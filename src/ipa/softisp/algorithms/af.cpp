/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, postmarketOS camera enablement
 *
 * Contrast-detection autofocus
 */

#include "af.h"

#include <algorithm>
#include <cmath>
#include <errno.h>
#include <limits>

#include <linux/v4l2-controls.h>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

namespace libcamera {

LOG_DEFINE_CATEGORY(IPASoftIspAf)

namespace ipa::softisp::algorithms {

int Af::init(IPAContext &context, const ValueNode &tuningData)
{
	const auto focus = context.lensControls.find(V4L2_CID_FOCUS_ABSOLUTE);
	if (focus == context.lensControls.end()) {
		LOG(IPASoftIspAf, Warning)
			<< "Focus tuning is present but no focus lens was found";
		return 0;
	}

	lensMin_ = focus->second.min().get<int32_t>();
	lensMax_ = focus->second.max().get<int32_t>();
	infinityPosition_ = std::clamp(
		tuningData["infinityPosition"].get<int32_t>(lensMin_),
		lensMin_, lensMax_);

	const int32_t direction = infinityPosition_ == lensMax_ ? -1 : 1;
	const int32_t defaultStep =
		std::max<int32_t>(1, (lensMax_ - lensMin_) / 32);
	oneMetrePosition_ = tuningData["oneMetrePosition"].get<int32_t>(
		infinityPosition_ + direction * defaultStep);
	oneMetrePosition_ = std::clamp(oneMetrePosition_, lensMin_, lensMax_);

	const int32_t steps = oneMetrePosition_ - infinityPosition_;
	if (!steps) {
		LOG(IPASoftIspAf, Error)
			<< "The one-metre and infinity lens positions must differ";
		return -EINVAL;
	}

	stepsPerDioptre_ = steps;
	const int32_t defaultMacro = steps > 0 ? lensMax_ : lensMin_;
	macroPosition_ = std::clamp(
		tuningData["macroPosition"].get<int32_t>(defaultMacro),
		lensMin_, lensMax_);

	if ((macroPosition_ - infinityPosition_) * steps <= 0) {
		LOG(IPASoftIspAf, Error)
			<< "The macro lens position is on the wrong side of infinity";
		return -EINVAL;
	}

	maxDioptres_ = lensPosition(macroPosition_);
	coarseSteps_ = tuningData["coarseSteps"].get<unsigned int>(coarseSteps_);
	fineRange_ = tuningData["fineRange"].get<int32_t>(fineRange_);
	fineStep_ = tuningData["fineStep"].get<int32_t>(fineStep_);
	settleFrames_ = tuningData["settleFrames"].get<unsigned int>(settleFrames_);
	retriggerRatio_ = tuningData["retriggerRatio"].get<double>(retriggerRatio_);
	retriggerFrames_ =
		tuningData["retriggerFrames"].get<unsigned int>(retriggerFrames_);

	if (!coarseSteps_ || fineRange_ <= 0 || fineStep_ <= 0 ||
	    retriggerRatio_ <= 0.0 || retriggerRatio_ >= 1.0 ||
	    !retriggerFrames_) {
		LOG(IPASoftIspAf, Error) << "Invalid autofocus tuning data";
		return -EINVAL;
	}

	context.ctrlMap[&controls::AfMode] =
		ControlInfo(controls::AfModeValues, controls::AfModeManual);
	context.ctrlMap[&controls::AfTrigger] =
		ControlInfo(controls::AfTriggerValues);
	context.ctrlMap[&controls::AfPause] =
		ControlInfo(controls::AfPauseValues);
	context.ctrlMap[&controls::LensPosition] =
		ControlInfo(0.0f, maxDioptres_, 0.0f);

	enabled_ = true;
	return 0;
}

int Af::configure(IPAContext &context,
		  [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	if (!enabled_)
		return 0;

	phase_ = Phase::Idle;
	positions_.clear();
	positionIndex_ = 0;
	settleRemaining_ = 0;
	bestPosition_ = infinityPosition_;
	bestFoM_ = -1;
	referenceFoM_ = 0;
	lowFoMFrames_ = 0;
	paused_ = false;
	pauseDeferred_ = false;

	context.activeState.af.mode = controls::AfModeManual;
	context.activeState.af.state = controls::AfStateIdle;
	context.activeState.af.pauseState = controls::AfPauseStateRunning;
	context.activeState.af.lensPosition = infinityPosition_;
	context.activeState.af.lensUpdate = true;

	return 0;
}

void Af::cancelScan(IPAContext &context)
{
	phase_ = Phase::Idle;
	positions_.clear();
	positionIndex_ = 0;
	settleRemaining_ = 0;
	lowFoMFrames_ = 0;
	context.activeState.af.state = controls::AfStateIdle;
}

void Af::setLensPosition(IPAContext &context, int32_t position)
{
	position = std::clamp(position, lensMin_, lensMax_);
	if (position == context.activeState.af.lensPosition)
		return;

	context.activeState.af.lensPosition = position;
	context.activeState.af.lensUpdate = true;
}

float Af::lensPosition(int32_t position) const
{
	return (position - infinityPosition_) / stepsPerDioptre_;
}

void Af::startScan(IPAContext &context)
{
	positions_.clear();
	for (unsigned int i = 0; i <= coarseSteps_; ++i) {
		const int64_t range = macroPosition_ - infinityPosition_;
		const int32_t position =
			infinityPosition_ + range * i / coarseSteps_;
		if (positions_.empty() || positions_.back() != position)
			positions_.push_back(position);
	}

	phase_ = Phase::Coarse;
	positionIndex_ = 0;
	settleRemaining_ = settleFrames_;
	bestPosition_ = infinityPosition_;
	bestFoM_ = -1;
	referenceFoM_ = 0;
	lowFoMFrames_ = 0;
	context.activeState.af.state = controls::AfStateScanning;
	setLensPosition(context, positions_.front());
}

void Af::startFineScan(IPAContext &context)
{
	const int32_t scanMin = std::min(infinityPosition_, macroPosition_);
	const int32_t scanMax = std::max(infinityPosition_, macroPosition_);
	const int32_t fineMin = std::max(scanMin, bestPosition_ - fineRange_);
	const int32_t fineMax = std::min(scanMax, bestPosition_ + fineRange_);

	positions_.clear();
	for (int32_t position = fineMin; position <= fineMax;
	     position += fineStep_)
		positions_.push_back(position);
	if (positions_.empty() || positions_.back() != fineMax)
		positions_.push_back(fineMax);

	phase_ = Phase::Fine;
	positionIndex_ = 0;
	settleRemaining_ = settleFrames_;
	bestFoM_ = -1;
	setLensPosition(context, positions_.front());
}

void Af::finishScan(IPAContext &context)
{
	setLensPosition(context, bestPosition_);
	phase_ = Phase::Settling;
	settleRemaining_ = settleFrames_;
}

void Af::queueRequest(IPAContext &context,
		      [[maybe_unused]] const uint32_t frame,
		      [[maybe_unused]] IPAFrameContext &frameContext,
		      const ControlList &requestControls)
{
	if (!enabled_)
		return;

	const auto &mode = requestControls.get(controls::AfMode);
	if (mode && *mode != context.activeState.af.mode) {
		context.activeState.af.mode = *mode;
		cancelScan(context);
		paused_ = false;
		pauseDeferred_ = false;
		context.activeState.af.pauseState = controls::AfPauseStateRunning;

		if (*mode == controls::AfModeContinuous)
			startScan(context);
	}

	const auto &trigger = requestControls.get(controls::AfTrigger);
	if (trigger && context.activeState.af.mode == controls::AfModeAuto) {
		if (*trigger == controls::AfTriggerStart &&
		    phase_ == Phase::Idle)
			startScan(context);
		else if (*trigger == controls::AfTriggerCancel &&
			 (phase_ == Phase::Coarse || phase_ == Phase::Fine ||
			  phase_ == Phase::Settling))
			cancelScan(context);
	}

	const auto &pause = requestControls.get(controls::AfPause);
	if (pause && context.activeState.af.mode == controls::AfModeContinuous) {
		switch (*pause) {
		case controls::AfPauseImmediate:
			paused_ = true;
			pauseDeferred_ = false;
			context.activeState.af.pauseState =
				controls::AfPauseStatePaused;
			break;
		case controls::AfPauseDeferred:
			if (paused_) {
				pauseDeferred_ = false;
				context.activeState.af.pauseState =
					controls::AfPauseStatePaused;
			} else if (phase_ == Phase::Coarse || phase_ == Phase::Fine ||
				   phase_ == Phase::Settling) {
				pauseDeferred_ = true;
				context.activeState.af.pauseState =
					controls::AfPauseStatePausing;
			} else {
				paused_ = true;
				pauseDeferred_ = false;
				context.activeState.af.pauseState =
					controls::AfPauseStatePaused;
			}
			break;
		case controls::AfPauseResume:
			paused_ = false;
			pauseDeferred_ = false;
			context.activeState.af.pauseState =
				controls::AfPauseStateRunning;
			if (phase_ == Phase::Idle)
				startScan(context);
			break;
		}
	}

	if (context.activeState.af.mode != controls::AfModeManual)
		return;

	const auto &position = requestControls.get(controls::LensPosition);
	if (!position)
		return;

	const float dioptres = std::clamp(*position, 0.0f, maxDioptres_);
	const int32_t rawPosition = std::lround(
		infinityPosition_ + dioptres * stepsPerDioptre_);
	setLensPosition(context, rawPosition);
}

void Af::process(IPAContext &context,
		 [[maybe_unused]] const uint32_t frame,
		 [[maybe_unused]] IPAFrameContext &frameContext,
		 const SwIspStats *stats,
		 ControlList &metadata)
{
	if (!enabled_)
		return;

	const int32_t frameLensPosition = context.activeState.af.lensPosition;
	metadata.set(controls::AfMode, context.activeState.af.mode);
	metadata.set(controls::AfState, context.activeState.af.state);
	metadata.set(controls::AfPauseState,
		     context.activeState.af.pauseState);
	metadata.set(controls::LensPosition, lensPosition(frameLensPosition));

	if (!stats->valid || !stats->focusLuminance)
		return;

	const uint64_t normalized =
		stats->focusGradient * 1000000 / stats->focusLuminance;
	const int32_t focusFoM = static_cast<int32_t>(
		std::min<uint64_t>(normalized, std::numeric_limits<int32_t>::max()));
	metadata.set(controls::FocusFoM, focusFoM);

	if (paused_)
		return;

	if (phase_ == Phase::Settling) {
		if (settleRemaining_) {
			--settleRemaining_;
			return;
		}

		referenceFoM_ = std::max(0, bestFoM_);
		context.activeState.af.state =
			bestFoM_ > 0 ? controls::AfStateFocused
				     : controls::AfStateFailed;
		phase_ = context.activeState.af.mode == controls::AfModeContinuous
				 ? Phase::Monitoring
				 : Phase::Idle;
		if (pauseDeferred_) {
			paused_ = true;
			pauseDeferred_ = false;
			context.activeState.af.pauseState =
				controls::AfPauseStatePaused;
		}
		return;
	}

	if (phase_ == Phase::Monitoring) {
		if (referenceFoM_ > 0 &&
		    focusFoM < referenceFoM_ * retriggerRatio_) {
			if (++lowFoMFrames_ >= retriggerFrames_)
				startScan(context);
		} else {
			lowFoMFrames_ = 0;
		}
		return;
	}

	if (phase_ != Phase::Coarse && phase_ != Phase::Fine)
		return;

	if (settleRemaining_) {
		--settleRemaining_;
		return;
	}

	if (focusFoM > bestFoM_) {
		bestFoM_ = focusFoM;
		bestPosition_ = positions_[positionIndex_];
	}

	if (++positionIndex_ < positions_.size()) {
		setLensPosition(context, positions_[positionIndex_]);
		settleRemaining_ = settleFrames_;
		return;
	}

	if (phase_ == Phase::Coarse)
		startFineScan(context);
	else
		finishScan(context);
}

REGISTER_IPA_ALGORITHM(Af, "Af")

} /* namespace ipa::softisp::algorithms */

} /* namespace libcamera */

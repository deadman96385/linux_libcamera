/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, postmarketOS camera enablement
 *
 * Contrast-detection autofocus
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <vector>

#include "algorithm.h"

namespace libcamera {

namespace ipa::softisp::algorithms {

class Af : public Algorithm
{
public:
	int init(IPAContext &context, const ValueNode &tuningData) override;
	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;

	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const SwIspStats *stats,
		     ControlList &metadata) override;

private:
	enum class Phase {
		Idle,
		Coarse,
		Fine,
		Settling,
		Monitoring,
	};

	void cancelScan(IPAContext &context);
	void startScan(IPAContext &context);
	void startFineScan(IPAContext &context);
	void finishScan(IPAContext &context);
	void setLensPosition(IPAContext &context, int32_t position);
	float lensPosition(int32_t position) const;

	bool enabled_ = false;
	int32_t lensMin_ = 0;
	int32_t lensMax_ = 0;
	int32_t infinityPosition_ = 0;
	int32_t oneMetrePosition_ = 0;
	int32_t macroPosition_ = 0;
	float stepsPerDioptre_ = 1.0f;
	float maxDioptres_ = 0.0f;

	unsigned int coarseSteps_ = 6;
	int32_t fineRange_ = 24;
	int32_t fineStep_ = 6;
	unsigned int settleFrames_ = 1;
	double retriggerRatio_ = 0.6;
	unsigned int retriggerFrames_ = 3;

	Phase phase_ = Phase::Idle;
	std::vector<int32_t> positions_;
	std::size_t positionIndex_ = 0;
	unsigned int settleRemaining_ = 0;
	int32_t bestPosition_ = 0;
	int32_t bestFoM_ = -1;
	int32_t referenceFoM_ = 0;
	unsigned int lowFoMFrames_ = 0;
	bool paused_ = false;
	bool pauseDeferred_ = false;
};

} /* namespace ipa::softisp::algorithms */

} /* namespace libcamera */

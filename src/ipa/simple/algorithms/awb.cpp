/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024-2026 Red Hat Inc.
 *
 * Auto white balance
 */

#include "awb.h"

#include <numeric>
#include <stdint.h>
#include <vector>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include "libipa/colours.h"
#include "simple/ipa_context.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPASoftAwb)

namespace ipa::soft::algorithms {

int Awb::init(IPAContext &context, const ValueNode &tuningData)
{
	context.ctrlMap[&controls::AwbEnable] = ControlInfo(false, true, true);
	context.ctrlMap[&controls::ColourGains] = ControlInfo(0.0f, 8.0f);

	const ValueNode &sets = tuningData["colourGains"];
	if (sets.isList() && sets.size()) {
		const ValueNode &set = sets[0];
		const std::vector<float> gains =
			set["gains"].get<std::vector<float>>().value_or(
				std::vector<float>{});
		if (gains.size() == 2) {
			initialGains_ = { { gains[0], 1.0f, gains[1] } };
			initialTemperatureK_ = set["ct"].get<unsigned int>(5000);
		}
	}

	return 0;
}

int Awb::configure(IPAContext &context,
		   [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	auto &gains = context.activeState.awb.gains;
	gains = initialGains_;
	context.activeState.awb.temperatureK = initialTemperatureK_;
	context.activeState.awb.automatic = true;

	return 0;
}

void Awb::queueRequest(IPAContext &context,
		       [[maybe_unused]] const uint32_t frame,
		       [[maybe_unused]] IPAFrameContext &frameContext,
		       const ControlList &requestControls)
{
	if (const auto &automatic = requestControls.get(controls::AwbEnable))
		context.activeState.awb.automatic = *automatic;

	if (!context.activeState.awb.automatic) {
		if (const auto &gains = requestControls.get(controls::ColourGains))
			context.activeState.awb.gains =
				{ { (*gains)[0], 1.0f, (*gains)[1] } };
	}
}

void Awb::prepare(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  DebayerParams *params)
{
	auto &gains = context.activeState.awb.gains;

	frameContext.gains = gains;
	params->gains = gains;
}

void Awb::process(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  const SwIspStats *stats,
		  ControlList &metadata)
{
	const SwIspStats::Histogram &histogram = stats->yHistogram;
	const uint8_t blackLevel = context.activeState.blc.level;

	metadata.set(controls::ColourGains, { frameContext.gains.r(),
					      frameContext.gains.b() });
	metadata.set(controls::AwbEnable, context.activeState.awb.automatic);
	metadata.set(controls::ColourTemperature,
		     context.activeState.awb.temperatureK);

	if (!stats->valid || !context.activeState.awb.automatic)
		return;

	/*
	 * Black level must be subtracted to get the correct AWB ratios, they
	 * would be off if they were computed from the whole brightness range
	 * rather than from the sensor range.
	 */
	const uint64_t nPixels = std::accumulate(
		histogram.begin(), histogram.end(), uint64_t(0));
	const uint64_t offset = blackLevel * nPixels;
	const uint64_t minValid = 1;
	/*
	 * Make sure the sums are at least minValid, while preventing unsigned
	 * integer underflow.
	 */
	const RGB<uint64_t> sum = stats->sum_.max(offset + minValid) - offset;

	/*
	 * Calculate red and blue gains for AWB.
	 * Clamp max gain at 4.0, this also avoids 0 division.
	 */
	auto &gains = context.activeState.awb.gains;
	gains = { {
		sum.r() <= sum.g() / 4 ? 4.0f : static_cast<float>(sum.g()) / sum.r(),
		1.0,
		sum.b() <= sum.g() / 4 ? 4.0f : static_cast<float>(sum.g()) / sum.b(),
	} };

	RGB<double> rgbGains{ { 1 / gains.r(), 1 / gains.g(), 1 / gains.b() } };
	context.activeState.awb.temperatureK = estimateCCT(rgbGains);
	metadata.set(controls::ColourTemperature, context.activeState.awb.temperatureK);

	LOG(IPASoftAwb, Debug)
		<< "gain R/B: " << gains << "; temperature: "
		<< context.activeState.awb.temperatureK;
}

REGISTER_IPA_ALGORITHM(Awb, "Awb")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */

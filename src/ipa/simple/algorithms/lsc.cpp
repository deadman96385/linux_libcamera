/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Lens shading correction
 */

#include "lsc.h"

#include <algorithm>
#include <cmath>

#include <libcamera/base/log.h>

namespace libcamera {

namespace ipa::soft::algorithms {

LOG_DEFINE_CATEGORY(IPASoftLsc)

int Lsc::init(IPAContext &context, const ValueNode &tuningData)
{
	int retR = lscR_.readYaml(tuningData["sets"], "ct", "r");
	int retG = lscG_.readYaml(tuningData["sets"], "ct", "g");
	int retB = lscB_.readYaml(tuningData["sets"], "ct", "b");
	if (retR < 0 || retG < 0 || retB < 0) {
		LOG(IPASoftLsc, Error)
			<< "Failed to parse LSC tables from tuning file";
		return -EINVAL;
	}

	context.lscEnabled = true;

	return 0;
}

int Lsc::configure([[maybe_unused]] IPAContext &context,
		   [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	return 0;
}

void Lsc::prepare(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  [[maybe_unused]] IPAFrameContext &frameContext,
		  DebayerParams *params)
{
	unsigned int ct = context.activeState.awb.temperatureK;
	if (ct == 0)
		ct = 5000;

	constexpr unsigned int minTemperatureChange = 100;
	if (params->lscLutVersion != 0 &&
	    utils::abs_diff(ct, lastAppliedCt_) < minTemperatureChange)
		return;

	const LscMatrix &red = lscR_.getInterpolated(ct);
	const LscMatrix &green = lscG_.getInterpolated(ct);
	const LscMatrix &blue = lscB_.getInterpolated(ct);

	DebayerParams::LscLookupTable &lut = params->lscLut;
	constexpr unsigned int gridSize = DebayerParams::kLscGridSize;
	for (unsigned int i = 0, j = 0; i < gridSize * gridSize; i++) {
		lut[j++] = std::clamp(std::lround(red.data()[i]), 0l, 255l);
		lut[j++] = std::clamp(std::lround(green.data()[i]), 0l, 255l);
		lut[j++] = std::clamp(std::lround(blue.data()[i]), 0l, 255l);
		lut[j++] = 0; /* padding */
	}
	params->lscLutVersion++;

	lastAppliedCt_ = ct;
}
REGISTER_IPA_ALGORITHM(Lsc, "Lsc")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */

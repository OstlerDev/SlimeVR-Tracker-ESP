/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 Gorbit99 & SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/

#include "magdriver.h"

namespace SlimeVR::Sensors::SoftFusion {

std::vector<MagDefinition> MagDriver::supportedMags{
	MagDefinition{
		.name = "QMC6309",

		.deviceId = 0x7c,

		.whoAmIReg = 0x00,
		.expectedWhoAmI = 0x90,
		.dummyBytes = 0,

		.dataWidth = MagDataWidth::SixByte,
		.dataReg = 0x01,

		.setup =
			[](MagInterface& interface) {
				interface.writeByte(0x0b, 0x80);
				interface.writeByte(0x0b, 0x00);  // Soft reset
				delay(10);
				interface.writeByte(0x0b, 0x48);  // Set/reset on, 8g full range, 200Hz
				interface.writeByte(
					0x0a,
					0x21
				);  // LP filter 2, 8x Oversampling, normal mode
				return true;
			},

		.resolution = 0.025f,
	},
	MagDefinition{
		.name = "IST8306",

		.deviceId = 0x19,

		.whoAmIReg = 0x00,
		.expectedWhoAmI = 0x06,
		.dummyBytes = 0,

		.dataWidth = MagDataWidth::SixByte,
		.dataReg = 0x11,

		.setup =
			[](MagInterface& interface) {
				interface.writeByte(0x32, 0x01);  // Soft reset
				delay(50);
				interface.writeByte(0x30, 0x20);  // Noise suppression: low
				interface.writeByte(0x41, 0x2d);  // Oversampling: 32X
				interface.writeByte(0x31, 0x02);  // Continuous measurement @ 10Hz
				return true;
			},

		.resolution = 0.3,
	},
	MagDefinition{
		.name = "BMM350",

		.deviceId = 0x14, // adsel low
		// .deviceId = 0x15, // adsel high

		.whoAmIReg = 0x00,
		.expectedWhoAmI = 0x33,
		.dummyBytes = 2, // always returns 2 dummy bytes to be discarded before good data is provided.

		.dataWidth = MagDataWidth::NineByte,
		.dataReg = 0x31, // data starts at 0x31

		.setup =
			[](MagInterface& interface) {
				// mirrors Bosch driver
				// soft reset
				interface.writeByte(0x7E, 0xB6); // CMD, CMD_SOFTRESET
				delay(24);
				interface.writeByte(0x50, 0x80); // OTP_CMD_REG, PWR_OFF_OTP
				// reset mag
				interface.writeByte(0x06, 0x00); // PMU_CMD, PMU_CMD_SUS (suspend)
				interface.writeByte(0x06, 0x07); // PMU_CMD, PMU_CMD_BR (bit reset)
				interface.writeByte(0x06, 0x05); // PMU_CMD, PMU_CMD_FGR (flux guide reset)
				// set interrupt mode 
				interface.writeByte(0x2E, 0b10000110); // INT_CTRL, (mag data ready, push-pull, polarity active high)
				// enable all axies
				interface.writeByte(0x05, 0b00000111); // PMU_CMD_AXIS_EN, (en_z, en_y, en_x)
				// TODO:
				// read OTP calibration data from chip, save calibration for later use (bmm3_read_otp_word(), bmm3_parse_compensation(), bmm3_update_odr());
				return true;
			},

		.resolution = 1,
	},
};

bool MagDriver::init(MagInterface&& interface, bool supports9ByteMags) {
	for (auto& mag : supportedMags) {
		interface.setDeviceId(mag.deviceId);
		interface.setDummyBytes(mag.dummyBytes);

		logger.info("Trying mag: %s", mag.name);

		uint8_t whoAmI = interface.readByte(mag.whoAmIReg);
		if (whoAmI != mag.expectedWhoAmI) {
			continue;
		}

		if (!supports9ByteMags && mag.dataWidth == MagDataWidth::NineByte) {
			logger.error("The sensor doesn't support this mag!");
			return false;
		}

		logger.info("Found mag %s! Initializing...", mag.name);

		if (!mag.setup(interface)) {
			logger.error("Mag %s failed to initialize!", mag.name);
			return false;
		}
		logger.info("Initialized mag %s!", mag.name);

		detectedMag = mag;

		break;
	}

	if (!detectedMag.has_value()) {
		logger.info("No mag detected.");
	}

	this->interface = interface;
	return detectedMag.has_value();
}

void MagDriver::startPolling() const {
	if (!detectedMag) {
		return;
	}

	interface.startPolling(detectedMag->dataReg, detectedMag->dataWidth);
}

void MagDriver::stopPolling() const {
	if (!detectedMag) {
		return;
	}

	interface.stopPolling();
}

void MagDriver::scaleMagSample(const uint8_t* magSample, float* scaled) const {
#pragma pack(push, 1)
	struct MagData6Byte {
		int16_t x;
		int16_t y;
		int16_t z;
	};
	struct MagData9Byte {
		int32_t x : 24;
		int32_t y : 24;
		int32_t z : 24;
	};
#pragma pack(pop)

	if (!detectedMag) {
		return;
	}

	if (detectedMag->dataWidth == MagDataWidth::SixByte) {
		const auto* data = reinterpret_cast<const MagData6Byte*>(magSample);
		scaled[0] = data->x * detectedMag->resolution;
		scaled[1] = data->y * detectedMag->resolution;
		scaled[2] = data->z * detectedMag->resolution;
	} else {
		const auto* data = reinterpret_cast<const MagData9Byte*>(magSample);
		scaled[0] = data->x * detectedMag->resolution;
		scaled[1] = data->y * detectedMag->resolution;
		scaled[2] = data->z * detectedMag->resolution;
	}
}

const char* MagDriver::getAttachedMagName() const {
	if (!detectedMag) {
		return nullptr;
	}

	return detectedMag->name;
}

bool MagDriver::isMagAttached() const { return detectedMag.has_value(); }

}  // namespace SlimeVR::Sensors::SoftFusion

/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 Gorbit99 & SlimeVR Contributors

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

#pragma once

#include <algorithm>
#include <array>
#include <cstdint>

#include "I2CDev.h"
#include "../../../sensorinterface/RegisterInterface.h"
#include "../magdriver.h"
#include "callbacks.h"

namespace SlimeVR::Sensors::SoftFusion::Drivers {

struct LSM6DSOutputHandler {
	LSM6DSOutputHandler(
		RegisterInterface& registerInterface,
		SlimeVR::Logging::Logger& logger
	)
		: m_RegisterInterface(registerInterface)
		, m_Logger(logger) {}

	RegisterInterface& m_RegisterInterface;
	SlimeVR::Logging::Logger& m_Logger;

	struct BaseRegs {
		struct IFCFG {
			static constexpr uint8_t reg = 0x03; // IF_CFG
			static constexpr uint8_t value = (0b01011000); // SHUB_PU_EN (pullup i2c), H_LACTIVE (int active low), PP_OD (int open drain)
		};
		struct CFGAccess {
			static constexpr uint8_t reg = 0x01; // FUNC_CFG_ACCESS
			static constexpr uint8_t main = (0b00000000); // default / main register bank
			static constexpr uint8_t shub = (0b01000000); // SHUB_REG_ACCESS
		};
		struct SHUBMasterConfig {
			static constexpr uint8_t reg = 0x14; // MASTER_CONFIG
			static constexpr uint8_t clear = (0b00000000);
			static constexpr uint8_t reset = (0b10000000); // RST_MASTER_REGS
			static constexpr uint8_t writeOnce = (0b01000000); // WRITE_ONCE
			static constexpr uint8_t passthrough = (0b00010000); // PASS_THROUGH_MODE
			static constexpr uint8_t masterOn = (0b00000100); // MASTER_ON
		};
		struct SHUBStatus {
			static constexpr uint8_t reg = (0x22); // STATUS_MASTER
		};
		struct SHUBSlv0Add {
			static constexpr uint8_t reg = (0x15); // SLV0_ADD
		};
		struct SHUBSlv0SubAdd {
			static constexpr uint8_t reg = (0x16); // SLV0_SUBADD
		};
		struct SHUBSlv0Config {
			static constexpr uint8_t reg = (0x17); // SLV0_CONFIG
			static constexpr uint8_t value = (0b10000000); // 120hz
			static constexpr uint8_t enableFifo = (0b00001000); // enable fifo data batching (BATCH_EXT_SENS_0_EN)
		};
		struct SHUBSlv0Datawrite {
			static constexpr uint8_t reg = (0x21); // DATAWRITE_SLV0
		};
		struct SHUBOut1 {
			static constexpr uint8_t reg = (0x02); // SENSOR_HUB_1
		};
	};

#pragma pack(push, 1)
	struct FifoEntryAligned {
		union {
			int16_t xyz[3];
			uint8_t raw[6];
		};
	};
#pragma pack(pop)

	static constexpr size_t FullFifoEntrySize = sizeof(FifoEntryAligned) + 1;

	template <typename Regs>
	bool bulkRead(
		DriverCallbacks<int16_t>&& callbacks,
		float GyrTs,
		float AccTs,
		float MagTs,
		float TempTs
	) {
		constexpr auto FIFO_SAMPLES_MASK = 0x3ff;
		constexpr auto FIFO_OVERRUN_LATCHED_MASK = 0x800;

		const auto fifo_status = m_RegisterInterface.readReg16(Regs::FifoStatus);
		const auto available_axes = fifo_status & FIFO_SAMPLES_MASK;
		const auto fifo_bytes = available_axes * FullFifoEntrySize;
		if (fifo_status & FIFO_OVERRUN_LATCHED_MASK) {
			// FIFO overrun is expected to happen during startup and calibration
			m_Logger.error(
				"FIFO OVERRUN! This occuring during normal usage is an issue."
			);
		}

		std::array<uint8_t, FullFifoEntrySize * 8> read_buffer;  // max 8 readings
		const auto bytes_to_read = std::min(
									   static_cast<size_t>(read_buffer.size()),
									   static_cast<size_t>(fifo_bytes)
								   )
								 / FullFifoEntrySize * FullFifoEntrySize;
		m_RegisterInterface
			.readBytes(Regs::FifoData, bytes_to_read, read_buffer.data());
		for (auto i = 0u; i < bytes_to_read; i += FullFifoEntrySize) {
			FifoEntryAligned entry;
			uint8_t tag = read_buffer[i] >> 3;
			memcpy(
				entry.raw,
				&read_buffer[i + 0x1],
				sizeof(FifoEntryAligned)
			);  // skip fifo header

			switch (tag) {
				case 0x01:  // Gyro NC
					callbacks.processGyroSample(entry.xyz, GyrTs);
					break;
				case 0x02:  // Accel NC
					callbacks.processAccelSample(entry.xyz, AccTs);
					break;
				case 0x03:  // Temperature
					callbacks.processTempSample(entry.xyz[0], TempTs);
					break;
				case 0x0e: // Sensor Hub Slave 0
					if (magPollingEnabled && magDataWidth == MagDataWidth::SixByte) {
						uint64_t now = millis();
						if (now - lastMagPollMillis >= MagTs * 1000) {
							callbacks.processMagSample(reinterpret_cast<uint8_t*>(entry.xyz), MagTs);
							lastMagPollMillis = now;
						}
					}
					break;
			}
		}
		// nine byte mags do not fit in a single fifo entry and are processed differently
		if (magPollingEnabled && magDataWidth == MagDataWidth::NineByte) {
			uint64_t now = millis();
			if (now - lastMagPollMillis >= MagTs * 1000) {
				size_t dataSize = 9 + auxDeviceDummyBytes;
				uint8_t auxSensorData[dataSize] = {0};
				readAuxData(magDataReg, dataSize, auxSensorData);
				// process sample after dummy byte offset
				callbacks.processMagSample(&auxSensorData[auxDeviceDummyBytes], MagTs);
				lastMagPollMillis = now;
			}
		}
		return fifo_bytes > bytes_to_read;
	}

	uint8_t auxDeviceId = 0x00;
	void setAuxId(uint8_t deviceId) {
		auxDeviceId = deviceId;
	}

	int auxDeviceDummyBytes = 0;
	void setAuxDummyBytes(int dummyBytes) {
		auxDeviceDummyBytes = dummyBytes;
	}

	// Setup SensorHub SLV0
	void setupAux(uint8_t address, bool writeMode) {
		m_RegisterInterface.writeReg(BaseRegs::CFGAccess::reg, BaseRegs::CFGAccess::main);
		m_RegisterInterface.writeReg(BaseRegs::IFCFG::reg, BaseRegs::IFCFG::value); // pullup i2c master & setup int pin
		// reset sensor hub
		m_RegisterInterface.writeReg(BaseRegs::CFGAccess::reg, BaseRegs::CFGAccess::shub); // switch to sensor hub regs
		m_RegisterInterface.writeReg(BaseRegs::SHUBMasterConfig::reg, BaseRegs::SHUBMasterConfig::reset); // trigger reset
		m_RegisterInterface.writeReg(BaseRegs::SHUBMasterConfig::reg, BaseRegs::SHUBMasterConfig::clear); // set back to 0 after reset
		// setup access to aux on slv0
		m_RegisterInterface.writeReg(BaseRegs::SHUBSlv0Add::reg, (auxDeviceId << 1) | (writeMode ? 0x00 : 0x01) ); // SLV0_ADD = deviceId, set to read or write mode
		m_RegisterInterface.writeReg(BaseRegs::SHUBSlv0SubAdd::reg, address); // SLV0_SUBADD = address
		// go back to page default page to be kind, later functions should switch to the sensor hub registers themselves.
		m_RegisterInterface.writeReg(BaseRegs::CFGAccess::reg, BaseRegs::CFGAccess::main);
	}

	// blocking i2c passthrough write
	void writeAux(uint8_t address, uint8_t value) {
		// readAux(address); // check value before change
		setupAux(address, true);
		m_RegisterInterface.writeReg(BaseRegs::CFGAccess::reg, BaseRegs::CFGAccess::shub); // enable reading from sensor hub
		m_RegisterInterface.writeReg(BaseRegs::SHUBSlv0Config::reg, BaseRegs::SHUBSlv0Config::value);
		m_RegisterInterface.writeReg(BaseRegs::SHUBSlv0Datawrite::reg, value); // set data to write
		m_RegisterInterface.writeReg(BaseRegs::SHUBMasterConfig::reg, BaseRegs::SHUBMasterConfig::masterOn | BaseRegs::SHUBMasterConfig::writeOnce);
		delay(50);
		uint8_t shub_status = m_RegisterInterface.readReg(BaseRegs::SHUBStatus::reg);
		bool err = false;
		if (shub_status & 0b00001000) { // verify SLAVE0_NACK
			m_Logger.debug("Sensor Hub SLV0 NACK!");
			err = true;
		}
		if (!(shub_status & 0b10000000)) { // verify WR_ONCE_DONE
			m_Logger.debug("Sensor Hub SLV0 did not set WR_ONCE_DONE");
			err = true;
		}
		if (!(shub_status & 0b00000001)) { // verify SENS_HUB_ENDOP
			m_Logger.debug("Sensor Hub SLV0 did not set SENS_HUB_ENDOP");
			err = true;
		}
		if (err){
			m_Logger.debug("Error during writeAux (auxDevice: 0x%x) (addr: 0x%x), (value: 0x%x), (shub_status: 0x%x)", auxDeviceId, address, value, shub_status);
		}
		if (!err) {
			m_Logger.debug("Sensor Hub SLV0 successfully wrote to aux device (auxDevice: 0x%x) (addr: 0x%x), (value: 0x%x), (shub_status: 0x%x)", auxDeviceId, address, value, shub_status);
			// readAux(address); // check value after set
		}
		m_RegisterInterface.writeReg(BaseRegs::CFGAccess::reg, BaseRegs::CFGAccess::main); // return to main register bank
		return;
	}

	uint8_t readAux(uint8_t address) {
		uint8_t buffer = 0;
		readAux(address, 1, &buffer);
		m_Logger.debug("readAux response (auxDevice: 0x%x) (addr: 0x%x), (res: 0x%x)", auxDeviceId, address, buffer);
		return buffer;
	}

	void readAux(uint8_t address, uint8_t size, uint8_t* buffer) {
		setupAux(address, false);
		m_RegisterInterface.writeReg(BaseRegs::CFGAccess::reg, BaseRegs::CFGAccess::shub); // enable reading from sensor hub
		m_RegisterInterface.writeReg(BaseRegs::SHUBSlv0Config::reg, BaseRegs::SHUBSlv0Config::value | (0x01 + auxDeviceDummyBytes)); // read 1 byte + dummy bytes.
		m_RegisterInterface.writeReg(BaseRegs::SHUBMasterConfig::reg, BaseRegs::SHUBMasterConfig::masterOn); // turn on sensor hub i2c master
		delay(50);
		uint8_t shub_status = m_RegisterInterface.readReg(BaseRegs::SHUBStatus::reg);
		if (shub_status & 0b00001000) { // verify SLAVE0_NACK
			m_Logger.debug("Sensor Hub SLV0 NACK! (auxDevice: 0x%x) (addr: 0x%x), (shub_status: 0x%x)", auxDeviceId, address, shub_status);
			return;
		}
		m_Logger.debug("SensorHub Status: 0x%x", shub_status);
		// read into buffer
		m_RegisterInterface.readBytes(BaseRegs::SHUBOut1::reg + auxDeviceDummyBytes, size, buffer); // offset read by dummy bytes
		m_RegisterInterface.writeReg(BaseRegs::CFGAccess::reg, BaseRegs::CFGAccess::main); // return to main register bank
		return;
	}

	bool magPollingEnabled = false;
	uint8_t magDataReg = 0x00;
	MagDataWidth magDataWidth;
	uint64_t lastMagPollMillis = 0;
	void startAuxPolling(uint8_t dataReg, MagDataWidth dataWidth) {
		magDataReg = dataReg;
		magDataWidth = dataWidth;
		lastMagPollMillis = millis();

		if (dataWidth == MagDataWidth::SixByte) {
			// process 6 byte mags in bulkRead fifo
			setupAux(dataReg, false);
			m_RegisterInterface.writeReg(BaseRegs::CFGAccess::reg, BaseRegs::CFGAccess::shub); // enable reading from sensor hub regs
			uint8_t magDataSize = 6; // does not support dummy bytes, max slave read is 7 bytes.
			// configure and start reading
			m_RegisterInterface.writeReg(BaseRegs::SHUBSlv0Config::reg, BaseRegs::SHUBSlv0Config::value | BaseRegs::SHUBSlv0Config::enableFifo | magDataSize);
			m_RegisterInterface.writeReg(BaseRegs::SHUBMasterConfig::reg, BaseRegs::SHUBMasterConfig::masterOn); // turn on sensor hub i2c master
		}
		else if (dataWidth == MagDataWidth::NineByte) {
			// process 9 byte mags using direct i2c passthrough and readAuxData()
			m_RegisterInterface.writeReg(BaseRegs::CFGAccess::reg, BaseRegs::CFGAccess::shub); // enable reading from sensor hub regs
			m_RegisterInterface.writeReg(BaseRegs::SHUBMasterConfig::reg, BaseRegs::SHUBMasterConfig::reset); // trigger reset
			m_RegisterInterface.writeReg(BaseRegs::SHUBMasterConfig::reg, BaseRegs::SHUBMasterConfig::clear); // set back to 0 after reset
			m_RegisterInterface.writeReg(BaseRegs::SHUBMasterConfig::reg, BaseRegs::SHUBMasterConfig::passthrough); // turn on i2c passthrough
		}
		m_RegisterInterface.writeReg(BaseRegs::CFGAccess::reg, BaseRegs::CFGAccess::main); // return to main register bank
		magPollingEnabled = true;
	}

	// used to read an aux data buffer directly from the sensor using i2c passthrough mode
	void readAuxData(uint8_t reg, size_t length, uint8_t* buffer) {
		// only allowed when polling is enabled and we are a 9 byte mag
		// 6 byte mags use fifo batching
		if (!magPollingEnabled || magDataWidth != MagDataWidth::NineByte) {
			return;
		}
		bool success = I2Cdev::readBytes(auxDeviceId, reg, length, buffer);
		if (!success) {
			m_Logger.debug("Aux device read using i2c passthrough failed!");
		}
	}

	void stopAuxPolling() { 
		// reset sensor hub
		m_RegisterInterface.writeReg(BaseRegs::CFGAccess::reg, BaseRegs::CFGAccess::shub); // switch to sensor hub regs
		m_RegisterInterface.writeReg(BaseRegs::SHUBMasterConfig::reg, BaseRegs::SHUBMasterConfig::reset); // trigger reset
		m_RegisterInterface.writeReg(BaseRegs::SHUBMasterConfig::reg, BaseRegs::SHUBMasterConfig::clear); // set back to 0 after reset
		m_RegisterInterface.writeReg(BaseRegs::CFGAccess::reg, BaseRegs::CFGAccess::main); // return to main register bank
		magPollingEnabled = false; 
	}
};

}  // namespace SlimeVR::Sensors::SoftFusion::Drivers

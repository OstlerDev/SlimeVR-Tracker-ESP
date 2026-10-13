/*
  SlimeVR Code is placed under the MIT license
  Copyright (c) 2026 SlimeVR Contributors

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

#include "I2CDev.h"
#include "lsm6dsv.h"

namespace SlimeVR::Sensors::SoftFusion::Drivers {

struct ISM330BX : LSM6DSV {
  static constexpr auto Name = "ISM330BX";
  static constexpr auto Type = SensorTypeID::ISM330BX;

  struct Regs : LSM6DSV::Regs {
    struct WhoAmI {
      static constexpr uint8_t reg = 0x0f;
      static constexpr uint8_t value = 0x71;
        // ISM330BX, LSM6DSV16B, LSM6DSVDTR
    };
  };

  using LSM6DSV::LSM6DSV;

  // No sensor hub, so just use direct i2c bus access
  void writeAux(uint8_t address, uint8_t value) {
    bool success = I2Cdev::writeByte(auxDeviceId, address, value);
    if (!success) {
      m_Logger.trace("Aux device write using i2c direct failed!");
    }
    if (success) {
      m_Logger.trace("Wrote to aux device (auxDevice: 0x%x) (addr: 0x%x), (value: 0x%x)", auxDeviceId, address, value);
    }
  }
  uint8_t readAux(uint8_t address) {
    uint8_t buffer = 0;
    readAuxData(address, 1, &buffer);
    m_Logger.trace("readAux response (auxDevice: 0x%x) (addr: 0x%x), (res: 0x%x)", auxDeviceId, address, buffer);
    return buffer;
  }
  void startAuxPolling(uint8_t dataReg, MagDataWidth dataWidth) {
    magDataReg = dataReg;
    magDataWidth = dataWidth;
    lastMagPollMillis = millis();
    magPollingEnabled = true;
  }
  void stopAuxPolling() {}

  bool bulkRead(DriverCallbacks<int16_t>&& cb) {
    DriverCallbacks<int16_t> callbacks = {
      .processAccelSample = [&](const auto sample[3], float AccTs) {
        // the ISM330BX, LSM6DSV16B, and LSM6DSVDTR return their accel in zyx format 
        // so we just need to flip em and return em.
        int16_t xyz[3] = {sample[2], sample[1], sample[0]};
        cb.processAccelSample(xyz, AccTs);
      },
      .processGyroSample = std::move(cb.processGyroSample),
      .processTempSample = std::move(cb.processTempSample),
      .processMagSample = std::move(cb.processMagSample),
    };

    return LSM6DSOutputHandler::template bulkRead<Regs>(
      std::move(callbacks),
      GyrTs,
      AccTs,
      MagTs,
      TempTs
    );
  }
};

}  // namespace SlimeVR::Sensors::SoftFusion::Drivers
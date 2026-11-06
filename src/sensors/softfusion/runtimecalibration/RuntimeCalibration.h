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

#include <vector3.h>

#include <cstdint>

#include "../../../GlobalVars.h"
#include "../../../configuration/Configuration.h"
#include "AccelBiasCalibrationStep.h"
#include "GyroBiasCalibrationStep.h"
#include "MotionlessCalibrationStep.h"
#include "NullCalibrationStep.h"
#include "SampleRateCalibrationStep.h"
#include "configuration/SensorConfig.h"
#include "logging/Logger.h"
#include "sensors/SensorFusion.h"
#include "sensors/softfusion/CalibrationBase.h"

namespace SlimeVR::Sensors::RuntimeCalibration {

template <typename IMU>
class RuntimeCalibrator : public Sensors::CalibrationBase<IMU> {
public:
	static constexpr bool HasUpsideDownCalibration = false;

	using Base = Sensors::CalibrationBase<IMU>;
	using Self = RuntimeCalibrator<IMU>;
	using Consts = typename Base::Consts;
	using RawSensorT = typename Consts::RawSensorT;
	using RawVectorT = typename Consts::RawVectorT;

	RuntimeCalibrator(
		SensorFusion& fusion,
		IMU& imu,
		uint8_t sensorId,
		Logging::Logger& logger,
		SensorToggleState& toggles
	)
		: Base{fusion, imu, sensorId, logger, toggles},
		magCalibrator{fusion.getVQF()} 
	{
		calibration.T_Ts = Consts::getDefaultTempTs();
		activeCalibration.T_Ts = Consts::getDefaultTempTs();
	}

	bool calibrationMatches(const Configuration::SensorConfig& sensorCalibration
	) final {
		return sensorCalibration.type
				== SlimeVR::Configuration::SensorConfigType::RUNTIME_CALIBRATION
			&& (sensorCalibration.data.sfusion.ImuType == IMU::Type)
			&& (sensorCalibration.data.sfusion.MotionlessDataLen
				== Base::MotionlessCalibDataSize());
	}

	void assignCalibration(const Configuration::SensorConfig& sensorCalibration) final {
		calibration = sensorCalibration.data.runtimeCalibration;
		activeCalibration = sensorCalibration.data.runtimeCalibration;
		if (!toggles.getToggle(SensorToggles::CalibrationEnabled)) {
			activeCalibration.gyroPointsCalibrated = 0;
			for (size_t i = 0; i < 3; i++) {
				activeCalibration.G_off1[i] = 0;
				activeCalibration.G_off2[i] = 0;
			}

			for (size_t i = 0; i < 3; i++) {
				activeCalibration.accelCalibrated[i] = false;
				activeCalibration.A_off[i] = 0;
			}
		} else {
			calculateZROChange();
		}

		magCalibrator.setMBias(activeCalibration.M_Bias);
		magCalibrator.setMAinv(activeCalibration.M_Ainv);
		magCalibrator.setHM(activeCalibration.M_hm);

		currentStep = &nullCalibrationStep;
	}

	void begin() final {
		startupMillis = millis();

		gyroBiasCalibrationStep.swapCalibrationIfNecessary();

		currentStep = &sampleRateCalibrationStep;
		currentStep->start();
		nextCalibrationStep = CalibrationStepEnum::SAMPLING_RATE;

		calculateZROChange();

		printCalibration();
	}

	void tick() final {
		if (skippedAStep && !lastTickRest && fusion.getRestDetected()) {
			computeNextCalibrationStep();
			skippedAStep = false;
		}

		if (millis() - startupMillis < initialStartupDelaySeconds * 1e3) {
			return;
		}

		if (!fusion.getRestDetected() && currentStep->requiresRest()) {
			if (isCalibrating) {
				currentStep->cancel();
				isCalibrating = false;
			}

			lastTickRest = fusion.getRestDetected();
			return;
		}

		if (!isCalibrating) {
			isCalibrating = true;
			currentStep->start();
		}

		if (currentStep->requiresRest() && !currentStep->restDetectionDelayElapsed()) {
			lastTickRest = fusion.getRestDetected();
			return;
		}

		auto result = currentStep->tick();

		switch (result) {
			case CalibrationStep<RawSensorT>::TickResult::DONE:
				if (nextCalibrationStep == CalibrationStepEnum::SAMPLING_RATE) {
					stepCalibrationForward(true, false);
					break;
				}
				stepCalibrationForward();
				break;
			case CalibrationStep<RawSensorT>::TickResult::SKIP:
				stepCalibrationForward(false, false);
				break;
			case CalibrationStep<RawSensorT>::TickResult::CONTINUE:
				break;
		}

		lastTickRest = fusion.getRestDetected();
	}

	void scaleAccelSample(sensor_real_t accelSample[3]) final {
		accelSample[0] = accelSample[0] * Consts::AScale - activeCalibration.A_off[0];
		accelSample[1] = accelSample[1] * Consts::AScale - activeCalibration.A_off[1];
		accelSample[2] = accelSample[2] * Consts::AScale - activeCalibration.A_off[2];
	}

	float getAccelTimestep() final { return activeCalibration.A_Ts; }

	void scaleGyroSample(sensor_real_t gyroSample[3]) final {
		gyroSample[0] = static_cast<sensor_real_t>(
			Consts::GScale * (gyroSample[0] - activeCalibration.G_off1[0])
		);
		gyroSample[1] = static_cast<sensor_real_t>(
			Consts::GScale * (gyroSample[1] - activeCalibration.G_off1[1])
		);
		gyroSample[2] = static_cast<sensor_real_t>(
			Consts::GScale * (gyroSample[2] - activeCalibration.G_off1[2])
		);
	}

	float getGyroTimestep() final { return activeCalibration.G_Ts; }

	uint64_t lastMagCalUpdate = 0;
	void scaleMagSample(float* magSample){ 
		magCalibrator.scaleMagSample(magSample);
		if (millis() - lastMagCalUpdate > 5000) {
			lastMagCalUpdate = millis();
			// only continue if it has changed
			if (calibration.M_Bias[0] == magCalibrator.Best_M_Bias[0]
				&& calibration.M_Bias[1] == magCalibrator.Best_M_Bias[1]
				&& calibration.M_Bias[2] == magCalibrator.Best_M_Bias[2]
			) { return; }

			calibration.M_Bias[0] = magCalibrator.Best_M_Bias[0];
			calibration.M_Bias[1] = magCalibrator.Best_M_Bias[1];
			calibration.M_Bias[2] = magCalibrator.Best_M_Bias[2];
			for (int i = 0; i < 3; i++) {
				calibration.M_Ainv[0][i] = magCalibrator.Best_M_Ainv[0][i];
				calibration.M_Ainv[1][i] = magCalibrator.Best_M_Ainv[1][i];
				calibration.M_Ainv[2][i] = magCalibrator.Best_M_Ainv[2][i];
			}
			calibration.M_hm = magCalibrator.Best_hm;
			saveCalibration();
			printCalibration(CalibrationPrintFlags::MAG_CAL);
		}
	}

	float getMagTimestep() final { return activeCalibration.M_Ts; }

	bool magHealthy() { return magCalibrator.magHealthy(); }

	float getTempTimestep() final { return activeCalibration.T_Ts; }

	const uint8_t* getMotionlessCalibrationData() final {
		return activeCalibration.MotionlessData;
	}

	void signalOverwhelmed() final {
		if (isCalibrating) {
			currentStep->signalOverwhelmed();
		}
	}

	void provideAccelSample(const RawSensorT accelSample[3]) final {
		if (isCalibrating) {
			currentStep->processAccelSample(accelSample);
		}
	}

	void provideGyroSample(const RawSensorT gyroSample[3]) final {
		if (isCalibrating) {
			currentStep->processGyroSample(gyroSample);
		}
	}

	void provideTempSample(float tempSample) final {
		if (isCalibrating) {
			currentStep->processTempSample(tempSample);
		}
	}

	void calculateZROChange() {
		if (activeCalibration.gyroPointsCalibrated < 2) {
			activeZROChange = IMU::TemperatureZROChange;
		}

		float diffX = (activeCalibration.G_off2[0] - activeCalibration.G_off1[0])
					* Consts::GScale;
		float diffY = (activeCalibration.G_off2[1] - activeCalibration.G_off1[1])
					* Consts::GScale;
		float diffZ = (activeCalibration.G_off2[2] - activeCalibration.G_off1[2])
					* Consts::GScale;

		float maxDiff
			= std::max(std::max(std::abs(diffX), std::abs(diffY)), std::abs(diffZ));

		activeZROChange = 0.1f / maxDiff
						/ (activeCalibration.gyroMeasurementTemperature2
						   - activeCalibration.gyroMeasurementTemperature1);
	}

	float getZROChange() final { return activeZROChange; }

private:
	enum class CalibrationStepEnum {
		NONE,
		SAMPLING_RATE,
		MOTIONLESS,
		GYRO_BIAS,
		ACCEL_BIAS,
	};

	void computeNextCalibrationStep() {
		if (!calibration.motionlessCalibrated && Base::HasMotionlessCalib) {
			nextCalibrationStep = CalibrationStepEnum::MOTIONLESS;
			currentStep = &motionlessCalibrationStep;
		} else if (calibration.gyroPointsCalibrated == 0) {
			nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
			currentStep = &gyroBiasCalibrationStep;
			// } else if (!accelBiasCalibrationStep.allAxesCalibrated()) {
			// 	nextCalibrationStep = CalibrationStepEnum::ACCEL_BIAS;
			// 	currentStep = &accelBiasCalibrationStep;
		} else {
			nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
			currentStep = &gyroBiasCalibrationStep;
		}
	}

	void stepCalibrationForward(bool print = true, bool save = true) {
		currentStep->cancel();
		switch (nextCalibrationStep) {
			case CalibrationStepEnum::NONE:
				return;
			case CalibrationStepEnum::SAMPLING_RATE:
				nextCalibrationStep = CalibrationStepEnum::MOTIONLESS;
				currentStep = &motionlessCalibrationStep;
				if (print) {
					printCalibration(CalibrationPrintFlags::TIMESTEPS);
				}
				break;
			case CalibrationStepEnum::MOTIONLESS:
				nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
				currentStep = &gyroBiasCalibrationStep;
				if (print) {
					printCalibration(CalibrationPrintFlags::MOTIONLESS);
				}
				break;
			case CalibrationStepEnum::GYRO_BIAS:
				if (calibration.gyroPointsCalibrated == 1) {
					// nextCalibrationStep = CalibrationStepEnum::ACCEL_BIAS;
					// currentStep = &accelBiasCalibrationStep;
					nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
					currentStep = &gyroBiasCalibrationStep;
				}

				if (print) {
					printCalibration(CalibrationPrintFlags::GYRO_BIAS);
				}
				break;
			case CalibrationStepEnum::ACCEL_BIAS:
				nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
				currentStep = &gyroBiasCalibrationStep;

				if (print) {
					printCalibration(CalibrationPrintFlags::ACCEL_BIAS);
				}

				if (!accelBiasCalibrationStep.allAxesCalibrated()) {
					skippedAStep = true;
				}
				break;
		}

		isCalibrating = false;

		if (save) {
			saveCalibration();
		}
	}

	void saveCalibration() {
		SlimeVR::Configuration::SensorConfig calibration{};
		calibration.type
			= SlimeVR::Configuration::SensorConfigType::RUNTIME_CALIBRATION;
		calibration.data.runtimeCalibration = this->calibration;
		configuration.setSensor(sensorId, calibration);
		configuration.save();
	}

	enum class CalibrationPrintFlags {
		TIMESTEPS = 1,
		MOTIONLESS = 2,
		GYRO_BIAS = 4,
		ACCEL_BIAS = 8,
		MAG_CAL = 16,
	};

	static constexpr CalibrationPrintFlags PrintAllCalibration
		= CalibrationPrintFlags::TIMESTEPS | CalibrationPrintFlags::MOTIONLESS
		| CalibrationPrintFlags::GYRO_BIAS | CalibrationPrintFlags::ACCEL_BIAS
		| CalibrationPrintFlags::MAG_CAL;

	void printCalibration(CalibrationPrintFlags toPrint = PrintAllCalibration) {
		if (any(toPrint & CalibrationPrintFlags::TIMESTEPS)) {
			if (activeCalibration.sensorTimestepsCalibrated) {
				logger.info(
					"Calibrated timesteps: Accel %f, Gyro %f, Temperature %f",
					activeCalibration.A_Ts,
					activeCalibration.G_Ts,
					activeCalibration.T_Ts
				);
			} else {
				logger.info("Sensor timesteps not calibrated");
			}
		}

		if (Base::HasMotionlessCalib
			&& any(toPrint & CalibrationPrintFlags::MOTIONLESS)) {
			if (calibration.motionlessCalibrated) {
				logger.info("Motionless calibration done");
			} else {
				logger.info("Motionless calibration not done");
			}
		}

		if (any(toPrint & CalibrationPrintFlags::GYRO_BIAS)) {
			if (calibration.gyroPointsCalibrated != 0) {
				logger.info(
					"Calibrated gyro bias at %fC: %f %f %f",
					calibration.gyroMeasurementTemperature1,
					calibration.G_off1[0],
					calibration.G_off1[1],
					calibration.G_off1[2]
				);
			} else {
				logger.info("Gyro bias not calibrated");
			}

			if (calibration.gyroPointsCalibrated == 2) {
				logger.info(
					"Calibrated gyro bias at %fC: %f %f %f",
					calibration.gyroMeasurementTemperature2,
					calibration.G_off2[0],
					calibration.G_off2[1],
					calibration.G_off2[2]
				);
			}
		}

		if (any(toPrint & CalibrationPrintFlags::ACCEL_BIAS)) {
			if (accelBiasCalibrationStep.allAxesCalibrated()) {
				logger.info(
					"Calibrated accel bias: %f %f %f",
					calibration.A_off[0],
					calibration.A_off[1],
					calibration.A_off[2]
				);
			} else if (accelBiasCalibrationStep.anyAxesCalibrated()) {
				logger.info(
					"Partially calibrated accel bias: %f %f %f",
					calibration.A_off[0],
					calibration.A_off[1],
					calibration.A_off[2]
				);
			} else {
				logger.info("Accel bias not calibrated");
			}
		}

		if (any(toPrint & CalibrationPrintFlags::MAG_CAL)) {
			if (calibration.M_Bias[0] == 0 && calibration.M_Bias[1] == 0 && calibration.M_Bias[2] == 0) {
				logger.info("Mag not calibrated yet, will calibrate automatically during use");
			} else {
				logger.info(
					"Calibrated mag hard-iron bias: [%f, %f, %f] (hm: %f)",
					calibration.M_Bias[0],
					calibration.M_Bias[1],
					calibration.M_Bias[2],
					calibration.M_hm
				);
				logger.info("Calibrated mag soft-iron ellipsoid: [");
				for (int i = 0; i < 3; i++) {
					logger.info(
						"    [%f, %f, %f],",
						calibration.M_Ainv[0][i],
						calibration.M_Ainv[1][i],
						calibration.M_Ainv[2][i]
					);
				}
				logger.info("]");
			}
		}
	}

	CalibrationStepEnum nextCalibrationStep = CalibrationStepEnum::SAMPLING_RATE;

	static constexpr float initialStartupDelaySeconds = 5;
	uint64_t startupMillis = millis();

	SampleRateCalibrationStep<RawSensorT> sampleRateCalibrationStep{activeCalibration};
	MotionlessCalibrationStep<IMU, RawSensorT> motionlessCalibrationStep{
		calibration,
		sensor
	};
	GyroBiasCalibrationStep<RawSensorT> gyroBiasCalibrationStep{calibration};
	AccelBiasCalibrationStep<RawSensorT> accelBiasCalibrationStep{
		calibration,
		static_cast<float>(Consts::AScale)
	};
	NullCalibrationStep<RawSensorT> nullCalibrationStep{calibration};

	CalibrationStep<RawSensorT>* currentStep = &nullCalibrationStep;

	MagCalibrator magCalibrator;

	bool isCalibrating = false;
	bool skippedAStep = false;
	bool lastTickRest = false;

	SlimeVR::Configuration::RuntimeCalibrationSensorConfig calibration{
		// let's create here transparent calibration that doesn't affect input data
		.ImuType = {IMU::Type},
		.MotionlessDataLen = {Base::MotionlessCalibDataSize()},

		.sensorTimestepsCalibrated = false,
		.A_Ts = IMU::AccTs,
		.G_Ts = IMU::GyrTs,
		.M_Ts = IMU::MagTs,
		.T_Ts = 0,

		.motionlessCalibrated = false,
		.MotionlessData = {},

		.gyroPointsCalibrated = 0,
		.gyroMeasurementTemperature1 = 0,
		.G_off1 = {0.0, 0.0, 0.0},
		.gyroMeasurementTemperature2 = 0,
		.G_off2 = {0.0, 0.0, 0.0},

		.accelCalibrated = {false, false, false},
		.A_off = {0.0, 0.0, 0.0},

		.M_Bias = {0.0, 0.0, 0.0},
		.M_Ainv = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}},
		.M_hm = 100
	};

	float activeZROChange = 0;

	Configuration::RuntimeCalibrationSensorConfig activeCalibration = calibration;

	using Base::fusion;
	using Base::logger;
	using Base::sensor;
	using Base::sensorId;
	using Base::toggles;
};

}  // namespace SlimeVR::Sensors::RuntimeCalibration

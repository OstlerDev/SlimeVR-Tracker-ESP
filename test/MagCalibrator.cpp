#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>

#include <unity.h>
#include <ArduinoFake.h>
using namespace fakeit;

#include "logging/Level.h"
#include "sensors/softfusion/runtimecalibration/MagCalibrator.h"

#define VQF_SINGLE_PRECISION // use float for processing

long millisTimeSimulate = 0;
uint64_t timeOffset = 0;
void setUp(void){
  When(Method(ArduinoFake(), millis)).AlwaysDo([](){
    return millisTimeSimulate;
  });
}
void tearDown(void){}

enum VisualizeType {
  SAMPLE_ACCEPT = 0,
  NEW_CALIBRATION_BEST = 1,
  NEW_CALIBRATION_CANDIDATE = 2
};

void createCalibration(void) {
  std::printf("Starting running calibration test! :D\n");

  // std::ifstream hmdFile("test/2026-11-04_20-59-54_LEFT_UPPER_ARM_F024F9CC2DF90.log");
  // std::ifstream file("test/2026-11-09_15-06-28_null_F024F9CC2DF90.log");
  // std::ifstream file("test/2026-11-11_14-33-50_null_F024F9CC2DF90.log");

  // std::ifstream file("test/2026-11-01_22-16-58_LEFT_UPPER_ARM_F024F9CC2DF90.log");
  // std::ifstream file("test/2026-11-03_01-40-38_LEFT_UPPER_ARM_F024F9CC2DF90.log");
  // std::ifstream file("test/2026-11-04_01-05-31_LEFT_UPPER_ARM_F024F9CC2DF90.log");
  // std::ifstream file("test/2026-11-04_20-59-54_LEFT_UPPER_ARM_F024F9CC2DF90.log");

  // std::ifstream file("test/2026-11-01_22-16-58_LEFT_UPPER_LEG_F024F9CC5D570.log");
  // std::ifstream file("test/2026-11-01_22-16-58_LEFT_FOOT_F024F9CC31CF0.log");
  // std::ifstream file("test/2026-11-04_01-05-31_LEFT_FOOT_F024F9CC31CF0.log");
  // std::ifstream file("test/2026-11-01_22-16-58_RIGHT_UPPER_LEG_F024F9CC3EB10.log");
  // std::ifstream file("test/2026-11-01_22-16-58_HIP_F024F9CC2ECB0.log");
  // std::ifstream file("test/2026-11-04_20-59-54_LEFT_LOWER_LEG_F024F9CCC0620.log");
  // std::ifstream file("test/2026-11-04_20-59-54_RIGHT_LOWER_LEG_F024F9CC3BC40.log");
  // std::ifstream file("test/2026-11-05_15-09-04_LEFT_FOOT_F024F9CC31CF0_taekwondo.log");
  // std::ifstream file("test/2026-11-05_15-09-04_LEFT_LOWER_LEG_F024F9CCC0620_taekwondo.log");

  // std::ifstream file("test/2026-11-12_01-35-12_LEFT_LOWER_LEG_F024F9CCC0620.log"); // rotates when calibration becomes healthy
  std::ifstream file("test/2026-11-12_01-35-12_LEFT_FOOT_F024F9CC31CF0.log");

  // std::ifstream file("test/2026-11-12_01-35-12_UPPER_CHEST_F024F9CC39DF0.log"); // rotates continuously for the start


  std::FILE* outfile = std::fopen("test/visualize-calibration.log", "w");
  std::FILE* trackerQuats = std::fopen("test/sensor-quats-9d.log", "w");
  std::FILE* trackerQuats6D = std::fopen("test/sensor-quats-6d.log", "w");
  if (!file.is_open() /*|| outfile == nullptr*/) {
    std::printf("Error: Could not open file\n");
    return;
  }

  std::string line;
  std::getline(file,line); // move past the first line since its always empty.
  sscanf(
    line.c_str(), 
    "%lld",
    &timeOffset
  );
  std::string hmdLine;

  float gyrTs = 1.0f/240;
  float accTs = 1.0f/120;
  float magTs = 1.0f/120;

  float outputEveryMs = 10;

  float xyz[3] = {0.0f, 0.0f, 0.0f};

  float qwxyz[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float qwxyz6D[4] = {1.0f, 0.0f, 0.0f, 0.0f};

  float HMDqwxyz[4] = {1.0f, 0.0f, 0.0f, 0.0f};

  VQF vqf(gyrTs, accTs, magTs);
  VQF vqf6D(gyrTs, accTs, magTs);
  SlimeVR::Sensors::RuntimeCalibration::MagCalibrator magCalibrator{&vqf};

  SlimeVR::Sensors::RuntimeCalibration::MagSampleChange changed;
  magCalibrator.onSampleAccept([&](SlimeVR::Sensors::RuntimeCalibration::MagSampleChange changed) {
    std::fprintf(outfile, "%lld,%d,%d,%f,%f,%f,%f,%f,%f,0,0,0,0,0,0\n", 
      timeOffset + changed.timestamp, 
      VisualizeType::SAMPLE_ACCEPT, 
      changed.index, 
      changed.newSample[0], changed.newSample[1], changed.newSample[2], 
      changed.oldSample[0], changed.oldSample[1], changed.oldSample[2]
    );
    // float hardIron[3] = {0.0f, 0.0f, 0.0f};
    // float softIron[3][3] = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    // MagCalibration best = magCalibrator.getCalibration(hardIron, softIron);
    // std::fprintf(outfile, "{type: 'CALIBRATION', hardIron: [%0.3f, %0.3f, %0.3f], softIron: [[%0.3f, %0.3f, %0.3f],[%0.3f, %0.3f, %0.3f],[%0.3f, %0.3f, %0.3f]]\n");
  });
  magCalibrator.onNewBest([&](SlimeVR::Sensors::RuntimeCalibration::MagCalibration best, long timestamp) {
    std::fprintf(outfile, "%lld,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
      timeOffset + timestamp,
      VisualizeType::NEW_CALIBRATION_BEST,
      best.M_Bias[0], best.M_Bias[1], best.M_Bias[2],
      best.M_Ainv[0][0], best.M_Ainv[0][1], best.M_Ainv[0][2],
      best.M_Ainv[1][0], best.M_Ainv[1][1], best.M_Ainv[1][2],
      best.M_Ainv[2][0], best.M_Ainv[2][1], best.M_Ainv[2][2],
      best.hm
    );
  });
  magCalibrator.onNewCandidate([&](SlimeVR::Sensors::RuntimeCalibration::MagCalibration candidate, long timestamp) {
    std::fprintf(outfile, "%lld,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
      timeOffset + timestamp,
      VisualizeType::NEW_CALIBRATION_CANDIDATE,
      candidate.M_Bias[0], candidate.M_Bias[1], candidate.M_Bias[2],
      candidate.M_Ainv[0][0], candidate.M_Ainv[0][1], candidate.M_Ainv[0][2],
      candidate.M_Ainv[1][0], candidate.M_Ainv[1][1], candidate.M_Ainv[1][2],
      candidate.M_Ainv[2][0], candidate.M_Ainv[2][1], candidate.M_Ainv[2][2],
      candidate.hm
    );
  });

  // SlimeVR::Sensors::RuntimeCalibration::MagCalibration calib{
  //   .M_Bias = {44.880871, -29.840500, -36.859253},
  //   .M_Ainv = {
  //     {1.917734, -0.164258, 0.118150},
  //     {-0.164258, 2.152893, 0.135736},
  //     {0.118150, 0.135736, 2.034069}
  //   },
  //   .hm = 88.522415
  // };
  // magCalibrator.setCalibration(calib);
  // vqf.setMagRef(90.9888, 1.0658); //chest
  // vqf.setMagRef(124.2942, 0.9881); //leg
  // vqf.setMagRef(69.7079, 1.1251); //chest

  int linesProcessed = 0;
  int magHealthyAt = 0;
  int vqfHealthyAt = 0;

  int magHealthyFor = 0;

  float magCorrectionRate = 0.0f;
  float totalMagCorrection = 0.0f;
  float magDisagreeAngle = 0.0f;
  float totalMagDisagree = 0.0f;
  float lastMagDelta = 0.0f;
  float magDelta = 0.0f;
  float totalMagDelta = 0.0f;

  int64_t sensorTime = 0;
  int sensorType = 0;
  float hmdTime = 0.0f;
  int64_t lastProcessed = 0;

  // simulate time at 100hz
  while (std::getline(file,line)) {
    sscanf(
      line.c_str(), 
      "%lld,%d,%f,%f,%f",
      &sensorTime,
      &sensorType,
      &xyz[0], &xyz[1], &xyz[2]
    );
    millisTimeSimulate = sensorTime - timeOffset; // use actual time

    if (sensorType == 0){
      vqf.updateGyr(xyz, gyrTs);
      vqf6D.updateGyr(xyz, gyrTs);
    }
    if (sensorType == 1){ 
      vqf.updateAcc(xyz);
      vqf6D.updateAcc(xyz);
    }
    if (sensorType == 2) {
      magCalibrator.scaleMagSample(&xyz[0]);
      vqf.updateMag(xyz);
      linesProcessed++;
    }

    // run updates at 100hz
    if (sensorTime - lastProcessed > outputEveryMs) {
      lastProcessed = sensorTime;
      // record both 9d and 6d for comparisons
      vqf.getQuat9D(qwxyz);
      std::fprintf(trackerQuats, "%lld,%f,%f,%f,%f\n", sensorTime, qwxyz[0], qwxyz[1], qwxyz[2], qwxyz[3]);

      vqf6D.getQuat6D(qwxyz6D);
      std::fprintf(trackerQuats6D, "%lld,%f,%f,%f,%f\n", sensorTime, qwxyz6D[0], qwxyz6D[1], qwxyz6D[2], qwxyz6D[3]);

      if (magHealthyAt == 0 && magCalibrator.magHealthy()) { magHealthyAt = millisTimeSimulate; }
      if (vqfHealthyAt == 0 && magCalibrator.vqfReady()) { vqfHealthyAt = millisTimeSimulate; }

      magCorrectionRate += std::fabs(vqf.getState().lastMagCorrAngularRate);
      if (magCalibrator.magHealthy()) { 
        magHealthyFor++;

        totalMagCorrection += std::fabs(vqf.getState().lastMagCorrAngularRate);
        if (linesProcessed % 10000 == 0) { 
          std::printf("[%d] magCorrectionRate: %f\n", linesProcessed, magCorrectionRate);
          magCorrectionRate = 0.0f;
        }
      }

      // if (linesProcessed % 100000 == 0) { magCalibrator.logCalibration(); } 

    }
  }

  float timePassed = millisTimeSimulate/(60.0f*60.0f*1000.0f);
  float timeMagHealthyAt = magHealthyAt/(60.0f*60.0f*1000.0f);
  float timeVQFHealthyAt = vqfHealthyAt/(60.0f*60.0f*1000.0f);
  float timeMagHealthyFor = magHealthyFor/(60.0f*60.0f*100.0f);
  float totalTimeHealthy = magCalibrator.getTotalHealthDuration()/(60.0f*60.0f);
  float totalTimeDisturbed = magCalibrator.getTotalDisturbedDuration()/(60.0f*60.0f);
  magCalibrator.logCalibration();
  std::printf("Processed %d samples (~%0.2fhrs) (vqf healthy at: ~%0.2fhrs, ~%0.2fhrs left) (total time vqf healthy: ~%0.2fhrs) (total time vqf disturbed: ~%0.2fhrs)\n", linesProcessed, timePassed, timeVQFHealthyAt, timePassed-timeVQFHealthyAt, totalTimeHealthy, totalTimeDisturbed);
  std::printf("Processed %d samples (~%0.2fhrs) (mag healthy at: ~%0.2fhrs, ~%0.2fhrs left) (total time mag healthy: ~%0.2fhrs) (total time mag unhealthy: ~%0.2fhrs)\n", linesProcessed, timePassed, timeMagHealthyAt, timePassed-timeMagHealthyAt, timeMagHealthyFor, (timePassed-timeMagHealthyAt)-timeMagHealthyFor);
  std::printf("Total mag correction for run: %0.2f\n", totalMagCorrection);
}

int main() {
  UNITY_BEGIN();

  RUN_TEST(createCalibration);

  return UNITY_END();
}
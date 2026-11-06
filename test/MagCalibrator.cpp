#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>

#include <unity.h>
#include <ArduinoFake.h>
using namespace fakeit;

#include "logging/Level.h"
#include "sensors/softfusion/runtimecalibration/MagCalibrator.h"

#define VQF_SINGLE_PRECISION // use float for processing

long millisTimeSimulate = 0;
void setUp(void){
  When(Method(ArduinoFake(), millis)).AlwaysDo([](){
    return millisTimeSimulate;
  });
}
void tearDown(void){}

void createCalibration(void) {
  std::printf("Starting running calibration test! :D\n");

  // std::ifstream file("test/2026-11-01_22-16-58_LEFT_UPPER_ARM_F024F9CC2DF90.log");
  // std::ifstream file("test/2026-11-01_22-16-58_LEFT_UPPER_LEG_F024F9CC5D570.log");
  // std::ifstream file("test/2026-11-01_22-16-58_LEFT_FOOT_F024F9CC31CF0.log");
  // std::ifstream file("test/2026-11-04_01-05-31_LEFT_FOOT_F024F9CC31CF0.log");
  // std::ifstream file("test/2026-11-01_22-16-58_RIGHT_UPPER_LEG_F024F9CC3EB10.log");
  // std::ifstream file("test/2026-11-01_22-16-58_HIP_F024F9CC2ECB0.log");
  // std::ifstream file("test/2026-11-05_15-09-04_LEFT_FOOT_F024F9CC31CF0_taekwondo.log");
  std::ifstream file("test/2026-11-05_15-09-04_LEFT_LOWER_LEG_F024F9CCC0620_taekwondo.log");
  std::FILE* outfile = std::fopen("test/visualization-output.json", "w");
  if (!file.is_open() /*|| outfile == nullptr*/) {
    std::printf("Error: Could not open file\n");
    return;
  }

  std::string line;
  std::getline(file,line); // move past the first line since its always empty.

  float gyrTs = 1.0f/240;
  float accTs = 1.0f/120;
  float magTs = 1.0f/120;

  float gyr[3] = {0.0f, 0.0f, 0.0f};
  float acc[3] = {0.0f, 0.0f, 0.0f};
  float mag[3] = {0.0f, 0.0f, 0.0f};

  float qwxyz[4] = {1.0f, 0.0f, 0.0f, 0.0f};

  VQF vqf(accTs);
  SlimeVR::Sensors::RuntimeCalibration::MagCalibrator magCalibrator{&vqf};

  // magCalibrator.onSampleAccept([&](float[3] newSample, float[3] oldSample, int index) {
  //   std::fprintf(outfile, "{type: 'SAMPLE_ACCEPT', index: %d, newSample: [%0.3f, %0.3f, %0.3f], oldSample: [%0.3f, %0.3f, %0.3f]},\n", index, newSample[0], newSample[1], newSample[2], oldSample[0], oldSample[1], oldSample[2]);
  //   float hardIron[3] = {0.0f, 0.0f, 0.0f};
  //   float softIron[3][3] = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
  //   magCalibrator.getCalibration(hardIron, softIron);
  //   std::fprintf(outfile, "{type: 'CALIBRATION', hardIron: [%0.3f, %0.3f, %0.3f], softIron: [[%0.3f, %0.3f, %0.3f],[%0.3f, %0.3f, %0.3f],[%0.3f, %0.3f, %0.3f]]\n");
  // });

  int linesProcessed = 0;
  int ignoreLines = 0;
  int magHealthyAt = 0;
  int vqfHealthyAt = 0;
  // simulate time at 100hz
  while (linesProcessed < ignoreLines) { linesProcessed++; millisTimeSimulate+=10; std::getline(file,line); } // ignore first 2 minutes
  while (/*linesProcessed < (180000 + ignoreLines) &&*/ std::getline(file,line)) {
    millisTimeSimulate+=10;
    sscanf(
      line.c_str(), 
      "%f,%f,%f,%f,%f,%f,%f,%f,%f",
      &gyr[0], &gyr[1], &gyr[2],
      &acc[0], &acc[1], &acc[2],
      &mag[0], &mag[1], &mag[2]
    );

    vqf.updateGyr(gyr, gyrTs);
    vqf.updateAcc(acc);

    magCalibrator.scaleMagSample(&mag[0]);
    vqf.updateMag(mag);

    // process quat to force vqf forward
    if (magCalibrator.magHealthy()) {
      vqf.getQuat9D(qwxyz);
    } else {
      vqf.getQuat6D(qwxyz);
    }
    if (magHealthyAt == 0 && magCalibrator.magHealthy()) {
      magHealthyAt = linesProcessed;
    }
    if (vqfHealthyAt == 0 && magCalibrator.vqfReady()) {
      vqfHealthyAt = linesProcessed;
    }

    if (linesProcessed % 100000 == 0) { magCalibrator.logCalibration(); } 

    // std::printf("parsed: gyro (%f,%f,%f) accel (%f,%f,%f) mag (%f,%f,%f)\n", gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], mag[0], mag[1], mag[2]);
    linesProcessed++;
  }
  float timePassed = linesProcessed/(60.0f*60.0f*100.0f);
  float timeMagHealthyAt = magHealthyAt/(60.0f*60.0f*100.0f);
  float timeVQFHealthyAt = vqfHealthyAt/(60.0f*60.0f*100.0f);
  float totalTimeHealthy = magCalibrator.getTotalHealthDuration()/(60.0f*60.0f);
  float totalTimeDisturbed = magCalibrator.getTotalDisturbedDuration()/(60.0f*60.0f);
  magCalibrator.logCalibration();
  std::printf("Processed %d samples (~%0.2fhrs) (vqf healthy at: ~%0.2fhrs) (total time vqf healthy: ~%0.2fhrs) (total time vqf disturbed: ~%0.2fhrs)\n", linesProcessed, timePassed, timeVQFHealthyAt, timePassed-timeVQFHealthyAt, totalTimeHealthy, totalTimeDisturbed);
  std::printf("Processed %d samples (~%0.2fhrs) (mag healthy at: ~%0.2fhrs, ~%0.2fhrs left)\n", linesProcessed, timePassed, timeMagHealthyAt, timePassed-timeMagHealthyAt);
}

int main() {
  UNITY_BEGIN();

  RUN_TEST(createCalibration);

  return UNITY_END();
}
/*
  SlimeVR Code is placed under the MIT license
  Copyright (c) 2026 puppy sky (OstlerDev) & SlimeVR Contributors

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

#include "magneto1.4.h"
#include "vqf.h"
#include "logging/Logger.h"

#include <functional>
#include <optional>

#define BENCHMARK_MAG_CALIBRATOR false
#if BENCHMARK_MAG_CALIBRATOR
#include "debugging/Benchmark.h"
#endif

#define SAMPLES_TOTAL (32)

namespace SlimeVR::Sensors::RuntimeCalibration {

class HealthSampler {
public:
  float getHealth() { return health; }
  void addSample(float score) {
    float sampleHealth = scoreToHealth(score);
    // simple moving average to keep things lightweight and fast
    float healthDelta = SMOOTHING_FACTOR * (sampleHealth - health);
    health += healthDelta;
  }
private:
  const int HEALTH_WINDOW = 1000;
  const float SMOOTHING_FACTOR = 2.0f/(HEALTH_WINDOW+1);
  const int ERROR_SCALER = 5;
  float health = 0.0f;

  // returns 0-1 based on the sample health (1 being healthy, 0 being very bad)
  float scoreToHealth(float deviationScore) {
    float errorAmount = 0.0f;
    if (deviationScore < 1) {
      errorAmount = 1.0f-deviationScore;
    } else {
      errorAmount = deviationScore-1.0f;
    }
    if (errorAmount >= 1) return 0;
    return 1.0f - errorAmount;
  }
};

class MagCalibrator {
public:
  MagCalibrator(VQF* vqf) : fusion(vqf) {
    resetCalibration(); // init
  }

  bool freeze = false;
  void scaleMagSample(float* sample) {
    if (sample[0] == 0 && sample[1] == 0 && sample[2] == 0) { return; }
    sampleNumber++;
    logStatus();
#if BENCHMARK_MAG_CALIBRATOR
    magCalibrateBM.before();
#endif
    provideMagSample(sample); // add to calibration if needed
#if BENCHMARK_MAG_CALIBRATOR
    magCalibrateBM.after();
#endif

    if (!startCalibration || !calibrationReady) {
      sample[0] = 0;
      sample[1] = 0;
      sample[2] = 0;
      return;
    }

    float movementFromLast = calculateDistance(sample[0], lastSample[0], sample[1], lastSample[1], sample[2], lastSample[2]);
    lastSample[0] = sample[0];
    lastSample[1] = sample[1];
    lastSample[2] = sample[2];

    float score = 0;
    scaleAndScore(sample, &score);

    // only adjust health when movement is detected (don't improve health for sitting still)
    if (movementFromLast > 3) activeHealth.addSample(score);
    // logger.trace("Sample Score: %f, Trust: %d, Frozen: %d", score, calibrationTrust, freeze);
    // logger.trace("Sample Score: %f", score);

    // if (improvementSamples < 50) {
    //   sample[0] = 0;
    //   sample[1] = 0;
    //   sample[2] = 0;
    //   return;
    // }
  }

  // Should we start using 9D fusion outputs or keep using 6D fusion.
  // We still pass mag data to VQF, this is just if we utilize it or not.
  bool magReady() { return vqfHasBeenHealthy; }

  // debug helpers
  void logCalibration() { logStatus(true); }
  float getTotalHealthDuration(){ return totalHealthyTime; }
  float getTotalDisturbedDuration(){ return totalDisturbedTime; }
  // // Will run this function if provided when new points are accepted into the calibration, useful for debugging/recording logs
  void onSampleAccept(std::function<void(float[3], float[3], int)>&& callback) {
    sampleAcceptCallback = callback;
  }
  // void getCalibration(float* hardIron, float* softIron) {
  //   hardIron = &M_Bias;
  //   softIron = &M_Ainv;
  // }

private:
#if BENCHMARK_MAG_CALIBRATOR
  SlimeVR::Debugging::Benchmark magCalibrateBM{"MagCalibrator.provideMagSample()"};
  SlimeVR::Debugging::Benchmark magnetoCalculateBM{"MagCalibrator.runMagneto()"};
#endif
  Logging::Logger logger{"MagCalibrator"};
  VQF* fusion; // reference to vqf for direct access

  // calls function with (float* newSample, float* oldSample)
  std::optional<std::function<void(float[3], float[3], int)>> sampleAcceptCallback;
  void emitSampleAccept(float newSample[3], float oldSample[3], int index){
    if (sampleAcceptCallback) {
      (*sampleAcceptCallback)(newSample, oldSample, index);
    }
  }

  HealthSampler activeHealth;
  // HealthSampler candidateHealth; // soon(tm)
  const int THROTTLE_ELLIPSOID_DURATION = 2 * 1000; // Throttle the frequency of calculating the ellipsoid to prevent lag.
  uint64_t lastEllipsoidCreate = 0;

  bool calibrationReady = false;
  bool isSoftIronOkay = false;
  float lastSample[3] = {0.0f, 0.0f, 0.0f};

  uint64_t startTime = 0;
  bool startCalibration = true;
  uint64_t lastLogTime = 0;
  uint64_t sampleNumber = 0;
  void logStatus(bool forcePrint = false){
    uint64_t now = millis();
    if (startTime == 0) {
      startTime = now;
    }
    // wait 2 minutes to collect datapoints
    if (now - startTime > 120*1000) {
      startCalibration = true;
    }
    // full log every 30 seconds
    if (now - lastLogTime > 120*1000 || forcePrint) {
      lastLogTime = now;
      const VQFState& vqfState = fusion->getState();
      float disturbedFor = (millis() - lastHealthyTime) / 1000.0f;
      logger.debug("[%d] VQF: Healthy Duration: %0.2f (current accepted mag field: norm %0.4f, dip %0.4f) Disturbed duration: %0.1f (candidate mag field: time %0.2f, norm %0.4f, dip %0.4f)", sampleNumber, vqfState.magUndisturbedT, vqfState.magRefNorm, vqfState.magRefDip, disturbedFor, vqfState.magCandidateT, vqfState.magCandidateNorm, vqfState.magCandidateDip);
      logger.debug("Active calibration health: %0.3f", activeHealth.getHealth());
      logger.debug("Magneto1.4 calibration: (%0.2f, %0.2f, %0.2f, spread: %0.1f, total error: %0.3f, improvement samples: %d)", M_Bias[0], M_Bias[1], M_Bias[2], getTotalSpread(), getErrorTotal(), improvementSamples);
      logger.debug("Magneto1.4 calibration matrix:");
      logger.debug("{");
      for (int i = 0; i < 3; i++) {
        logger.debug(
          "  [%f, %f, %f],",
          M_Ainv[0][i],
          M_Ainv[1][i],
          M_Ainv[2][i]
        );
      }
      logger.debug("}");
      logger.debug("Point Cloud: {");
      for (int i = 0; i < SAMPLES_TOTAL; i++) {
        logger.debug("  [%0.4f, %0.4f, %0.4f],", x_vec[i], y_vec[i], z_vec[i]);
      }
      logger.debug("}");
    }
  }

  /**
   * Helper functions to accumulate new points and adjust them over time. 
   */
  int calibrationTrust = 500;
  int improvementSamples = 0;
  float x_vec[SAMPLES_TOTAL];
  float y_vec[SAMPLES_TOTAL];
  float z_vec[SAMPLES_TOTAL];

  float minDist_vec[SAMPLES_TOTAL];
  float pointDeviation[SAMPLES_TOTAL];
  float pointTrustScore[SAMPLES_TOTAL];

  void resetCalibration(){
    calibrationTrust = 100;
    improvementSamples = 0;
    for (int i = 0; i < SAMPLES_TOTAL; i++) {
      x_vec[i] = 0;
      y_vec[i] = 0;
      z_vec[i] = 0;
      pointTrustScore[i] = 0;
    }
    hm = 0;
    M_Bias[0] = 0;
    M_Bias[1] = 0;
    M_Bias[2] = 0;
    for (int i = 0; i < 3; i++) {
      M_Ainv[0][i] = 0;
      M_Ainv[1][i] = 0;
      M_Ainv[2][i] = 0;
    }
  }
  int getFirstEmpty(){
    for (int i = 0; i < SAMPLES_TOTAL; i++) {
      if (x_vec[i] == 0 && y_vec[i] == 0 && z_vec[i] == 0) {
        return i;
      }
    }
    return -1;
  }
  int countEmpty(){
    int emptyCount = 0;
    for (int i = 0; i < SAMPLES_TOTAL; i++) {
      if (x_vec[i] == 0 && y_vec[i] == 0 && z_vec[i] == 0) {
        emptyCount++;
      }
    }
    return emptyCount;
  }
  // Tries to clean out severely disrupted samples from the startup calibration.
  void cleanStartupSamples() {
    for (int i = 0; i < SAMPLES_TOTAL; i++) {
      bool sampleIsBad = false;
      float errAmount = getErrorAmt(pointDeviation[i]);
      logger.debug("Checking startup sample [%d]! err: %0.3f, dist: %0.2f (%0.2f, %0.2f, %0.2f) ", i, errAmount, minDist_vec[i], x_vec[i], y_vec[i], z_vec[i]);
      // if (errAmount > 0.15) {
      //   sampleIsBad = true;
      // }
      // if there is an extreme magnitude sample isolated from all other samples, reject it immediately
      if (minDist_vec[i] > 100) {
        sampleIsBad = true;
      }
      if (sampleIsBad) {
        logger.debug("Cleaning out bad sample [%d] from startup calibration array! err: %0.3f, dist: %0.2f (%0.2f, %0.2f, %0.2f) ", i, errAmount, minDist_vec[i], x_vec[i], y_vec[i], z_vec[i]);
        x_vec[i] = 0;
        y_vec[i] = 0;
        z_vec[i] = 0;
        minDist_vec[i] = 0;
        pointDeviation[i] = 0;
        pointTrustScore[i] = 0;
      }
    }
  }
  // find the worst sample that we should replace within the point cloud.
  // Either find the sample with the lowest spread for replacement,
  // or find the sample that severely deviates from 
  int getWorstSample(){
    float currentBestDist = minDist_vec[0];
    int bestDistIndex = 0;
    float currentWorst = pointTrustScore[0];
    int worstSample = 0;
    float worstErrorScore = pointDeviation[0];
    int worstErrorSample = 0;

    for (int i = 1; i < SAMPLES_TOTAL; i++) {
      if (minDist_vec[i] > currentBestDist) {
        currentBestDist = minDist_vec[i];
        bestDistIndex = i;
      }
      if (pointTrustScore[i] < currentWorst) {
        currentWorst = pointTrustScore[i];
        worstSample = i;
      }
      if (pointDeviation[i] > worstErrorScore) {
        worstErrorScore = pointDeviation[i];
        worstErrorSample = i;
      }
    }
    // Replace highly deviated samples first to clear out samples with errors 
    // that might have been accumulated when the mag was being disturbed
    // first, sanity check using extreme spread distances, since those are likely to be errors
    // verify using the most deviated samples by using a basic average
    if (currentBestDist > 100 && currentBestDist > averageDistance*5) {
      logger.debug("Force replacing deviated distance: %0.2f (avg: %0.2f), index: %d", currentBestDist, averageDistance, bestDistIndex);
      pointTrustScore[bestDistIndex] = 0;
      return bestDistIndex;
    }
    // if there are no isolated samples, next reject samples that deviate from ellipsoid skin.
    if (getErrorAmt(worstErrorScore) > 0.15 && isGoodSoftCal()) {
      logger.debug("Force replacing deviated score: %0.4f, index: %d", worstErrorScore, worstErrorSample);
      // Set the weighted trust score to zero to force replacement of this sample.
      pointTrustScore[worstErrorSample] = 0;
      return worstErrorSample;
    }
    // If all samples are within threshold, then just replace the least spread point.
    return worstSample;
  }
  // get the minimum distance to any other point
  float getMinDistance(float* sample, int ignoreIndx = -1) {
    float minDistance = 1000;
    for (int i = 0; i < SAMPLES_TOTAL; i++) {
      if (i == ignoreIndx || (x_vec[i] == 0 && y_vec[i] == 0 && z_vec[i] == 0)) continue;
      float dist = calculateDistance(sample[0], x_vec[i], sample[1], y_vec[i], sample[2], z_vec[i]);
      if (dist < minDistance || minDistance == 1000) {
        minDistance = dist;
      }
    }
    return minDistance;
  }
  // recalculate the min distance for all points
  float averageDistance = 0.0f;
  void recalculateMinDistance() {
    for (int i = 0; i < SAMPLES_TOTAL; i++) {
      float sample[3] = {x_vec[i], y_vec[i], z_vec[i]};
      minDist_vec[i] = getMinDistance(sample, i);
    }
    float distanceAccumulator = 0;
    for (int i = 0; i < SAMPLES_TOTAL; i++) {
      if (x_vec[i] == 0 && y_vec[i] == 0 && z_vec[i] == 0) continue; // ignore samples with no distance
      distanceAccumulator += minDist_vec[i];
    }
    averageDistance = distanceAccumulator/SAMPLES_TOTAL;
  }
  void recalculateTrustScores() { 
    recalculateMinDistance();
    for (int i = 0; i < SAMPLES_TOTAL; i++) {
      float sample[3] = {x_vec[i], y_vec[i], z_vec[i]};
      float score;
      scaleAndScore(sample, &score);
      pointDeviation[i] = score;
      pointTrustScore[i] = calculateTrust(minDist_vec[i], score);
    }
  }
  float getTotalSpread(){
    float spread = 0;
    for (int i = 0; i < SAMPLES_TOTAL; i++) {
      if (x_vec[i] == 0 && y_vec[i] == 0 && z_vec[i] == 0) continue;
      spread += minDist_vec[i];
    }
    totalSpread = spread;
    return spread;
  }
  float getErrorTotal(){
    float totalError = 0;
    for (int i = 0; i < SAMPLES_TOTAL; i++) {
      totalError += getErrorAmt(pointDeviation[i]);
    }
    return totalError;
  }
  // get a scale for the deviation from a point
  float getErrorAmt(float deviation) {
    if (deviation < 1) {
      return 1.0f-deviation;
    } else {
      return deviation-1.0f;
    }
  }
  // calculates raw distance between 2 3d points
  float calculateDistance(float x1, float x2, float y1, float y2, float z1, float z2){
    return ((x1-x2)*(x1-x2)) + ((y1-y2)*(y1-y2)) + ((z1-z2)*(z1-z2));
  };

  // experimental sanity check on soft-iron ellipsoid values (https://github.com/ysoldak/magcal/blob/master/magcal.go#L197)
  bool isGoodSoftCal() {
    // soft iron sanity checks
    float x = M_Ainv[0][0];
    float y = M_Ainv[1][1];
    float z = M_Ainv[2][2];

    if (x < 0.5 || y < 0.5 || z < 0.5 || x > 3 || y > 3 || z > 3) {
      return false;
    }
    if (x > y*2 || x > z*2 || y > x*2 || y > z*2 || z > x*2 || z > y*2) {
      return false;
    }
    // sanity check diagonals
    float d1 = M_Ainv[0][1] * 5;
    float d2 = M_Ainv[0][2] * 5;
    float d3 = M_Ainv[1][2] * 5;
    if (d1 > x || d1 > y || d1 > z || d2 > x || d2 > y || d2 > z || d3 > x || d3 > y || d3 > z) { 
      return false; 
    }

    return true;
  }

  /**
   * Handle creation of the calibration
   */
  const int sampleRate = 10; // consider changing to time based collection metric, also consider subsample averaging of all datapoints
  int samplerDelay = 0;
  double lastSampleTime;
  bool vqfHasBeenHealthy = false;
  float healthyDuration = 0;
  float totalHealthyTime = 0;
  float disturbedDuration = 0;
  float totalDisturbedTime = 0;
  long lastHealthyTime = 0;
  void provideMagSample(float* sample) {
    // delay to start calibration
    if (!startCalibration) { return; }
    // only collect a datapoint every X samples provided (unless we are filling initial array)
    samplerDelay++;
    if (samplerDelay % sampleRate != 0) { return; }
    samplerDelay = 0;

    if (freeze) {
      return;
    }


    // Ignore new samples if mag is disturbed
    if (calibrationReady) {
      const VQFState& vqfState = fusion->getState();
      if (vqfState.magDistDetected && vqfState.kMagInit != 1) {
        // Log duration that we were healthy!
        if (healthyDuration > 0) {
          logger.debug("[%d] VQF: Mag is now disturbed! Was healthy for: %0.2f (current accepted mag field: norm %0.4f, dip: %0.4f)", sampleNumber, healthyDuration, vqfState.magRefNorm, vqfState.magRefDip);
          totalHealthyTime += healthyDuration;
          healthyDuration = 0;
        }
        // Consider also tossing out any samples added to the array in the last 0.5 seconds
        // in case any bad samples have snuck in before VQF could trigger mag disturbance rejections

        // NOTE! Mag disturbance detection DOES NOT handle if a magnet is brought close by but doesn't change the
        // angle. We /might/ still want some way to detect if the sensor readings spiked out of reasonable ranges.
        // ignore provided sample
        // only start ignoring once VQF has been undisturbed a single time
        if (vqfHasBeenHealthy) {
          if (sampleNumber % 10000 == 0) {
            float disturbedFor = (millis() - lastHealthyTime) / 1000.0f;
            logger.debug("[%d] VQF: Mag is Disturbed! Disturbed duration: %0.1f (new candidate: time %0.2f, norm %0.4f, dip: %0.4f)", sampleNumber, disturbedFor, vqfState.magCandidateT, vqfState.magCandidateNorm, vqfState.magCandidateDip);
          }
          disturbedDuration = vqfState.magRejectT;

          // dont ignore using vqf at all?
          // // If we have a healthy calibration, then listen to vqf and ignore the sample, otherwise, keep processing
          // // this filters out momentary disturbances, but allows updating when the disturbance is consistent.
          // // There is a chance that VQF will report disturbances before we have a good soft-iron calibration, and the
          // // disturbance reporting can filter out points that we need to add into our point cloud.
          // if (activeHealth.getHealth() > 0.92) {
          //   return;
          // }
        } else {
          if (sampleNumber % 10000 == 0) logger.debug("[%d] VQF: Calibrating for the first time (current candidate: time %0.2f, norm %0.4f, dip: %0.4f)", sampleNumber, vqfState.magCandidateT, vqfState.magCandidateNorm, vqfState.magCandidateDip);
        }
      } else {
        long timeNow = millis();
        if (!vqfHasBeenHealthy) logger.info("[%d] VQF: Accepted mag heading for first time! Calibration health: %0.3f, Accepted candidate norm: %0.4f, dip: %0.4f", sampleNumber, activeHealth.getHealth(), vqfState.magRefNorm, vqfState.magRefDip);
        if (disturbedDuration > 0) {
          float disturbedFor = (timeNow - lastHealthyTime) / 1000.0f;
          logger.debug("[%d] VQF: Mag is now healthy again! Was disturbed for: %0.2f (current accepted mag field: norm %0.4f, dip: %0.4f)", sampleNumber, disturbedFor, vqfState.magRefNorm, vqfState.magRefDip);
          totalDisturbedTime += disturbedFor;
          disturbedDuration = 0;
        }
        lastHealthyTime = timeNow;
        vqfHasBeenHealthy = true;
        if (sampleNumber % 10000 == 0) logger.debug("[%d] VQF: Mag is healthy :3! Healthy duration: %0.2f, lastMagDisAngle: %0.2f,  Accepted candidate norm: %0.4f, dip: %0.4f", sampleNumber, vqfState.magUndisturbedT, vqfState.lastMagDisAngle, vqfState.magRefNorm, vqfState.magRefDip);
        healthyDuration = vqfState.magUndisturbedT;
      }
    }

    // first check if there are any still empty
    int emptyIndex = getFirstEmpty();
    float minDistance = getMinDistance(sample);
    // when building the array require a spread sample.
    if (minDistance < 5 && emptyIndex != -1) return;
    if (emptyIndex != -1) {
      x_vec[emptyIndex] = sample[0];
      y_vec[emptyIndex] = sample[1];
      z_vec[emptyIndex] = sample[2];

      logger.debug("[%d] Adding new datapoint to initial point cloud: %0.2f, %0.2f, %0.2f (dist: %0.2f) (indexFilled: %d)", sampleNumber, sample[0], sample[1], sample[2], minDistance, emptyIndex);

      recalculateMinDistance(); // recalculates distance
      if (countEmpty() == 0) {
        runMagneto();
        recalculateTrustScores();
        // force log first calibration
        logCalibration();

        // remove extreme samples that snuck in
        // cleanStartupSamples(); // dont do this!
      }
      return;
    }

    // DISABLE calibration trust resetting the array, just keep it around for now as a static value
    // for monitoring purposes. It seemed to reset the data too often instead of allowing slow improvements
    // it is likely better to calculate based on the percentage of the ellipsoid that has datapoints
    // so that we know how much is left to "fill".
    // Calibration trust was meant to track how well fit our ellipsoid was to the sphere, but it seemed to
    // actually degrade the calibration by removing good datapoints from the array.
    // In the future we should only change calibration trust when the mag is actually moving instead of
    // changing trust when the mag is holding still.

    // Maybe instead of just raw checking samples score against the current ellipsoid we should
    // do a "test" of the new point to see if it improves the fit or if it makes the fit of the 
    // ellipsoid worse by checking every points score against a "test" ellipsoid.
    // We could have two ellipsoids that we keep in memory, one as the "active" and the second as the "test"
    // that we use to update with a test sample, check fit, then if its bad, revert the change.
    // This would help us validate things a little more maybe?

    // toss out samples that don't fit, and lower trust score.


    // Disable now that we have VQF disturbance rejection
    // // if ((score < 0.9 || score > 1.1) && improvementSamples > 60) {
    // //   if (calibrationTrust > 0) {
    // //     calibrationTrust--;
    // //   }
    // //   // disable trust based resets

    // //   // if trust drops too much, clear out array and start again from scratch
    // //   // if (calibrationTrust == 0) {
    // //   //   resetCalibration();
    // //   //   calibrationTrust = 500;
    // //   // }

    // //   return;
    // // } else if ((score > 0.9 && score < 1.1) && improvementSamples > 60) {
    // //   if (calibrationTrust < 2500) {
    // //     calibrationTrust++;
    // //   }
    // //   // disable freeze for now...

    // //   // if (calibrationTrust > 2500) {
    // //   //   freeze = true;
    // //   // }
    // // }

    // if there are not any empty slots, check if we are better than the worst sample, if so, replace it.
    int worstSample = getWorstSample();

    // test how well the new point fits the existing ellipsoid
    float score;
    float testSample[3] = {sample[0], sample[1], sample[2]};
    scaleAndScore(&testSample[0], &score);

    float weightedTrustScore = calculateTrust(minDistance, score);
    if (sampleNumber % 10000 == 0 || pointTrustScore[worstSample] <= 0.0f) logger.debug("[%d] Weighted Trust: %0.4f, Deviation Score: %0.4f, Raw Distance: %0.2f | Worst index: %d, worst trust: %0.4f, worst dist: %0.2f, worst score: %0.4f | New Point: (%0.2f, %0.2f, %0.2f)", sampleNumber, weightedTrustScore, score, minDistance, worstSample, pointTrustScore[worstSample], minDist_vec[worstSample], pointDeviation[worstSample], sample[0], sample[1], sample[2]);

    // hard ignore points that deviate from the ellipsoid skin **UNLESS** we are force replacing a bad point
    if ((score < 0.7 || score > 1.3) && isGoodSoftCal() && pointTrustScore[worstSample] > 0.0f) { return; }
    // don't accept bad samples in the first place.
    if (minDistance > averageDistance*5) { return; }
    // if we are force replacing a point, make sure we are accepting a less deviated sample into the array instead of a worse sample
    if (pointTrustScore[worstSample] <= 0.0f && (getErrorAmt(score) > 0.15 && getErrorAmt(score) > getErrorAmt(pointDeviation[worstSample])) && isGoodSoftCal()) { return; }

    // See if it is a better point than an existing one
    if ((weightedTrustScore > pointTrustScore[worstSample] || pointTrustScore[worstSample] <= 0.0f) && !freeze) {
      logger.debug("[%d] Replacing!! Current health: %0.3f, Weighted Trust: %0.4f, Deviation Score: %0.4f, Raw Distance: %0.2f, New Point: (%0.2f, %0.2f, %0.2f) | Worst index: %d, worst trust: %0.4f, worst dist: %0.2f, worst score: %0.4f, worst Point: (%0.2f, %0.2f, %0.2f) ", sampleNumber, activeHealth.getHealth(), weightedTrustScore, score, minDistance, sample[0], sample[1], sample[2], worstSample, pointTrustScore[worstSample], minDist_vec[worstSample], pointDeviation[worstSample], x_vec[worstSample], y_vec[worstSample], z_vec[worstSample]);
      float oldSample[3] = {x_vec[worstSample], y_vec[worstSample], z_vec[worstSample]}; // for emitSampleAccept
      float newSample[3] = {sample[0], sample[1], sample[2]};

        // logger.trace("Replacing current worst sample i:%d (%0.2f) for new sample (%0.2f)!", worstSample, minDist_vec[worstSample], minDistance);
      // update least square accumulator
        // leastSquareAccumulate(x_vec[worstSample], y_vec[worstSample], z_vec[worstSample], false); // remove old
        // leastSquareAccumulate(sample[0], sample[1], sample[2], true); // add new
      // replace the worst sample
      x_vec[worstSample] = sample[0];
      y_vec[worstSample] = sample[1];
      z_vec[worstSample] = sample[2];
      if (weightedTrustScore>5) improvementSamples++; // only count improvements that have decent spread
      // update the min distance for all points now that we have accepted a new point:
      recalculateTrustScores();
      // logger.trace("best points:");
      // for (int i = 0; i < SAMPLES_TOTAL; i++){
      //   logger.trace("    #%d - minDist: %0.2f, x: %0.1f, y: %0.1f, z: %0.1f", i, minDist_vec[i], x_vec[i], y_vec[i], z_vec[i]);
      // }
      // recalculate best sphere to use as hard-iron offset
        // accumulateAll(); // rerun accumulator...
        // bestSphereUpdate();
      
      // use magneto
#if BENCHMARK_MAG_CALIBRATOR
      magnetoCalculateBM.before();
#endif
      runMagneto();
#if BENCHMARK_MAG_CALIBRATOR
      magnetoCalculateBM.after();
#endif

      // @TODO: Have some better way to determine when the calibration is ready to provide samples to VQF
      // we want to provide samples as soon as we have a good enough hard iron calibration, but we don't
      // want to wait too long, otherwise we can't utilize the mag disturbance rejection
      // VQF WILL NOT CHANGE IF WE HAVE THIS DUMMY
      // if (!calibrationReady && improvementSamples > 15) {
        calibrationReady = true; // TEST: approve immediately 
      // }

      emitSampleAccept(newSample, oldSample, worstSample); // emit after magneto is updated

      // find the worst fitting sample
      // consider occasionally tossing out the worst fitting sample so that we can improve fit over time
      // might backfire if the "bad fitting sample" is actually important to the best sphere fit.
      float maxError = 0;
      for (int i = 0; i < SAMPLES_TOTAL; i++) {
        float pointScore = 0;
        float checkSample[3] = {x_vec[i], y_vec[i], z_vec[i]};
        scaleAndScore(checkSample, &pointScore);
        if (getErrorAmt(pointScore) > maxError) {
          maxError = getErrorAmt(pointScore);
        }
      }
      logger.debug("Ellipoid calibrated! Max point error: %0.4f", maxError);
    }
  };

  // scale with score
  void scaleAndScore(float* sample, float* score) {
    // logStatus(); // attempt to log whenever we are scaling a sample c:
    // require minimum calibration before being ready to serve samples to vqf
    // if (totalSpread < 10000 || improvementSamples < 200 || (radius < 98 || radius > 102)) {
    
    float mag_x = sample[0];
    float mag_y = sample[1];
    float mag_z = sample[2];
    // apply hard iron
    mag_x -= M_Bias[0];
    mag_y -= M_Bias[1];
    mag_z -= M_Bias[2];

    // apply soft iron
    // @TODO: Consider not applying soft-iron until we are within a normal ellipsoid shape (not blown out on any axis)
    //        or consider having trust for each of the axies seperately? idk.
    float cx = M_Ainv[0][0] * mag_x + M_Ainv[0][1] * mag_y + M_Ainv[0][2] * mag_z;
    float cy = M_Ainv[1][0] * mag_x + M_Ainv[1][1] * mag_y + M_Ainv[1][2] * mag_z;
    float cz = M_Ainv[2][0] * mag_x + M_Ainv[2][1] * mag_y + M_Ainv[2][2] * mag_z;

    float clength = sqrtf(cx * cx + cy * cy + cz * cz);
    *score = clength / hm;

    // logger.trace("Sample Score: %f", *score);

    // // only apply soft iron bias if it is within sane values.
    if (!isSoftIronOkay) {
      sample[0] = mag_x;
      sample[1] = mag_y;
      sample[2] = mag_z;
      return;
    }
    sample[0] = cx;
    sample[1] = cy;
    sample[2] = cz;
  }

  // Takes the raw distance from other points as well as the deviation from the ellipsoid skin
  // then creates a weighted penalty score and applys that to the spread.
  // Returns an overall calculated trust score. 
  const float PENALTY_SCALE = 1.0f / 0.1f; // a deviation of 0.1 will halve the spread score as a penalty
  float calculateTrust(float spreadScore, float deviationScore) {
    float internalScore = deviationScore;
    if (!isGoodSoftCal()) { internalScore = 1; }
    float deviationNorm = PENALTY_SCALE * (internalScore-1.0f);
    return spreadScore / (1.0f+(deviationNorm*deviationNorm));
  }

  /**
   * Code to calculate the least squares sphere to use as the hard iron bias for the mag calibration.
   * Efficient least squares sphere calculation algorithm: https://arxiv.org/pdf/1506.02776
   * Reused some code from: https://github.com/alinja/magnetometer_calib
   */

  // the hard iron bias/sphere center
  float totalSpread = 0;
  float radius = 0;
  float M_Bias[3] = {0.0, 0.0, 0.0};
  float M_Ainv[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
  double hm = 0;

  int ellipsoidThrottleAttempts = 0;
  void runMagneto() {
    // throttle how frequently the magneto algorithm is run (points are still adjusted)
    if (millis() - lastEllipsoidCreate <= THROTTLE_ELLIPSOID_DURATION) { 
      ellipsoidThrottleAttempts++;
      logger.debug("Throttling Ellipsoid creation! Throttled %d times during this period.", ellipsoidThrottleAttempts);
      return; 
    }
    ellipsoidThrottleAttempts = 0;
    lastEllipsoidCreate = millis();

    // create a new calibration
    MagnetoCalibration* magneto = new MagnetoCalibration();
    // logger.debug("{");
    for (int i = 0; i < SAMPLES_TOTAL; i++) {
      magneto->sample(x_vec[i], y_vec[i], z_vec[i]);  
      // logger.debug("  [%0.4f, %0.4f, %0.4f],", x_vec[i], y_vec[i], z_vec[i]);
    }
    // logger.debug("}");
    float M_BAinv[4][3];
    magneto->current_calibration(M_BAinv, &hm);
    logger.trace("hm: %f", hm);
    delete magneto;

    // logger.debug("Magneto1.4 calibration: (x: %0.2f, y: %0.2f, z: %0.2f, spread: %0.1f, improvement samples: %d)", M_BAinv[0][0], M_BAinv[0][1], M_BAinv[0][2], getTotalSpread(), improvementSamples);
    M_Bias[0] = M_BAinv[0][0];
    M_Bias[1] = M_BAinv[0][1];
    M_Bias[2] = M_BAinv[0][2];

    for (int i = 0; i < 3; i++){
      M_Ainv[0][i] = M_BAinv[1][i];
      M_Ainv[1][i] = M_BAinv[2][i];
      M_Ainv[2][i] = M_BAinv[3][i];
    }
    // logger.debug("Magneto1.4 calibration matrix:");
    // logger.debug("{");
    // for (int i = 0; i < 3; i++) {
    //   logger.debug(
    //     "  [%f, %f, %f]",
    //     M_BAinv[1][i],
    //     M_BAinv[2][i],
    //     M_BAinv[3][i]
    //   );
    // }
    // logger.debug("}");
  }
};

}  // namespace SlimeVR::Sensors::RuntimeCalibration
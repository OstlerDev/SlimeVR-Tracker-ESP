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

#define BENCHMARK_MAG_CALIBRATOR true
#if BENCHMARK_MAG_CALIBRATOR
#include "debugging/Benchmark.h"
#endif


/* How many points should we collect to try and find the calibration */
#define SAMPLES_TOTAL (32)

namespace SlimeVR::Sensors::RuntimeCalibration {

class HealthSampler {
public:
  void setHealth(float newHealth) { health = newHealth; }
  float getHealth() { return health; }
  void addSample(float score) {
    float sampleHealth = scoreToHealth(score);
    // simple moving average to keep things lightweight and fast
    float healthDelta = SMOOTHING_FACTOR * (sampleHealth - health);
    health += healthDelta;
  }
private:
  const int HEALTH_WINDOW = 2000;
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

struct MagCalParams {
  /* The initial health for the Best calibration, higher values speed up the "warmup" of saved calibrations */
  float StartHealth = 0.6f;
  /* How much movement is required for a sample to affect the rolling health of active/candidate calibrations */
  float HealthMovementThreshold = 3.0f;
  /* The health threshold for mag to be determined to be healthy, controls when 9D fusion is used */
  float MagHealthyThreshold = 0.9f;

  /* The delay on startup before points will start to be collected */
  int StartupCollectionDelay = 0 * 1000;
  /* How frequently we should log all of the calibration data for debugging */
  int LogDumpFrequency = 15 * 1000;

  /* How frequently should we test incoming samples to determine if we should add them to our point cloud */
  int SampleFrequency = 10;
  /* The miniumum distance allowed when collecting new points to fill empty slots in our point cloud */
  float MinDistanceFreshPoints = 5;
  /* How much should we penalize deviation from the ellipsoid skin when creating a weighted trust score for an incoming point */
  float DeviationScorePenalty = 1.0f / 0.1f; // a deviation of 0.1 will halve the spread score as a penalty
  /* The max deviation allowed before points are required to be thrown out to prevent bad calibrations */
  float MaxDeviationNewPoints = 0.3;
  /* How much deviation is allowed in our point cloud from the ellipsoid before we should force remove it */
  float ForceRemoveDeviation = 0.15;
  /* The minimum distance allowed when force removing the most deviated point during worst sample selection */
  int ForceRemoveMinDistance = 100;
  /* How much deviation from the average distance is allowed before force removing a point for deviated distance */
  float ForceRemoveDistanceMultiplier = 5;
  /* How many times should we try to find a better point when forcing a point to be removed before attempting to */
  /* clean the bad point cloud by wiping a portion of the point cloud and wipe the bad point */
  int ForceRemoveMaxRetries = 100;


  /* The minimum amount of health that is required for a candidate to be promoted. Lower values encourages more frequent */
  /* reconvergence during calibration warmup, higher values make the first promotion wait longer. Lower values are recommended. */
  float CandidateMinPromotionHealth = 0.2f;
  /* After promoting a candidate calibration what should we reset the candidate health to? */
  float CandidateHealthAfterPromotion = 0.0f;  
  /* When wiping part of the point cloud, how many points should we remove? This is performed after a candidate is */
  /* accepted as well as when forcing cleanup fails */
  int CleaningInterval = 2; // clear out every second datapoint

  /* Soft Calibration Requirements to be deemed healthy. Stops unhealthy ellipsoids from "locking in" too early. */
  /* Once a soft calibration is determined to be healthy we start restricting new point deviation from surface. */
  /* Also prevents the soft calibration from being a very distorted ellipsoid */
  float GoodSoftCalMajorMinimum = 0.5;
  float GoodSoftCalMajorMaximum = 3;
  float GoodSoftCalMajorMultiplier = 2;
  float GoodSoftCalDiagMultiplier = 5;

  /* Throttle running heavy calculations for determining the best calibration for our set of points to this duration */
  /* This helps prevent lag when many new points are coming in */
  int ThrottleEllipsoidCalculation = 2 * 1000;
};

class MagCalibrator {
public:
  MagCalibrator(VQF* vqf) : fusion(vqf) {
    resetCalibration(); // init
    activeHealth.setHealth(params.StartHealth);
  }

  // public because returning them is annoying...
  float Best_M_Bias[3] = {0.0f, 0.0f, 0.0f};
  float Best_M_Ainv[3][3] = {{1.0f,0.0f,0.0f},{0.0f,1.0f,0.0f},{0.0f,0.0f,1.0f}};
  double Best_hm = 100;
  float Candidate_M_Bias[3] = {0.0f, 0.0f, 0.0f};
  float Candidate_M_Ainv[3][3] = {{1.0f,0.0f,0.0f},{0.0f,1.0f,0.0f},{0.0f,0.0f,1.0f}};
  double Candidate_hm = 100;

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

    float movementFromLast = calculateDistance(sample[0], lastSample[0], sample[1], lastSample[1], sample[2], lastSample[2]);
    lastSample[0] = sample[0];
    lastSample[1] = sample[1];
    lastSample[2] = sample[2];

    float score = 0.0f;
    scaleAndScore(sample, &score, Best_M_Bias, Best_M_Ainv, Best_hm);

    // only adjust health when movement is detected (don't improve health for sitting still)
    if (movementFromLast > params.HealthMovementThreshold){
      activeHealth.addSample(score);

      float candidateScore = 0.0f;
      float candidateSample[3] = {lastSample[0], lastSample[1], lastSample[2]};
      scaleAndScore(candidateSample, &candidateScore, Candidate_M_Bias, Candidate_M_Ainv, Candidate_hm);
      candidateHealth.addSample(candidateScore);
    }
  }

  // Should we start using 9D fusion outputs or keep using 6D fusion.
  // We still pass mag data to VQF, this is just if we utilize it or not.
  bool magHealthy() { 
    return activeHealth.getHealth() > params.MagHealthyThreshold;
    // return vqfHasBeenHealthy; // check if vqf has found a mag heading or not
  }
  bool vqfReady() { return vqfHasBeenHealthy; }

  void setMBias(float M_Bias[3]) {
    Best_M_Bias[0] = M_Bias[0];
    Best_M_Bias[1] = M_Bias[1];
    Best_M_Bias[2] = M_Bias[2];
  }
  void setMAinv(float M_A_inv[3][3]) {
    for (int i = 0; i < 3; i++) {
      Best_M_Ainv[0][i] = M_A_inv[0][i];
      Best_M_Ainv[1][i] = M_A_inv[1][i];
      Best_M_Ainv[2][i] = M_A_inv[2][i];
    }
  }
  void setHM(float M_hm) {
    Best_hm = M_hm;
  }

  // debug helpers
  void logCalibration() { logStatus(true); }
  float getTotalHealthDuration(){ return totalHealthyTime; }
  float getTotalDisturbedDuration(){
    float disturbedFor = (millis() - lastHealthyTime) / 1000.0f;
    return totalDisturbedTime + disturbedFor;
  }
  // // Will run this function if provided when new points are accepted into the calibration, useful for debugging/recording logs
  void onSampleAccept(std::function<void(float[3], float[3], int)>&& callback) {
    sampleAcceptCallback = callback;
  }

private:
#if BENCHMARK_MAG_CALIBRATOR
  SlimeVR::Debugging::Benchmark magCalibrateBM{"MagCalibrator.provideMagSample()"};
  SlimeVR::Debugging::Benchmark magnetoCalculateBM{"MagCalibrator.runMagneto()"};
#endif
  Logging::Logger logger{"MagCalibrator"};
  VQF* fusion; // reference to vqf for direct access
  MagCalParams params;

  // calls function with (float* newSample, float* oldSample)
  std::optional<std::function<void(float[3], float[3], int)>> sampleAcceptCallback;
  void emitSampleAccept(float newSample[3], float oldSample[3], int index){
    if (sampleAcceptCallback) {
      (*sampleAcceptCallback)(newSample, oldSample, index);
    }
  }

  HealthSampler activeHealth;
  HealthSampler candidateHealth;

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
    // wait to collect datapoints
    if (now - startTime > params.StartupCollectionDelay) {
      startCalibration = true;
    }
    // full log 
    if (now - lastLogTime > params.LogDumpFrequency || forcePrint) {
      lastLogTime = now;
      const VQFState& vqfState = fusion->getState();
      float disturbedFor = (millis() - lastHealthyTime) / 1000.0f;
      logger.debug("[%d] VQF: Healthy Duration: %0.2f (current accepted mag field: norm %0.4f, dip %0.4f) Disturbed duration: %0.1f (candidate mag field: time %0.2f, norm %0.4f, dip %0.4f)", sampleNumber, vqfState.magUndisturbedT, vqfState.magRefNorm, vqfState.magRefDip, disturbedFor, vqfState.magCandidateT, vqfState.magCandidateNorm, vqfState.magCandidateDip);
      logger.debug("Active calibration health: %0.3f, Candidate calibration health: %0.3f", activeHealth.getHealth(), candidateHealth.getHealth());
      
      // log current best
      logger.debug("Best Magneto1.4 calibration: (%0.2f, %0.2f, %0.2f, spread: %0.1f, total error: %0.3f, improvement samples: %d)", Best_M_Bias[0], Best_M_Bias[1], Best_M_Bias[2], getTotalSpread(), getErrorTotal(), improvementSamples);
      logger.debug("Best Magneto1.4 calibration matrix:");
      logger.debug("{");
      for (int i = 0; i < 3; i++) {
        logger.debug(
          "  [%f, %f, %f],",
          Best_M_Ainv[0][i],
          Best_M_Ainv[1][i],
          Best_M_Ainv[2][i]
        );
      }
      logger.debug("}");

      // log current candidate
      logger.debug("Candidate Magneto1.4 calibration: (%0.2f, %0.2f, %0.2f, spread: %0.1f, total error: %0.3f, improvement samples: %d)", Candidate_M_Bias[0], Candidate_M_Bias[1], Candidate_M_Bias[2], getTotalSpread(), getErrorTotal(), improvementSamples);
      logger.debug("Candidate Magneto1.4 calibration matrix:");
      logger.debug("{");
      for (int i = 0; i < 3; i++) {
        logger.debug(
          "  [%f, %f, %f],",
          Candidate_M_Ainv[0][i],
          Candidate_M_Ainv[1][i],
          Candidate_M_Ainv[2][i]
        );
      }
      logger.debug("}");

      // log point cloud
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
  int improvementSamples = 0;
  float x_vec[SAMPLES_TOTAL];
  float y_vec[SAMPLES_TOTAL];
  float z_vec[SAMPLES_TOTAL];

  float minDist_vec[SAMPLES_TOTAL];
  float pointDeviation[SAMPLES_TOTAL];
  float pointTrustScore[SAMPLES_TOTAL];

  // scale and calculate score for how fit it is to the ellipsoid skin
  void scaleAndScore(float* sample, float* score, float M_Bias[3], float M_Ainv[3][3], float hm) {
    *score = 0.0f;
    // sanity check, don't run if we have not created a calibration yet.
    if (M_Bias[0] == 0 && M_Bias[1] == 0 && M_Bias[2] == 0) { return; }
    
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

    // only apply soft iron bias if it is within sane values.
    if (!isGoodSoftCal(Best_M_Ainv)) {
      sample[0] = mag_x;
      sample[1] = mag_y;
      sample[2] = mag_z;
      return;
    }
    sample[0] = cx;
    sample[1] = cy;
    sample[2] = cz;
  }

  void resetCalibration(){
    improvementSamples = 0;
    for (int i = 0; i < SAMPLES_TOTAL; i++) {
      x_vec[i] = 0;
      y_vec[i] = 0;
      z_vec[i] = 0;
      pointTrustScore[i] = 0;
    }

    Best_hm = 100;
    Best_M_Bias[0] = 0.0f;
    Best_M_Bias[1] = 0.0f;
    Best_M_Bias[2] = 0.0f;
    for (int i = 0; i < 3; i++) {
      Best_M_Ainv[0][i] = 0.0f;
      Best_M_Ainv[1][i] = 0.0f;
      Best_M_Ainv[2][i] = 0.0f;
    }
    Best_M_Ainv[0][0] = 1.0f;
    Best_M_Ainv[1][1] = 1.0f;
    Best_M_Ainv[2][2] = 1.0f;

    Candidate_hm = 100;
    Candidate_M_Bias[0] = 0.0f;
    Candidate_M_Bias[1] = 0.0f;
    Candidate_M_Bias[2] = 0.0f;
    for (int i = 0; i < 3; i++) {
      Candidate_M_Ainv[0][i] = 0.0f;
      Candidate_M_Ainv[1][i] = 0.0f;
      Candidate_M_Ainv[2][i] = 0.0f;
    }
    Candidate_M_Ainv[0][0] = 1.0f;
    Candidate_M_Ainv[1][1] = 1.0f;
    Candidate_M_Ainv[2][2] = 1.0f;
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
  // find the worst sample that we should replace within the point cloud.
  // Either find the sample with the lowest spread for replacement,
  // or find the sample that severely deviates from 
  int forceReplaceCount = 0;
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
    if (currentBestDist > params.ForceRemoveMinDistance && currentBestDist > averageDistance * params.ForceRemoveDistanceMultiplier) {
      logger.debug("Force replacing deviated distance: %0.2f (avg: %0.2f), index: %d", currentBestDist, averageDistance, bestDistIndex);
      pointTrustScore[bestDistIndex] = 0;
      forceReplaceCount++;
      return bestDistIndex;
    }
    // if there are no isolated samples, next reject samples that deviate from ellipsoid skin.
    if (getErrorAmt(worstErrorScore) > params.ForceRemoveDeviation && isGoodSoftCal(Candidate_M_Ainv)) {
      logger.debug("Force replacing deviated score: %0.4f, index: %d", worstErrorScore, worstErrorSample);
      // Set the weighted trust score to zero to force replacement of this sample.
      pointTrustScore[worstErrorSample] = 0;
      forceReplaceCount++;
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
      float score = 0.0f;
      scaleAndScore(sample, &score, Candidate_M_Bias, Candidate_M_Ainv, Candidate_hm);
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
    return spread;
  }
  float getErrorTotal(){
    float totalError = 0;
    for (int i = 0; i < SAMPLES_TOTAL; i++) {
      totalError += getErrorAmt(pointDeviation[i]);
    }
    return totalError;
  }
  // Takes the raw distance from other points as well as the deviation from the ellipsoid skin
  // then creates a weighted penalty score and applys that to the spread.
  // Returns an overall calculated trust score. 
  float calculateTrust(float spreadScore, float deviationScore) {
    float internalScore = deviationScore;
    if (!isGoodSoftCal(Candidate_M_Ainv)) { internalScore = 1; }
    float deviationNorm = params.DeviationScorePenalty * (internalScore-1.0f);
    return spreadScore / (1.0f+(deviationNorm*deviationNorm));
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
  bool isGoodSoftCal(float M_Ainv[3][3]) {
    // soft iron sanity checks
    float x = M_Ainv[0][0];
    float y = M_Ainv[1][1];
    float z = M_Ainv[2][2];

    float min = params.GoodSoftCalMajorMinimum;
    float max = params.GoodSoftCalMajorMinimum;
    float mmult = params.GoodSoftCalMajorMultiplier;
    float dmult = params.GoodSoftCalDiagMultiplier;
    if (x < min || y < min || z < min || x > max || y > max || z > max) {
      return false;
    }
    if (x > y*mmult || x > z*mmult || y > x*mmult || y > z*mmult || z > x*mmult || z > y*mmult) {
      return false;
    }
    // sanity check diagonals
    float d1 = M_Ainv[0][1] * dmult;
    float d2 = M_Ainv[0][2] * dmult;
    float d3 = M_Ainv[1][2] * dmult;
    if (d1 > x || d1 > y || d1 > z || d2 > x || d2 > y || d2 > z || d3 > x || d3 > y || d3 > z) { 
      return false; 
    }

    return true;
  }

  bool doCalibrationsMatch() {
    // If we have not initialized the best calibration then return false
    if (!hasBestCalibration()) return false;
    // check if they are an exact match or not
    if (Candidate_M_Bias[0] != Best_M_Bias[0]) return false;
    if (Candidate_M_Bias[1] != Best_M_Bias[1]) return false;
    if (Candidate_M_Bias[2] != Best_M_Bias[2]) return false;
    for (int i = 0; i < 3; i++) {
      if (Candidate_M_Ainv[i][0] != Best_M_Ainv[i][0]) return false;
      if (Candidate_M_Ainv[i][1] != Best_M_Ainv[i][1]) return false;
      if (Candidate_M_Ainv[i][2] != Best_M_Ainv[i][2]) return false;
    }
    if (Candidate_hm != Best_hm) return false;
    return true;
  }
  bool hasBestCalibration() {
    if (Best_M_Bias[0] == 0 && Best_M_Bias[1] == 0 && Best_M_Bias[2] == 0) return false;
    return true;
  }

  /**
   * Handle creation of the calibration
   */
  int samplerDelay = 0;
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
    if (samplerDelay % params.SampleFrequency != 0) { return; }
    samplerDelay = 0;

    if (freeze) {
      return;
    }

    
    // If we have a best calibration, and that best calibration has a good soft calibration, don't get rid of it in favor of a soft
    // calibration that is not a good soft calibration. Don't toss out good for bad.
    bool betterSoftCal = true;
    if (hasBestCalibration() && isGoodSoftCal(Best_M_Ainv) && !isGoodSoftCal(Candidate_M_Ainv)) { betterSoftCal = false; }
    // Check if the current candidate ellipsoid is the best one we have right now
    // We do this before accepting a new point and calculating a new ellipsoid
    // This means early on we will get lots of calibrations accepted then it becomes harder
    if (
      !doCalibrationsMatch() 
      && activeHealth.getHealth() < candidateHealth.getHealth() 
      && candidateHealth.getHealth() > params.CandidateMinPromotionHealth
      && betterSoftCal
    ) {
      logger.debug("Promoting Candidate Ellipsoid to Best Ellipsoid! Active Health: %0.3f, Candidate Health: %0.3f", activeHealth.getHealth(), candidateHealth.getHealth());
      Best_M_Bias[0] = Candidate_M_Bias[0];
      Best_M_Bias[1] = Candidate_M_Bias[1];
      Best_M_Bias[2] = Candidate_M_Bias[2];
      Candidate_M_Bias[0] = 0.0f;
      Candidate_M_Bias[1] = 0.0f;
      Candidate_M_Bias[2] = 0.0f;
      for (int i = 0; i < 3; i++) {
        Best_M_Ainv[0][i] = Candidate_M_Ainv[0][i];
        Best_M_Ainv[1][i] = Candidate_M_Ainv[1][i];
        Best_M_Ainv[2][i] = Candidate_M_Ainv[2][i];
        Candidate_M_Ainv[0][i] = 0.0f;
        Candidate_M_Ainv[1][i] = 0.0f;
        Candidate_M_Ainv[2][i] = 0.0f;
      }
      Candidate_M_Ainv[0][0] = 1.0f;
      Candidate_M_Ainv[1][1] = 1.0f;
      Candidate_M_Ainv[2][2] = 1.0f;
      Best_hm = Candidate_hm;
      Candidate_hm = 100;

      // Wipe out part of the point cloud to force reconvergence of calibration
      for (int i = 0; i < SAMPLES_TOTAL; i++) {
        if (i % params.CleaningInterval == 0) continue; // clear out every X datapoint
        x_vec[i] = 0;
        y_vec[i] = 0;
        z_vec[i] = 0;
        pointTrustScore[i] = 0;
      }

      // Reset candidate health to make the new candidate prove it's healthy over a new time period.
      candidateHealth.setHealth(0.0f);
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

          // Don't ignore incoming points by using VQF as the rejector. Good idea in theory, bad in practice. Too many points get
          // rejected which slows down convergence on good calibrations because VQF does not know the correct mag vector.
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
        if (!vqfHasBeenHealthy) logger.info("[%d] VQF: Accepted mag heading for first time! Active calibration health: %0.3f, Candidate calibration health: %0.3f, Accepted candidate norm: %0.4f, dip: %0.4f", sampleNumber, activeHealth.getHealth(), candidateHealth.getHealth(), vqfState.magRefNorm, vqfState.magRefDip);
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
    if (minDistance < params.MinDistanceFreshPoints && emptyIndex != -1) return;
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
      }
      return;
    }

    // if there are not any empty slots, check if we are better than the worst sample, if so, replace it.
    int worstSample = getWorstSample();

    // if we are stuck in a bad state, go ahead and wipe out half of the points and specifically the point that is bad.
    if (forceReplaceCount > params.ForceRemoveMaxRetries) {
      logger.debug("Force replacing stuck in bad state, wiping half of the point cloud to clear bad state...");
      forceReplaceCount = 0;
      // Wipe out 1/2rd of the point cloud to force reconvergence
      for (int i = 0; i < SAMPLES_TOTAL; i++) {
        if (i % 2 == 0) continue; // skip every other
        x_vec[i] = 0;
        y_vec[i] = 0;
        z_vec[i] = 0;
        pointTrustScore[i] = 0;
      }
      x_vec[worstSample] = 0;
      y_vec[worstSample] = 0;
      z_vec[worstSample] = 0;
      pointTrustScore[worstSample] = 0;
    }

    // test how well the new point fits the existing ellipsoid
    float score = 0.0f;
    float testSample[3] = {sample[0], sample[1], sample[2]};
    scaleAndScore(&testSample[0], &score, Candidate_M_Bias, Candidate_M_Ainv, Candidate_hm);

    float weightedTrustScore = calculateTrust(minDistance, score);
    if (sampleNumber % 10000 == 0 || pointTrustScore[worstSample] <= 0.0f) logger.debug("[%d] Weighted Trust: %0.4f, Deviation Score: %0.4f, Raw Distance: %0.2f | Worst index: %d, worst trust: %0.4f, worst dist: %0.2f, worst score: %0.4f | New Point: (%0.2f, %0.2f, %0.2f)", sampleNumber, weightedTrustScore, score, minDistance, worstSample, pointTrustScore[worstSample], minDist_vec[worstSample], pointDeviation[worstSample], sample[0], sample[1], sample[2]);

    // hard ignore points that deviate from the **best** ellipsoid skin **UNLESS** we are force replacing a bad point
    if (getErrorAmt(score) > params.MaxDeviationNewPoints && isGoodSoftCal(Candidate_M_Ainv) && pointTrustScore[worstSample] > 0.0f) { return; }
    // don't accept bad samples in the first place.
    if (minDistance > averageDistance * params.ForceRemoveDistanceMultiplier) { return; }
    // if we are force replacing a point, make sure we are accepting a less deviated sample into the array instead of a worse sample
    if (pointTrustScore[worstSample] <= 0.0f && (getErrorAmt(score) > params.ForceRemoveDeviation && getErrorAmt(score) > getErrorAmt(pointDeviation[worstSample])) && isGoodSoftCal(Candidate_M_Ainv)) { return; }

    // See if it is a better point than an existing one
    if ((weightedTrustScore > pointTrustScore[worstSample] || pointTrustScore[worstSample] <= 0.0f) && !freeze) {
      forceReplaceCount = 0;
      logger.debug("[%d] Replacing!! Active health: %0.3f, Candidate health: %0.3f, Weighted Trust: %0.4f, Deviation Score: %0.4f, Raw Distance: %0.2f, New Point: (%0.2f, %0.2f, %0.2f) | Worst index: %d, worst trust: %0.4f, worst dist: %0.2f, worst score: %0.4f, worst Point: (%0.2f, %0.2f, %0.2f) ", sampleNumber, activeHealth.getHealth(), candidateHealth.getHealth(), weightedTrustScore, score, minDistance, sample[0], sample[1], sample[2], worstSample, pointTrustScore[worstSample], minDist_vec[worstSample], pointDeviation[worstSample], x_vec[worstSample], y_vec[worstSample], z_vec[worstSample]);
      float oldSample[3] = {x_vec[worstSample], y_vec[worstSample], z_vec[worstSample]}; // for emitSampleAccept
      float newSample[3] = {sample[0], sample[1], sample[2]};

      // replace the worst sample
      x_vec[worstSample] = sample[0];
      y_vec[worstSample] = sample[1];
      z_vec[worstSample] = sample[2];
      improvementSamples++;
      // update the min distance for all points now that we have accepted a new point:
      recalculateTrustScores();
      
      // use magneto
#if BENCHMARK_MAG_CALIBRATOR
      magnetoCalculateBM.before();
#endif
      runMagneto();
#if BENCHMARK_MAG_CALIBRATOR
      magnetoCalculateBM.after();
#endif

      calibrationReady = true;

      emitSampleAccept(newSample, oldSample, worstSample); // emit after magneto is updated
    }
  };

  // the hard iron bias/sphere center
  int ellipsoidThrottleAttempts = 0;
  void runMagneto() {
    // throttle how frequently the magneto algorithm is run (points are still adjusted)
    if (millis() - lastEllipsoidCreate <= params.ThrottleEllipsoidCalculation) { 
      ellipsoidThrottleAttempts++;
      logger.debug("Throttling Ellipsoid creation! Throttled %d times during this period.", ellipsoidThrottleAttempts);
      return; 
    }
    ellipsoidThrottleAttempts = 0;
    lastEllipsoidCreate = millis();

    // create a new calibration
    MagnetoCalibration* magneto = new MagnetoCalibration();
    for (int i = 0; i < SAMPLES_TOTAL; i++) {
      magneto->sample(x_vec[i], y_vec[i], z_vec[i]);  
    }
    float M_BAinv[4][3];
    magneto->current_calibration(M_BAinv, &Candidate_hm);
    delete magneto;

    Candidate_M_Bias[0] = M_BAinv[0][0];
    Candidate_M_Bias[1] = M_BAinv[0][1];
    Candidate_M_Bias[2] = M_BAinv[0][2];

    for (int i = 0; i < 3; i++){
      Candidate_M_Ainv[0][i] = M_BAinv[1][i];
      Candidate_M_Ainv[1][i] = M_BAinv[2][i];
      Candidate_M_Ainv[2][i] = M_BAinv[3][i];
    }

    logger.debug("Candidate Ellipoid calibrated! Active Health: %0.3f, Candidate Health: %0.3f", activeHealth.getHealth(), candidateHealth.getHealth());
  }
};

}  // namespace SlimeVR::Sensors::RuntimeCalibration
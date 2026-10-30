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

#define SAMPLES_TOTAL (64)

namespace SlimeVR::Sensors::RuntimeCalibration {

class MagCalibrator {
public:
  MagCalibrator(VQF* vqf) : fusion(vqf) {}

  bool freeze = false;
  void scaleMagSample(float* sample) {
    logStatus();
    provideMagSample(sample); // add to calibration if needed

    if (!startCalibration || !calibrationReady) {
      sample[0] = 0;
      sample[1] = 0;
      sample[2] = 0;
      return;
    }

    float score = 0;
    scaleAndScore(sample, &score);
    // logger.trace("Sample Score: %f, Trust: %d, Frozen: %d", score, calibrationTrust, freeze);
    // logger.trace("Sample Score: %f", score);

    // if (improvementSamples < 50) {
    //   sample[0] = 0;
    //   sample[1] = 0;
    //   sample[2] = 0;
    //   return;
    // }
  }

private:
  Logging::Logger logger{"MagCalibrator"};
  VQF* fusion; // reference to vqf for direct access

  bool calibrationReady = false;

  uint64_t startTime = 0;
  bool startCalibration = false;
  uint64_t lastLogTime = 0;
  void logStatus(){
    uint64_t now = millis();
    if (startTime == 0) {
      startTime = now;
    }
    // wait 2 minutes to collect datapoints
    if (now - startTime > 60*0.2*1000) {
      startCalibration = true;
    }
    // full log every 30 seconds
    if (now - lastLogTime > 60*0.5*1000) {
      lastLogTime = now;
      logger.debug("Magneto1.4 calibration: (%0.2f, %0.2f, %0.2f, spread: %0.1f, total error: %0.1f, improvement samples: %d)", M_Bias[0], M_Bias[1], M_Bias[2], getTotalSpread(), getErrorTotal(), improvementSamples);
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
  // find the sample that is closest to another sample so that we can replace it.
  int getWorstSample(){
    float currentWorst = pointTrustScore[0];
    int worstSample = 0;
    for (int i = 1; i < SAMPLES_TOTAL; i++) {
      if (pointTrustScore[i] < currentWorst) {
        currentWorst = pointTrustScore[i];
        worstSample = i;
      }
    }
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
  void recalculateTrustScores() { 
    for (int i = 0; i < SAMPLES_TOTAL; i++) {
      float sample[3] = {x_vec[i], y_vec[i], z_vec[i]};
      minDist_vec[i] = getMinDistance(sample, i);

      float score;
      scaleAndScore(sample, &score);
      pointDeviation[i] = score;
      pointTrustScore[i] = calculateTrust(minDist_vec[i], score);
    }
  }
  float getTotalSpread(){
    float spread = 0;
    for (int i = 0; i < SAMPLES_TOTAL; i++) {
      spread += minDist_vec[i];
    }
    totalSpread = spread;
    return spread;
  }
  float getErrorTotal(){
    float totalError = 0;
    for (int i = 0; i < SAMPLES_TOTAL; i++) {
      float devScore = pointDeviation[i];
      if (devScore < 1) {
        totalError += (1-devScore);
      } else {
        totalError += (devScore-1);
      }
    }
    return totalError;
  }
  // calculates raw distance between 2 3d points
  float calculateDistance(float x1, float x2, float y1, float y2, float z1, float z2){
    return ((x1-x2)*(x1-x2)) + ((y1-y2)*(y1-y2)) + ((z1-z2)*(z1-z2));
  };

  /**
   * Handle creation of the calibration
   */
  const int sampleRate = 10; // consider changing to time based collection metric, also consider subsample averaging of all datapoints
  int samplerDelay = 0;
  double lastSampleTime;
  bool vqfHasBeenHealthy = false;
  void provideMagSample(float* sample) {
    // delay to start calibration
    if (!startCalibration) {
      return;
    }
    // only collect a datapoint every X samples provided
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
        // Consider also tossing out any samples added to the array in the last 0.5 seconds
        // in case any bad samples have snuck in before VQF could trigger mag disturbance rejections

        // NOTE! Mag disturbance detection DOES NOT handle if a magnet is brought close by but doesn't change the
        // angle. We /might/ still want some way to detect if the sensor readings spiked out of reasonable ranges.
        // ignore provided sample
        // only start ignoring once VQF has been undisturbed a single time
        if (vqfHasBeenHealthy) {
          logger.trace("VQF: Mag is Disturbed! Ignoring Sample... Disturbed duration: %0.1f", vqfState.magRejectT);
          return;
        } else {
          logger.trace("VQF: Calibrating for the first time, Candidate time: %0.5f", vqfState.magCandidateT);
        }
      } else {
        vqfHasBeenHealthy = true;
        logger.trace("VQF: Mag is healthy :3! Healthy duration: %0.5f, lastMagDisAngle: %0.5f", vqfState.magUndisturbedT, (fusion->getState()).lastMagDisAngle);
      }
    }

    // first check if there are any still empty
    int emptyIndex = getFirstEmpty();
    float minDistance = getMinDistance(sample);
    // when building the array require a spread sample.
    if (minDistance < 3 && emptyIndex != -1) return;
    if (emptyIndex != -1) {
      x_vec[emptyIndex] = sample[0];
      y_vec[emptyIndex] = sample[1];
      z_vec[emptyIndex] = sample[2];

      logger.trace("Adding new datapoint to initial point cloud: %0.2f, %0.2f, %0.2f (dist: %0.2f)", sample[0], sample[1], sample[2], minDistance);

      recalculateTrustScores(); // recalculates distance
      if (emptyIndex == SAMPLES_TOTAL-1) {
        runMagneto();
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
    logger.trace("Weighted Trust: %0.4f, Deviation Score: %0.4f, Raw Distance, %0.2f Frozen: %d", weightedTrustScore, score, minDistance, freeze);

    // hard ignore points that deviate from the ellipsoid skin
    if (score < 0.75 || score > 1.25) { return; }

    // See if it is a better point than an existing one
    if (weightedTrustScore > pointTrustScore[worstSample] && !freeze) {
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
      runMagneto();

      // @TODO: Have some better way to determine when the calibration is ready to provide samples to VQF
      // we want to provide samples as soon as we have a good enough hard iron calibration, but we don't
      // want to wait too long, otherwise we can't utilize the mag disturbance rejection
      // VQF WILL NOT CHANGE IF WE HAVE THIS DUMMY
      if (!calibrationReady && improvementSamples > 15) {
        calibrationReady = true;
      }

      // find the worst fitting sample
      // consider occasionally tossing out the worst fitting sample so that we can improve fit over time
      // might backfire if the "bad fitting sample" is actually important to the best sphere fit.
      float minScore = 1;
      float runningScoreTotal = 0;
      for (int i = 0; i < SAMPLES_TOTAL; i++) {
        float pointScore = 0;
        float checkSample[3] = {x_vec[i], y_vec[i], z_vec[i]};
        scaleAndScore(checkSample, &pointScore);
        runningScoreTotal += pointScore;

        if (pointScore < minScore) {
          minScore = pointScore;
        }
      }
      runningScoreTotal = runningScoreTotal / SAMPLES_TOTAL;
      logger.trace("New fit avg score: %0.4f | Min score: %0.4f", runningScoreTotal, minScore);
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

    sample[0] = cx;
    sample[1] = cy;
    sample[2] = cz;
  }

  // Takes the raw distance from other points as well as the deviation from the ellipsoid skin
  // then creates a weighted penalty score and applys that to the spread.
  // Returns an overall calculated trust score. 
  const float PENALTY_SCALE = 1.0f / 0.1f; // a deviation of 0.1 will halve the spread score as a penalty
  float calculateTrust(float spreadScore, float deviationScore) {
    float deviationNorm = PENALTY_SCALE * (deviationScore-1.0f);
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

  void runMagneto() {
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
// Copyright (c) 2014, Freescale Semiconductor, Inc.
// All rights reserved.
// vim: set ts=4:
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Freescale Semiconductor, Inc. nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL FREESCALE SEMICONDUCTOR, INC. BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// This file contains magnetic calibration functions.  It is STRONGLY RECOMMENDED
// that the casual developer NOT TOUCH THIS FILE.  The mathematics behind this file
// is extremely complex, and it will be very easy (almost inevitable) that you screw
// it up.
//
// Haha - This file has been edited!  Please do not blame or pester NXP (formerly
//        Freescale) about the "almost inevitable" issues!

#ifndef MOTIONCAL_MAGCAL_H_
#define MOTIONCAL_MAGCAL_H_

// puppy sky note: This algorithm has been stripped down to it's base components and was initially from: https://github.com/PaulStoffregen/MotionCal/blob/master/magcal.c

// #define FXOS8700_UTPERCOUNT  0.1f
// #define DEFAULTB 50.0F        // default geomagnetic field (uT)
#define MAGBUFFSIZE 64 // used by motioncal_magcal
#define X 0                         // vector components
#define Y 1
#define Z 2
// magnetic calibration & buffer structure
typedef struct {
    // float V[3];                  // current hard iron offset x, y, z, (uT)
    // float invW[3][3];            // current inverse soft iron matrix
    // float B;                     // current geomagnetic field magnitude (uT)
    // float FourBsq;               // current 4*B*B (uT^2)
    // float FitError;              // current fit error %
    // float FitErrorAge;           // current fit error % (grows automatically with age)
    float trV[3];                // trial value of hard iron offset z, y, z (uT)
    float trinvW[3][3];          // trial inverse soft iron matrix size
    float trB;                   // trial value of geomagnetic field magnitude in uT
    float trFitErrorpc;          // trial value of fit error %
    // float A[3][3];               // ellipsoid matrix A
    // float invA[3][3];            // inverse of ellipsoid matrix A
    float matA[10][10];          // scratch 10x10 matrix used by calibration algorithms
    float matB[10][10];          // scratch 10x10 matrix used by calibration algorithms
    float vecA[10];              // scratch 10x1 vector used by calibration algorithms
    float vecB[4];               // scratch 4x1 vector used by calibration algorithms
    // int8_t ValidMagCal;          // integer value 0, 4, 7, 10 denoting both valid calibration and solver used
    float BpFast[3][MAGBUFFSIZE];   // uncalibrated magnetometer readings
    int  valid[MAGBUFFSIZE];        // 1=has data, 0=empty slot
    int MagBufferCount;           // number of magnetometer readings
} MotionCal_MagCalibration_t;


// function sets the matrix A to the identity matrix
static inline void fmatrixAeqI(float *A[], int rc)
{
  // rc = rows and columns in A

  float *pAij;  // pointer to A[i][j]
  int i, j;  // loop counters

  for (i = 0; i < rc; i++) {
    // set pAij to &A[i][j=0]
    pAij = A[i];
    for (j = 0; j < rc; j++) {
      *(pAij++) = 0.0F;
    }
    A[i][i] = 1.0F;
  }
}

// function uses Gauss-Jordan elimination to compute the inverse of matrix A in situ
// on exit, A is replaced with its inverse
static inline void fmatrixAeqInvA(float *A[], int iColInd[], int iRowInd[], int iPivot[], int isize)
{
  float largest;          // largest element used for pivoting
  float scaling;          // scaling factor in pivoting
  float recippiv;         // reciprocal of pivot element
  float ftmp;           // temporary variable used in swaps
  int i, j, k, l, m;     // index counters
  int iPivotRow, iPivotCol;  // row and column of pivot element

  // to avoid compiler warnings
  iPivotRow = iPivotCol = 0;

  // initialize the pivot array to 0
  for (j = 0; j < isize; j++) {
    iPivot[j] = 0;
  }

  // main loop i over the dimensions of the square matrix A
  for (i = 0; i < isize; i++) {
    // zero the largest element found for pivoting
    largest = 0.0F;
    // loop over candidate rows j
    for (j = 0; j < isize; j++) {
      // check if row j has been previously pivoted
      if (iPivot[j] != 1) {
        // loop over candidate columns k
        for (k = 0; k < isize; k++) {
          // check if column k has previously been pivoted
          if (iPivot[k] == 0) {
            // check if the pivot element is the largest found so far
            if (fabs(A[j][k]) >= largest) {
              // and store this location as the current best candidate for pivoting
              iPivotRow = j;
              iPivotCol = k;
              largest = (float) fabs(A[iPivotRow][iPivotCol]);
            }
          } else if (iPivot[k] > 1) {
            // zero determinant situation: exit with identity matrix
            fmatrixAeqI(A, isize);
            return;
          }
        }
      }
    }
    // increment the entry in iPivot to denote it has been selected for pivoting
    iPivot[iPivotCol]++;

    // check the pivot rows iPivotRow and iPivotCol are not the same before swapping
    if (iPivotRow != iPivotCol) {
      // loop over columns l
      for (l = 0; l < isize; l++) {
        // and swap all elements of rows iPivotRow and iPivotCol
        ftmp = A[iPivotRow][l];
        A[iPivotRow][l] = A[iPivotCol][l];
        A[iPivotCol][l] = ftmp;
      }
    }

    // record that on the i-th iteration rows iPivotRow and iPivotCol were swapped
    iRowInd[i] = iPivotRow;
    iColInd[i] = iPivotCol;

    // check for zero on-diagonal element (singular matrix) and return with identity matrix if detected
    if (A[iPivotCol][iPivotCol] == 0.0F) {
      // zero determinant situation: exit with identity matrix
      fmatrixAeqI(A, isize);
      return;
    }

    // calculate the reciprocal of the pivot element knowing it's non-zero
    recippiv = 1.0F / A[iPivotCol][iPivotCol];
    // by definition, the diagonal element normalizes to 1
    A[iPivotCol][iPivotCol] = 1.0F;
    // multiply all of row iPivotCol by the reciprocal of the pivot element including the diagonal element
    // the diagonal element A[iPivotCol][iPivotCol] now has value equal to the reciprocal of its previous value
    for (l = 0; l < isize; l++) {
      A[iPivotCol][l] *= recippiv;
    }
    // loop over all rows m of A
    for (m = 0; m < isize; m++) {
      if (m != iPivotCol) {
        // scaling factor for this row m is in column iPivotCol
        scaling = A[m][iPivotCol];
        // zero this element
        A[m][iPivotCol] = 0.0F;
        // loop over all columns l of A and perform elimination
        for (l = 0; l < isize; l++) {
          A[m][l] -= A[iPivotCol][l] * scaling;
        }
      }
    }
  } // end of loop i over the matrix dimensions

  // finally, loop in inverse order to apply the missing column swaps
  for (l = isize - 1; l >= 0; l--) {
    // set i and j to the two columns to be swapped
    i = iRowInd[l];
    j = iColInd[l];

    // check that the two columns i and j to be swapped are not the same
    if (i != j) {
      // loop over all rows k to swap columns i and j of A
      for (k = 0; k < isize; k++) {
        ftmp = A[k][i];
        A[k][i] = A[k][j];
        A[k][j] = ftmp;
      }
    }
  }
}

// 4 element calibration using 4x4 matrix inverse
static void fUpdateCalibration4INV(MotionCal_MagCalibration_t *MagCal);
static void fUpdateCalibration4INV(MotionCal_MagCalibration_t *MagCal)
{
  float fBp2;         // fBp[X]^2+fBp[Y]^2+fBp[Z]^2
  float fSumBp4;        // sum of fBp2
  float fscaling;       // set to FUTPERCOUNT * FMATRIXSCALING
  float fE;         // error function = r^T.r
  float iOffset[3];     // offset to remove large DC hard iron bias in matrix
  int iCount;       // number of measurements counted
  int i, j, k;        // loop counters

  // working arrays for 4x4 matrix inversion
  float *pfRows[4];
  int iColInd[4];
  int iRowInd[4];
  int iPivot[4];

  // compute fscaling to reduce multiplications later
  // fscaling = FXOS8700_UTPERCOUNT / DEFAULTB;

  // the trial inverse soft iron matrix invW always equals
  // the identity matrix for 4 element calibration
  // f3x3matrixAeqI(MagCal->trinvW);
  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++) {
      MagCal->trinvW[i][j] = 0.0F;
    }
    MagCal->trinvW[i][i] = 1.0F;
  }

  // zero fSumBp4=Y^T.Y, vecB=X^T.Y (4x1) and on and above
  // diagonal elements of matA=X^T*X (4x4)
  fSumBp4 = 0.0F;
  for (i = 0; i < 4; i++) {
    MagCal->vecB[i] = 0.0F;
    for (j = i; j < 4; j++) {
      MagCal->matA[i][j] = 0.0F;
    }
  }

  // the offsets are guaranteed to be set from the first element but to avoid compiler error
  iOffset[X] = iOffset[Y] = iOffset[Z] = 0;

  // use from MINEQUATIONS up to MAXEQUATIONS entries from magnetic buffer to compute matrices
  iCount = 0;
  for (j = 0; j < MAGBUFFSIZE; j++) {
    if (MagCal->valid[j]) {
      // use first valid magnetic buffer entry as estimate (in counts) for offset
      if (iCount == 0) {
        for (k = X; k <= Z; k++) {
          iOffset[k] = MagCal->BpFast[k][j];
        }
      }

      // store scaled and offset fBp[XYZ] in vecA[0-2] and fBp[XYZ]^2 in vecA[3-5]
      for (k = X; k <= Z; k++) {
        // MagCal->vecA[k] = (float)((int32_t)MagCal->BpFast[k][j] - (int32_t)iOffset[k]) * fscaling;
        MagCal->vecA[k] = MagCal->BpFast[k][j] - iOffset[k]; // float by default, likely don't need scaling...
        MagCal->vecA[k + 3] = MagCal->vecA[k] * MagCal->vecA[k];
      }

      // calculate fBp2 = Bp[X]^2 + Bp[Y]^2 + Bp[Z]^2 (scaled uT^2)
      fBp2 = MagCal->vecA[3] + MagCal->vecA[4] + MagCal->vecA[5];

      // accumulate fBp^4 over all measurements into fSumBp4=Y^T.Y
      fSumBp4 += fBp2 * fBp2;

      // now we have fBp2, accumulate vecB[0-2] = X^T.Y =sum(Bp2.Bp[XYZ])
      for (k = X; k <= Z; k++) {
        MagCal->vecB[k] += MagCal->vecA[k] * fBp2;
      }

      //accumulate vecB[3] = X^T.Y =sum(fBp2)
      MagCal->vecB[3] += fBp2;

      // accumulate on and above-diagonal terms of matA = X^T.X ignoring matA[3][3]
      MagCal->matA[0][0] += MagCal->vecA[X + 3];
      MagCal->matA[0][1] += MagCal->vecA[X] * MagCal->vecA[Y];
      MagCal->matA[0][2] += MagCal->vecA[X] * MagCal->vecA[Z];
      MagCal->matA[0][3] += MagCal->vecA[X];
      MagCal->matA[1][1] += MagCal->vecA[Y + 3];
      MagCal->matA[1][2] += MagCal->vecA[Y] * MagCal->vecA[Z];
      MagCal->matA[1][3] += MagCal->vecA[Y];
      MagCal->matA[2][2] += MagCal->vecA[Z + 3];
      MagCal->matA[2][3] += MagCal->vecA[Z];

      // increment the counter for next iteration
      iCount++;
    }
  }

  // set the last element of the measurement matrix to the number of buffer elements used
  MagCal->matA[3][3] = (float) iCount;

  // store the number of measurements accumulated
  MagCal->MagBufferCount = iCount;

  // use above diagonal elements of symmetric matA to set both matB and matA to X^T.X
  for (i = 0; i < 4; i++) {
    for (j = i; j < 4; j++) {
      MagCal->matB[i][j] = MagCal->matB[j][i]
        = MagCal->matA[j][i] = MagCal->matA[i][j];
    }
  }

  // calculate in situ inverse of matB = inv(X^T.X) (4x4) while matA still holds X^T.X
  for (i = 0; i < 4; i++) {
    pfRows[i] = MagCal->matB[i];
  }
  fmatrixAeqInvA(pfRows, iColInd, iRowInd, iPivot, 4);

  // calculate vecA = solution beta (4x1) = inv(X^T.X).X^T.Y = matB * vecB
  for (i = 0; i < 4; i++) {
    MagCal->vecA[i] = 0.0F;
    for (k = 0; k < 4; k++) {
      MagCal->vecA[i] += MagCal->matB[i][k] * MagCal->vecB[k];
    }
  }

  // calculate P = r^T.r = Y^T.Y - 2 * beta^T.(X^T.Y) + beta^T.(X^T.X).beta
  // = fSumBp4 - 2 * vecA^T.vecB + vecA^T.matA.vecA
  // first set P = Y^T.Y - 2 * beta^T.(X^T.Y) = SumBp4 - 2 * vecA^T.vecB
  fE = 0.0F;
  for (i = 0; i < 4; i++) {
    fE += MagCal->vecA[i] * MagCal->vecB[i];
  }
  fE = fSumBp4 - 2.0F * fE;

  // set vecB = (X^T.X).beta = matA.vecA
  for (i = 0; i < 4; i++) {
    MagCal->vecB[i] = 0.0F;
    for (k = 0; k < 4; k++) {
      MagCal->vecB[i] += MagCal->matA[i][k] * MagCal->vecA[k];
    }
  }

  // complete calculation of P by adding beta^T.(X^T.X).beta = vecA^T * vecB
  for (i = 0; i < 4; i++) {
    fE += MagCal->vecB[i] * MagCal->vecA[i];
  }

  // compute the hard iron vector (in uT but offset and scaled by FMATRIXSCALING)
  for (k = X; k <= Z; k++) {
    MagCal->trV[k] = 0.5F * MagCal->vecA[k];
  }

  // compute the scaled geomagnetic field strength B (in uT but scaled by FMATRIXSCALING)
  MagCal->trB = sqrtf(MagCal->vecA[3] + MagCal->trV[X] * MagCal->trV[X] +
      MagCal->trV[Y] * MagCal->trV[Y] + MagCal->trV[Z] * MagCal->trV[Z]);

  // calculate the trial fit error (percent) normalized to number of measurements
  // and scaled geomagnetic field strength
  MagCal->trFitErrorpc = sqrtf(fE / (float) MagCal->MagBufferCount) * 100.0F /
      (2.0F * MagCal->trB * MagCal->trB);

  // correct the hard iron estimate for FMATRIXSCALING and the offsets applied (result in uT)
  for (k = X; k <= Z; k++) {
    // MagCal->trV[k] = MagCal->trV[k] * DEFAULTB + (float)iOffset[k] * FXOS8700_UTPERCOUNT;
    MagCal->trV[k] = MagCal->trV[k] + iOffset[k];
  }

  // correct the geomagnetic field strength B to correct scaling (result in uT)
  // MagCal->trB *= DEFAULTB;
}

#endif
/*
 * TVC6DOF_sim.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "TVC6DOF_sim".
 *
 * Model version              : 1.173
 * Simulink Coder version : 9.4 (R2020b) 29-Jul-2020
 * C source code generated on : Mon Apr  6 21:18:10 2026
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objective: Debugging
 * Validation result: Not run
 */

#include "TVC6DOF_sim.h"
#include "TVC6DOF_sim_private.h"

/* Block signals (default storage) */
B_TVC6DOF_sim_T TVC6DOF_sim_B;

/* Continuous states */
X_TVC6DOF_sim_T TVC6DOF_sim_X;

/* Block states (default storage) */
DW_TVC6DOF_sim_T TVC6DOF_sim_DW;

/* Previous zero-crossings (trigger) states */
PrevZCX_TVC6DOF_sim_T TVC6DOF_sim_PrevZCX;

/* Real-time model */
static RT_MODEL_TVC6DOF_sim_T TVC6DOF_sim_M_;
RT_MODEL_TVC6DOF_sim_T *const TVC6DOF_sim_M = &TVC6DOF_sim_M_;

/* Forward declaration for local functions */
static void TVC6DOF_sim_cosd(real_T *x);
static void TVC6DOF_sim_sind(real_T *x);
static real_T TVC6DOF_sim_eml_rand_shr3cong(uint32_T state[2]);
static void TVC6DOF_sim_genrandu(uint32_T s, uint32_T *state, real_T *r);
static void TVC6DOF_s_genrand_uint32_vector(uint32_T mt[625], uint32_T u[2]);
static real_T TVC6DOF_sim_genrandu_f(uint32_T mt[625]);
static real_T TVC6DOF_sim_eml_rand_mt19937ar(uint32_T state[625]);
static void TVC6DOF_sim_randn(real_T r[3]);
static real_T TVC6DOF_sim_randn_l(void);
static void rate_scheduler(void);

/*
 *   This function updates active task flag for each subrate.
 * The function is called at model base rate, hence the
 * generated code self-manages all its subrates.
 */
static void rate_scheduler(void)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (TVC6DOF_sim_M->Timing.TaskCounters.TID[2])++;
  if ((TVC6DOF_sim_M->Timing.TaskCounters.TID[2]) > 7) {/* Sample time: [0.08s, 0.0s] */
    TVC6DOF_sim_M->Timing.TaskCounters.TID[2] = 0;
  }
}

/*
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = (ODE3_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 16;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  TVC6DOF_sim_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  TVC6DOF_sim_step();
  TVC6DOF_sim_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  TVC6DOF_sim_step();
  TVC6DOF_sim_derivatives();

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

real_T rt_urand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  uint32_T hi;
  uint32_T lo;

  /* Uniform random number generator (random number between 0 and 1)

     #define IA      16807                      magic multiplier = 7^5
     #define IM      2147483647                 modulus = 2^31-1
     #define IQ      127773                     IM div IA
     #define IR      2836                       IM modulo IA
     #define S       4.656612875245797e-10      reciprocal of 2^31-1
     test = IA * (seed % IQ) - IR * (seed/IQ)
     seed = test < 0 ? (test + IM) : test
     return (seed*S)
   */
  lo = *u % 127773U * 16807U;
  hi = *u / 127773U * 2836U;
  if (lo < hi) {
    *u = 2147483647U - (hi - lo);
  } else {
    *u = lo - hi;
  }

  return (real_T)*u * 4.6566128752457969E-10;
}

real_T rt_nrand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  real_T si;
  real_T sr;
  real_T y;

  /* Normal (Gaussian) random number generator */
  do {
    sr = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = sr * sr + si * si;
  } while (si > 1.0);

  y = sqrt(-2.0 * log(si) / si) * sr;
  return y;
}

real_T rt_remd_snf(real_T u0, real_T u1)
{
  real_T q;
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1) || rtIsInf(u0)) {
    y = (rtNaN);
  } else if (rtIsInf(u1)) {
    y = u0;
  } else if ((u1 != 0.0) && (u1 != trunc(u1))) {
    q = fabs(u0 / u1);
    if (!(fabs(q - floor(q + 0.5)) > DBL_EPSILON * q)) {
      y = 0.0 * u0;
    } else {
      y = fmod(u0, u1);
    }
  } else {
    y = fmod(u0, u1);
  }

  return y;
}

/* Function for MATLAB Function: '<S3>/Equations of Motion' */
static void TVC6DOF_sim_cosd(real_T *x)
{
  real_T absx;
  real_T b_x;
  int8_T n;
  if (rtIsInf(*x) || rtIsNaN(*x)) {
    *x = (rtNaN);
  } else {
    b_x = rt_remd_snf(*x, 360.0);
    absx = fabs(b_x);
    if (absx > 180.0) {
      if (b_x > 0.0) {
        b_x -= 360.0;
      } else {
        b_x += 360.0;
      }

      absx = fabs(b_x);
    }

    if (absx <= 45.0) {
      b_x *= 0.017453292519943295;
      *x = cos(b_x);
    } else {
      if (absx <= 135.0) {
        if (b_x > 0.0) {
          b_x = (b_x - 90.0) * 0.017453292519943295;
          n = 1;
        } else {
          b_x = (b_x + 90.0) * 0.017453292519943295;
          n = -1;
        }
      } else if (b_x > 0.0) {
        b_x = (b_x - 180.0) * 0.017453292519943295;
        n = 2;
      } else {
        b_x = (b_x + 180.0) * 0.017453292519943295;
        n = -2;
      }

      if (n == 1) {
        *x = -sin(b_x);
      } else if (n == -1) {
        *x = sin(b_x);
      } else {
        *x = -cos(b_x);
      }
    }
  }
}

/* Function for MATLAB Function: '<S3>/Equations of Motion' */
static void TVC6DOF_sim_sind(real_T *x)
{
  real_T absx;
  real_T c_x;
  int8_T n;
  if (rtIsInf(*x) || rtIsNaN(*x)) {
    *x = (rtNaN);
  } else {
    c_x = rt_remd_snf(*x, 360.0);
    absx = fabs(c_x);
    if (absx > 180.0) {
      if (c_x > 0.0) {
        c_x -= 360.0;
      } else {
        c_x += 360.0;
      }

      absx = fabs(c_x);
    }

    if (absx <= 45.0) {
      c_x *= 0.017453292519943295;
      *x = sin(c_x);
    } else {
      if (absx <= 135.0) {
        if (c_x > 0.0) {
          c_x = (c_x - 90.0) * 0.017453292519943295;
          n = 1;
        } else {
          c_x = (c_x + 90.0) * 0.017453292519943295;
          n = -1;
        }
      } else if (c_x > 0.0) {
        c_x = (c_x - 180.0) * 0.017453292519943295;
        n = 2;
      } else {
        c_x = (c_x + 180.0) * 0.017453292519943295;
        n = -2;
      }

      if (n == 1) {
        *x = cos(c_x);
      } else if (n == -1) {
        *x = -cos(c_x);
      } else {
        *x = -sin(c_x);
      }
    }
  }
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static real_T TVC6DOF_sim_eml_rand_shr3cong(uint32_T state[2])
{
  static const real_T b[65] = { 0.340945, 0.4573146, 0.5397793, 0.6062427,
    0.6631691, 0.7136975, 0.7596125, 0.8020356, 0.8417227, 0.8792102, 0.9148948,
    0.9490791, 0.9820005, 1.0138492, 1.044781, 1.0749254, 1.1043917, 1.1332738,
    1.161653, 1.189601, 1.2171815, 1.2444516, 1.2714635, 1.298265, 1.3249008,
    1.3514125, 1.3778399, 1.4042211, 1.4305929, 1.4569915, 1.4834527, 1.5100122,
    1.5367061, 1.5635712, 1.5906454, 1.617968, 1.6455802, 1.6735255, 1.7018503,
    1.7306045, 1.7598422, 1.7896223, 1.8200099, 1.851077, 1.8829044, 1.9155831,
    1.9492166, 1.9839239, 2.0198431, 2.0571356, 2.095993, 2.136645, 2.1793713,
    2.2245175, 2.2725186, 2.3239338, 2.3795008, 2.4402218, 2.5075117, 2.5834658,
    2.6713916, 2.7769942, 2.7769942, 2.7769942, 2.7769942 };

  real_T r;
  real_T s;
  real_T x;
  real_T y;
  int32_T j;
  uint32_T icng;
  uint32_T jsr;
  uint32_T ui;
  icng = 69069U * state[0] + 1234567U;
  jsr = state[1] << 13 ^ state[1];
  jsr ^= jsr >> 17;
  jsr ^= jsr << 5;
  ui = icng + jsr;
  j = (int32_T)((ui & 63U) + 1U);
  r = (real_T)(int32_T)ui * 4.6566128730773926E-10 * b[j];
  if (!(fabs(r) <= b[j - 1])) {
    x = (fabs(r) - b[j - 1]) / (b[j] - b[j - 1]);
    icng = 69069U * icng + 1234567U;
    jsr ^= jsr << 13;
    jsr ^= jsr >> 17;
    jsr ^= jsr << 5;
    y = (real_T)(int32_T)(icng + jsr) * 2.328306436538696E-10 + 0.5;
    s = x + y;
    if (s > 1.301198) {
      if (r < 0.0) {
        r = 0.4878992 * x - 0.4878992;
      } else {
        r = 0.4878992 - 0.4878992 * x;
      }
    } else {
      if (!(s <= 0.9689279)) {
        x = 0.4878992 - 0.4878992 * x;
        if (y > 12.67706 - exp(-0.5 * x * x) * 12.37586) {
          if (r < 0.0) {
            r = -x;
          } else {
            r = x;
          }
        } else {
          if (!(exp(-0.5 * b[j] * b[j]) + y * 0.01958303 / b[j] <= exp(-0.5 * r *
                r))) {
            do {
              icng = 69069U * icng + 1234567U;
              jsr ^= jsr << 13;
              jsr ^= jsr >> 17;
              jsr ^= jsr << 5;
              x = log((real_T)(int32_T)(icng + jsr) * 2.328306436538696E-10 +
                      0.5) / 2.776994;
              icng = 69069U * icng + 1234567U;
              jsr ^= jsr << 13;
              jsr ^= jsr >> 17;
              jsr ^= jsr << 5;
            } while (!(log((real_T)(int32_T)(icng + jsr) * 2.328306436538696E-10
                           + 0.5) * -2.0 > x * x));

            if (r < 0.0) {
              r = x - 2.776994;
            } else {
              r = 2.776994 - x;
            }
          }
        }
      }
    }
  }

  state[0] = icng;
  state[1] = jsr;
  return r;
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static void TVC6DOF_sim_genrandu(uint32_T s, uint32_T *state, real_T *r)
{
  int32_T hi;
  uint32_T test1;
  uint32_T test2;
  hi = (int32_T)(s / 127773U);
  test1 = (s - hi * 127773U) * 16807U;
  test2 = 2836U * hi;
  if (test1 < test2) {
    *state = ~(test2 - test1) & 2147483647U;
  } else {
    *state = test1 - test2;
  }

  *r = (real_T)*state * 4.6566128752457969E-10;
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static void TVC6DOF_s_genrand_uint32_vector(uint32_T mt[625], uint32_T u[2])
{
  int32_T j;
  int32_T kk;
  uint32_T mti;
  uint32_T y;
  for (j = 0; j < 2; j++) {
    mti = mt[624] + 1U;
    if (mti >= 625U) {
      for (kk = 0; kk < 227; kk++) {
        y = (mt[kk + 1] & 2147483647U) | (mt[kk] & 2147483648U);
        if ((y & 1U) == 0U) {
          mti = y >> 1U;
        } else {
          mti = y >> 1U ^ 2567483615U;
        }

        mt[kk] = mt[kk + 397] ^ mti;
      }

      for (kk = 0; kk < 396; kk++) {
        y = (mt[kk + 227] & 2147483648U) | (mt[kk + 228] & 2147483647U);
        if ((y & 1U) == 0U) {
          mti = y >> 1U;
        } else {
          mti = y >> 1U ^ 2567483615U;
        }

        mt[kk + 227] = mt[kk] ^ mti;
      }

      y = (mt[623] & 2147483648U) | (mt[0] & 2147483647U);
      if ((y & 1U) == 0U) {
        mti = y >> 1U;
      } else {
        mti = y >> 1U ^ 2567483615U;
      }

      mt[623] = mt[396] ^ mti;
      mti = 1U;
    }

    y = mt[(int32_T)mti - 1];
    mt[624] = mti;
    y ^= y >> 11U;
    y ^= y << 7U & 2636928640U;
    y ^= y << 15U & 4022730752U;
    y ^= y >> 18U;
    u[j] = y;
  }
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static real_T TVC6DOF_sim_genrandu_f(uint32_T mt[625])
{
  real_T r;
  int32_T exitg1;
  int32_T k;
  uint32_T u[2];
  uint32_T b_r;
  boolean_T b_isvalid;
  boolean_T exitg2;

  /* ========================= COPYRIGHT NOTICE ============================ */
  /*  This is a uniform (0,1) pseudorandom number generator based on:        */
  /*                                                                         */
  /*  A C-program for MT19937, with initialization improved 2002/1/26.       */
  /*  Coded by Takuji Nishimura and Makoto Matsumoto.                        */
  /*                                                                         */
  /*  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      */
  /*  All rights reserved.                                                   */
  /*                                                                         */
  /*  Redistribution and use in source and binary forms, with or without     */
  /*  modification, are permitted provided that the following conditions     */
  /*  are met:                                                               */
  /*                                                                         */
  /*    1. Redistributions of source code must retain the above copyright    */
  /*       notice, this list of conditions and the following disclaimer.     */
  /*                                                                         */
  /*    2. Redistributions in binary form must reproduce the above copyright */
  /*       notice, this list of conditions and the following disclaimer      */
  /*       in the documentation and/or other materials provided with the     */
  /*       distribution.                                                     */
  /*                                                                         */
  /*    3. The names of its contributors may not be used to endorse or       */
  /*       promote products derived from this software without specific      */
  /*       prior written permission.                                         */
  /*                                                                         */
  /*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    */
  /*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      */
  /*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  */
  /*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  */
  /*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  */
  /*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       */
  /*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  */
  /*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  */
  /*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    */
  /*  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE */
  /*  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  */
  /*                                                                         */
  /* =============================   END   ================================= */
  do {
    exitg1 = 0;
    TVC6DOF_s_genrand_uint32_vector(mt, u);
    r = ((real_T)(u[0] >> 5U) * 6.7108864E+7 + (real_T)(u[1] >> 6U)) *
      1.1102230246251565E-16;
    if (r == 0.0) {
      if ((mt[624] >= 1U) && (mt[624] < 625U)) {
        b_isvalid = false;
        k = 1;
        exitg2 = false;
        while ((!exitg2) && (k < 625)) {
          if (mt[k - 1] == 0U) {
            k++;
          } else {
            b_isvalid = true;
            exitg2 = true;
          }
        }
      } else {
        b_isvalid = false;
      }

      if (!b_isvalid) {
        b_r = 5489U;
        mt[0] = 5489U;
        for (k = 0; k < 623; k++) {
          b_r = ((b_r >> 30U ^ b_r) * 1812433253U + k) + 1U;
          mt[k + 1] = b_r;
        }

        mt[624] = 624U;
      }
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return r;
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static real_T TVC6DOF_sim_eml_rand_mt19937ar(uint32_T state[625])
{
  static const real_T b[257] = { 0.0, 0.215241895984875, 0.286174591792068,
    0.335737519214422, 0.375121332878378, 0.408389134611989, 0.43751840220787,
    0.46363433679088, 0.487443966139235, 0.50942332960209, 0.529909720661557,
    0.549151702327164, 0.567338257053817, 0.584616766106378, 0.601104617755991,
    0.61689699000775, 0.63207223638606, 0.646695714894993, 0.660822574244419,
    0.674499822837293, 0.687767892795788, 0.700661841106814, 0.713212285190975,
    0.725446140909999, 0.737387211434295, 0.749056662017815, 0.760473406430107,
    0.771654424224568, 0.782615023307232, 0.793369058840623, 0.80392911698997,
    0.814306670135215, 0.824512208752291, 0.834555354086381, 0.844444954909153,
    0.854189171008163, 0.863795545553308, 0.87327106808886, 0.882622229585165,
    0.891855070732941, 0.900975224461221, 0.909987953496718, 0.91889818364959,
    0.927710533401999, 0.936429340286575, 0.945058684468165, 0.953602409881086,
    0.96206414322304, 0.970447311064224, 0.978755155294224, 0.986990747099062,
    0.99515699963509, 1.00325667954467, 1.01129241744, 1.01926671746548,
    1.02718196603564, 1.03504043983344, 1.04284431314415, 1.05059566459093,
    1.05829648333067, 1.06594867476212, 1.07355406579244, 1.0811144097034,
    1.08863139065398, 1.09610662785202, 1.10354167942464, 1.11093804601357,
    1.11829717411934, 1.12562045921553, 1.13290924865253, 1.14016484436815,
    1.14738850542085, 1.15458145035993, 1.16174485944561, 1.16887987673083,
    1.17598761201545, 1.18306914268269, 1.19012551542669, 1.19715774787944,
    1.20416683014438, 1.2111537262437, 1.21811937548548, 1.22506469375653,
    1.23199057474614, 1.23889789110569, 1.24578749554863, 1.2526602218949,
    1.25951688606371, 1.26635828701823, 1.27318520766536, 1.27999841571382,
    1.28679866449324, 1.29358669373695, 1.30036323033084, 1.30712898903073,
    1.31388467315022, 1.32063097522106, 1.32736857762793, 1.33409815321936,
    1.3408203658964, 1.34753587118059, 1.35424531676263, 1.36094934303328,
    1.36764858359748, 1.37434366577317, 1.38103521107586, 1.38772383568998,
    1.39441015092814, 1.40109476367925, 1.4077782768464, 1.41446128977547,
    1.42114439867531, 1.42782819703026, 1.43451327600589, 1.44120022484872,
    1.44788963128058, 1.45458208188841, 1.46127816251028, 1.46797845861808,
    1.47468355569786, 1.48139403962819, 1.48811049705745, 1.49483351578049,
    1.50156368511546, 1.50830159628131, 1.51504784277671, 1.521803020761,
    1.52856772943771, 1.53534257144151, 1.542128153229, 1.54892508547417,
    1.55573398346918, 1.56255546753104, 1.56939016341512, 1.57623870273591,
    1.58310172339603, 1.58997987002419, 1.59687379442279, 1.60378415602609,
    1.61071162236983, 1.61765686957301, 1.62462058283303, 1.63160345693487,
    1.63860619677555, 1.64562951790478, 1.65267414708306, 1.65974082285818,
    1.66683029616166, 1.67394333092612, 1.68108070472517, 1.68824320943719,
    1.69543165193456, 1.70264685479992, 1.7098896570713, 1.71716091501782,
    1.72446150294804, 1.73179231405296, 1.73915426128591, 1.74654827828172,
    1.75397532031767, 1.76143636531891, 1.76893241491127, 1.77646449552452,
    1.78403365954944, 1.79164098655216, 1.79928758454972, 1.80697459135082,
    1.81470317596628, 1.82247454009388, 1.83028991968276, 1.83815058658281,
    1.84605785028518, 1.8540130597602, 1.86201760539967, 1.87007292107127,
    1.878180486293, 1.88634182853678, 1.8945585256707, 1.90283220855043,
    1.91116456377125, 1.91955733659319, 1.92801233405266, 1.93653142827569,
    1.94511656000868, 1.95376974238465, 1.96249306494436, 1.97128869793366,
    1.98015889690048, 1.98910600761744, 1.99813247135842, 2.00724083056053,
    2.0164337349062, 2.02571394786385, 2.03508435372962, 2.04454796521753,
    2.05410793165065, 2.06376754781173, 2.07353026351874, 2.0833996939983,
    2.09337963113879, 2.10347405571488, 2.11368715068665, 2.12402331568952,
    2.13448718284602, 2.14508363404789, 2.15581781987674, 2.16669518035431,
    2.17772146774029, 2.18890277162636, 2.20024554661128, 2.21175664288416,
    2.22344334009251, 2.23531338492992, 2.24737503294739, 2.25963709517379,
    2.27210899022838, 2.28480080272449, 2.29772334890286, 2.31088825060137,
    2.32430801887113, 2.33799614879653, 2.35196722737914, 2.36623705671729,
    2.38082279517208, 2.39574311978193, 2.41101841390112, 2.42667098493715,
    2.44272531820036, 2.4592083743347, 2.47614993967052, 2.49358304127105,
    2.51154444162669, 2.53007523215985, 2.54922155032478, 2.56903545268184,
    2.58957598670829, 2.61091051848882, 2.63311639363158, 2.65628303757674,
    2.68051464328574, 2.70593365612306, 2.73268535904401, 2.76094400527999,
    2.79092117400193, 2.82287739682644, 2.85713873087322, 2.89412105361341,
    2.93436686720889, 2.97860327988184, 3.02783779176959, 3.08352613200214,
    3.147889289518, 3.2245750520478, 3.32024473383983, 3.44927829856143,
    3.65415288536101, 3.91075795952492 };

  static const real_T c[257] = { 1.0, 0.977101701267673, 0.959879091800108,
    0.9451989534423, 0.932060075959231, 0.919991505039348, 0.908726440052131,
    0.898095921898344, 0.887984660755834, 0.878309655808918, 0.869008688036857,
    0.860033621196332, 0.851346258458678, 0.842915653112205, 0.834716292986884,
    0.826726833946222, 0.818929191603703, 0.811307874312656, 0.803849483170964,
    0.796542330422959, 0.789376143566025, 0.782341832654803, 0.775431304981187,
    0.768637315798486, 0.761953346836795, 0.755373506507096, 0.748892447219157,
    0.742505296340151, 0.736207598126863, 0.729995264561476, 0.72386453346863,
    0.717811932630722, 0.711834248878248, 0.705928501332754, 0.700091918136512,
    0.694321916126117, 0.688616083004672, 0.682972161644995, 0.677388036218774,
    0.671861719897082, 0.66639134390875, 0.660975147776663, 0.655611470579697,
    0.650298743110817, 0.645035480820822, 0.639820277453057, 0.634651799287624,
    0.629528779924837, 0.624450015547027, 0.619414360605834, 0.614420723888914,
    0.609468064925773, 0.604555390697468, 0.599681752619125, 0.594846243767987,
    0.590047996332826, 0.585286179263371, 0.580559996100791, 0.575868682972354,
    0.571211506735253, 0.566587763256165, 0.561996775814525, 0.557437893618766,
    0.552910490425833, 0.548413963255266, 0.543947731190026, 0.539511234256952,
    0.535103932380458, 0.530725304403662, 0.526374847171684, 0.522052074672322,
    0.517756517229756, 0.513487720747327, 0.509245245995748, 0.505028667943468,
    0.500837575126149, 0.49667156905249, 0.492530263643869, 0.488413284705458,
    0.484320269426683, 0.480250865909047, 0.476204732719506, 0.47218153846773,
    0.468180961405694, 0.464202689048174, 0.460246417812843, 0.456311852678716,
    0.452398706861849, 0.448506701507203, 0.444635565395739, 0.440785034665804,
    0.436954852547985, 0.433144769112652, 0.429354541029442, 0.425583931338022,
    0.421832709229496, 0.418100649837848, 0.414387534040891, 0.410693148270188,
    0.407017284329473, 0.403359739221114, 0.399720314980197, 0.396098818515832,
    0.392495061459315, 0.388908860018789, 0.385340034840077, 0.381788410873393,
    0.378253817245619, 0.374736087137891, 0.371235057668239, 0.367750569779032,
    0.364282468129004, 0.360830600989648, 0.357394820145781, 0.353974980800077,
    0.350570941481406, 0.347182563956794, 0.343809713146851, 0.340452257044522,
    0.337110066637006, 0.333783015830718, 0.330470981379163, 0.327173842813601,
    0.323891482376391, 0.320623784956905, 0.317370638029914, 0.314131931596337,
    0.310907558126286, 0.307697412504292, 0.30450139197665, 0.301319396100803,
    0.298151326696685, 0.294997087799962, 0.291856585617095, 0.288729728482183,
    0.285616426815502, 0.282516593083708, 0.279430141761638, 0.276356989295668,
    0.273297054068577, 0.270250256365875, 0.267216518343561, 0.264195763997261,
    0.261187919132721, 0.258192911337619, 0.255210669954662, 0.252241126055942,
    0.249284212418529, 0.246339863501264, 0.24340801542275, 0.240488605940501,
    0.237581574431238, 0.23468686187233, 0.231804410824339, 0.228934165414681,
    0.226076071322381, 0.223230075763918, 0.220396127480152, 0.217574176724331,
    0.214764175251174, 0.211966076307031, 0.209179834621125, 0.206405406397881,
    0.203642749310335, 0.200891822494657, 0.198152586545776, 0.195425003514135,
    0.192709036903589, 0.190004651670465, 0.187311814223801, 0.1846304924268,
    0.181960655599523, 0.179302274522848, 0.176655321443735, 0.174019770081839,
    0.171395595637506, 0.168782774801212, 0.166181285764482, 0.163591108232366,
    0.161012223437511, 0.158444614155925, 0.15588826472448, 0.153343161060263,
    0.150809290681846, 0.148286642732575, 0.145775208005994, 0.143274978973514,
    0.140785949814445, 0.138308116448551, 0.135841476571254, 0.133386029691669,
    0.130941777173644, 0.12850872228, 0.126086870220186, 0.123676228201597,
    0.12127680548479, 0.11888861344291, 0.116511665625611, 0.114145977827839,
    0.111791568163838, 0.109448457146812, 0.107116667774684, 0.104796225622487,
    0.102487158941935, 0.10018949876881, 0.0979032790388625, 0.095628536713009,
    0.093365311912691, 0.0911136480663738, 0.0888735920682759,
    0.0866451944505581, 0.0844285095703535, 0.082223595813203,
    0.0800305158146631, 0.0778493367020961, 0.0756801303589272,
    0.0735229737139814, 0.0713779490588905, 0.0692451443970068,
    0.0671246538277886, 0.065016577971243, 0.0629210244377582, 0.06083810834954,
    0.0587679529209339, 0.0567106901062031, 0.0546664613248891,
    0.0526354182767924, 0.0506177238609479, 0.0486135532158687,
    0.0466230949019305, 0.0446465522512946, 0.0426841449164746,
    0.0407361106559411, 0.0388027074045262, 0.0368842156885674,
    0.0349809414617162, 0.0330932194585786, 0.0312214171919203,
    0.0293659397581334, 0.0275272356696031, 0.0257058040085489,
    0.0239022033057959, 0.0221170627073089, 0.0203510962300445,
    0.0186051212757247, 0.0168800831525432, 0.0151770883079353,
    0.0134974506017399, 0.0118427578579079, 0.0102149714397015,
    0.00861658276939875, 0.00705087547137324, 0.00552240329925101,
    0.00403797259336304, 0.00260907274610216, 0.0012602859304986,
    0.000477467764609386 };

  real_T c_u;
  real_T r;
  real_T x;
  int32_T exitg1;
  int32_T i;
  uint32_T u32[2];
  do {
    exitg1 = 0;
    TVC6DOF_s_genrand_uint32_vector(state, u32);
    i = (int32_T)((u32[1] >> 24U) + 1U);
    r = (((real_T)(u32[0] >> 3U) * 1.6777216E+7 + (real_T)((int32_T)u32[1] &
           16777215)) * 2.2204460492503131E-16 - 1.0) * b[i];
    if (fabs(r) <= b[i - 1]) {
      exitg1 = 1;
    } else if (i < 256) {
      x = TVC6DOF_sim_genrandu_f(state);
      if ((c[i - 1] - c[i]) * x + c[i] < exp(-0.5 * r * r)) {
        exitg1 = 1;
      }
    } else {
      do {
        x = TVC6DOF_sim_genrandu_f(state);
        x = log(x) * 0.273661237329758;
        c_u = TVC6DOF_sim_genrandu_f(state);
      } while (!(-2.0 * log(c_u) > x * x));

      if (r < 0.0) {
        r = x - 3.65415288536101;
      } else {
        r = 3.65415288536101 - x;
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return r;
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static void TVC6DOF_sim_randn(real_T r[3])
{
  real_T c_r;
  real_T t;
  int32_T b_k;
  uint32_T c_state;
  uint32_T e_r;
  if (!TVC6DOF_sim_DW.method_not_empty) {
    TVC6DOF_sim_DW.method = 0U;
    TVC6DOF_sim_DW.method_not_empty = true;
    TVC6DOF_sim_DW.state[0] = 362436069U;
    TVC6DOF_sim_DW.state[1] = 0U;
    if (TVC6DOF_sim_DW.state[1] == 0U) {
      TVC6DOF_sim_DW.state[1] = 521288629U;
    }
  }

  if (TVC6DOF_sim_DW.method == 0U) {
    if (TVC6DOF_sim_DW.method_i == 4U) {
      c_state = TVC6DOF_sim_DW.state_p;
      do {
        TVC6DOF_sim_genrandu(c_state, &e_r, &c_r);
        TVC6DOF_sim_genrandu(e_r, &c_state, &t);
        c_r = 2.0 * c_r - 1.0;
        t = 2.0 * t - 1.0;
        t = t * t + c_r * c_r;
      } while (!(t <= 1.0));

      c_r *= sqrt(-2.0 * log(t) / t);
      TVC6DOF_sim_DW.state_p = c_state;
      r[0] = c_r;
      c_state = TVC6DOF_sim_DW.state_p;
      do {
        TVC6DOF_sim_genrandu(c_state, &e_r, &c_r);
        TVC6DOF_sim_genrandu(e_r, &c_state, &t);
        c_r = 2.0 * c_r - 1.0;
        t = 2.0 * t - 1.0;
        t = t * t + c_r * c_r;
      } while (!(t <= 1.0));

      c_r *= sqrt(-2.0 * log(t) / t);
      TVC6DOF_sim_DW.state_p = c_state;
      r[1] = c_r;
      c_state = TVC6DOF_sim_DW.state_p;
      do {
        TVC6DOF_sim_genrandu(c_state, &e_r, &c_r);
        TVC6DOF_sim_genrandu(e_r, &c_state, &t);
        c_r = 2.0 * c_r - 1.0;
        t = 2.0 * t - 1.0;
        t = t * t + c_r * c_r;
      } while (!(t <= 1.0));

      c_r *= sqrt(-2.0 * log(t) / t);
      TVC6DOF_sim_DW.state_p = c_state;
      r[2] = c_r;
    } else if (TVC6DOF_sim_DW.method_i == 5U) {
      t = TVC6DOF_sim_eml_rand_shr3cong(TVC6DOF_sim_DW.state_f);
      r[0] = t;
      t = TVC6DOF_sim_eml_rand_shr3cong(TVC6DOF_sim_DW.state_f);
      r[1] = t;
      t = TVC6DOF_sim_eml_rand_shr3cong(TVC6DOF_sim_DW.state_f);
      r[2] = t;
    } else {
      if (!TVC6DOF_sim_DW.state_not_empty) {
        memset(&TVC6DOF_sim_DW.state_d[0], 0, 625U * sizeof(uint32_T));
        e_r = 5489U;
        TVC6DOF_sim_DW.state_d[0] = 5489U;
        for (b_k = 0; b_k < 623; b_k++) {
          e_r = ((e_r >> 30U ^ e_r) * 1812433253U + b_k) + 1U;
          TVC6DOF_sim_DW.state_d[b_k + 1] = e_r;
        }

        TVC6DOF_sim_DW.state_d[624] = 624U;
        TVC6DOF_sim_DW.state_not_empty = true;
      }

      t = TVC6DOF_sim_eml_rand_mt19937ar(TVC6DOF_sim_DW.state_d);
      r[0] = t;
      t = TVC6DOF_sim_eml_rand_mt19937ar(TVC6DOF_sim_DW.state_d);
      r[1] = t;
      t = TVC6DOF_sim_eml_rand_mt19937ar(TVC6DOF_sim_DW.state_d);
      r[2] = t;
    }
  } else if (TVC6DOF_sim_DW.method == 4U) {
    c_state = TVC6DOF_sim_DW.state[0];
    do {
      TVC6DOF_sim_genrandu(c_state, &e_r, &c_r);
      TVC6DOF_sim_genrandu(e_r, &c_state, &t);
      c_r = 2.0 * c_r - 1.0;
      t = 2.0 * t - 1.0;
      t = t * t + c_r * c_r;
    } while (!(t <= 1.0));

    c_r *= sqrt(-2.0 * log(t) / t);
    TVC6DOF_sim_DW.state[0] = c_state;
    r[0] = c_r;
    c_state = TVC6DOF_sim_DW.state[0];
    do {
      TVC6DOF_sim_genrandu(c_state, &e_r, &c_r);
      TVC6DOF_sim_genrandu(e_r, &c_state, &t);
      c_r = 2.0 * c_r - 1.0;
      t = 2.0 * t - 1.0;
      t = t * t + c_r * c_r;
    } while (!(t <= 1.0));

    c_r *= sqrt(-2.0 * log(t) / t);
    TVC6DOF_sim_DW.state[0] = c_state;
    r[1] = c_r;
    c_state = TVC6DOF_sim_DW.state[0];
    do {
      TVC6DOF_sim_genrandu(c_state, &e_r, &c_r);
      TVC6DOF_sim_genrandu(e_r, &c_state, &t);
      c_r = 2.0 * c_r - 1.0;
      t = 2.0 * t - 1.0;
      t = t * t + c_r * c_r;
    } while (!(t <= 1.0));

    c_r *= sqrt(-2.0 * log(t) / t);
    TVC6DOF_sim_DW.state[0] = c_state;
    r[2] = c_r;
  } else {
    t = TVC6DOF_sim_eml_rand_shr3cong(TVC6DOF_sim_DW.state);
    r[0] = t;
    t = TVC6DOF_sim_eml_rand_shr3cong(TVC6DOF_sim_DW.state);
    r[1] = t;
    t = TVC6DOF_sim_eml_rand_shr3cong(TVC6DOF_sim_DW.state);
    r[2] = t;
  }
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static real_T TVC6DOF_sim_randn_l(void)
{
  real_T c_r;
  real_T r;
  real_T t;
  int32_T mti;
  uint32_T c_state;
  uint32_T f_r;
  if (!TVC6DOF_sim_DW.method_not_empty) {
    TVC6DOF_sim_DW.method = 0U;
    TVC6DOF_sim_DW.method_not_empty = true;
    TVC6DOF_sim_DW.state[0] = 362436069U;
    TVC6DOF_sim_DW.state[1] = 0U;
    if (TVC6DOF_sim_DW.state[1] == 0U) {
      TVC6DOF_sim_DW.state[1] = 521288629U;
    }
  }

  if (TVC6DOF_sim_DW.method == 0U) {
    if (TVC6DOF_sim_DW.method_i == 4U) {
      c_state = TVC6DOF_sim_DW.state_p;
      do {
        TVC6DOF_sim_genrandu(c_state, &f_r, &c_r);
        TVC6DOF_sim_genrandu(f_r, &c_state, &t);
        r = 2.0 * c_r - 1.0;
        t = 2.0 * t - 1.0;
        t = t * t + r * r;
      } while (!(t <= 1.0));

      r *= sqrt(-2.0 * log(t) / t);
      TVC6DOF_sim_DW.state_p = c_state;
    } else if (TVC6DOF_sim_DW.method_i == 5U) {
      r = TVC6DOF_sim_eml_rand_shr3cong(TVC6DOF_sim_DW.state_f);
    } else {
      if (!TVC6DOF_sim_DW.state_not_empty) {
        memset(&TVC6DOF_sim_DW.state_d[0], 0, 625U * sizeof(uint32_T));
        f_r = 5489U;
        TVC6DOF_sim_DW.state_d[0] = 5489U;
        for (mti = 0; mti < 623; mti++) {
          f_r = ((f_r >> 30U ^ f_r) * 1812433253U + mti) + 1U;
          TVC6DOF_sim_DW.state_d[mti + 1] = f_r;
        }

        TVC6DOF_sim_DW.state_d[624] = 624U;
        TVC6DOF_sim_DW.state_not_empty = true;
      }

      r = TVC6DOF_sim_eml_rand_mt19937ar(TVC6DOF_sim_DW.state_d);
    }
  } else if (TVC6DOF_sim_DW.method == 4U) {
    c_state = TVC6DOF_sim_DW.state[0];
    do {
      TVC6DOF_sim_genrandu(c_state, &f_r, &c_r);
      TVC6DOF_sim_genrandu(f_r, &c_state, &t);
      r = 2.0 * c_r - 1.0;
      t = 2.0 * t - 1.0;
      t = t * t + r * r;
    } while (!(t <= 1.0));

    r *= sqrt(-2.0 * log(t) / t);
    TVC6DOF_sim_DW.state[0] = c_state;
  } else {
    r = TVC6DOF_sim_eml_rand_shr3cong(TVC6DOF_sim_DW.state);
  }

  return r;
}

/* Model step function */
void TVC6DOF_sim_step(void)
{
  real_T h_0[9];
  real_T scale_0[9];
  real_T scale_1[9];
  real_T q_rot[4];
  real_T r[3];
  real_T tmp[3];
  real_T absxk;
  real_T g;
  real_T h;
  real_T i;
  real_T j;
  real_T k;
  real_T m;
  real_T scale;
  real_T t;
  int32_T i_0;
  int32_T i_1;
  ZCEventType zcEvent;
  if (rtmIsMajorTimeStep(TVC6DOF_sim_M)) {
    /* set solver stop time */
    if (!(TVC6DOF_sim_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&TVC6DOF_sim_M->solverInfo,
                            ((TVC6DOF_sim_M->Timing.clockTickH0 + 1) *
        TVC6DOF_sim_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&TVC6DOF_sim_M->solverInfo,
                            ((TVC6DOF_sim_M->Timing.clockTick0 + 1) *
        TVC6DOF_sim_M->Timing.stepSize0 + TVC6DOF_sim_M->Timing.clockTickH0 *
        TVC6DOF_sim_M->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(TVC6DOF_sim_M)) {
    TVC6DOF_sim_M->Timing.t[0] = rtsiGetT(&TVC6DOF_sim_M->solverInfo);
  }

  /* Integrator: '<S3>/Integrator1' */
  TVC6DOF_sim_B.Integrator1[0] = TVC6DOF_sim_X.Integrator1_CSTATE[0];
  TVC6DOF_sim_B.Integrator1[1] = TVC6DOF_sim_X.Integrator1_CSTATE[1];
  TVC6DOF_sim_B.Integrator1[2] = TVC6DOF_sim_X.Integrator1_CSTATE[2];
  if (rtmIsMajorTimeStep(TVC6DOF_sim_M) &&
      TVC6DOF_sim_M->Timing.TaskCounters.TID[1] == 0) {
    /* Scope: '<Root>/Scope1' */
    if (rtmIsMajorTimeStep(TVC6DOF_sim_M)) {
      StructLogVar *svar = (StructLogVar *)
        TVC6DOF_sim_DW.Scope1_PWORK.LoggedData;
      LogVar *var = svar->signals.values;

      /* signals */
      {
        real_T up0[3];
        up0[0] = TVC6DOF_sim_B.Integrator1[0];
        up0[1] = TVC6DOF_sim_B.Integrator1[1];
        up0[2] = TVC6DOF_sim_B.Integrator1[2];
        rt_UpdateLogVar((LogVar *)var, up0, 0);
      }
    }
  }

  /* Integrator: '<S3>/Integrator' */
  TVC6DOF_sim_B.Integrator[0] = TVC6DOF_sim_X.Integrator_CSTATE[0];
  TVC6DOF_sim_B.Integrator[1] = TVC6DOF_sim_X.Integrator_CSTATE[1];
  TVC6DOF_sim_B.Integrator[2] = TVC6DOF_sim_X.Integrator_CSTATE[2];
  if (rtmIsMajorTimeStep(TVC6DOF_sim_M) &&
      TVC6DOF_sim_M->Timing.TaskCounters.TID[1] == 0) {
    /* Scope: '<Root>/Scope2' */
    if (rtmIsMajorTimeStep(TVC6DOF_sim_M)) {
      StructLogVar *svar = (StructLogVar *)
        TVC6DOF_sim_DW.Scope2_PWORK.LoggedData;
      LogVar *var = svar->signals.values;

      /* signals */
      {
        real_T up0[3];
        up0[0] = TVC6DOF_sim_B.Integrator[0];
        up0[1] = TVC6DOF_sim_B.Integrator[1];
        up0[2] = TVC6DOF_sim_B.Integrator[2];
        rt_UpdateLogVar((LogVar *)var, up0, 0);
      }
    }
  }

  /* FromWorkspace: '<S9>/FromWs' */
  {
    real_T *pDataValues = (real_T *) TVC6DOF_sim_DW.FromWs_PWORK.DataPtr;
    real_T *pTimeValues = (real_T *) TVC6DOF_sim_DW.FromWs_PWORK.TimePtr;
    int_T currTimeIndex = TVC6DOF_sim_DW.FromWs_IWORK.PrevIndex;
    real_T t = TVC6DOF_sim_M->Timing.t[0];

    /* Get index */
    if (t <= pTimeValues[0]) {
      currTimeIndex = 0;
    } else if (t >= pTimeValues[50]) {
      currTimeIndex = 49;
    } else {
      if (t < pTimeValues[currTimeIndex]) {
        while (t < pTimeValues[currTimeIndex]) {
          currTimeIndex--;
        }
      } else {
        while (t >= pTimeValues[currTimeIndex + 1]) {
          currTimeIndex++;
        }
      }
    }

    TVC6DOF_sim_DW.FromWs_IWORK.PrevIndex = currTimeIndex;

    /* Post output */
    {
      real_T t1 = pTimeValues[currTimeIndex];
      real_T t2 = pTimeValues[currTimeIndex + 1];
      if (t1 == t2) {
        if (t < t1) {
          TVC6DOF_sim_B.FromWs = pDataValues[currTimeIndex];
        } else {
          TVC6DOF_sim_B.FromWs = pDataValues[currTimeIndex + 1];
        }
      } else {
        real_T f1 = (t2 - t) / (t2 - t1);
        real_T f2 = 1.0 - f1;
        real_T d1;
        real_T d2;
        int_T TimeIndex= currTimeIndex;
        d1 = pDataValues[TimeIndex];
        d2 = pDataValues[TimeIndex + 1];
        TVC6DOF_sim_B.FromWs = (real_T) rtInterpolate(d1, d2, f1, f2);
        pDataValues += 51;
      }
    }
  }

  if (rtmIsMajorTimeStep(TVC6DOF_sim_M) &&
      TVC6DOF_sim_M->Timing.TaskCounters.TID[2] == 0) {
    /* Delay: '<Root>/Actuator Delay' */
    TVC6DOF_sim_B.ActuatorDelay = TVC6DOF_sim_DW.ActuatorDelay_DSTATE;

    /* Saturate: '<Root>/Saturation' */
    m = TVC6DOF_sim_B.ActuatorDelay;
    scale = TVC6DOF_sim_P.Saturation_LowerSat;
    absxk = TVC6DOF_sim_P.Saturation_UpperSat;
    if (m > absxk) {
      /* Saturate: '<Root>/Saturation' */
      TVC6DOF_sim_B.Saturation = absxk;
    } else if (m < scale) {
      /* Saturate: '<Root>/Saturation' */
      TVC6DOF_sim_B.Saturation = scale;
    } else {
      /* Saturate: '<Root>/Saturation' */
      TVC6DOF_sim_B.Saturation = m;
    }

    /* End of Saturate: '<Root>/Saturation' */

    /* Delay: '<Root>/Actuator Delay2' */
    TVC6DOF_sim_B.ActuatorDelay2 = TVC6DOF_sim_DW.ActuatorDelay2_DSTATE;

    /* Saturate: '<Root>/Saturation1' */
    m = TVC6DOF_sim_B.ActuatorDelay2;
    scale = TVC6DOF_sim_P.Saturation1_LowerSat;
    absxk = TVC6DOF_sim_P.Saturation1_UpperSat;
    if (m > absxk) {
      /* Saturate: '<Root>/Saturation1' */
      TVC6DOF_sim_B.Saturation1 = absxk;
    } else if (m < scale) {
      /* Saturate: '<Root>/Saturation1' */
      TVC6DOF_sim_B.Saturation1 = scale;
    } else {
      /* Saturate: '<Root>/Saturation1' */
      TVC6DOF_sim_B.Saturation1 = m;
    }

    /* End of Saturate: '<Root>/Saturation1' */
  }

  /* FromWorkspace: '<S5>/FromWs' */
  {
    real_T *pDataValues = (real_T *) TVC6DOF_sim_DW.FromWs_PWORK_a.DataPtr;
    real_T *pTimeValues = (real_T *) TVC6DOF_sim_DW.FromWs_PWORK_a.TimePtr;
    int_T currTimeIndex = TVC6DOF_sim_DW.FromWs_IWORK_a.PrevIndex;
    real_T t = TVC6DOF_sim_M->Timing.t[0];

    /* Get index */
    if (t <= pTimeValues[0]) {
      currTimeIndex = 0;
    } else if (t >= pTimeValues[53]) {
      currTimeIndex = 52;
    } else {
      if (t < pTimeValues[currTimeIndex]) {
        while (t < pTimeValues[currTimeIndex]) {
          currTimeIndex--;
        }
      } else {
        while (t >= pTimeValues[currTimeIndex + 1]) {
          currTimeIndex++;
        }
      }
    }

    TVC6DOF_sim_DW.FromWs_IWORK_a.PrevIndex = currTimeIndex;

    /* Post output */
    {
      real_T t1 = pTimeValues[currTimeIndex];
      real_T t2 = pTimeValues[currTimeIndex + 1];
      if (t1 == t2) {
        if (t < t1) {
          TVC6DOF_sim_B.FromWs_i = pDataValues[currTimeIndex];
        } else {
          TVC6DOF_sim_B.FromWs_i = pDataValues[currTimeIndex + 1];
        }
      } else {
        real_T f1 = (t2 - t) / (t2 - t1);
        real_T f2 = 1.0 - f1;
        real_T d1;
        real_T d2;
        int_T TimeIndex= currTimeIndex;
        d1 = pDataValues[TimeIndex];
        d2 = pDataValues[TimeIndex + 1];
        TVC6DOF_sim_B.FromWs_i = (real_T) rtInterpolate(d1, d2, f1, f2);
        pDataValues += 54;
      }
    }
  }

  /* Integrator: '<S3>/Integrator6' */
  TVC6DOF_sim_B.Integrator6[0] = TVC6DOF_sim_X.Integrator6_CSTATE[0];
  TVC6DOF_sim_B.Integrator6[1] = TVC6DOF_sim_X.Integrator6_CSTATE[1];
  TVC6DOF_sim_B.Integrator6[2] = TVC6DOF_sim_X.Integrator6_CSTATE[2];
  TVC6DOF_sim_B.Integrator6[3] = TVC6DOF_sim_X.Integrator6_CSTATE[3];

  /* Integrator: '<S3>/Integrator3' */
  TVC6DOF_sim_B.Integrator3[0] = TVC6DOF_sim_X.Integrator3_CSTATE[0];
  TVC6DOF_sim_B.Integrator3[1] = TVC6DOF_sim_X.Integrator3_CSTATE[1];
  TVC6DOF_sim_B.Integrator3[2] = TVC6DOF_sim_X.Integrator3_CSTATE[2];

  /* MATLAB Function: '<S3>/Equations of Motion' incorporates:
   *  Constant: '<S3>/Constant'
   */
  /* :  Ix=params(1); */
  /* :  Iy=params(2); */
  /* :  Iz=params(3); */
  /* :  g=params(4); */
  /* :  l=params(5); */
  /* :  mis_y=params(6); */
  /* :  mis_z=params(7); */
  /* :  m_dry=params(8); */
  /* :  m=m_dry+m_fuel/1000; */
  m = TVC6DOF_sim_B.FromWs / 1000.0 + TVC6DOF_sim_P.Constant_Value[7];

  /* :  wx=w1; */
  /* :  wy=w2; */
  /* :  wz=w3; */
  /* :  if size(q_in)==1 */
  /* :  gCb=[cosd(th_z) sind(th_z) 0;-sind(th_z) cosd(th_z) 0;0 0 1]*[cosd(th_y) 0 -sind(th_y);0 1 0;sind(th_y) 0 cosd(th_y)]; */
  /* :  Tb=gCb.'*[T;0;0]; */
  scale = TVC6DOF_sim_B.Saturation1;
  TVC6DOF_sim_cosd(&scale);
  absxk = TVC6DOF_sim_B.Saturation1;
  TVC6DOF_sim_sind(&absxk);
  t = TVC6DOF_sim_B.Saturation1;
  TVC6DOF_sim_sind(&t);
  g = TVC6DOF_sim_B.Saturation1;
  TVC6DOF_sim_cosd(&g);
  h = TVC6DOF_sim_B.Saturation;
  TVC6DOF_sim_cosd(&h);
  i = TVC6DOF_sim_B.Saturation;
  TVC6DOF_sim_sind(&i);
  j = TVC6DOF_sim_B.Saturation;
  TVC6DOF_sim_sind(&j);
  k = TVC6DOF_sim_B.Saturation;
  TVC6DOF_sim_cosd(&k);
  scale_0[0] = scale;
  scale_0[3] = absxk;
  scale_0[6] = 0.0;
  scale_0[1] = -t;
  scale_0[4] = g;
  scale_0[7] = 0.0;
  h_0[0] = h;
  h_0[3] = 0.0;
  h_0[6] = -i;
  scale_0[2] = 0.0;
  h_0[1] = 0.0;
  scale_0[5] = 0.0;
  h_0[4] = 1.0;
  scale_0[8] = 1.0;
  h_0[7] = 0.0;
  h_0[2] = j;
  h_0[5] = 0.0;
  h_0[8] = k;
  tmp[0] = TVC6DOF_sim_B.FromWs_i;
  tmp[1] = 0.0;
  tmp[2] = 0.0;

  /* :  r=[-l;mis_y;mis_z]; */
  r[0] = -TVC6DOF_sim_P.Constant_Value[4];
  r[1] = TVC6DOF_sim_P.Constant_Value[5];
  r[2] = TVC6DOF_sim_P.Constant_Value[6];

  /* :  Mb=cross(r,Tb); */
  /* :  Ti=hamilton(hamilton(q_in,[0;Tb]),conjugate(q_in)); */
  for (i_0 = 0; i_0 < 3; i_0++) {
    TVC6DOF_sim_B.Tb[i_0] = 0.0;
    for (i_1 = 0; i_1 < 3; i_1++) {
      scale_1[i_0 + 3 * i_1] = 0.0;
      scale = scale_1[3 * i_1 + i_0];
      scale += h_0[3 * i_0] * scale_0[i_1];
      scale_1[i_0 + 3 * i_1] = scale;
      scale = scale_1[3 * i_1 + i_0];
      scale += h_0[3 * i_0 + 1] * scale_0[i_1 + 3];
      scale_1[i_0 + 3 * i_1] = scale;
      scale = scale_1[3 * i_1 + i_0];
      scale += h_0[3 * i_0 + 2] * scale_0[i_1 + 6];
      scale_1[i_0 + 3 * i_1] = scale;
      TVC6DOF_sim_B.Tb[i_0] += scale_1[3 * i_1 + i_0] * tmp[i_1];
    }

    q_rot[i_0 + 1] = TVC6DOF_sim_B.Tb[i_0];
  }

  TVC6DOF_sim_B.Mb[0] = r[1] * TVC6DOF_sim_B.Tb[2] - r[2] * TVC6DOF_sim_B.Tb[1];
  TVC6DOF_sim_B.Mb[1] = r[2] * TVC6DOF_sim_B.Tb[0] - r[0] * TVC6DOF_sim_B.Tb[2];
  TVC6DOF_sim_B.Mb[2] = r[0] * TVC6DOF_sim_B.Tb[1] - r[1] * TVC6DOF_sim_B.Tb[0];

  /* :  product=[r(1)*q(1)-r(2)*q(2)-r(3)*q(3)-r(4)*q(4); */
  /* :                   r(1)*q(2)+r(2)*q(1)-r(3)*q(4)+r(4)*q(3); */
  /* :                   r(1)*q(3)+r(2)*q(4)+r(3)*q(1)-r(4)*q(2); */
  /* :                   r(1)*q(4)-r(2)*q(3)+r(3)*q(2)+r(4)*q(1)]; */
  t = ((0.0 * TVC6DOF_sim_B.Integrator6[0] - q_rot[1] *
        TVC6DOF_sim_B.Integrator6[1]) - q_rot[2] * TVC6DOF_sim_B.Integrator6[2])
    - q_rot[3] * TVC6DOF_sim_B.Integrator6[3];
  g = ((0.0 * TVC6DOF_sim_B.Integrator6[1] + q_rot[1] *
        TVC6DOF_sim_B.Integrator6[0]) - q_rot[2] * TVC6DOF_sim_B.Integrator6[3])
    + q_rot[3] * TVC6DOF_sim_B.Integrator6[2];
  h = ((0.0 * TVC6DOF_sim_B.Integrator6[2] + q_rot[1] *
        TVC6DOF_sim_B.Integrator6[3]) + q_rot[2] * TVC6DOF_sim_B.Integrator6[0])
    - q_rot[3] * TVC6DOF_sim_B.Integrator6[1];
  i = ((0.0 * TVC6DOF_sim_B.Integrator6[3] - q_rot[1] *
        TVC6DOF_sim_B.Integrator6[2]) + q_rot[2] * TVC6DOF_sim_B.Integrator6[1])
    + q_rot[3] * TVC6DOF_sim_B.Integrator6[0];

  /* :  q_star=[q(1);-q(2);-q(3);-q(4)]; */
  q_rot[0] = TVC6DOF_sim_B.Integrator6[0];
  q_rot[1] = -TVC6DOF_sim_B.Integrator6[1];
  q_rot[2] = -TVC6DOF_sim_B.Integrator6[2];
  q_rot[3] = -TVC6DOF_sim_B.Integrator6[3];

  /* :  product=[r(1)*q(1)-r(2)*q(2)-r(3)*q(3)-r(4)*q(4); */
  /* :                   r(1)*q(2)+r(2)*q(1)-r(3)*q(4)+r(4)*q(3); */
  /* :                   r(1)*q(3)+r(2)*q(4)+r(3)*q(1)-r(4)*q(2); */
  /* :                   r(1)*q(4)-r(2)*q(3)+r(3)*q(2)+r(4)*q(1)]; */
  scale = ((q_rot[0] * g + q_rot[1] * t) - q_rot[2] * i) + q_rot[3] * h;
  absxk = ((q_rot[0] * h + q_rot[1] * i) + q_rot[2] * t) - q_rot[3] * g;
  t = ((q_rot[0] * i - q_rot[1] * h) + q_rot[2] * g) + q_rot[3] * t;

  /* :  xdd=Ti(2)/m-g; */
  /* :  ydd=Ti(3)/m; */
  /* :  zdd=Ti(4)/m; */
  /* :  rdd=[xdd;ydd;zdd]; */
  TVC6DOF_sim_B.rdd[0] = scale / m - TVC6DOF_sim_P.Constant_Value[3];
  TVC6DOF_sim_B.rdd[1] = absxk / m;
  TVC6DOF_sim_B.rdd[2] = t / m;

  /* :  wxd=(1/Ix)*(Mb(1)+(Iy-Iz)*wy*wz); */
  /* :  wyd=(1/Iy)*(Mb(2)+(Iz-Ix)*wx*wz); */
  /* :  wzd=(1/Iz)*(Mb(3)+(Ix-Iy)*wx*wy); */
  /* :  wd=[wxd;wyd;wzd]; */
  TVC6DOF_sim_B.wd[0] = ((TVC6DOF_sim_P.Constant_Value[1] -
    TVC6DOF_sim_P.Constant_Value[2]) * TVC6DOF_sim_B.Integrator3[1] *
    TVC6DOF_sim_B.Integrator3[2] + TVC6DOF_sim_B.Mb[0]) * (1.0 /
    TVC6DOF_sim_P.Constant_Value[0]);
  TVC6DOF_sim_B.wd[1] = ((TVC6DOF_sim_P.Constant_Value[2] -
    TVC6DOF_sim_P.Constant_Value[0]) * TVC6DOF_sim_B.Integrator3[0] *
    TVC6DOF_sim_B.Integrator3[2] + TVC6DOF_sim_B.Mb[1]) * (1.0 /
    TVC6DOF_sim_P.Constant_Value[1]);
  TVC6DOF_sim_B.wd[2] = ((TVC6DOF_sim_P.Constant_Value[0] -
    TVC6DOF_sim_P.Constant_Value[1]) * TVC6DOF_sim_B.Integrator3[0] *
    TVC6DOF_sim_B.Integrator3[1] + TVC6DOF_sim_B.Mb[2]) * (1.0 /
    TVC6DOF_sim_P.Constant_Value[2]);

  /* :  qd=0.5*hamilton(q_in,[0;wx;wy;wz]); */
  q_rot[1] = TVC6DOF_sim_B.Integrator3[0];
  q_rot[2] = TVC6DOF_sim_B.Integrator3[1];
  q_rot[3] = TVC6DOF_sim_B.Integrator3[2];

  /* :  product=[r(1)*q(1)-r(2)*q(2)-r(3)*q(3)-r(4)*q(4); */
  /* :                   r(1)*q(2)+r(2)*q(1)-r(3)*q(4)+r(4)*q(3); */
  /* :                   r(1)*q(3)+r(2)*q(4)+r(3)*q(1)-r(4)*q(2); */
  /* :                   r(1)*q(4)-r(2)*q(3)+r(3)*q(2)+r(4)*q(1)]; */
  TVC6DOF_sim_B.qd[0] = (((0.0 * TVC6DOF_sim_B.Integrator6[0] - q_rot[1] *
    TVC6DOF_sim_B.Integrator6[1]) - q_rot[2] * TVC6DOF_sim_B.Integrator6[2]) -
    q_rot[3] * TVC6DOF_sim_B.Integrator6[3]) * 0.5;
  TVC6DOF_sim_B.qd[1] = (((0.0 * TVC6DOF_sim_B.Integrator6[1] + q_rot[1] *
    TVC6DOF_sim_B.Integrator6[0]) - q_rot[2] * TVC6DOF_sim_B.Integrator6[3]) +
    q_rot[3] * TVC6DOF_sim_B.Integrator6[2]) * 0.5;
  TVC6DOF_sim_B.qd[2] = (((0.0 * TVC6DOF_sim_B.Integrator6[2] + q_rot[1] *
    TVC6DOF_sim_B.Integrator6[3]) + q_rot[2] * TVC6DOF_sim_B.Integrator6[0]) -
    q_rot[3] * TVC6DOF_sim_B.Integrator6[1]) * 0.5;
  TVC6DOF_sim_B.qd[3] = (((0.0 * TVC6DOF_sim_B.Integrator6[3] - q_rot[1] *
    TVC6DOF_sim_B.Integrator6[2]) + q_rot[2] * TVC6DOF_sim_B.Integrator6[1]) +
    q_rot[3] * TVC6DOF_sim_B.Integrator6[0]) * 0.5;

  /* End of MATLAB Function: '<S3>/Equations of Motion' */
  if (rtmIsMajorTimeStep(TVC6DOF_sim_M) &&
      TVC6DOF_sim_M->Timing.TaskCounters.TID[1] == 0) {
    /* Scope: '<Root>/Scope3' */
    if (rtmIsMajorTimeStep(TVC6DOF_sim_M)) {
      StructLogVar *svar = (StructLogVar *)
        TVC6DOF_sim_DW.Scope3_PWORK.LoggedData;
      LogVar *var = svar->signals.values;

      /* signals */
      {
        real_T up0[3];
        up0[0] = TVC6DOF_sim_B.rdd[0];
        up0[1] = TVC6DOF_sim_B.rdd[1];
        up0[2] = TVC6DOF_sim_B.rdd[2];
        rt_UpdateLogVar((LogVar *)var, up0, 0);
      }
    }

    /* Scope: '<Root>/Scope4' */
    if (rtmIsMajorTimeStep(TVC6DOF_sim_M)) {
      StructLogVar *svar = (StructLogVar *)
        TVC6DOF_sim_DW.Scope4_PWORK.LoggedData;
      LogVar *var = svar->signals.values;

      /* signals */
      {
        real_T up0[3];
        up0[0] = TVC6DOF_sim_B.Integrator3[0];
        up0[1] = TVC6DOF_sim_B.Integrator3[1];
        up0[2] = TVC6DOF_sim_B.Integrator3[2];
        rt_UpdateLogVar((LogVar *)var, up0, 0);
      }
    }

    /* Scope: '<Root>/Scope5' */
    if (rtmIsMajorTimeStep(TVC6DOF_sim_M)) {
      StructLogVar *svar = (StructLogVar *)
        TVC6DOF_sim_DW.Scope5_PWORK.LoggedData;
      LogVar *var = svar->signals.values;

      /* signals */
      {
        real_T up0[4];
        up0[0] = TVC6DOF_sim_B.Integrator6[0];
        up0[1] = TVC6DOF_sim_B.Integrator6[1];
        up0[2] = TVC6DOF_sim_B.Integrator6[2];
        up0[3] = TVC6DOF_sim_B.Integrator6[3];
        rt_UpdateLogVar((LogVar *)var, up0, 0);
      }
    }

    /* RandomNumber: '<S1>/Random Number' */
    TVC6DOF_sim_B.RandomNumber = TVC6DOF_sim_DW.NextOutput;

    /* ZeroOrderHold: '<S1>/Zero-Order Hold' */
    TVC6DOF_sim_B.ZeroOrderHold[0] = TVC6DOF_sim_B.rdd[0];

    /* Sum: '<S1>/Add' */
    TVC6DOF_sim_B.Add[0] = TVC6DOF_sim_B.ZeroOrderHold[0] +
      TVC6DOF_sim_B.RandomNumber;

    /* ZeroOrderHold: '<S1>/Zero-Order Hold1' */
    TVC6DOF_sim_B.ZeroOrderHold1[0] = TVC6DOF_sim_B.Integrator3[0];

    /* ZeroOrderHold: '<S1>/Zero-Order Hold' */
    TVC6DOF_sim_B.ZeroOrderHold[1] = TVC6DOF_sim_B.rdd[1];

    /* Sum: '<S1>/Add' */
    TVC6DOF_sim_B.Add[1] = TVC6DOF_sim_B.ZeroOrderHold[1] +
      TVC6DOF_sim_B.RandomNumber;

    /* ZeroOrderHold: '<S1>/Zero-Order Hold1' */
    TVC6DOF_sim_B.ZeroOrderHold1[1] = TVC6DOF_sim_B.Integrator3[1];

    /* ZeroOrderHold: '<S1>/Zero-Order Hold' */
    TVC6DOF_sim_B.ZeroOrderHold[2] = TVC6DOF_sim_B.rdd[2];

    /* Sum: '<S1>/Add' */
    TVC6DOF_sim_B.Add[2] = TVC6DOF_sim_B.ZeroOrderHold[2] +
      TVC6DOF_sim_B.RandomNumber;

    /* ZeroOrderHold: '<S1>/Zero-Order Hold1' */
    TVC6DOF_sim_B.ZeroOrderHold1[2] = TVC6DOF_sim_B.Integrator3[2];

    /* ZeroOrderHold: '<S1>/Zero-Order Hold2' */
    TVC6DOF_sim_B.ZeroOrderHold2[0] = TVC6DOF_sim_B.Integrator6[0];
    TVC6DOF_sim_B.ZeroOrderHold2[1] = TVC6DOF_sim_B.Integrator6[1];
    TVC6DOF_sim_B.ZeroOrderHold2[2] = TVC6DOF_sim_B.Integrator6[2];
    TVC6DOF_sim_B.ZeroOrderHold2[3] = TVC6DOF_sim_B.Integrator6[3];

    /* MATLAB Function: '<S1>/MATLAB Function' */
    /* :  angle_var=0.01; */
    /* :  axis=randn(3,1); */
    TVC6DOF_sim_randn(r);

    /* :  axis=axis/norm(axis); */
    scale = 3.3121686421112381E-170;
    absxk = fabs(r[0]);
    if (absxk > 3.3121686421112381E-170) {
      m = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      m = t * t;
    }

    absxk = fabs(r[1]);
    if (absxk > scale) {
      t = scale / absxk;
      m = m * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      m += t * t;
    }

    absxk = fabs(r[2]);
    if (absxk > scale) {
      t = scale / absxk;
      m = m * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      m += t * t;
    }

    m = scale * sqrt(m);
    scale = r[0];
    scale /= m;
    r[0] = scale;
    scale = r[1];
    scale /= m;
    r[1] = scale;
    scale = r[2];
    scale /= m;
    r[2] = scale;

    /* :  angle=randn*angle_var; */
    m = TVC6DOF_sim_randn_l() * 0.01;

    /* :  q_rot=[cos(angle/2);axis(1)*sin(angle/2);axis(2)*sin(angle/2);axis(3)*sin(angle/2)]; */
    q_rot[0] = cos(m / 2.0);
    q_rot[1] = sin(m / 2.0) * r[0];
    q_rot[2] = sin(m / 2.0) * r[1];
    q_rot[3] = sin(m / 2.0) * r[2];

    /* :  q_n=hamilton(q,q_rot); */
    /* :  product=[r(1)*q(1)-r(2)*q(2)-r(3)*q(3)-r(4)*q(4); */
    /* :               r(1)*q(2)+r(2)*q(1)-r(3)*q(4)+r(4)*q(3); */
    /* :               r(1)*q(3)+r(2)*q(4)+r(3)*q(1)-r(4)*q(2); */
    /* :               r(1)*q(4)-r(2)*q(3)+r(3)*q(2)+r(4)*q(1)]; */
    TVC6DOF_sim_B.q_n[0] = ((q_rot[0] * TVC6DOF_sim_B.ZeroOrderHold2[0] - q_rot
      [1] * TVC6DOF_sim_B.ZeroOrderHold2[1]) - q_rot[2] *
      TVC6DOF_sim_B.ZeroOrderHold2[2]) - q_rot[3] *
      TVC6DOF_sim_B.ZeroOrderHold2[3];
    TVC6DOF_sim_B.q_n[1] = ((q_rot[0] * TVC6DOF_sim_B.ZeroOrderHold2[1] + q_rot
      [1] * TVC6DOF_sim_B.ZeroOrderHold2[0]) - q_rot[2] *
      TVC6DOF_sim_B.ZeroOrderHold2[3]) + q_rot[3] *
      TVC6DOF_sim_B.ZeroOrderHold2[2];
    TVC6DOF_sim_B.q_n[2] = ((q_rot[0] * TVC6DOF_sim_B.ZeroOrderHold2[2] + q_rot
      [1] * TVC6DOF_sim_B.ZeroOrderHold2[3]) + q_rot[2] *
      TVC6DOF_sim_B.ZeroOrderHold2[0]) - q_rot[3] *
      TVC6DOF_sim_B.ZeroOrderHold2[1];
    TVC6DOF_sim_B.q_n[3] = ((q_rot[0] * TVC6DOF_sim_B.ZeroOrderHold2[3] - q_rot
      [1] * TVC6DOF_sim_B.ZeroOrderHold2[2]) + q_rot[2] *
      TVC6DOF_sim_B.ZeroOrderHold2[1]) + q_rot[3] *
      TVC6DOF_sim_B.ZeroOrderHold2[0];

    /* End of MATLAB Function: '<S1>/MATLAB Function' */

    /* MATLAB Function: '<S4>/MATLAB Function' incorporates:
     *  Constant: '<S4>/Constant'
     */
    /* :  w_est=w_n; */
    /* :  T_est=norm(rdd_n)*m_meas; */
    scale = 3.3121686421112381E-170;
    TVC6DOF_sim_B.w_est[0] = TVC6DOF_sim_B.ZeroOrderHold1[0];
    absxk = fabs(TVC6DOF_sim_B.Add[0]);
    if (absxk > 3.3121686421112381E-170) {
      m = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      m = t * t;
    }

    TVC6DOF_sim_B.w_est[1] = TVC6DOF_sim_B.ZeroOrderHold1[1];
    absxk = fabs(TVC6DOF_sim_B.Add[1]);
    if (absxk > scale) {
      t = scale / absxk;
      m = m * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      m += t * t;
    }

    TVC6DOF_sim_B.w_est[2] = TVC6DOF_sim_B.ZeroOrderHold1[2];
    absxk = fabs(TVC6DOF_sim_B.Add[2]);
    if (absxk > scale) {
      t = scale / absxk;
      m = m * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      m += t * t;
    }

    m = scale * sqrt(m);
    TVC6DOF_sim_B.T_est = m * TVC6DOF_sim_P.m_dry_n;

    /* :  q_est=q_n; */
    TVC6DOF_sim_B.q_est[0] = TVC6DOF_sim_B.q_n[0];
    TVC6DOF_sim_B.q_est[1] = TVC6DOF_sim_B.q_n[1];
    TVC6DOF_sim_B.q_est[2] = TVC6DOF_sim_B.q_n[2];
    TVC6DOF_sim_B.q_est[3] = TVC6DOF_sim_B.q_n[3];

    /* End of MATLAB Function: '<S4>/MATLAB Function' */

    /* Gain: '<S2>/Gain10' */
    TVC6DOF_sim_B.Gain10[0] = TVC6DOF_sim_P.Gain10_Gain * TVC6DOF_sim_B.w_est[0];
    TVC6DOF_sim_B.Gain10[1] = TVC6DOF_sim_P.Gain10_Gain * TVC6DOF_sim_B.w_est[1];
    TVC6DOF_sim_B.Gain10[2] = TVC6DOF_sim_P.Gain10_Gain * TVC6DOF_sim_B.w_est[2];
    for (i_0 = 0; i_0 < 3; i_0++) {
      /* Gain: '<S2>/Gain' */
      TVC6DOF_sim_B.Gain[i_0] = 0.0;
      TVC6DOF_sim_B.Gain[i_0] += TVC6DOF_sim_P.Gain_Gain[i_0] *
        TVC6DOF_sim_B.q_est[0];
      TVC6DOF_sim_B.Gain[i_0] += TVC6DOF_sim_P.Gain_Gain[i_0 + 3] *
        TVC6DOF_sim_B.q_est[1];
      TVC6DOF_sim_B.Gain[i_0] += TVC6DOF_sim_P.Gain_Gain[i_0 + 6] *
        TVC6DOF_sim_B.q_est[2];
      TVC6DOF_sim_B.Gain[i_0] += TVC6DOF_sim_P.Gain_Gain[i_0 + 9] *
        TVC6DOF_sim_B.q_est[3];

      /* Gain: '<S2>/Gain11' */
      TVC6DOF_sim_B.Gain11[i_0] = TVC6DOF_sim_P.Gain11_Gain *
        TVC6DOF_sim_B.Gain[i_0];
    }

    /* Gain: '<S2>/Gain1' */
    TVC6DOF_sim_B.Gain1 = TVC6DOF_sim_P.kp * TVC6DOF_sim_B.Gain11[0];

    /* Gain: '<S2>/Gain3' */
    TVC6DOF_sim_B.Gain3 = TVC6DOF_sim_P.kd * TVC6DOF_sim_B.Gain10[0];
  }

  /* Integrator: '<S2>/Integrator' */
  TVC6DOF_sim_B.Integrator_i = TVC6DOF_sim_X.Integrator_CSTATE_b;

  /* Gain: '<S2>/Gain2' */
  TVC6DOF_sim_B.Gain2 = TVC6DOF_sim_P.ki * TVC6DOF_sim_B.Integrator_i;

  /* Sum: '<S2>/Sum' */
  TVC6DOF_sim_B.Sum = (TVC6DOF_sim_B.Gain1 + TVC6DOF_sim_B.Gain2) +
    TVC6DOF_sim_B.Gain3;
  if (rtmIsMajorTimeStep(TVC6DOF_sim_M) &&
      TVC6DOF_sim_M->Timing.TaskCounters.TID[1] == 0) {
    /* Gain: '<S2>/Gain4' */
    TVC6DOF_sim_B.Gain4 = TVC6DOF_sim_P.kp * TVC6DOF_sim_B.Gain11[1];

    /* Gain: '<S2>/Gain6' */
    TVC6DOF_sim_B.Gain6 = TVC6DOF_sim_P.kd * TVC6DOF_sim_B.Gain10[1];
  }

  /* Integrator: '<S2>/Integrator1' */
  TVC6DOF_sim_B.Integrator1_f = TVC6DOF_sim_X.Integrator1_CSTATE_n;

  /* Gain: '<S2>/Gain5' */
  TVC6DOF_sim_B.Gain5 = TVC6DOF_sim_P.ki * TVC6DOF_sim_B.Integrator1_f;

  /* Sum: '<S2>/Sum1' */
  TVC6DOF_sim_B.Sum1 = (TVC6DOF_sim_B.Gain4 + TVC6DOF_sim_B.Gain5) +
    TVC6DOF_sim_B.Gain6;
  if (rtmIsMajorTimeStep(TVC6DOF_sim_M) &&
      TVC6DOF_sim_M->Timing.TaskCounters.TID[1] == 0) {
    /* Gain: '<S2>/Gain7' */
    TVC6DOF_sim_B.Gain7 = TVC6DOF_sim_P.kp * TVC6DOF_sim_B.Gain11[2];

    /* Gain: '<S2>/Gain9' */
    TVC6DOF_sim_B.Gain9 = TVC6DOF_sim_P.kd * TVC6DOF_sim_B.Gain10[2];
  }

  /* Integrator: '<S2>/Integrator2' */
  TVC6DOF_sim_B.Integrator2 = TVC6DOF_sim_X.Integrator2_CSTATE;

  /* Gain: '<S2>/Gain8' */
  TVC6DOF_sim_B.Gain8 = TVC6DOF_sim_P.ki * TVC6DOF_sim_B.Integrator2;

  /* Sum: '<S2>/Sum2' */
  TVC6DOF_sim_B.Sum2 = (TVC6DOF_sim_B.Gain7 + TVC6DOF_sim_B.Gain8) +
    TVC6DOF_sim_B.Gain9;

  /* MATLAB Function: '<S2>/MATLAB Function' incorporates:
   *  Constant: '<S2>/Constant'
   */
  /* :  Ix=params(1); */
  /* :  Iy=params(2); */
  /* :  Iz=params(3); */
  /* :  l=params(4); */
  /* :  g=params(5); */
  /* :  Tb=[0;Mz/l;-My/l]; */
  r[1] = TVC6DOF_sim_B.Sum2 / TVC6DOF_sim_P.Constant_Value_a[3];
  r[2] = -TVC6DOF_sim_B.Sum1 / TVC6DOF_sim_P.Constant_Value_a[3];

  /* :  th_y_com=asin(Tb(3)/T_est)*(180/pi); */
  m = asin(r[2] / TVC6DOF_sim_B.T_est) * 57.295779513082323;

  /* :  th_z_com=asin(Tb(2)/(-T_est*cos((pi*th_y_com)/180)))*(180/pi); */
  TVC6DOF_sim_B.th_z_com = asin(r[1] / (cos(3.1415926535897931 * m / 180.0) *
    -TVC6DOF_sim_B.T_est)) * 57.295779513082323;
  TVC6DOF_sim_B.th_y_com = m;

  /* Sum: '<S2>/Add' incorporates:
   *  Constant: '<S2>/Constant1'
   */
  TVC6DOF_sim_B.Add_c = TVC6DOF_sim_B.th_y_com + TVC6DOF_sim_P.static_error_y_mc;

  /* Sum: '<S2>/Add1' incorporates:
   *  Constant: '<S2>/Constant2'
   */
  TVC6DOF_sim_B.Add1 = TVC6DOF_sim_B.th_z_com + TVC6DOF_sim_P.static_error_z_mc;
  if (rtmIsMajorTimeStep(TVC6DOF_sim_M) &&
      TVC6DOF_sim_M->Timing.TaskCounters.TID[1] == 0) {
    /* HitCross: '<S3>/Hit  Crossing' */
    zcEvent = rt_ZCFcn(FALLING_ZERO_CROSSING,
                       &TVC6DOF_sim_PrevZCX.HitCrossing_Input_ZCE,
                       (TVC6DOF_sim_B.Integrator[0] -
                        TVC6DOF_sim_P.HitCrossing_Offset));
    if (TVC6DOF_sim_DW.HitCrossing_MODE == 0) {
      if (zcEvent != NO_ZCEVENT) {
        /* HitCross: '<S3>/Hit  Crossing' */
        TVC6DOF_sim_B.HitCrossing = !TVC6DOF_sim_B.HitCrossing;
        TVC6DOF_sim_DW.HitCrossing_MODE = 1;
      } else {
        if (TVC6DOF_sim_B.HitCrossing) {
          /* HitCross: '<S3>/Hit  Crossing' */
          TVC6DOF_sim_B.HitCrossing = ((!(TVC6DOF_sim_B.Integrator[0] !=
            TVC6DOF_sim_P.HitCrossing_Offset)) && TVC6DOF_sim_B.HitCrossing);
        }
      }
    } else {
      /* HitCross: '<S3>/Hit  Crossing' */
      TVC6DOF_sim_B.HitCrossing = ((!(TVC6DOF_sim_B.Integrator[0] !=
        TVC6DOF_sim_P.HitCrossing_Offset)) && TVC6DOF_sim_B.HitCrossing);
      TVC6DOF_sim_DW.HitCrossing_MODE = 0;
    }

    /* End of HitCross: '<S3>/Hit  Crossing' */

    /* Stop: '<S3>/Stop Simulation' */
    if (TVC6DOF_sim_B.HitCrossing) {
      rtmSetStopRequested(TVC6DOF_sim_M, 1);
    }

    /* End of Stop: '<S3>/Stop Simulation' */
  }

  if (rtmIsMajorTimeStep(TVC6DOF_sim_M)) {
    /* Matfile logging */
    rt_UpdateTXYLogVars(TVC6DOF_sim_M->rtwLogInfo, (TVC6DOF_sim_M->Timing.t));
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(TVC6DOF_sim_M)) {
    if (rtmIsMajorTimeStep(TVC6DOF_sim_M) &&
        TVC6DOF_sim_M->Timing.TaskCounters.TID[2] == 0) {
      /* Update for Delay: '<Root>/Actuator Delay' */
      TVC6DOF_sim_DW.ActuatorDelay_DSTATE = TVC6DOF_sim_B.Add_c;

      /* Update for Delay: '<Root>/Actuator Delay2' */
      TVC6DOF_sim_DW.ActuatorDelay2_DSTATE = TVC6DOF_sim_B.Add1;
    }

    if (rtmIsMajorTimeStep(TVC6DOF_sim_M) &&
        TVC6DOF_sim_M->Timing.TaskCounters.TID[1] == 0) {
      /* Update for RandomNumber: '<S1>/Random Number' */
      TVC6DOF_sim_DW.NextOutput = rt_nrand_Upu32_Yd_f_pw_snf
        (&TVC6DOF_sim_DW.RandSeed) * TVC6DOF_sim_P.RandomNumber_StdDev +
        TVC6DOF_sim_P.RandomNumber_Mean;
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(TVC6DOF_sim_M)) {
    /* signal main to stop simulation */
    {                                  /* Sample time: [0.0s, 0.0s] */
      if ((rtmGetTFinal(TVC6DOF_sim_M)!=-1) &&
          !((rtmGetTFinal(TVC6DOF_sim_M)-(((TVC6DOF_sim_M->Timing.clockTick1+
               TVC6DOF_sim_M->Timing.clockTickH1* 4294967296.0)) * 0.01)) >
            (((TVC6DOF_sim_M->Timing.clockTick1+
               TVC6DOF_sim_M->Timing.clockTickH1* 4294967296.0)) * 0.01) *
            (DBL_EPSILON))) {
        rtmSetErrorStatus(TVC6DOF_sim_M, "Simulation finished");
      }
    }

    rt_ertODEUpdateContinuousStates(&TVC6DOF_sim_M->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++TVC6DOF_sim_M->Timing.clockTick0)) {
      ++TVC6DOF_sim_M->Timing.clockTickH0;
    }

    TVC6DOF_sim_M->Timing.t[0] = rtsiGetSolverStopTime
      (&TVC6DOF_sim_M->solverInfo);

    {
      /* Update absolute timer for sample time: [0.01s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.01, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      TVC6DOF_sim_M->Timing.clockTick1++;
      if (!TVC6DOF_sim_M->Timing.clockTick1) {
        TVC6DOF_sim_M->Timing.clockTickH1++;
      }
    }

    rate_scheduler();
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void TVC6DOF_sim_derivatives(void)
{
  XDot_TVC6DOF_sim_T *_rtXdot;
  _rtXdot = ((XDot_TVC6DOF_sim_T *) TVC6DOF_sim_M->derivs);

  /* Derivatives for Integrator: '<S3>/Integrator1' */
  _rtXdot->Integrator1_CSTATE[0] = TVC6DOF_sim_B.Integrator[0];

  /* Derivatives for Integrator: '<S3>/Integrator' */
  _rtXdot->Integrator_CSTATE[0] = TVC6DOF_sim_B.rdd[0];

  /* Derivatives for Integrator: '<S3>/Integrator1' */
  _rtXdot->Integrator1_CSTATE[1] = TVC6DOF_sim_B.Integrator[1];

  /* Derivatives for Integrator: '<S3>/Integrator' */
  _rtXdot->Integrator_CSTATE[1] = TVC6DOF_sim_B.rdd[1];

  /* Derivatives for Integrator: '<S3>/Integrator1' */
  _rtXdot->Integrator1_CSTATE[2] = TVC6DOF_sim_B.Integrator[2];

  /* Derivatives for Integrator: '<S3>/Integrator' */
  _rtXdot->Integrator_CSTATE[2] = TVC6DOF_sim_B.rdd[2];

  /* Derivatives for Integrator: '<S3>/Integrator6' */
  _rtXdot->Integrator6_CSTATE[0] = TVC6DOF_sim_B.qd[0];
  _rtXdot->Integrator6_CSTATE[1] = TVC6DOF_sim_B.qd[1];
  _rtXdot->Integrator6_CSTATE[2] = TVC6DOF_sim_B.qd[2];
  _rtXdot->Integrator6_CSTATE[3] = TVC6DOF_sim_B.qd[3];

  /* Derivatives for Integrator: '<S3>/Integrator3' */
  _rtXdot->Integrator3_CSTATE[0] = TVC6DOF_sim_B.wd[0];
  _rtXdot->Integrator3_CSTATE[1] = TVC6DOF_sim_B.wd[1];
  _rtXdot->Integrator3_CSTATE[2] = TVC6DOF_sim_B.wd[2];

  /* Derivatives for Integrator: '<S2>/Integrator' */
  _rtXdot->Integrator_CSTATE_b = TVC6DOF_sim_B.Gain11[0];

  /* Derivatives for Integrator: '<S2>/Integrator1' */
  _rtXdot->Integrator1_CSTATE_n = TVC6DOF_sim_B.Gain11[1];

  /* Derivatives for Integrator: '<S2>/Integrator2' */
  _rtXdot->Integrator2_CSTATE = TVC6DOF_sim_B.Gain11[2];
}

/* Model initialize function */
void TVC6DOF_sim_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)TVC6DOF_sim_M, 0,
                sizeof(RT_MODEL_TVC6DOF_sim_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&TVC6DOF_sim_M->solverInfo,
                          &TVC6DOF_sim_M->Timing.simTimeStep);
    rtsiSetTPtr(&TVC6DOF_sim_M->solverInfo, &rtmGetTPtr(TVC6DOF_sim_M));
    rtsiSetStepSizePtr(&TVC6DOF_sim_M->solverInfo,
                       &TVC6DOF_sim_M->Timing.stepSize0);
    rtsiSetdXPtr(&TVC6DOF_sim_M->solverInfo, &TVC6DOF_sim_M->derivs);
    rtsiSetContStatesPtr(&TVC6DOF_sim_M->solverInfo, (real_T **)
                         &TVC6DOF_sim_M->contStates);
    rtsiSetNumContStatesPtr(&TVC6DOF_sim_M->solverInfo,
      &TVC6DOF_sim_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&TVC6DOF_sim_M->solverInfo,
      &TVC6DOF_sim_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&TVC6DOF_sim_M->solverInfo,
      &TVC6DOF_sim_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&TVC6DOF_sim_M->solverInfo,
      &TVC6DOF_sim_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&TVC6DOF_sim_M->solverInfo, (&rtmGetErrorStatus
      (TVC6DOF_sim_M)));
    rtsiSetRTModelPtr(&TVC6DOF_sim_M->solverInfo, TVC6DOF_sim_M);
  }

  rtsiSetSimTimeStep(&TVC6DOF_sim_M->solverInfo, MAJOR_TIME_STEP);
  TVC6DOF_sim_M->intgData.y = TVC6DOF_sim_M->odeY;
  TVC6DOF_sim_M->intgData.f[0] = TVC6DOF_sim_M->odeF[0];
  TVC6DOF_sim_M->intgData.f[1] = TVC6DOF_sim_M->odeF[1];
  TVC6DOF_sim_M->intgData.f[2] = TVC6DOF_sim_M->odeF[2];
  TVC6DOF_sim_M->contStates = ((X_TVC6DOF_sim_T *) &TVC6DOF_sim_X);
  rtsiSetSolverData(&TVC6DOF_sim_M->solverInfo, (void *)&TVC6DOF_sim_M->intgData);
  rtsiSetSolverName(&TVC6DOF_sim_M->solverInfo,"ode3");
  rtmSetTPtr(TVC6DOF_sim_M, &TVC6DOF_sim_M->Timing.tArray[0]);
  rtmSetTFinal(TVC6DOF_sim_M, 7.0);
  TVC6DOF_sim_M->Timing.stepSize0 = 0.01;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = NULL;
    TVC6DOF_sim_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(TVC6DOF_sim_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(TVC6DOF_sim_M->rtwLogInfo, (NULL));
    rtliSetLogT(TVC6DOF_sim_M->rtwLogInfo, "tout");
    rtliSetLogX(TVC6DOF_sim_M->rtwLogInfo, "");
    rtliSetLogXFinal(TVC6DOF_sim_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(TVC6DOF_sim_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(TVC6DOF_sim_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(TVC6DOF_sim_M->rtwLogInfo, 0);
    rtliSetLogDecimation(TVC6DOF_sim_M->rtwLogInfo, 1);
    rtliSetLogY(TVC6DOF_sim_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(TVC6DOF_sim_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(TVC6DOF_sim_M->rtwLogInfo, (NULL));
  }

  /* block I/O */
  (void) memset(((void *) &TVC6DOF_sim_B), 0,
                sizeof(B_TVC6DOF_sim_T));

  /* states (continuous) */
  {
    (void) memset((void *)&TVC6DOF_sim_X, 0,
                  sizeof(X_TVC6DOF_sim_T));
  }

  /* states (dwork) */
  (void) memset((void *)&TVC6DOF_sim_DW, 0,
                sizeof(DW_TVC6DOF_sim_T));

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(TVC6DOF_sim_M->rtwLogInfo, 0.0, rtmGetTFinal
    (TVC6DOF_sim_M), TVC6DOF_sim_M->Timing.stepSize0, (&rtmGetErrorStatus
    (TVC6DOF_sim_M)));

  /* SetupRuntimeResources for Scope: '<Root>/Scope1' */
  {
    RTWLogSignalInfo rt_ScopeSignalInfo;
    static int_T rt_ScopeSignalWidths[] = { 3 };

    static int_T rt_ScopeSignalNumDimensions[] = { 1 };

    static int_T rt_ScopeSignalDimensions[] = { 3 };

    static void *rt_ScopeCurrSigDims[] = { (NULL) };

    static int_T rt_ScopeCurrSigDimsSize[] = { 4 };

    static const char_T *rt_ScopeSignalLabels[] = { "" };

    static char_T rt_ScopeSignalTitles[] = "";
    static int_T rt_ScopeSignalTitleLengths[] = { 0 };

    static boolean_T rt_ScopeSignalIsVarDims[] = { 0 };

    static int_T rt_ScopeSignalPlotStyles[] = { 0, 0, 0 };

    BuiltInDTypeId dTypes[1] = { SS_DOUBLE };

    static char_T rt_ScopeBlockName[] = "TVC6DOF_sim/Scope1";
    static int_T rt_ScopeFrameData[] = { 0 };

    static RTWPreprocessingFcnPtr rt_ScopeSignalLoggingPreprocessingFcnPtrs[] =
      {
      (NULL)
    };

    rt_ScopeSignalInfo.numSignals = 1;
    rt_ScopeSignalInfo.numCols = rt_ScopeSignalWidths;
    rt_ScopeSignalInfo.numDims = rt_ScopeSignalNumDimensions;
    rt_ScopeSignalInfo.dims = rt_ScopeSignalDimensions;
    rt_ScopeSignalInfo.isVarDims = rt_ScopeSignalIsVarDims;
    rt_ScopeSignalInfo.currSigDims = rt_ScopeCurrSigDims;
    rt_ScopeSignalInfo.currSigDimsSize = rt_ScopeCurrSigDimsSize;
    rt_ScopeSignalInfo.dataTypes = dTypes;
    rt_ScopeSignalInfo.complexSignals = (NULL);
    rt_ScopeSignalInfo.frameData = rt_ScopeFrameData;
    rt_ScopeSignalInfo.preprocessingPtrs =
      rt_ScopeSignalLoggingPreprocessingFcnPtrs;
    rt_ScopeSignalInfo.labels.cptr = rt_ScopeSignalLabels;
    rt_ScopeSignalInfo.titles = rt_ScopeSignalTitles;
    rt_ScopeSignalInfo.titleLengths = rt_ScopeSignalTitleLengths;
    rt_ScopeSignalInfo.plotStyles = rt_ScopeSignalPlotStyles;
    rt_ScopeSignalInfo.blockNames.cptr = (NULL);
    rt_ScopeSignalInfo.stateNames.cptr = (NULL);
    rt_ScopeSignalInfo.crossMdlRef = (NULL);
    rt_ScopeSignalInfo.dataTypeConvert = (NULL);
    TVC6DOF_sim_DW.Scope1_PWORK.LoggedData = rt_CreateStructLogVar(
      TVC6DOF_sim_M->rtwLogInfo,
      0.0,
      rtmGetTFinal(TVC6DOF_sim_M),
      TVC6DOF_sim_M->Timing.stepSize0,
      (&rtmGetErrorStatus(TVC6DOF_sim_M)),
      "position",
      0,
      0,
      1,
      0.01,
      &rt_ScopeSignalInfo,
      rt_ScopeBlockName);
    if (TVC6DOF_sim_DW.Scope1_PWORK.LoggedData == (NULL))
      return;
  }

  /* SetupRuntimeResources for Scope: '<Root>/Scope2' */
  {
    RTWLogSignalInfo rt_ScopeSignalInfo;
    static int_T rt_ScopeSignalWidths[] = { 3 };

    static int_T rt_ScopeSignalNumDimensions[] = { 1 };

    static int_T rt_ScopeSignalDimensions[] = { 3 };

    static void *rt_ScopeCurrSigDims[] = { (NULL) };

    static int_T rt_ScopeCurrSigDimsSize[] = { 4 };

    static const char_T *rt_ScopeSignalLabels[] = { "" };

    static char_T rt_ScopeSignalTitles[] = "";
    static int_T rt_ScopeSignalTitleLengths[] = { 0 };

    static boolean_T rt_ScopeSignalIsVarDims[] = { 0 };

    static int_T rt_ScopeSignalPlotStyles[] = { 0, 0, 0 };

    BuiltInDTypeId dTypes[1] = { SS_DOUBLE };

    static char_T rt_ScopeBlockName[] = "TVC6DOF_sim/Scope2";
    static int_T rt_ScopeFrameData[] = { 0 };

    static RTWPreprocessingFcnPtr rt_ScopeSignalLoggingPreprocessingFcnPtrs[] =
      {
      (NULL)
    };

    rt_ScopeSignalInfo.numSignals = 1;
    rt_ScopeSignalInfo.numCols = rt_ScopeSignalWidths;
    rt_ScopeSignalInfo.numDims = rt_ScopeSignalNumDimensions;
    rt_ScopeSignalInfo.dims = rt_ScopeSignalDimensions;
    rt_ScopeSignalInfo.isVarDims = rt_ScopeSignalIsVarDims;
    rt_ScopeSignalInfo.currSigDims = rt_ScopeCurrSigDims;
    rt_ScopeSignalInfo.currSigDimsSize = rt_ScopeCurrSigDimsSize;
    rt_ScopeSignalInfo.dataTypes = dTypes;
    rt_ScopeSignalInfo.complexSignals = (NULL);
    rt_ScopeSignalInfo.frameData = rt_ScopeFrameData;
    rt_ScopeSignalInfo.preprocessingPtrs =
      rt_ScopeSignalLoggingPreprocessingFcnPtrs;
    rt_ScopeSignalInfo.labels.cptr = rt_ScopeSignalLabels;
    rt_ScopeSignalInfo.titles = rt_ScopeSignalTitles;
    rt_ScopeSignalInfo.titleLengths = rt_ScopeSignalTitleLengths;
    rt_ScopeSignalInfo.plotStyles = rt_ScopeSignalPlotStyles;
    rt_ScopeSignalInfo.blockNames.cptr = (NULL);
    rt_ScopeSignalInfo.stateNames.cptr = (NULL);
    rt_ScopeSignalInfo.crossMdlRef = (NULL);
    rt_ScopeSignalInfo.dataTypeConvert = (NULL);
    TVC6DOF_sim_DW.Scope2_PWORK.LoggedData = rt_CreateStructLogVar(
      TVC6DOF_sim_M->rtwLogInfo,
      0.0,
      rtmGetTFinal(TVC6DOF_sim_M),
      TVC6DOF_sim_M->Timing.stepSize0,
      (&rtmGetErrorStatus(TVC6DOF_sim_M)),
      "velocity",
      0,
      0,
      1,
      0.01,
      &rt_ScopeSignalInfo,
      rt_ScopeBlockName);
    if (TVC6DOF_sim_DW.Scope2_PWORK.LoggedData == (NULL))
      return;
  }

  /* SetupRuntimeResources for Scope: '<Root>/Scope3' */
  {
    RTWLogSignalInfo rt_ScopeSignalInfo;
    static int_T rt_ScopeSignalWidths[] = { 3 };

    static int_T rt_ScopeSignalNumDimensions[] = { 1 };

    static int_T rt_ScopeSignalDimensions[] = { 3 };

    static void *rt_ScopeCurrSigDims[] = { (NULL) };

    static int_T rt_ScopeCurrSigDimsSize[] = { 4 };

    static const char_T *rt_ScopeSignalLabels[] = { "" };

    static char_T rt_ScopeSignalTitles[] = "";
    static int_T rt_ScopeSignalTitleLengths[] = { 0 };

    static boolean_T rt_ScopeSignalIsVarDims[] = { 0 };

    static int_T rt_ScopeSignalPlotStyles[] = { 0, 0, 0 };

    BuiltInDTypeId dTypes[1] = { SS_DOUBLE };

    static char_T rt_ScopeBlockName[] = "TVC6DOF_sim/Scope3";
    static int_T rt_ScopeFrameData[] = { 0 };

    static RTWPreprocessingFcnPtr rt_ScopeSignalLoggingPreprocessingFcnPtrs[] =
      {
      (NULL)
    };

    rt_ScopeSignalInfo.numSignals = 1;
    rt_ScopeSignalInfo.numCols = rt_ScopeSignalWidths;
    rt_ScopeSignalInfo.numDims = rt_ScopeSignalNumDimensions;
    rt_ScopeSignalInfo.dims = rt_ScopeSignalDimensions;
    rt_ScopeSignalInfo.isVarDims = rt_ScopeSignalIsVarDims;
    rt_ScopeSignalInfo.currSigDims = rt_ScopeCurrSigDims;
    rt_ScopeSignalInfo.currSigDimsSize = rt_ScopeCurrSigDimsSize;
    rt_ScopeSignalInfo.dataTypes = dTypes;
    rt_ScopeSignalInfo.complexSignals = (NULL);
    rt_ScopeSignalInfo.frameData = rt_ScopeFrameData;
    rt_ScopeSignalInfo.preprocessingPtrs =
      rt_ScopeSignalLoggingPreprocessingFcnPtrs;
    rt_ScopeSignalInfo.labels.cptr = rt_ScopeSignalLabels;
    rt_ScopeSignalInfo.titles = rt_ScopeSignalTitles;
    rt_ScopeSignalInfo.titleLengths = rt_ScopeSignalTitleLengths;
    rt_ScopeSignalInfo.plotStyles = rt_ScopeSignalPlotStyles;
    rt_ScopeSignalInfo.blockNames.cptr = (NULL);
    rt_ScopeSignalInfo.stateNames.cptr = (NULL);
    rt_ScopeSignalInfo.crossMdlRef = (NULL);
    rt_ScopeSignalInfo.dataTypeConvert = (NULL);
    TVC6DOF_sim_DW.Scope3_PWORK.LoggedData = rt_CreateStructLogVar(
      TVC6DOF_sim_M->rtwLogInfo,
      0.0,
      rtmGetTFinal(TVC6DOF_sim_M),
      TVC6DOF_sim_M->Timing.stepSize0,
      (&rtmGetErrorStatus(TVC6DOF_sim_M)),
      "acceleration",
      0,
      0,
      1,
      0.01,
      &rt_ScopeSignalInfo,
      rt_ScopeBlockName);
    if (TVC6DOF_sim_DW.Scope3_PWORK.LoggedData == (NULL))
      return;
  }

  /* SetupRuntimeResources for Scope: '<Root>/Scope4' */
  {
    RTWLogSignalInfo rt_ScopeSignalInfo;
    static int_T rt_ScopeSignalWidths[] = { 3 };

    static int_T rt_ScopeSignalNumDimensions[] = { 1 };

    static int_T rt_ScopeSignalDimensions[] = { 3 };

    static void *rt_ScopeCurrSigDims[] = { (NULL) };

    static int_T rt_ScopeCurrSigDimsSize[] = { 4 };

    static const char_T *rt_ScopeSignalLabels[] = { "" };

    static char_T rt_ScopeSignalTitles[] = "";
    static int_T rt_ScopeSignalTitleLengths[] = { 0 };

    static boolean_T rt_ScopeSignalIsVarDims[] = { 0 };

    static int_T rt_ScopeSignalPlotStyles[] = { 0, 0, 0 };

    BuiltInDTypeId dTypes[1] = { SS_DOUBLE };

    static char_T rt_ScopeBlockName[] = "TVC6DOF_sim/Scope4";
    static int_T rt_ScopeFrameData[] = { 0 };

    static RTWPreprocessingFcnPtr rt_ScopeSignalLoggingPreprocessingFcnPtrs[] =
      {
      (NULL)
    };

    rt_ScopeSignalInfo.numSignals = 1;
    rt_ScopeSignalInfo.numCols = rt_ScopeSignalWidths;
    rt_ScopeSignalInfo.numDims = rt_ScopeSignalNumDimensions;
    rt_ScopeSignalInfo.dims = rt_ScopeSignalDimensions;
    rt_ScopeSignalInfo.isVarDims = rt_ScopeSignalIsVarDims;
    rt_ScopeSignalInfo.currSigDims = rt_ScopeCurrSigDims;
    rt_ScopeSignalInfo.currSigDimsSize = rt_ScopeCurrSigDimsSize;
    rt_ScopeSignalInfo.dataTypes = dTypes;
    rt_ScopeSignalInfo.complexSignals = (NULL);
    rt_ScopeSignalInfo.frameData = rt_ScopeFrameData;
    rt_ScopeSignalInfo.preprocessingPtrs =
      rt_ScopeSignalLoggingPreprocessingFcnPtrs;
    rt_ScopeSignalInfo.labels.cptr = rt_ScopeSignalLabels;
    rt_ScopeSignalInfo.titles = rt_ScopeSignalTitles;
    rt_ScopeSignalInfo.titleLengths = rt_ScopeSignalTitleLengths;
    rt_ScopeSignalInfo.plotStyles = rt_ScopeSignalPlotStyles;
    rt_ScopeSignalInfo.blockNames.cptr = (NULL);
    rt_ScopeSignalInfo.stateNames.cptr = (NULL);
    rt_ScopeSignalInfo.crossMdlRef = (NULL);
    rt_ScopeSignalInfo.dataTypeConvert = (NULL);
    TVC6DOF_sim_DW.Scope4_PWORK.LoggedData = rt_CreateStructLogVar(
      TVC6DOF_sim_M->rtwLogInfo,
      0.0,
      rtmGetTFinal(TVC6DOF_sim_M),
      TVC6DOF_sim_M->Timing.stepSize0,
      (&rtmGetErrorStatus(TVC6DOF_sim_M)),
      "angularVelocity",
      0,
      0,
      1,
      0.01,
      &rt_ScopeSignalInfo,
      rt_ScopeBlockName);
    if (TVC6DOF_sim_DW.Scope4_PWORK.LoggedData == (NULL))
      return;
  }

  /* SetupRuntimeResources for Scope: '<Root>/Scope5' */
  {
    RTWLogSignalInfo rt_ScopeSignalInfo;
    static int_T rt_ScopeSignalWidths[] = { 4 };

    static int_T rt_ScopeSignalNumDimensions[] = { 1 };

    static int_T rt_ScopeSignalDimensions[] = { 4 };

    static void *rt_ScopeCurrSigDims[] = { (NULL) };

    static int_T rt_ScopeCurrSigDimsSize[] = { 4 };

    static const char_T *rt_ScopeSignalLabels[] = { "" };

    static char_T rt_ScopeSignalTitles[] = "";
    static int_T rt_ScopeSignalTitleLengths[] = { 0 };

    static boolean_T rt_ScopeSignalIsVarDims[] = { 0 };

    static int_T rt_ScopeSignalPlotStyles[] = { 0, 0, 0, 0 };

    BuiltInDTypeId dTypes[1] = { SS_DOUBLE };

    static char_T rt_ScopeBlockName[] = "TVC6DOF_sim/Scope5";
    static int_T rt_ScopeFrameData[] = { 0 };

    static RTWPreprocessingFcnPtr rt_ScopeSignalLoggingPreprocessingFcnPtrs[] =
      {
      (NULL)
    };

    rt_ScopeSignalInfo.numSignals = 1;
    rt_ScopeSignalInfo.numCols = rt_ScopeSignalWidths;
    rt_ScopeSignalInfo.numDims = rt_ScopeSignalNumDimensions;
    rt_ScopeSignalInfo.dims = rt_ScopeSignalDimensions;
    rt_ScopeSignalInfo.isVarDims = rt_ScopeSignalIsVarDims;
    rt_ScopeSignalInfo.currSigDims = rt_ScopeCurrSigDims;
    rt_ScopeSignalInfo.currSigDimsSize = rt_ScopeCurrSigDimsSize;
    rt_ScopeSignalInfo.dataTypes = dTypes;
    rt_ScopeSignalInfo.complexSignals = (NULL);
    rt_ScopeSignalInfo.frameData = rt_ScopeFrameData;
    rt_ScopeSignalInfo.preprocessingPtrs =
      rt_ScopeSignalLoggingPreprocessingFcnPtrs;
    rt_ScopeSignalInfo.labels.cptr = rt_ScopeSignalLabels;
    rt_ScopeSignalInfo.titles = rt_ScopeSignalTitles;
    rt_ScopeSignalInfo.titleLengths = rt_ScopeSignalTitleLengths;
    rt_ScopeSignalInfo.plotStyles = rt_ScopeSignalPlotStyles;
    rt_ScopeSignalInfo.blockNames.cptr = (NULL);
    rt_ScopeSignalInfo.stateNames.cptr = (NULL);
    rt_ScopeSignalInfo.crossMdlRef = (NULL);
    rt_ScopeSignalInfo.dataTypeConvert = (NULL);
    TVC6DOF_sim_DW.Scope5_PWORK.LoggedData = rt_CreateStructLogVar(
      TVC6DOF_sim_M->rtwLogInfo,
      0.0,
      rtmGetTFinal(TVC6DOF_sim_M),
      TVC6DOF_sim_M->Timing.stepSize0,
      (&rtmGetErrorStatus(TVC6DOF_sim_M)),
      "quaternion",
      0,
      0,
      1,
      0.01,
      &rt_ScopeSignalInfo,
      rt_ScopeBlockName);
    if (TVC6DOF_sim_DW.Scope5_PWORK.LoggedData == (NULL))
      return;
  }

  /* Start for FromWorkspace: '<S9>/FromWs' */
  {
    static real_T pTimeValues0[] = { 0.0, 0.148, 0.148, 0.228, 0.228, 0.294,
      0.294, 0.419, 0.419, 0.477, 0.477, 0.52, 0.52, 0.593, 0.593, 0.688, 0.688,
      0.855, 0.855, 1.037, 1.037, 1.205, 1.205, 1.423, 1.423, 1.452, 1.452,
      1.503, 1.503, 1.736, 1.736, 1.955, 1.955, 2.21, 2.21, 2.494, 2.494, 2.763,
      2.763, 3.12, 3.12, 3.382, 3.382, 3.404, 3.404, 3.418, 3.418, 3.45, 3.45,
      9.8, 10.0 } ;

    static real_T pDataValues0[] = { 60.0, 59.3164, 59.3164, 58.3541, 58.3541,
      57.2108, 57.2108, 54.0771, 54.0771, 52.3818, 52.3818, 51.2397, 51.2397,
      49.4767, 49.4767, 47.3744, 47.3744, 43.9685, 43.9685, 40.4849, 40.4849,
      37.3989, 37.3989, 33.5, 33.5, 32.9674, 32.9674, 32.026, 32.026, 27.8823,
      27.8823, 24.0514, 24.0514, 19.6652, 19.6652, 14.8632, 14.8632, 10.4455,
      10.4455, 4.68731, 4.68731, 0.51289, 0.51289, 0.215344, 0.215344, 0.0947253,
      0.0947253, 0.0, 0.0, 0.0, 0.0 } ;

    TVC6DOF_sim_DW.FromWs_PWORK.TimePtr = (void *) pTimeValues0;
    TVC6DOF_sim_DW.FromWs_PWORK.DataPtr = (void *) pDataValues0;
    TVC6DOF_sim_DW.FromWs_IWORK.PrevIndex = 0;
  }

  /* Start for FromWorkspace: '<S5>/FromWs' */
  {
    static real_T pTimeValues0[] = { 0.0, 0.148, 0.148, 0.228, 0.228, 0.294,
      0.294, 0.353, 0.353, 0.382, 0.382, 0.419, 0.419, 0.477, 0.477, 0.52, 0.52,
      0.593, 0.593, 0.688, 0.688, 0.855, 0.855, 1.037, 1.037, 1.205, 1.205,
      1.423, 1.423, 1.452, 1.452, 1.503, 1.503, 1.736, 1.736, 1.955, 1.955, 2.21,
      2.21, 2.494, 2.494, 2.763, 2.763, 3.12, 3.12, 3.382, 3.382, 3.404, 3.404,
      3.418, 3.418, 3.45, 3.45, 10.0 } ;

    static real_T pDataValues0[] = { 0.0, 7.638, 7.638, 12.253, 12.253, 16.391,
      16.391, 20.21, 20.21, 22.756, 22.756, 25.26, 25.26, 23.074, 23.074, 20.845,
      20.845, 19.093, 19.093, 17.5, 17.5, 16.225, 16.225, 15.427, 15.427, 14.948,
      14.948, 14.627, 14.627, 15.741, 15.741, 14.785, 14.785, 14.623, 14.623,
      14.303, 14.303, 14.141, 14.141, 13.819, 13.819, 13.338, 13.338, 13.334,
      13.334, 13.013, 13.013, 9.352, 9.352, 4.895, 4.895, 0.0, 0.0, 0.0 } ;

    TVC6DOF_sim_DW.FromWs_PWORK_a.TimePtr = (void *) pTimeValues0;
    TVC6DOF_sim_DW.FromWs_PWORK_a.DataPtr = (void *) pDataValues0;
    TVC6DOF_sim_DW.FromWs_IWORK_a.PrevIndex = 0;
  }

  TVC6DOF_sim_PrevZCX.HitCrossing_Input_ZCE = UNINITIALIZED_ZCSIG;

  {
    real_T tmp;
    int32_T r;
    int32_T t;
    uint32_T tseed;

    /* InitializeConditions for Integrator: '<S3>/Integrator1' */
    TVC6DOF_sim_X.Integrator1_CSTATE[0] = TVC6DOF_sim_P.Integrator1_IC[0];

    /* InitializeConditions for Integrator: '<S3>/Integrator' */
    TVC6DOF_sim_X.Integrator_CSTATE[0] = TVC6DOF_sim_P.Integrator_IC[0];

    /* InitializeConditions for Integrator: '<S3>/Integrator1' */
    TVC6DOF_sim_X.Integrator1_CSTATE[1] = TVC6DOF_sim_P.Integrator1_IC[1];

    /* InitializeConditions for Integrator: '<S3>/Integrator' */
    TVC6DOF_sim_X.Integrator_CSTATE[1] = TVC6DOF_sim_P.Integrator_IC[1];

    /* InitializeConditions for Integrator: '<S3>/Integrator1' */
    TVC6DOF_sim_X.Integrator1_CSTATE[2] = TVC6DOF_sim_P.Integrator1_IC[2];

    /* InitializeConditions for Integrator: '<S3>/Integrator' */
    TVC6DOF_sim_X.Integrator_CSTATE[2] = TVC6DOF_sim_P.Integrator_IC[2];

    /* InitializeConditions for Delay: '<Root>/Actuator Delay' */
    TVC6DOF_sim_DW.ActuatorDelay_DSTATE =
      TVC6DOF_sim_P.ActuatorDelay_InitialCondition;

    /* InitializeConditions for Delay: '<Root>/Actuator Delay2' */
    TVC6DOF_sim_DW.ActuatorDelay2_DSTATE =
      TVC6DOF_sim_P.ActuatorDelay2_InitialCondition;

    /* InitializeConditions for Integrator: '<S3>/Integrator6' */
    TVC6DOF_sim_X.Integrator6_CSTATE[0] = TVC6DOF_sim_P.Integrator6_IC[0];
    TVC6DOF_sim_X.Integrator6_CSTATE[1] = TVC6DOF_sim_P.Integrator6_IC[1];
    TVC6DOF_sim_X.Integrator6_CSTATE[2] = TVC6DOF_sim_P.Integrator6_IC[2];
    TVC6DOF_sim_X.Integrator6_CSTATE[3] = TVC6DOF_sim_P.Integrator6_IC[3];

    /* InitializeConditions for Integrator: '<S3>/Integrator3' */
    TVC6DOF_sim_X.Integrator3_CSTATE[0] = TVC6DOF_sim_P.Integrator3_IC[0];
    TVC6DOF_sim_X.Integrator3_CSTATE[1] = TVC6DOF_sim_P.Integrator3_IC[1];
    TVC6DOF_sim_X.Integrator3_CSTATE[2] = TVC6DOF_sim_P.Integrator3_IC[2];

    /* InitializeConditions for RandomNumber: '<S1>/Random Number' */
    tmp = floor(TVC6DOF_sim_P.RandomNumber_Seed);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? (uint32_T)-(int32_T)(uint32_T)-tmp : (uint32_T)tmp;
    r = (int32_T)(tseed >> 16U);
    t = (int32_T)(tseed & 32768U);
    tseed = ((((tseed - ((uint32_T)r << 16U)) + t) << 16U) + t) + r;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    TVC6DOF_sim_DW.RandSeed = tseed;
    TVC6DOF_sim_DW.NextOutput = rt_nrand_Upu32_Yd_f_pw_snf
      (&TVC6DOF_sim_DW.RandSeed) * TVC6DOF_sim_P.RandomNumber_StdDev +
      TVC6DOF_sim_P.RandomNumber_Mean;

    /* End of InitializeConditions for RandomNumber: '<S1>/Random Number' */

    /* InitializeConditions for Integrator: '<S2>/Integrator' */
    TVC6DOF_sim_X.Integrator_CSTATE_b = TVC6DOF_sim_P.Integrator_IC_m;

    /* InitializeConditions for Integrator: '<S2>/Integrator1' */
    TVC6DOF_sim_X.Integrator1_CSTATE_n = TVC6DOF_sim_P.Integrator1_IC_o;

    /* InitializeConditions for Integrator: '<S2>/Integrator2' */
    TVC6DOF_sim_X.Integrator2_CSTATE = TVC6DOF_sim_P.Integrator2_IC;

    /* SystemInitialize for MATLAB Function: '<S1>/MATLAB Function' */
    TVC6DOF_sim_DW.method_not_empty = false;
    TVC6DOF_sim_DW.state_not_empty = false;
    TVC6DOF_sim_DW.method_i = 7U;
    TVC6DOF_sim_DW.state_p = 1144108930U;
    TVC6DOF_sim_DW.state_f[0] = 362436069U;
    TVC6DOF_sim_DW.state_f[1] = 521288629U;
  }
}

/* Model terminate function */
void TVC6DOF_sim_terminate(void)
{
  /* (no terminate code required) */
}

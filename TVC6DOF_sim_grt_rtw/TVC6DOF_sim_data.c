/*
 * TVC6DOF_sim_data.c
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

/* Block parameters (default storage) */
P_TVC6DOF_sim_T TVC6DOF_sim_P = {
  /* Variable: kd
   * Referenced by:
   *   '<S2>/Gain3'
   *   '<S2>/Gain6'
   *   '<S2>/Gain9'
   */
  0.1,

  /* Variable: ki
   * Referenced by:
   *   '<S2>/Gain2'
   *   '<S2>/Gain5'
   *   '<S2>/Gain8'
   */
  1.0,

  /* Variable: kp
   * Referenced by:
   *   '<S2>/Gain1'
   *   '<S2>/Gain4'
   *   '<S2>/Gain7'
   */
  1.0,

  /* Variable: m_dry_n
   * Referenced by: '<S4>/Constant'
   */
  0.94,

  /* Variable: static_error_y_mc
   * Referenced by: '<S2>/Constant1'
   */
  0.0,

  /* Variable: static_error_z_mc
   * Referenced by: '<S2>/Constant2'
   */
  0.0,

  /* Expression: [0 0 0]
   * Referenced by: '<S3>/Integrator1'
   */
  { 0.0, 0.0, 0.0 },

  /* Expression: [0 0 0]
   * Referenced by: '<S3>/Integrator'
   */
  { 0.0, 0.0, 0.0 },

  /* Expression: [Ix_mc Iy_mc Iz_mc g l_mc mis_y_mc mis_z_mc m_dry_mc]
   * Referenced by: '<S3>/Constant'
   */
  { 0.1, 0.1, 0.1, 9.81, 0.3, 0.0, 0.0, 0.94 },

  /* Expression: 0
   * Referenced by: '<Root>/Actuator Delay'
   */
  0.0,

  /* Expression: 10
   * Referenced by: '<Root>/Saturation'
   */
  10.0,

  /* Expression: -10
   * Referenced by: '<Root>/Saturation'
   */
  -10.0,

  /* Expression: 0
   * Referenced by: '<Root>/Actuator Delay2'
   */
  0.0,

  /* Expression: 10
   * Referenced by: '<Root>/Saturation1'
   */
  10.0,

  /* Expression: -10
   * Referenced by: '<Root>/Saturation1'
   */
  -10.0,

  /* Expression: [1 0 0 0]
   * Referenced by: '<S3>/Integrator6'
   */
  { 1.0, 0.0, 0.0, 0.0 },

  /* Expression: [0 0 0]
   * Referenced by: '<S3>/Integrator3'
   */
  { 0.0, 0.0, 0.0 },

  /* Expression: 0.01
   * Referenced by: '<S1>/Random Number'
   */
  0.01,

  /* Computed Parameter: RandomNumber_StdDev
   * Referenced by: '<S1>/Random Number'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<S1>/Random Number'
   */
  0.0,

  /* Expression: -1
   * Referenced by: '<S2>/Gain10'
   */
  -1.0,

  /* Expression: [0 1 0 0;0 0 1 0;0 0 0 1]
   * Referenced by: '<S2>/Gain'
   */
  { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 },

  /* Expression: -1
   * Referenced by: '<S2>/Gain11'
   */
  -1.0,

  /* Expression: [0]
   * Referenced by: '<S2>/Integrator'
   */
  0.0,

  /* Expression: [0]
   * Referenced by: '<S2>/Integrator1'
   */
  0.0,

  /* Expression: [0]
   * Referenced by: '<S2>/Integrator2'
   */
  0.0,

  /* Expression: [Ix_n Iy_n Iz_n l_n g]
   * Referenced by: '<S2>/Constant'
   */
  { 0.1, 0.1, 0.1, 0.3, 9.81 },

  /* Expression: 0.01
   * Referenced by: '<S3>/Hit  Crossing'
   */
  0.01
};

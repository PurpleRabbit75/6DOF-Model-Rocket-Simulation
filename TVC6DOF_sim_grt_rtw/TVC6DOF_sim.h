/*
 * TVC6DOF_sim.h
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

#ifndef RTW_HEADER_TVC6DOF_sim_h_
#define RTW_HEADER_TVC6DOF_sim_h_
#include <math.h>
#include <stddef.h>
#include <float.h>
#include <string.h>
#ifndef TVC6DOF_sim_COMMON_INCLUDES_
#define TVC6DOF_sim_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#endif                                 /* TVC6DOF_sim_COMMON_INCLUDES_ */

#include "TVC6DOF_sim_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rt_nonfinite.h"
#include "rt_zcfcn.h"
#include "rtGetInf.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContStateDisabled
#define rtmGetContStateDisabled(rtm)   ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
#define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
#define rtmGetContStates(rtm)          ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
#define rtmSetContStates(rtm, val)     ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
#define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
#define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetIntgData
#define rtmGetIntgData(rtm)            ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
#define rtmSetIntgData(rtm, val)       ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
#define rtmGetOdeF(rtm)                ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
#define rtmSetOdeF(rtm, val)           ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
#define rtmGetOdeY(rtm)                ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
#define rtmSetOdeY(rtm, val)           ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
#define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
#define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
#define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
#define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetRTWLogInfo
#define rtmGetRTWLogInfo(rtm)          ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetZCCacheNeedsReset
#define rtmGetZCCacheNeedsReset(rtm)   ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
#define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
#define rtmGetdX(rtm)                  ((rtm)->derivs)
#endif

#ifndef rtmSetdX
#define rtmSetdX(rtm, val)             ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

/* Block signals (default storage) */
typedef struct {
  real_T Integrator1[3];               /* '<S3>/Integrator1' */
  real_T Integrator[3];                /* '<S3>/Integrator' */
  real_T FromWs;                       /* '<S9>/FromWs' */
  real_T ActuatorDelay;                /* '<Root>/Actuator Delay' */
  real_T Saturation;                   /* '<Root>/Saturation' */
  real_T ActuatorDelay2;               /* '<Root>/Actuator Delay2' */
  real_T Saturation1;                  /* '<Root>/Saturation1' */
  real_T FromWs_i;                     /* '<S5>/FromWs' */
  real_T Integrator6[4];               /* '<S3>/Integrator6' */
  real_T Integrator3[3];               /* '<S3>/Integrator3' */
  real_T ZeroOrderHold[3];             /* '<S1>/Zero-Order Hold' */
  real_T RandomNumber;                 /* '<S1>/Random Number' */
  real_T Add[3];                       /* '<S1>/Add' */
  real_T ZeroOrderHold1[3];            /* '<S1>/Zero-Order Hold1' */
  real_T ZeroOrderHold2[4];            /* '<S1>/Zero-Order Hold2' */
  real_T Gain10[3];                    /* '<S2>/Gain10' */
  real_T Gain[3];                      /* '<S2>/Gain' */
  real_T Gain11[3];                    /* '<S2>/Gain11' */
  real_T Gain1;                        /* '<S2>/Gain1' */
  real_T Integrator_i;                 /* '<S2>/Integrator' */
  real_T Gain2;                        /* '<S2>/Gain2' */
  real_T Gain3;                        /* '<S2>/Gain3' */
  real_T Sum;                          /* '<S2>/Sum' */
  real_T Gain4;                        /* '<S2>/Gain4' */
  real_T Integrator1_f;                /* '<S2>/Integrator1' */
  real_T Gain5;                        /* '<S2>/Gain5' */
  real_T Gain6;                        /* '<S2>/Gain6' */
  real_T Sum1;                         /* '<S2>/Sum1' */
  real_T Gain7;                        /* '<S2>/Gain7' */
  real_T Integrator2;                  /* '<S2>/Integrator2' */
  real_T Gain8;                        /* '<S2>/Gain8' */
  real_T Gain9;                        /* '<S2>/Gain9' */
  real_T Sum2;                         /* '<S2>/Sum2' */
  real_T Add_c;                        /* '<S2>/Add' */
  real_T Add1;                         /* '<S2>/Add1' */
  real_T w_est[3];                     /* '<S4>/MATLAB Function' */
  real_T q_est[4];                     /* '<S4>/MATLAB Function' */
  real_T T_est;                        /* '<S4>/MATLAB Function' */
  real_T rdd[3];                       /* '<S3>/Equations of Motion' */
  real_T wd[3];                        /* '<S3>/Equations of Motion' */
  real_T qd[4];                        /* '<S3>/Equations of Motion' */
  real_T Mb[3];                        /* '<S3>/Equations of Motion' */
  real_T Tb[3];                        /* '<S3>/Equations of Motion' */
  real_T th_y_com;                     /* '<S2>/MATLAB Function' */
  real_T th_z_com;                     /* '<S2>/MATLAB Function' */
  real_T q_n[4];                       /* '<S1>/MATLAB Function' */
  boolean_T HitCrossing;               /* '<S3>/Hit  Crossing' */
} B_TVC6DOF_sim_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T ActuatorDelay_DSTATE;         /* '<Root>/Actuator Delay' */
  real_T ActuatorDelay2_DSTATE;        /* '<Root>/Actuator Delay2' */
  real_T NextOutput;                   /* '<S1>/Random Number' */
  struct {
    void *LoggedData;
  } Scope1_PWORK;                      /* '<Root>/Scope1' */

  struct {
    void *LoggedData;
  } Scope2_PWORK;                      /* '<Root>/Scope2' */

  struct {
    void *TimePtr;
    void *DataPtr;
    void *RSimInfoPtr;
  } FromWs_PWORK;                      /* '<S9>/FromWs' */

  struct {
    void *TimePtr;
    void *DataPtr;
    void *RSimInfoPtr;
  } FromWs_PWORK_a;                    /* '<S5>/FromWs' */

  struct {
    void *LoggedData;
  } Scope3_PWORK;                      /* '<Root>/Scope3' */

  struct {
    void *LoggedData;
  } Scope4_PWORK;                      /* '<Root>/Scope4' */

  struct {
    void *LoggedData;
  } Scope5_PWORK;                      /* '<Root>/Scope5' */

  uint32_T RandSeed;                   /* '<S1>/Random Number' */
  uint32_T method;                     /* '<S1>/MATLAB Function' */
  uint32_T state[2];                   /* '<S1>/MATLAB Function' */
  uint32_T method_i;                   /* '<S1>/MATLAB Function' */
  uint32_T state_p;                    /* '<S1>/MATLAB Function' */
  uint32_T state_f[2];                 /* '<S1>/MATLAB Function' */
  uint32_T state_d[625];               /* '<S1>/MATLAB Function' */
  struct {
    int_T PrevIndex;
  } FromWs_IWORK;                      /* '<S9>/FromWs' */

  struct {
    int_T PrevIndex;
  } FromWs_IWORK_a;                    /* '<S5>/FromWs' */

  int_T HitCrossing_MODE;              /* '<S3>/Hit  Crossing' */
  boolean_T method_not_empty;          /* '<S1>/MATLAB Function' */
  boolean_T state_not_empty;           /* '<S1>/MATLAB Function' */
} DW_TVC6DOF_sim_T;

/* Continuous states (default storage) */
typedef struct {
  real_T Integrator1_CSTATE[3];        /* '<S3>/Integrator1' */
  real_T Integrator_CSTATE[3];         /* '<S3>/Integrator' */
  real_T Integrator6_CSTATE[4];        /* '<S3>/Integrator6' */
  real_T Integrator3_CSTATE[3];        /* '<S3>/Integrator3' */
  real_T Integrator_CSTATE_b;          /* '<S2>/Integrator' */
  real_T Integrator1_CSTATE_n;         /* '<S2>/Integrator1' */
  real_T Integrator2_CSTATE;           /* '<S2>/Integrator2' */
} X_TVC6DOF_sim_T;

/* State derivatives (default storage) */
typedef struct {
  real_T Integrator1_CSTATE[3];        /* '<S3>/Integrator1' */
  real_T Integrator_CSTATE[3];         /* '<S3>/Integrator' */
  real_T Integrator6_CSTATE[4];        /* '<S3>/Integrator6' */
  real_T Integrator3_CSTATE[3];        /* '<S3>/Integrator3' */
  real_T Integrator_CSTATE_b;          /* '<S2>/Integrator' */
  real_T Integrator1_CSTATE_n;         /* '<S2>/Integrator1' */
  real_T Integrator2_CSTATE;           /* '<S2>/Integrator2' */
} XDot_TVC6DOF_sim_T;

/* State disabled  */
typedef struct {
  boolean_T Integrator1_CSTATE[3];     /* '<S3>/Integrator1' */
  boolean_T Integrator_CSTATE[3];      /* '<S3>/Integrator' */
  boolean_T Integrator6_CSTATE[4];     /* '<S3>/Integrator6' */
  boolean_T Integrator3_CSTATE[3];     /* '<S3>/Integrator3' */
  boolean_T Integrator_CSTATE_b;       /* '<S2>/Integrator' */
  boolean_T Integrator1_CSTATE_n;      /* '<S2>/Integrator1' */
  boolean_T Integrator2_CSTATE;        /* '<S2>/Integrator2' */
} XDis_TVC6DOF_sim_T;

/* Zero-crossing (trigger) state */
typedef struct {
  ZCSigState HitCrossing_Input_ZCE;    /* '<S3>/Hit  Crossing' */
} PrevZCX_TVC6DOF_sim_T;

#ifndef ODE3_INTG
#define ODE3_INTG

/* ODE3 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[3];                        /* derivatives */
} ODE3_IntgData;

#endif

/* Parameters (default storage) */
struct P_TVC6DOF_sim_T_ {
  real_T kd;                           /* Variable: kd
                                        * Referenced by:
                                        *   '<S2>/Gain3'
                                        *   '<S2>/Gain6'
                                        *   '<S2>/Gain9'
                                        */
  real_T ki;                           /* Variable: ki
                                        * Referenced by:
                                        *   '<S2>/Gain2'
                                        *   '<S2>/Gain5'
                                        *   '<S2>/Gain8'
                                        */
  real_T kp;                           /* Variable: kp
                                        * Referenced by:
                                        *   '<S2>/Gain1'
                                        *   '<S2>/Gain4'
                                        *   '<S2>/Gain7'
                                        */
  real_T m_dry_n;                      /* Variable: m_dry_n
                                        * Referenced by: '<S4>/Constant'
                                        */
  real_T static_error_y_mc;            /* Variable: static_error_y_mc
                                        * Referenced by: '<S2>/Constant1'
                                        */
  real_T static_error_z_mc;            /* Variable: static_error_z_mc
                                        * Referenced by: '<S2>/Constant2'
                                        */
  real_T Integrator1_IC[3];            /* Expression: [0 0 0]
                                        * Referenced by: '<S3>/Integrator1'
                                        */
  real_T Integrator_IC[3];             /* Expression: [0 0 0]
                                        * Referenced by: '<S3>/Integrator'
                                        */
  real_T Constant_Value[8];
            /* Expression: [Ix_mc Iy_mc Iz_mc g l_mc mis_y_mc mis_z_mc m_dry_mc]
             * Referenced by: '<S3>/Constant'
             */
  real_T ActuatorDelay_InitialCondition;/* Expression: 0
                                         * Referenced by: '<Root>/Actuator Delay'
                                         */
  real_T Saturation_UpperSat;          /* Expression: 10
                                        * Referenced by: '<Root>/Saturation'
                                        */
  real_T Saturation_LowerSat;          /* Expression: -10
                                        * Referenced by: '<Root>/Saturation'
                                        */
  real_T ActuatorDelay2_InitialCondition;/* Expression: 0
                                          * Referenced by: '<Root>/Actuator Delay2'
                                          */
  real_T Saturation1_UpperSat;         /* Expression: 10
                                        * Referenced by: '<Root>/Saturation1'
                                        */
  real_T Saturation1_LowerSat;         /* Expression: -10
                                        * Referenced by: '<Root>/Saturation1'
                                        */
  real_T Integrator6_IC[4];            /* Expression: [1 0 0 0]
                                        * Referenced by: '<S3>/Integrator6'
                                        */
  real_T Integrator3_IC[3];            /* Expression: [0 0 0]
                                        * Referenced by: '<S3>/Integrator3'
                                        */
  real_T RandomNumber_Mean;            /* Expression: 0.01
                                        * Referenced by: '<S1>/Random Number'
                                        */
  real_T RandomNumber_StdDev;         /* Computed Parameter: RandomNumber_StdDev
                                       * Referenced by: '<S1>/Random Number'
                                       */
  real_T RandomNumber_Seed;            /* Expression: 0
                                        * Referenced by: '<S1>/Random Number'
                                        */
  real_T Gain10_Gain;                  /* Expression: -1
                                        * Referenced by: '<S2>/Gain10'
                                        */
  real_T Gain_Gain[12];                /* Expression: [0 1 0 0;0 0 1 0;0 0 0 1]
                                        * Referenced by: '<S2>/Gain'
                                        */
  real_T Gain11_Gain;                  /* Expression: -1
                                        * Referenced by: '<S2>/Gain11'
                                        */
  real_T Integrator_IC_m;              /* Expression: [0]
                                        * Referenced by: '<S2>/Integrator'
                                        */
  real_T Integrator1_IC_o;             /* Expression: [0]
                                        * Referenced by: '<S2>/Integrator1'
                                        */
  real_T Integrator2_IC;               /* Expression: [0]
                                        * Referenced by: '<S2>/Integrator2'
                                        */
  real_T Constant_Value_a[5];          /* Expression: [Ix_n Iy_n Iz_n l_n g]
                                        * Referenced by: '<S2>/Constant'
                                        */
  real_T HitCrossing_Offset;           /* Expression: 0.01
                                        * Referenced by: '<S3>/Hit  Crossing'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_TVC6DOF_sim_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;
  RTWSolverInfo solverInfo;
  X_TVC6DOF_sim_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[16];
  real_T odeF[3][16];
  ODE3_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    struct {
      uint8_T TID[3];
    } TaskCounters;

    time_T tFinal;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[3];
  } Timing;
};

/* Block parameters (default storage) */
extern P_TVC6DOF_sim_T TVC6DOF_sim_P;

/* Block signals (default storage) */
extern B_TVC6DOF_sim_T TVC6DOF_sim_B;

/* Continuous states (default storage) */
extern X_TVC6DOF_sim_T TVC6DOF_sim_X;

/* Block states (default storage) */
extern DW_TVC6DOF_sim_T TVC6DOF_sim_DW;

/* Zero-crossing (trigger) state */
extern PrevZCX_TVC6DOF_sim_T TVC6DOF_sim_PrevZCX;

/* Model entry point functions */
extern void TVC6DOF_sim_initialize(void);
extern void TVC6DOF_sim_step(void);
extern void TVC6DOF_sim_terminate(void);

/* Real-time Model object */
extern RT_MODEL_TVC6DOF_sim_T *const TVC6DOF_sim_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'TVC6DOF_sim'
 * '<S1>'   : 'TVC6DOF_sim/Avionics'
 * '<S2>'   : 'TVC6DOF_sim/Controller'
 * '<S3>'   : 'TVC6DOF_sim/Dynamics Model'
 * '<S4>'   : 'TVC6DOF_sim/Estimator'
 * '<S5>'   : 'TVC6DOF_sim/Thrust Curve'
 * '<S6>'   : 'TVC6DOF_sim/Avionics/MATLAB Function'
 * '<S7>'   : 'TVC6DOF_sim/Controller/MATLAB Function'
 * '<S8>'   : 'TVC6DOF_sim/Dynamics Model/Equations of Motion'
 * '<S9>'   : 'TVC6DOF_sim/Dynamics Model/Signal Builder'
 * '<S10>'  : 'TVC6DOF_sim/Estimator/MATLAB Function'
 */
#endif                                 /* RTW_HEADER_TVC6DOF_sim_h_ */

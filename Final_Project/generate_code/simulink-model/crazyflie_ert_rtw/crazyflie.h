/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: crazyflie.h
 *
 * Code generated for Simulink model 'crazyflie'.
 *
 * Model version                  : 9.6
 * Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
 * C/C++ source code generated on : Thu May  2 11:48:02 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_crazyflie_h_
#define RTW_HEADER_crazyflie_h_
#ifndef crazyflie_COMMON_INCLUDES_
#define crazyflie_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* crazyflie_COMMON_INCLUDES_ */

/* Forward declaration for rtModel */
typedef struct tag_RTM_crazyflie_T RT_MODEL_crazyflie_T;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T Delay_DSTATE;                 /* '<S2>/Delay' */
  real_T Delay_DSTATE_p;               /* '<S3>/Delay' */
} DW_crazyflie_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: -K
   * Referenced by: '<Root>/Gain'
   */
  real_T Gain_Gain[20];
} ConstP_crazyflie_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T Base_Thrust;                  /* '<Root>/Base_Thrust' */
  real_T Ref_Roll;                     /* '<Root>/Ref_Roll' */
  real_T Ref_Pitch;                    /* '<Root>/Ref_Pitch' */
  real_T Ref_YawRate;                  /* '<Root>/Ref_YawRate' */
  real_T Acc_x;                        /* '<Root>/Acc_x' */
  real_T Acc_y;                        /* '<Root>/Acc_y' */
  real_T Acc_z;                        /* '<Root>/Acc_z' */
  real_T Gyro_x;                       /* '<Root>/Gyro_x' */
  real_T Gyro_y;                       /* '<Root>/Gyro_y' */
  real_T Gyro_z;                       /* '<Root>/Gyro_z' */
  int16_T Flow_x;                      /* '<Root>/Flow_x' */
  int16_T Flow_y;                      /* '<Root>/Flow_y' */
} ExtU_crazyflie_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  uint16_T Motor_1;                    /* '<Root>/Motor_1' */
  uint16_T Motor_2;                    /* '<Root>/Motor_2' */
  uint16_T Motor_3;                    /* '<Root>/Motor_3' */
  uint16_T Motor_4;                    /* '<Root>/Motor_4' */
  real_T Log1;                         /* '<Root>/Log1' */
  real_T Log2;                         /* '<Root>/Log2' */
  real_T Log3;                         /* '<Root>/Log3' */
  real_T Log4;                         /* '<Root>/Log4' */
  real_T Log5;                         /* '<Root>/Log5' */
  real_T Log6;                         /* '<Root>/Log6' */
} ExtY_crazyflie_T;

/* Real-time Model Data Structure */
struct tag_RTM_crazyflie_T {
  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    struct {
      uint8_T TID[2];
    } TaskCounters;
  } Timing;
};

/* Block signals and states (default storage) */
extern DW_crazyflie_T crazyflie_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_crazyflie_T crazyflie_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_crazyflie_T crazyflie_Y;

/* Constant parameters (default storage) */
extern const ConstP_crazyflie_T crazyflie_ConstP;

/* Model entry point functions */
extern void crazyflie_initialize(void);
extern void crazyflie_step(void);
extern void crazyflie_terminate(void);

/* Real-time Model object */
extern RT_MODEL_crazyflie_T *const crazyflie_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S1>/Discrete-Time Integrator3' : Unused code path elimination
 * Block '<S1>/Discrete-Time Integrator4' : Unused code path elimination
 * Block '<S1>/Discrete-Time Integrator5' : Unused code path elimination
 * Block '<S1>/Pitch_Gyro1' : Unused code path elimination
 * Block '<S1>/Roll_Gyro1' : Unused code path elimination
 * Block '<Root>/ToDouble' : Eliminate redundant data type conversion
 * Block '<Root>/ToDouble1' : Eliminate redundant data type conversion
 * Block '<Root>/ToDouble2' : Eliminate redundant data type conversion
 * Block '<Root>/ToDouble3' : Eliminate redundant data type conversion
 * Block '<Root>/ToDouble4' : Eliminate redundant data type conversion
 * Block '<Root>/ToDouble5' : Eliminate redundant data type conversion
 */

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
 * '<Root>' : 'crazyflie'
 * '<S1>'   : 'crazyflie/Subsystem'
 * '<S2>'   : 'crazyflie/Subsystem/ComplementaryFilter2'
 * '<S3>'   : 'crazyflie/Subsystem/ComplementaryFilter3'
 * '<S4>'   : 'crazyflie/Subsystem/MATLAB Function1'
 */
#endif                                 /* RTW_HEADER_crazyflie_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

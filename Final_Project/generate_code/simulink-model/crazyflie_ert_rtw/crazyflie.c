/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: crazyflie.c
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

#include "crazyflie.h"
#include <math.h>
#include "rtwtypes.h"

/* Block signals and states (default storage) */
DW_crazyflie_T crazyflie_DW;

/* External inputs (root inport signals with default storage) */
ExtU_crazyflie_T crazyflie_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_crazyflie_T crazyflie_Y;

/* Real-time model */
static RT_MODEL_crazyflie_T crazyflie_M_;
RT_MODEL_crazyflie_T *const crazyflie_M = &crazyflie_M_;
static void rate_scheduler(void);

/*
 *         This function updates active task flag for each subrate.
 *         The function is called at model base rate, hence the
 *         generated code self-manages all its subrates.
 */
static void rate_scheduler(void)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (crazyflie_M->Timing.TaskCounters.TID[1])++;
  if ((crazyflie_M->Timing.TaskCounters.TID[1]) > 9) {/* Sample time: [0.01s, 0.0s] */
    crazyflie_M->Timing.TaskCounters.TID[1] = 0;
  }
}

/* Model step function */
void crazyflie_step(void)
{
  real_T tmp[5];
  real_T rtb_Sum3_m[4];
  real_T rtb_Sum3;
  real_T rtb_Sum3_l;
  real_T tmp_0;
  int32_T i;
  int32_T i_0;
  if (crazyflie_M->Timing.TaskCounters.TID[1] == 0) {
    /* Sum: '<S2>/Sum3' incorporates:
     *  Constant: '<S2>/Gamma'
     *  Constant: '<S2>/SamplingRate'
     *  Delay: '<S2>/Delay'
     *  Inport: '<Root>/Acc_y'
     *  Inport: '<Root>/Acc_z'
     *  Inport: '<Root>/Gyro_x'
     *  Product: '<S2>/Product'
     *  Product: '<S2>/Product1'
     *  Product: '<S2>/Product2'
     *  Sum: '<S2>/Sum2'
     *  Trigonometry: '<S1>/Atan4'
     */
    rtb_Sum3 = (0.01 * crazyflie_U.Gyro_x + crazyflie_DW.Delay_DSTATE) * 0.1 +
      atan2(crazyflie_U.Acc_y, crazyflie_U.Acc_z) * 0.9;

    /* Sum: '<S3>/Sum3' incorporates:
     *  Constant: '<S3>/Gamma'
     *  Constant: '<S3>/SamplingRate'
     *  Delay: '<S3>/Delay'
     *  Gain: '<S1>/Gain1'
     *  Inport: '<Root>/Acc_x'
     *  Inport: '<Root>/Acc_y'
     *  Inport: '<Root>/Acc_z'
     *  Inport: '<Root>/Gyro_y'
     *  MATLAB Function: '<S1>/MATLAB Function1'
     *  Product: '<S3>/Product'
     *  Product: '<S3>/Product1'
     *  Product: '<S3>/Product2'
     *  Sum: '<S3>/Sum2'
     *  Trigonometry: '<S1>/Atan3'
     */
    /* MATLAB Function 'Subsystem/MATLAB Function1': '<S4>:1' */
    /* '<S4>:1:3' y = sqrt(u*u+v*v); */
    rtb_Sum3_l = atan2(-crazyflie_U.Acc_x, sqrt(crazyflie_U.Acc_y *
      crazyflie_U.Acc_y + crazyflie_U.Acc_z * crazyflie_U.Acc_z)) * 0.9 + (0.01 *
      crazyflie_U.Gyro_y + crazyflie_DW.Delay_DSTATE_p) * 0.1;

    /* SignalConversion generated from: '<Root>/Gain' incorporates:
     *  Gain: '<S1>/Gain'
     *  Gain: '<S1>/Gain2'
     *  Inport: '<Root>/Gyro_x'
     *  Inport: '<Root>/Gyro_y'
     *  Inport: '<Root>/Gyro_z'
     *  Inport: '<Root>/Ref_Pitch'
     *  Inport: '<Root>/Ref_Roll'
     *  Inport: '<Root>/Ref_YawRate'
     *  Sum: '<Root>/Sum'
     *  Sum: '<Root>/Sum1'
     *  Sum: '<Root>/Sum2'
     */
    tmp[0] = 57.295779513082323 * rtb_Sum3 - crazyflie_U.Ref_Roll;
    tmp[1] = 57.295779513082323 * rtb_Sum3_l - crazyflie_U.Ref_Pitch;
    tmp[2] = crazyflie_U.Gyro_x;
    tmp[3] = crazyflie_U.Gyro_y;
    tmp[4] = crazyflie_U.Gyro_z - crazyflie_U.Ref_YawRate;

    /* Sum: '<Root>/Sum3' incorporates:
     *  Constant: '<Root>/Constant'
     *  Gain: '<Root>/Gain'
     */
    for (i = 0; i < 4; i++) {
      tmp_0 = 0.0;
      for (i_0 = 0; i_0 < 5; i_0++) {
        tmp_0 += crazyflie_ConstP.Gain_Gain[(i_0 << 2) + i] * tmp[i_0];
      }

      rtb_Sum3_m[i] = tmp_0 + 10.0;
    }

    /* End of Sum: '<Root>/Sum3' */

    /* DataTypeConversion: '<Root>/ToUint16' */
    if (rtb_Sum3_m[0] < 65536.0) {
      if (rtb_Sum3_m[0] >= 0.0) {
        /* Outport: '<Root>/Motor_1' */
        crazyflie_Y.Motor_1 = (uint16_T)rtb_Sum3_m[0];
      } else {
        /* Outport: '<Root>/Motor_1' */
        crazyflie_Y.Motor_1 = 0U;
      }
    } else {
      /* Outport: '<Root>/Motor_1' */
      crazyflie_Y.Motor_1 = MAX_uint16_T;
    }

    /* End of DataTypeConversion: '<Root>/ToUint16' */

    /* DataTypeConversion: '<Root>/ToUint16_1' */
    if (rtb_Sum3_m[1] < 65536.0) {
      if (rtb_Sum3_m[1] >= 0.0) {
        /* Outport: '<Root>/Motor_2' */
        crazyflie_Y.Motor_2 = (uint16_T)rtb_Sum3_m[1];
      } else {
        /* Outport: '<Root>/Motor_2' */
        crazyflie_Y.Motor_2 = 0U;
      }
    } else {
      /* Outport: '<Root>/Motor_2' */
      crazyflie_Y.Motor_2 = MAX_uint16_T;
    }

    /* End of DataTypeConversion: '<Root>/ToUint16_1' */

    /* DataTypeConversion: '<Root>/ToUint16_2' */
    if (rtb_Sum3_m[2] < 65536.0) {
      if (rtb_Sum3_m[2] >= 0.0) {
        /* Outport: '<Root>/Motor_3' */
        crazyflie_Y.Motor_3 = (uint16_T)rtb_Sum3_m[2];
      } else {
        /* Outport: '<Root>/Motor_3' */
        crazyflie_Y.Motor_3 = 0U;
      }
    } else {
      /* Outport: '<Root>/Motor_3' */
      crazyflie_Y.Motor_3 = MAX_uint16_T;
    }

    /* End of DataTypeConversion: '<Root>/ToUint16_2' */

    /* DataTypeConversion: '<Root>/ToUint16_3' */
    if (rtb_Sum3_m[3] < 65536.0) {
      if (rtb_Sum3_m[3] >= 0.0) {
        /* Outport: '<Root>/Motor_4' */
        crazyflie_Y.Motor_4 = (uint16_T)rtb_Sum3_m[3];
      } else {
        /* Outport: '<Root>/Motor_4' */
        crazyflie_Y.Motor_4 = 0U;
      }
    } else {
      /* Outport: '<Root>/Motor_4' */
      crazyflie_Y.Motor_4 = MAX_uint16_T;
    }

    /* End of DataTypeConversion: '<Root>/ToUint16_3' */

    /* Update for Delay: '<S2>/Delay' */
    crazyflie_DW.Delay_DSTATE = rtb_Sum3;

    /* Update for Delay: '<S3>/Delay' */
    crazyflie_DW.Delay_DSTATE_p = rtb_Sum3_l;
  }

  rate_scheduler();
}

/* Model initialize function */
void crazyflie_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void crazyflie_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

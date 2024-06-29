/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: crazyflieModel.c
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

#define DEBUG_MODULE                   "STAB"
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "system.h"
#include "log.h"
#include "param.h"
#include "debug.h"
#include "motors.h"
#include "pm.h"
#include "crazyflie.h"
#include "crazyflieModel.h"
#include "sensors.h"
#include "commander.h"
#include "crtp_commander_high_level.h"
#include "crtp_localization_service.h"
#include "controller.h"
#include "power_distribution.h"
#include "collision_avoidance.h"
#include "health.h"
#include "supervisor.h"
#include "estimator.h"
#include "usddeck.h"
#include "quatcompress.h"
#include "statsCnt.h"
#include "static_mem.h"
#include "rateSupervisor.h"

static bool isInit;
static bool emergencyStop = false;
static int emergencyStopTimeout = EMERGENCY_STOP_TIMEOUT_DISABLED;
static uint32_t inToOutLatency;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;

// For scratch storage - never logged or passed to other subsystems.
//static setpoint_t tempSetpoint;
static StateEstimatorType estimatorType;
static ControllerType controllerType;
static STATS_CNT_RATE_DEFINE(stabilizerRate, 500);
static rateSupervisor_t rateSupervisorContext;
static bool rateWarningDisplayed = false;

// SSY191
static float l1, l2, l3, l4, l5, l6;
static uint16_t m1, m2, m3, m4;
STATIC_MEM_TASK_ALLOC(crazyflieModelTask, CRAZYFLIEMODEL_TASK_STACKSIZE);
static void crazyflieModelTask(void* param);
static void calcSensorToOutputLatency(const sensorData_t *sensorData)
{
  uint64_t outTimestamp = usecTimestamp();
  inToOutLatency = outTimestamp - sensorData->interruptTimestamp;
}

void crazyflieModelInit(StateEstimatorType estimator)
{
  if (isInit)
    return;
  crazyflie_initialize();
  sensorsInit();
  stateEstimatorInit(estimator);
  controllerInit(ControllerTypeAny);
  powerDistributionInit();
  collisionAvoidanceInit();
  estimatorType = getStateEstimator();
  controllerType = getControllerType();
  crazyflie_initialize();
  STATIC_MEM_TASK_CREATE(crazyflieModelTask, crazyflieModelTask,
    CRAZYFLIEMODEL_TASK_NAME, NULL, CRAZYFLIEMODEL_TASK_PRI);
  isInit = true;
}

bool crazyflieModelTest(void)
{
  bool pass = true;
  pass &= sensorsTest();
  pass &= stateEstimatorTest();
  pass &= controllerTest();
  pass &= powerDistributionTest();
  pass &= collisionAvoidanceTest();
  return pass;
}

static void checkEmergencyStopTimeout()
{
  if (emergencyStopTimeout >= 0) {
    emergencyStopTimeout -= 1;
    if (emergencyStopTimeout == 0) {
      emergencyStop = true;
    }
  }
}

/* The stabilizer loop runs at 1kHz (stock) or 500Hz (kalman). It is the
 * responsibility of the different functions to run slower by skipping call
 * (ie. returning without modifying the output structure).
 */
static void crazyflieModelTask(void* param)
{
  uint32_t tick;
  uint32_t lastWakeTime;
  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();
  DEBUG_PRINT("Wait for sensor calibration...\n");

  // Wait for sensors to be calibrated
  lastWakeTime = xTaskGetTickCount();
  while (!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }

  // Initialize tick to something else then 0
  tick = 1;
  rateSupervisorInit(&rateSupervisorContext, xTaskGetTickCount(), M2T(1000), 997,
                     1003, 1);
  DEBUG_PRINT("Ready to fly.\n");
  while (1) {
    // The sensor should unlock at 1kHz
    sensorsWaitDataReady();

    // update sensorData struct (for logging variables)
    sensorsAcquire(&sensorData, tick);
    if (healthShallWeRunTest()) {
      healthRunTests(&sensorData);
    } else {
      stateEstimator(&state, tick);
      commanderGetSetpoint(&setpoint, &state);
      crazyflie_U.Acc_x = sensorData.acc.x;
      crazyflie_U.Acc_y = sensorData.acc.y;
      crazyflie_U.Acc_z = sensorData.acc.z;
      crazyflie_U.Gyro_x = sensorData.gyro.x;
      crazyflie_U.Gyro_y = sensorData.gyro.y;
      crazyflie_U.Gyro_z = sensorData.gyro.z;
      crazyflie_U.Base_Thrust = setpoint.thrust;
      crazyflie_U.Ref_Roll = setpoint.attitude.roll;
      crazyflie_U.Ref_Pitch = setpoint.attitude.pitch;
      crazyflie_U.Ref_YawRate = setpoint.attitudeRate.yaw;

      //DEBUG_PRINT("Acc Y: %f\n", sensorData.acc.y);
      crazyflie_step();

      /* Get model outputs here */
      l1 = crazyflie_Y.Log1;
      l2 = crazyflie_Y.Log2;
      l3 = crazyflie_Y.Log3;
      l4 = crazyflie_Y.Log4;
      l5 = crazyflie_Y.Log5;
      l6 = crazyflie_Y.Log6;
      m1 = (uint16_t)crazyflie_Y.Motor_1;
      m2 = (uint16_t)crazyflie_Y.Motor_2;
      m3 = (uint16_t)crazyflie_Y.Motor_3;
      m4 = (uint16_t)crazyflie_Y.Motor_4;
      checkEmergencyStopTimeout();

      //
      // The supervisor module keeps track of Crazyflie state such as if
      // we are ok to fly, or if the Crazyflie is in flight.
      //
      supervisorUpdate(&sensorData);
      if (emergencyStop || (systemIsArmed() == false)) {
        powerStop();
      } else {
        motorsSetRatio(MOTOR_M1, m1);
        motorsSetRatio(MOTOR_M2, m2);
        motorsSetRatio(MOTOR_M3, m3);
        motorsSetRatio(MOTOR_M4, m4);
      }

#ifdef CONFIG_DECK_USD

      // Log data to uSD card if configured
      if (usddeckLoggingEnabled()
          && usddeckLoggingMode() == usddeckLoggingMode_SynchronousStabilizer
          && RATE_DO_EXECUTE(usddeckFrequency(), tick)) {
        usddeckTriggerLogging();
      }

#endif

      calcSensorToOutputLatency(&sensorData);
      tick++;
      STATS_CNT_RATE_EVENT(&stabilizerRate);
      if (!rateSupervisorValidate(&rateSupervisorContext, xTaskGetTickCount()))
      {
        if (!rateWarningDisplayed) {
          DEBUG_PRINT("WARNING: stabilizer loop rate is off (%lu)\n",
                      rateSupervisorLatestCount(&rateSupervisorContext));
          rateWarningDisplayed = true;
        }
      }
    }
  }
}

void stabilizerSetEmergencyStop()
{
  emergencyStop = true;
}

void stabilizerResetEmergencyStop()
{
  emergencyStop = false;
}

void stabilizerSetEmergencyStopTimeout(int timeout)
{
  emergencyStop = false;
  emergencyStopTimeout = timeout;
}

/**
 * Parameters to set the estimator and controller type
 * for the stabilizer module, or to do an emergency stop
 */
PARAM_GROUP_START(stabilizer)
/**
 * @brief Estimator type Any(0), complementary(1), kalman(2) (Default: 0)
 */
  PARAM_ADD_CORE(PARAM_UINT8, estimator, &estimatorType)
/**
 * @brief Controller type Any(0), PID(1), Mellinger(2), INDI(3) (Default: 0)
 */
  PARAM_ADD_CORE(PARAM_UINT8, controller, &controllerType)
/**
 * @brief If set to nonzero will turn off power
 */
  PARAM_ADD_CORE(PARAM_UINT8, stop, &emergencyStop)
  PARAM_GROUP_STOP(stabilizer)
/**
 * Log group for the current controller target
 *
 * Note: all members may not be updated depending on how the system is used
 */
  LOG_GROUP_START(ctrltarget)
/**
 * @brief Desired position X [m]
 */
  LOG_ADD_CORE(LOG_FLOAT, x, &setpoint.position.x)
/**
 * @brief Desired position Y [m]
 */
  LOG_ADD_CORE(LOG_FLOAT, y, &setpoint.position.y)
/**
 * @brief Desired position X [m]
 */
  LOG_ADD_CORE(LOG_FLOAT, z, &setpoint.position.z)
/**
 * @brief Desired velocity X [m/s]
 */
  LOG_ADD_CORE(LOG_FLOAT, vx, &setpoint.velocity.x)
/**
 * @brief Desired velocity Y [m/s]
 */
  LOG_ADD_CORE(LOG_FLOAT, vy, &setpoint.velocity.y)
/**
 * @brief Desired velocity Z [m/s]
 */
  LOG_ADD_CORE(LOG_FLOAT, vz, &setpoint.velocity.z)
/**
 * @brief Desired acceleration X [m/s^2]
 */
  LOG_ADD_CORE(LOG_FLOAT, ax, &setpoint.acceleration.x)
/**
 * @brief Desired acceleration Y [m/s^2]
 */
  LOG_ADD_CORE(LOG_FLOAT, ay, &setpoint.acceleration.y)
/**
 * @brief Desired acceleration Z [m/s^2]
 */
  LOG_ADD_CORE(LOG_FLOAT, az, &setpoint.acceleration.z)
/**
 * @brief Desired attitude, roll [deg]
 */
  LOG_ADD_CORE(LOG_FLOAT, roll, &setpoint.attitude.roll)
/**
 * @brief Desired attitude, pitch [deg]
 */
  LOG_ADD_CORE(LOG_FLOAT, pitch, &setpoint.attitude.pitch)
/**
 * @brief Desired attitude rate, yaw rate [deg/s]
 */
  LOG_ADD_CORE(LOG_FLOAT, yaw, &setpoint.attitudeRate.yaw)
  LOG_GROUP_STOP(ctrltarget)
/**
 * Logs to set the estimator and controller type
 * for the stabilizer module
 */
  LOG_GROUP_START(stabilizer)
/**
 * @brief Estimated roll
 *   Note: Same as stateEstimate.roll
 */
  LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
/**
 * @brief Estimated pitch
 *   Note: Same as stateEstimate.pitch
 */
  LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
/**
 * @brief Estimated yaw
 *   Note: same as stateEstimate.yaw
 */
  LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)
/**
 * @brief Current thrust
 */
  LOG_ADD(LOG_FLOAT, thrust, &control.thrust)
/**
 * @brief Rate of stabilizer loop
 */
  STATS_CNT_RATE_LOG_ADD(rtStab, &stabilizerRate)
/**
 * @brief Latency from sampling of sensor to motor output
 *    Note: Used for debugging but could also be used as a system test
 */
  LOG_ADD(LOG_UINT32, intToOut, &inToOutLatency)
  LOG_GROUP_STOP(stabilizer)
/**
 * Log group for accelerometer sensor measurement, based on body frame.
 * Compensated for a miss-alignment by gravity at startup.
 *
 * For data on measurement noise please see information from the sensor
 * manufacturer. To see what accelerometer sensor is in your Crazyflie or Bolt
 * please check documentation on the Bitcraze webpage or check the parameter
 * group `imu_sensors`.
 */
  LOG_GROUP_START(acc)
/**
 * @brief Acceleration in X [Gs]
 */
  LOG_ADD_CORE(LOG_FLOAT, x, &sensorData.acc.x)
/**
 * @brief Acceleration in Y [Gs]
 */
  LOG_ADD_CORE(LOG_FLOAT, y, &sensorData.acc.y)
/**
 * @brief Acceleration in Z [Gs]
 */
  LOG_ADD_CORE(LOG_FLOAT, z, &sensorData.acc.z)
  LOG_GROUP_STOP(acc)
#ifdef LOG_SEC_IMU
  LOG_GROUP_START(accSec)
  LOG_ADD(LOG_FLOAT, x, &sensorData.accSec.x)
  LOG_ADD(LOG_FLOAT, y, &sensorData.accSec.y)
  LOG_ADD(LOG_FLOAT, z, &sensorData.accSec.z)
  LOG_GROUP_STOP(accSec)
#endif
/**
 * Log group for the barometer.
 *
 * For data on measurement noise please see information from the sensor
 * manufacturer. To see what barometer sensor is in your Crazyflie or Bolt
 * please check documentation on the Bitcraze webpage or check the parameter
 * group `imu_sensors`.
 */
  LOG_GROUP_START(baro)
/**
 * @brief Altitude above Sea Level [m]
 */
  LOG_ADD_CORE(LOG_FLOAT, asl, &sensorData.baro.asl)
/**
 * @brief Temperature [degrees Celsius]
 */
  LOG_ADD(LOG_FLOAT, temp, &sensorData.baro.temperature)
/**
 * @brief Air preassure [mbar]
 */
  LOG_ADD_CORE(LOG_FLOAT, pressure, &sensorData.baro.pressure)
  LOG_GROUP_STOP(baro)
/**
 * Log group for gyroscopes.
 *
 * For data on measurement noise please see information from the sensor
 * manufacturer. To see what gyroscope sensor is in your Crazyflie or Bolt
 * please check documentation on the Bitcraze webpage or check the parameter
 * group `imu_sensors`.
 */
  LOG_GROUP_START(gyro)
/**
 * @brief Angular velocity (rotation) around the X-axis, after filtering [deg/s]
 */
  LOG_ADD_CORE(LOG_FLOAT, x, &sensorData.gyro.x)
/**
 * @brief Angular velocity (rotation) around the Y-axis, after filtering [deg/s]
 */
  LOG_ADD_CORE(LOG_FLOAT, y, &sensorData.gyro.y)
/**
 * @brief Angular velocity (rotation) around the Z-axis, after filtering [deg/s]
 */
  LOG_ADD_CORE(LOG_FLOAT, z, &sensorData.gyro.z)
  LOG_GROUP_STOP(gyro)
#ifdef LOG_SEC_IMU
  LOG_GROUP_START(gyroSec)
  LOG_ADD(LOG_FLOAT, x, &sensorData.gyroSec.x)
  LOG_ADD(LOG_FLOAT, y, &sensorData.gyroSec.y)
  LOG_ADD(LOG_FLOAT, z, &sensorData.gyroSec.z)
  LOG_GROUP_STOP(gyroSec)
#endif
/**
 * Log group for magnetometer.
 *
 * Currently only present on Crazyflie 2.0
 */
  LOG_GROUP_START(mag)
/**
 * @brief Magnetometer X axis, after filtering [gauss]
 */
  LOG_ADD_CORE(LOG_FLOAT, x, &sensorData.mag.x)
/**
 * @brief Magnetometer Y axis, after filtering [gauss]
 */
  LOG_ADD_CORE(LOG_FLOAT, y, &sensorData.mag.y)
/**
 * @brief Magnetometer Z axis, after filtering [gauss]
 */
  LOG_ADD_CORE(LOG_FLOAT, z, &sensorData.mag.z)
  LOG_GROUP_STOP(mag)
  LOG_GROUP_START(controller)
  LOG_ADD(LOG_INT16, ctr_yaw, &control.yaw)
  LOG_GROUP_STOP(controller)
/**
 * Log group for the state estimator, the currently estimated state of the platform.
 *
 * Note: all values may not be updated depending on which estimator that is used.
 */
  LOG_GROUP_START(stateEstimate)
/**
 * @brief The estimated position of the platform in the global reference frame, X [m]
 */
  LOG_ADD_CORE(LOG_FLOAT, x, &state.position.x)
/**
 * @brief The estimated position of the platform in the global reference frame, Y [m]
 */
  LOG_ADD_CORE(LOG_FLOAT, y, &state.position.y)
/**
 * @brief The estimated position of the platform in the global reference frame, Z [m]
 */
  LOG_ADD_CORE(LOG_FLOAT, z, &state.position.z)
/**
 * @brief The velocity of the Crazyflie in the global reference frame, X [m/s]
 */
  LOG_ADD_CORE(LOG_FLOAT, vx, &state.velocity.x)
/**
 * @brief The velocity of the Crazyflie in the global reference frame, Y [m/s]
 */
  LOG_ADD_CORE(LOG_FLOAT, vy, &state.velocity.y)
/**
 * @brief The velocity of the Crazyflie in the global reference frame, Z [m/s]
 */
  LOG_ADD_CORE(LOG_FLOAT, vz, &state.velocity.z)
/**
 * @brief The acceleration of the Crazyflie in the global reference frame, X [Gs]
 */
  LOG_ADD_CORE(LOG_FLOAT, ax, &state.acc.x)
/**
 * @brief The acceleration of the Crazyflie in the global reference frame, Y [Gs]
 */
  LOG_ADD_CORE(LOG_FLOAT, ay, &state.acc.y)
/**
 * @brief The acceleration of the Crazyflie in the global reference frame, without considering gravity, Z [Gs]
 */
  LOG_ADD_CORE(LOG_FLOAT, az, &state.acc.z)
/**
 * @brief Attitude, roll angle [deg]
 */
  LOG_ADD_CORE(LOG_FLOAT, roll, &state.attitude.roll)
/**
 * @brief Attitude, pitch angle (legacy CF2 body coordinate system, where pitch is inverted) [deg]
 */
  LOG_ADD_CORE(LOG_FLOAT, pitch, &state.attitude.pitch)
/**
 * @brief Attitude, yaw angle [deg]
 */
  LOG_ADD_CORE(LOG_FLOAT, yaw, &state.attitude.yaw)
/**
 * @brief Attitude as a quaternion, x
 */
  LOG_ADD_CORE(LOG_FLOAT, qx, &state.attitudeQuaternion.x)
/**
 * @brief Attitude as a quaternion, y
 */
  LOG_ADD_CORE(LOG_FLOAT, qy, &state.attitudeQuaternion.y)
/**
 * @brief Attitude as a quaternion, z
 */
  LOG_ADD_CORE(LOG_FLOAT, qz, &state.attitudeQuaternion.z)
/**
 * @brief Attitude as a quaternion, w
 */
  LOG_ADD_CORE(LOG_FLOAT, qw, &state.attitudeQuaternion.w)
  LOG_GROUP_STOP(stateEstimate)
/* SSY191: Add Simulink plots
 */
  LOG_GROUP_START(simulink)
  LOG_ADD(LOG_FLOAT, log_1, &l1)
  LOG_ADD(LOG_FLOAT, log_2, &l2)
  LOG_ADD(LOG_FLOAT, log_3, &l3)
  LOG_ADD(LOG_FLOAT, log_4, &l4)
  LOG_ADD(LOG_FLOAT, log_5, &l5)
  LOG_ADD(LOG_FLOAT, log_6, &l6)
  LOG_GROUP_STOP(simulink)
/*
 * File trailer for generated code.
 *
 * [EOF]
 */

#include "your_code.h"


/***
 *
 * This file is where you should add you tasks. You already know the structure
 * Required to do so from the work with the simulator.
 *
 * The function yourCodeInit() is set to automatically execute when the
 * quadrotor is started. This is where you need to create your tasks. The
 * scheduler that runs the tasks is already up and running so you should
 * NOT make a call to vTaskStartScheduler();.
 *
 * Below that you can find a few examples of useful function calls and code snippets.
 *
 * For further reference on how this is done: Look into the file stabilizer.c
 * which usually handles the control of the crazyflie.
 *
 ***/
static sensorData_t sensorData;
// static uint16_t motor_val;
static SemaphoreHandle_t motors_sem;
static SemaphoreHandle_t references_sem;
static SemaphoreHandle_t sensors_sem;
static SemaphoreHandle_t estimate_sem;
static uint16_t motors[4];
static state_t state;
static setpoint_t setpoint;


// NO NEED TO CHANGE THIS
static bool emergencyStop = false;
static int emergencyStopTimeout = EMERGENCY_STOP_TIMEOUT_DISABLED;
static StateEstimatorType estimatorType;
// END NO NEED TO CHANGE THIS

STATIC_MEM_TASK_ALLOC(filterTask, configMINIMAL_STACK_SIZE);
STATIC_MEM_TASK_ALLOC(referenceTask, configMINIMAL_STACK_SIZE);
STATIC_MEM_TASK_ALLOC(controlSystemTask, configMINIMAL_STACK_SIZE);
static void filterTask(void *pvParameters);
static void referenceTask(void *pvParameters);
static void controlSystemTask(void *pvParameters);




void yourCodeInit(void)
{
    estimatorType = getStateEstimator();
    motors_sem = xSemaphoreCreateMutex();
    references_sem = xSemaphoreCreateMutex();
    sensors_sem = xSemaphoreCreateMutex();
    estimate_sem = xSemaphoreCreateMutex();
    
	
	/*
   * CREATE AND EXECUTE YOUR TASKS FROM HERE
   */
  STATIC_MEM_TASK_CREATE(referenceTask, referenceTask, "referenceTask", NULL, 1);
  STATIC_MEM_TASK_CREATE(controlSystemTask, controlSystemTask, "controlSystemTask", NULL, 2);
  STATIC_MEM_TASK_CREATE(filterTask, filterTask, "filterTask", NULL, 3);
  
	
}

static void referenceTask(void *pvParameters) {
  uint32_t tick;
  uint32_t lastWakeTime;
  lastWakeTime = xTaskGetTickCount();

  while(!sensorsAreCalibrated()) {
      vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }
  tick = 1;

 while(1) {
        vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));
        xSemaphoreTake(references_sem, portMAX_DELAY);
        setpoint.attitude.roll=0.0 ;
        setpoint.attitude.pitch=0.0 ;
        xSemaphoreGive(references_sem);
 }

}

static void filterTask(void *pvParameters) {
  uint32_t tick;
  uint32_t lastWakeTime;
  lastWakeTime = xTaskGetTickCount();

 
  while(!sensorsAreCalibrated()) {
      vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }
  tick = 1;
    double gyro_data[3];
    double acc_data[3];
    double acc_angle[3];
    double h = 0.01;
    double gamma = 0.95;
    double estimate[3] = {0.0};

    while(true){
      vTaskDelayUntil(&lastWakeTime, F2T(POSITION_RATE));

      sensorsAcquire(&sensorData, tick);
      xSemaphoreTake(sensors_sem,portMAX_DELAY);
      acc_data[0] = sensorData.acc.x;
      acc_data[1] = sensorData.acc.y;
      acc_data[2] = sensorData.acc.z;
      gyro_data[0] = sensorData.gyro.x;
      gyro_data[1] = sensorData.gyro.y;
      gyro_data[2] = sensorData.gyro.z;
      xSemaphoreGive(sensors_sem);
      acc_angle[0] = (180/3.14)*atan2(acc_data[1],acc_data[2]);
      acc_angle[1] = (180/3.14)*atan2(-acc_data[0],sqrt(pow(acc_data[1],2)+pow(acc_data[2],2)));
      acc_angle[2] = 0;

      for (int i = 0; i < 3; i++)
        {
            estimate[i] = ((1-gamma)*acc_angle[i]+gamma*(estimate[i]+h*gyro_data[i])); 
        }
      estimate[2] += 0.01 * gyro_data[2];
      xSemaphoreTake(estimate_sem,portMAX_DELAY);

      state.attitude.roll = estimate[0];
      state.attitude.pitch = estimate[1];
      state.attitude.yaw = estimate[2];

      xSemaphoreGive(estimate_sem);
      LOG_GROUP_START(estim)
      LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
      LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
      LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)
      LOG_GROUP_STOP(estim)
    }
  }

static void controlSystemTask(void *pvParameters) {
    //struct ControlSystemParams *params = (struct ControlSystemParams*)pvParameters;
    uint32_t tick;
    uint32_t lastWakeTime;

    lastWakeTime = xTaskGetTickCount ();
    while(!sensorsAreCalibrated()) {
      vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
    }
    tick = 1;
    double r_rpdy[3];
    double estimate[3] = {0.0};
    double base_thrust=10000;
    //unsigned short motors[4];
    double gyro_data[3];

    double c1,c2,c3,c4,c5;
    c1 = 358.2652;
    c2 = 520.972;
    c3 = 69.620;
    c4 = 100.919;
    c5 = 0.4;
    double gain[4][5]={
    { c1,  c2,  c3,  c4,  c5},
    { c1, -c2,  c3, -c4, -c5},
    {-c1, -c2, -c3, -c4,  c5},
    {-c1,  c2, -c3,  c4, -c5}
    };
    while(1)
{    
  vTaskDelayUntil(&lastWakeTime, F2T(POSITION_RATE));
        sensorsAcquire(&sensorData, tick);
        stateEstimator(&state, tick);
        commanderGetSetpoint(&setpoint, &state);
        xSemaphoreTake(sensors_sem,portMAX_DELAY);
          gyro_data[0] = sensorData.gyro.x;
          gyro_data[1] = sensorData.gyro.y;
          gyro_data[2] = sensorData.gyro.z;
        xSemaphoreGive(sensors_sem);
        xSemaphoreTake(estimate_sem, portMAX_DELAY);
        estimate[0] = state.attitude.roll;
        estimate[1] = state.attitude.pitch;
        estimate[2] = state.attitude.yaw;
        xSemaphoreGive(estimate_sem);
        xSemaphoreTake(references_sem, portMAX_DELAY);
        r_rpdy[0]=setpoint.attitude.roll;
        r_rpdy[1]=setpoint.attitude.pitch;
        r_rpdy[2]=0;
        xSemaphoreGive(references_sem);
  double state[5][1];
        state[0][0] = -r_rpdy[0]+estimate[0];
        state[1][0] = -r_rpdy[1]+estimate[1];
        state[2][0] = gyro_data[0];
        state[3][0] = gyro_data[1];
        state[4][0] = -r_rpdy[2]+gyro_data[2];

        double error[4][1];
        error[0][0] = gain[0][0]*state[0][0]+gain[0][1]*state[1][0]+gain[0][2]*state[2][0]+gain[0][3]*state[3][0]+gain[0][4]*state[4][0];
        error[1][0] = gain[1][0]*state[0][0]+gain[1][1]*state[1][0]+gain[1][2]*state[2][0]+gain[1][3]*state[3][0]+gain[1][4]*state[4][0];
        error[2][0] = gain[2][0]*state[0][0]+gain[2][1]*state[1][0]+gain[2][2]*state[2][0]+gain[2][3]*state[3][0]+gain[2][4]*state[4][0];
        error[3][0] = gain[3][0]*state[0][0]+gain[3][1]*state[1][0]+gain[3][2]*state[2][0]+gain[3][3]*state[3][0]+gain[3][4]*state[4][0];


        motors[0] = (uint16_t)error[0][0]+base_thrust; 
        motors[1] = (uint16_t)error[1][0]+base_thrust; 
        motors[2] = (uint16_t)error[2][0]+base_thrust; 
        motors[3] = (uint16_t)error[3][0]+base_thrust; 

        xSemaphoreTake(motors_sem,portMAX_DELAY);
          motorsSetRatio(MOTOR_M1, motors[0]);
          motorsSetRatio(MOTOR_M2, motors[1]);
          motorsSetRatio(MOTOR_M3, motors[2]);
          motorsSetRatio(MOTOR_M4, motors[3]);
        xSemaphoreGive(motors_sem);
    
}
    
    
    
    }

/* 
* ADD TASKS HERE
*/
// static void task(void* param)
// {
//   uint32_t tick;
//   uint32_t lastWakeTime;

//   lastWakeTime = xTaskGetTickCount ();
//   while(!sensorsAreCalibrated()) {
//     vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP)); //executed att 1000Hz
//  }
//  // Initialize tick to something else then 0
//   tick = 1;

 
//   while (1)//main code
//   {
//     vTaskDelayUntil(&lastWakeTime, F2T(POSITION_RATE)); //executed at 100Hz

    
//     sensorsAcquire(&sensorData, tick);
//     FilterParams params;
//     params->gyro_data = sensorData.gyro;
//     params->acc_data = sensorData.acc;
//     filterTask(params);
//     motor_val=(uint16_t) 3000;

//     xSemaphoreTake(motors_sem, portMAX_DELAY);
//     motorsSetRatio(MOTOR_M1, motor_val);
//     motorsSetRatio(MOTOR_M2, motor_val);
//     motorsSetRatio(MOTOR_M3, motor_val);
//     motorsSetRatio(MOTOR_M4, motor_val);
//     xSemaphoreGive(motors_sem);

//   }
// }

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

/*************************************************
 * CREATE A TASK (Example, a task with priority 5)
 ************************************************/
 //STATIC_MEM_TASK_ALLOC(yourTaskName, configMINIMAL_STACK_SIZE);
 //STATIC_MEM_TASK_CREATE(yourTaskName, yourTaskName,
 //   "TASK_IDENTIFIER_NAME", NULL, 5);

/*************************************************
 * WAIT FOR SENSORS TO BE CALIBRATED
 ************************************************/
// lastWakeTime = xTaskGetTickCount ();
// while(!sensorsAreCalibrated()) {
//     vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
// }



/*************************************************
 * RETRIEVE THE MOST RECENT SENSOR DATA
 *
 * The code creates a variable called sensorData and then calls a function
 * that fills this variable with the latest data from the sensors.
 *
 * sensorData_t sensorData = struct {
 *     Axis3f acc;
 *     Axis3f gyro;
 *     Axis3f mag;
 *     baro_t baro;
 *     zDistance_t zrange;
 *     point_t position;
 * }
 *
 * Before starting the loop, initialize tick:
 * uint32_t tick;
 * tick = 1;
 ************************************************/
// sensorData_t sensorData;
// sensorsAcquire(&sensorData, tick);



/*************************************************
 * RETRIEVE THE SET POINT FROM ANY EXTERNAL COMMAND INTERFACE
 *
 * The code creates a variable called setpoint and then calls a function
 * that fills this variable with the latest command input.
 *
 * setpoint_t setpoint = struct {
 *     uint32_t timestamp;
 *
 *     attitude_t attitude;      // deg
 *     attitude_t attitudeRate;  // deg/s
 *     quaternion_t attitudeQuaternion;
 *     float thrust;
 *     point_t position;         // m
 *     velocity_t velocity;      // m/s
 *     acc_t acceleration;       // m/s^2
 *     bool velocity_body;       // true if velocity is given in body frame; false if velocity is given in world frame
 *
 *     struct {
 *         stab_mode_t x;
 *         stab_mode_t y;
 *         stab_mode_t z;
 *         stab_mode_t roll;
 *         stab_mode_t pitch;
 *         stab_mode_t yaw;
 *         stab_mode_t quat;
 *     } mode;
 * }
 *
 ************************************************/
// state_t state;
// setpoint_t setpoint;
// stateEstimator(&state, tick);
// commanderGetSetpoint(&setpoint, &state);



/*************************************************
 * SENDING OUTPUT TO THE MOTORS
 *
 * The code sends an output to each motor. The output should have the be
 * of the typ unsigned 16-bit integer, i.e. use variables such as:
 * uint16_t value_i
 *
 ************************************************/
// motorsSetRatio(MOTOR_M1, value_1);
// motorsSetRatio(MOTOR_M2, value_2);
// motorsSetRatio(MOTOR_M3, value_3);
// motorsSetRatio(MOTOR_M4, value_4);


/*************************************************
 * LOGGING VALUES THAT CAN BE PLOTTEN IN PYTHON CLIENT
 *
 * We have already set up three log blocks to for the accelerometer data, the
 * gyro data and the setpoints, just uncomment the block to start logging. Use
 * them as reference if you want to add custom blocks.
 *
 ************************************************/

/*
LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &sensorData.acc.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.acc.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.acc.z)
LOG_GROUP_STOP(acc)
*/

/*
LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyro.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyro.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyro.z)
LOG_GROUP_STOP(gyro)
*/

/*
LOG_GROUP_START(ctrltarget)
LOG_ADD(LOG_FLOAT, roll, &setpoint.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &setpoint.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &setpoint.attitudeRate.yaw)
LOG_GROUP_STOP(ctrltarget)
*/

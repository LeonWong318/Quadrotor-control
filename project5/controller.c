#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <memory.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "controller.h"
#include "constants.h"

void controlSystemTask(void *pvParameters) {
    struct ControlSystemParams *params =
        (struct ControlSystemParams*)pvParameters;
    // we keep local copies of the global state + semaphores
    unsigned short motors[4];
    double gyro_data[3];
    double acc_data[3];
    double r_rpdy[3];
    double estimate[3] = {0.0};
    unsigned short base_thrust = 29000;
    // copy the semaphore handles for convenience
    SemaphoreHandle_t motors_sem = params->motors_sem;
    SemaphoreHandle_t references_sem = params->references_sem;
    SemaphoreHandle_t sensors_sem = params->sensors_sem;
    SemaphoreHandle_t estimate_sem = params->estimate_sem;

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
    //gain[4][5] = -gain[4][5];
    // double gain[4][5]={
    // {-c1, -c2, -c3, -c4, -c5},
    // {-c1,  c2, -c3,  c4,  c5},
    // { c1,  c2,  c3,  c4, -c5},
    // { c1, -c2,  c3, -c4,  c5}
    // };

    TickType_t cLastWakeTime;
    cLastWakeTime = xTaskGetTickCount();
    while(1) {
        // read sensor data (gyro)
        xSemaphoreTake(sensors_sem, portMAX_DELAY);
        memcpy(gyro_data, params->gyro_data, sizeof(gyro_data));
        xSemaphoreGive(sensors_sem);
        // read filter data (angle estimates)
        xSemaphoreTake(estimate_sem, portMAX_DELAY);
        memcpy(estimate, params->estimate, sizeof(estimate));
        xSemaphoreGive(estimate_sem);
        // read latest references
        xSemaphoreTake(references_sem, portMAX_DELAY);
        memcpy(r_rpdy, params->r_rpdy, sizeof(r_rpdy));
        xSemaphoreGive(references_sem);
        // Convert to radians
        // for(int i = 0; i < 3; i++){
        //     r_rpdy[i] = r_rpdy[i]*M_PI/180; 
        //     gyro_data[i] = gyro_data[i]*M_PI/180;        
        // }
        // compute error
        
        double state[5][1];
        state[0][0] = -r_rpdy[0]+estimate[0];
        state[1][0] = -r_rpdy[1]+estimate[1];
        state[2][0] = gyro_data[0];
        state[3][0] = gyro_data[1];
        state[4][0] = -r_rpdy[2]+gyro_data[2];

        // matrix multiple
        
        double error[4][1];
        error[0][0] = gain[0][0]*state[0][0]+gain[0][1]*state[1][0]+gain[0][2]*state[2][0]+gain[0][3]*state[3][0]+gain[0][4]*state[4][0];
        error[1][0] = gain[1][0]*state[0][0]+gain[1][1]*state[1][0]+gain[1][2]*state[2][0]+gain[1][3]*state[3][0]+gain[1][4]*state[4][0];
        error[2][0] = gain[2][0]*state[0][0]+gain[2][1]*state[1][0]+gain[2][2]*state[2][0]+gain[2][3]*state[3][0]+gain[2][4]*state[4][0];
        error[3][0] = gain[3][0]*state[0][0]+gain[3][1]*state[1][0]+gain[3][2]*state[2][0]+gain[3][3]*state[3][0]+gain[3][4]*state[4][0];
        // for (int w = 0; w < 4; w++) {
        //   for (int j = 0; j < 1; j++) {
        //         error[w][j] = 0;
        //       for (int k = 0; k < 5; k++) {
        //          error[w][j] += gain[w][k] * state[k][j];
        //       }
        //     }
        // }
        //matrix_multiply(4,5,5,1,gain,state);
        // // example of how to log some intermediate calculation
        // // and use the provided constants
        // params->log_data[0] = crazyflie_constants.m * crazyflie_constants.g;
        // params->log_data[2] = r_rpdy[0];
        // compute motor outputs
        
        motors[0] = error[0][0]+base_thrust; 
        motors[1] = error[1][0]+base_thrust; 
        motors[2] = error[2][0]+base_thrust; 
        motors[3] = error[3][0]+base_thrust; 
       
        // // write motor output
        // memcpy(params->motors, motors, sizeof(motors));
        // write motor output
        params->log_data[0] = state[0][0];
        params->log_data[1] = state[1][0];
        params->log_data[2] = error[2][0];
        params->log_data[3] = error[3][0];
        xSemaphoreTake(motors_sem,portMAX_DELAY);
        memcpy(params->motors, motors, sizeof(motors));
        xSemaphoreGive(motors_sem);
        // sleep 10ms to make this task run at 100Hz
        vTaskDelayUntil( &cLastWakeTime, 10 / portTICK_PERIOD_MS);
    }
}
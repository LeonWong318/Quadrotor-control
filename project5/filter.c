#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <memory.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "filter.h"
#include "constants.h"
void filterTask(void *pvParameters) {
    struct FilterParams *params =
        (struct FilterParams*)pvParameters;
    // we keep local copies of the global state + semaphores
    double gyro_data[3];
    double acc_data[3];
    double acc_angle[3];
    double h = 0.01;
    double gamma = 0.95;
    // copy the semaphore handles for convenience
    SemaphoreHandle_t sensors_sem = params->sensors_sem;
    SemaphoreHandle_t estimate_sem = params->estimate_sem;
    // local internal state.
    double estimate[3] = {0.0};
    TickType_t fLastWakeTime;
    fLastWakeTime = xTaskGetTickCount();
    while(1) {
        // read sensor data
        xSemaphoreTake(sensors_sem, portMAX_DELAY);
        memcpy(gyro_data, params->gyro_data, sizeof(gyro_data));
        memcpy(acc_data, params->acc_data, sizeof(acc_data));
        xSemaphoreGive(sensors_sem);
        // apply filter
        // Calculating the euler angles
        acc_angle[0] = (180/3.14)*atan2(acc_data[1],acc_data[2]);
        acc_angle[1] = (180/3.14)*atan2(-acc_data[0],sqrt(pow(acc_data[1],2)+pow(acc_data[2],2)));
        acc_angle[2] = 0;
        // Estimation after the filter
        for (int i = 0; i < 3; i++)
        {
            estimate[i] = ((1-gamma)*acc_angle[i]+gamma*(estimate[i]+h*gyro_data[i])); 
        }
        //convert to degree
        // for (int i = 0; i < 3; i++)
        // {
        // estimate[i]=estimate[i]*(180/M_PI);
        // }
        
        // estimate of the yaw angle provided as an example
        estimate[2] += 0.01 * gyro_data[2];
        
        // // example of how to log some intermediate calculation
        // params->log_data[1] = 0.01 * gyro_data[2];
        // write estimates output
        xSemaphoreTake(estimate_sem, portMAX_DELAY);
        memcpy(params->estimate, estimate, sizeof(estimate));
        xSemaphoreGive(estimate_sem);
        // sleep 10ms to make this task run at 100Hz
       //vTaskDelayUntil( &fLastWakeTime, 10 / portTICK_PERIOD_MS);
       vTaskDelayUntil( &fLastWakeTime, 1 / portTICK_PERIOD_MS);
    }}
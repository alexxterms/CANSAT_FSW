#ifndef GAS_SENSOR_TASK_H
#define GAS_SENSOR_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Function to initialize and run the gas sensor task
void gas_sensor_task(void *arg);

#endif // GAS_SENSOR_TASK_H
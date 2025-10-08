#ifndef BAROMETER_TASK_H
#define BAROMETER_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Function to initialize and run the barometer task
void barometer_task(void *arg);

#endif // BAROMETER_TASK_H
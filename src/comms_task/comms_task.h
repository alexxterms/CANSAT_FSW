#ifndef COMMS_TASK_H
#define COMMS_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Function to initialize and run the comms task
void comms_task(void *arg);

#endif // COMMS_TASK_H
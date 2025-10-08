#ifndef FLIGHT_STATE_TASK_H
#define FLIGHT_STATE_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Function to initialize and run the flight state task
void flight_state_task(void *arg);

// Function to update the flight state based on sensor data
void update_flight_state(float acceleration, float altitude, float velocity);

#endif // FLIGHT_STATE_TASK_H
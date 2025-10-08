#ifndef IMU_TASK_H
#define IMU_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bmx160.h"
#include "ahrs.h"

// Function to initialize and run the IMU task
void imu_task(void *arg);

// Transformation functions for BMX160 orientation
void transform_accel_gyro(bmx160_vector_t *v);
void transform_mag(bmx160_vector_t *v);

// Main IMU processing function
void run_imu(void);

#endif // IMU_TASK_H
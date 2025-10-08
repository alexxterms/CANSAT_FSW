#ifndef PARAGLIDER_TASK_H
#define PARAGLIDER_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Function to initialize and run the paraglider navigation task
void paraglider_navigation_task(void *arg);

#endif // PARAGLIDER_TASK_H
/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"

void vTask1(void *pvParameters)
{
    /* ulCount is declared volatile to ensure it is not optimized out. */
    for (;;)
    {
        /* Print out the name of the current task task. */
        ESP_LOGI("TASK1", "Task 1 is running");
        //printf("Task 1 is running\n");
        /* Delay for a period. */
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 segundo
    }
}

void vTask2(void *pvParameters)
{
    /* As per most tasks, this task is implemented in an infinite loop. */
    for (;;)
    {
        /* Print out the name of this task. */
        ESP_LOGI("TASK2", "Task 2 is running");
        //printf("Task 2 is running\n");
        /* Delay for a period. */
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 segundo
    }
}

void app_main(void)
{
    /*
     * Variables declared here may no longer exist after starting the FreeRTOS
     * scheduler. Do not attempt to access variables declared on the stack used
     * by main() from tasks.
     */
    /*
     * Create one of the two tasks. Note that a real application should check
     * the return value of the xTaskCreate() call to ensure the task was
     * created successfully.
     */
    xTaskCreate(vTask1,   /* Pointer to the function that implements the task.*/
                "Task 1", /* Text name for the task. */
                2048,     /* Stack depth in words. */
                NULL,     /* This example does not use the task parameter. */
                1,        /* This task will run at priority 1. */
                NULL);    /* This example does not use the task handle. */

    /* Create the other task in exactly the same way and at the same priority.*/
    xTaskCreate(vTask2, "Task 2", 2048, NULL, 1, NULL);

    /*
     * If all is well main() will not reach here because the scheduler will now
     * be running the created tasks. If main() does reach here then there was
     * not enough heap memory to create either the idle or timer tasks
     * (described later in this book). Chapter 3 provides more information on
     * heap memory management.
     */

}
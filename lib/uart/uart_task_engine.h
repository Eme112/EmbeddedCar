/*
 * uart_task.h
 *
 *  Created on: 13 mrt. 2021
 *      Author: jancu
 */

#ifndef UART_TASK_H_
#define UART_TASK_H_

#include "FreeRTOS.h"
#include "semphr.h"

/*******************************************************************************
* Macros
********************************************************************************/
/* Task parameters for UART Task. */
#define UART_TASK_PRIORITY       (2)
#define UART_TASK_STACK_SIZE     (1024 * 3)

/* application dependent UART settings */
#define UART_BUFFER_SIZE 26

/*******************************************************************************
* Function Prototypes
********************************************************************************/
void uart_task1(void *pvParameters);
void uart_task2_send(void *pvParameters);
void uart_task2_receive(void *pvParameters);

void initUART1();
void initUART2();
void UART_Isr1();
void UART_Isr2();

// Data to be sent and received
static SemaphoreHandle_t mutex_calculated_speed;
static int calculated_speed = 0.0;
static SemaphoreHandle_t mutex_mileage;
static int mileage = 0.0;
static SemaphoreHandle_t mutex_speed;
static int speed = 0.0;
static SemaphoreHandle_t mutex_engine_state;
static char engine_state = 'N';

#endif /* UART_TASK_H_ */

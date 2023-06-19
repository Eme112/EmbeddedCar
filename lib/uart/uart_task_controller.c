/*
 * uart_task.c
 */

/* FreeRTOS header files */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "uart_task_controller.h"

#include "hardware/uart.h"
#include "hardware/irq.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define UART_ID_1 uart0
#define UART_ID_2 uart1
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE
#define UART_TX_PIN_1 12
#define UART_RX_PIN_1 13
#define UART_TX_PIN_2 8
#define UART_RX_PIN_2 9


/* Stores the handle of the task that will be notified when the
 receive is complete. */
volatile TaskHandle_t xTaskToNotify_UART_1 = NULL;
volatile TaskHandle_t xTaskToNotify_UART_2 = NULL;


uint8_t rxChar_1;
uint8_t rxChar_2;


void UART_receive1();
void UART_receive2();


/******************************************************************************
 * Function Name: uart_task
 ******************************************************************************
 * Summary:
 *  Task for handling initialization & connection of UART.
 *
 * Parameters:
 *  void *pvParameters : Task parameter defined during task creation (unused)
 *
 * Return:
 *  void
 *
 ******************************************************************************/
void uart_task1(void *pvParameters) {

    /* To avoid compiler warnings */
    (void) pvParameters;
    uint32_t ulNotificationValue;
    xTaskToNotify_UART_1 = NULL;

    while (true) {
        /* Start the receiving from UART. */
        UART_receive1();
        /* Wait to be notified that the receive is complete.  Note
         the first parameter is pdTRUE, which has the effect of clearing
         the task's notification value back to 0, making the notification
         value act like a binary (rather than a counting) semaphore.  */
        // printf("UART task running 1\n");
        
        ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (ulNotificationValue == 1) {
            /* Handle received data */
            while (uart_is_readable(UART_ID_1)) 
            {
                rxChar_1 = uart_getc(UART_ID_1);
                
                if (uart_is_writable(UART_ID_1))
                {
                    rxChar_1++;
                    uart_putc(UART_ID_1, rxChar_1);
                }
            }
        }
    }
}

void uart_task2_receive(void *pvParameters) {

    /* To avoid compiler warnings */
    (void) pvParameters;
    uint32_t ulNotificationValue;
    xTaskToNotify_UART_2 = NULL;

    while (true) {
        /* Start the receiving from UART. */
        UART_receive2();
        /* Wait to be notified that the receive is complete.  Note
         the first parameter is pdTRUE, which has the effect of clearing
         the task's notification value back to 0, making the notification
         value act like a binary (rather than a counting) semaphore.  */
        // printf("UART task running 2\n");
        
        ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (ulNotificationValue == 1) {
            /* Handle received data */
            char* buffer = malloc(100);
            int i = 0;
            while (uart_is_readable(UART_ID_2)) 
            {
                buffer[i++] = uart_getc(UART_ID_2);
            }
            // Check msg integrity start with ## and end with ##
            if (buffer[0] == '#' && buffer[1] == '#') {
                if (buffer[i-2] == '#' && buffer[i-1] == '#') {
                    // Parse double value from buffer
                    char* ptr;
                    double calculated_speed_ = strtod(buffer+2, &ptr);
                    if (mutex_calculated_speed != NULL) {
                        if (xSemaphoreTake(mutex_calculated_speed, portMAX_DELAY) == pdTRUE) {
                            calculated_speed = calculated_speed_;
                            xSemaphoreGive(mutex_calculated_speed);
                        }
                    }
                }
            }


        }
    }
}

void uart_task2_send(void *pvParameters) {
    /* To avoid compiler warnings */
    (void) pvParameters;
    uint32_t ulNotificationValue;
    xTaskToNotify_UART_2 = NULL;

    int count = 0;
    double calculated_speed_;
    int speed_;
    int direction_;
    char engine_state_;
    int buttons_;
    while (1) {
        // Handle Rate 10ms
        if (mutex_direction != NULL) {
            if (xSemaphoreTake(mutex_direction, portMAX_DELAY) == pdTRUE) {
                direction_ = direction;
                xSemaphoreGive(mutex_direction);
            }
        }
        if (mutex_speed != NULL) {
            if (xSemaphoreTake(mutex_speed, portMAX_DELAY) == pdTRUE) {
                speed_ = speed;
                xSemaphoreGive(mutex_speed);
            }
        }
        if (count != 0 && uart_is_writable(UART_ID_2))
        {
            char* buffer = malloc(100);
            sprintf(buffer, "##%d,%d##", speed_, direction_);
            uart_puts(UART_ID_2, buffer);
        }

        // Handle Rate 50ms
        if (count == 0 && uart_is_writable(UART_ID_2)) {
            if (mutex_buttons_state != NULL) {
                if (xSemaphoreTake(mutex_buttons_state, portMAX_DELAY) == pdTRUE) {
                    buttons_ = buttons;
                    xSemaphoreGive(mutex_buttons_state);
                }
            }
            if (mutex_engine_state != NULL) {
                if (xSemaphoreTake(mutex_engine_state, portMAX_DELAY) == pdTRUE) {
                    engine_state_ = engine_state;
                    xSemaphoreGive(mutex_engine_state);
                }
            }
            char* buffer = malloc(100);
            sprintf(buffer, "##%d,%d,%c,%d##", speed_, direction_, engine_state_, buttons_);
            uart_puts(UART_ID_2, buffer);
        }
        vTaskDelay(10);
        count = (count + 1) % 5;
    }
}

void initUART1() {
    uart_init(UART_ID_1, BAUD_RATE);
    gpio_set_function(UART_TX_PIN_1, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN_1, GPIO_FUNC_UART);
    // Set UART flow control CTS/RTS, we don't want these, so turn them off
	uart_set_hw_flow(UART_ID_1, false, false);
    // Set data format
    uart_set_format(UART_ID_1, DATA_BITS, STOP_BITS, PARITY);        
    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID_1, false);
    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID_1 == uart0 ? UART0_IRQ : UART1_IRQ;
    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, UART_Isr1);
    irq_set_enabled(UART_IRQ, true);
}
void initUART2() {
    uart_init(UART_ID_2, BAUD_RATE);
    gpio_set_function(UART_TX_PIN_2, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN_2, GPIO_FUNC_UART);
    // Set UART flow control CTS/RTS, we don't want these, so turn them off
	uart_set_hw_flow(UART_ID_2, false, false);
    // Set data format
    uart_set_format(UART_ID_2, DATA_BITS, STOP_BITS, PARITY);        
    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID_2, false);
    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID_2 == uart0 ? UART0_IRQ : UART1_IRQ;
    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, UART_Isr2);
    irq_set_enabled(UART_IRQ, true);
}

// UART interrupt handler
void UART_Isr1() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Now disable the UART to send interrupts
    uart_set_irq_enables(UART_ID_1, false, false);

    if (xTaskToNotify_UART_1 != NULL) {

        /* Notify the task that the receive is complete. */
        vTaskNotifyGiveFromISR(xTaskToNotify_UART_1, &xHigherPriorityTaskWoken);
        /* There are no receive in progress, so no tasks to notify. */
        xTaskToNotify_UART_1 = NULL;

        /* If xHigherPriorityTaskWoken is now set to pdTRUE then a
         context switch should be performed to ensure the interrupt
         returns directly to the highest priority task.  The macro used
         for this purpose is dependent on the port in use and may be
         called portEND_SWITCHING_ISR(). */
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
void UART_Isr2() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Now disable the UART to send interrupts
    uart_set_irq_enables(UART_ID_2, false, false);

    if (xTaskToNotify_UART_2 != NULL) {

        /* Notify the task that the receive is complete. */
        vTaskNotifyGiveFromISR(xTaskToNotify_UART_2, &xHigherPriorityTaskWoken);
        /* There are no receive in progress, so no tasks to notify. */
        xTaskToNotify_UART_2 = NULL;

        /* If xHigherPriorityTaskWoken is now set to pdTRUE then a
         context switch should be performed to ensure the interrupt
         returns directly to the highest priority task.  The macro used
         for this purpose is dependent on the port in use and may be
         called portEND_SWITCHING_ISR(). */
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// UART activate a receive with interrupt. Wait for ever for UART_BUFFER_SIZE bytes
void UART_receive1() {
    /* At this point xTaskToNotify should be NULL as no receive
     is in progress.  A mutex can be used to guard access to the
     peripheral if necessary. */
    configASSERT(xTaskToNotify_UART_1 == NULL);

    /* Store the handle of the calling task. */
    xTaskToNotify_UART_1 = xTaskGetCurrentTaskHandle();
    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID_1, true, false);
}

// UART activate a receive with interrupt. Wait for ever for UART_BUFFER_SIZE bytes
void UART_receive2() {
    /* At this point xTaskToNotify should be NULL as no receive
     is in progress.  A mutex can be used to guard access to the
     peripheral if necessary. */
    configASSERT(xTaskToNotify_UART_2 == NULL);

    /* Store the handle of the calling task. */
    xTaskToNotify_UART_2 = xTaskGetCurrentTaskHandle();
    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID_2, true, false);
}


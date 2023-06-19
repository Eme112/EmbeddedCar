/*************************************************/
// UART CODE
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "uart_task.h"

#include "hardware/uart.h"
#include "hardware/irq.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "semphr.h"

#define UART_ID_1 uart0
#define UART_ID_2 uart1
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE
// Engine
#define UART_TX_PIN_1 12
#define UART_RX_PIN_1 13
// Controller
#define UART_TX_PIN_2 8
#define UART_RX_PIN_2 9

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
// Receive from Engine
void uart_task1_receive(void *pvParameters) {
    /* To avoid compiler warnings */
    (void) pvParameters;

    char* buffer = malloc(100);
    char current_char;
    int i = 0;
    while (true) {

        // Parse the following uart input s{%d}&  m{%d}&. IdLetter then the number and it ends with &.
        while (uart_is_readable(UART_ID_1))
        {
            current_char = uart_getc(UART_ID_1);
            if (current_char == 's' || current_char == 'm') {
                i = 0;
                buffer[i++] = current_char;
                continue;
            }
            if (current_char == '&') {
                buffer[i] = '\0';
                if (buffer[0] == 's') {
                    char* ptr;
                    int calculated_speed_ = strtol(buffer+1, &ptr, 10);
                    xSemaphoreTake(mutex_speed, portMAX_DELAY);
                    calculated_speed = calculated_speed_;
                    xSemaphoreGive(mutex_speed);
                    // printf("Speed: %d\n", speed);
                }
                else if (buffer[0] == 'm') {
                    char* ptr;
                    int mileage_ = strtol(buffer+1, &ptr, 10);
                    xSemaphoreTake(mutex_mileage, portMAX_DELAY);
                    mileage = mileage_;
                    xSemaphoreGive(mutex_mileage);
                    // printf("Mileage: %d\n", mileage);
                }
                i = 0;
                continue;
            }
            // If not numeric, reset
            if ( (current_char < '0' || current_char > '9') && current_char != '-') {
                i = 0;
                continue;
            }
            buffer[i++] = current_char;
        }
        vTaskDelay(20);
    }
    free(buffer);
}

// Send to Engine
void uart_task1_send(void *pvParameters) {
    /* To avoid compiler warnings */
    (void) pvParameters;

    int speed_ = 0;
    int engine_state_ = (int)'N';
    while (1) {
        if (uart_is_writable(UART_ID_1)) {
            xSemaphoreTake( mutex_speed, portMAX_DELAY );
            speed_ = speed;
            xSemaphoreGive(mutex_speed);
            
            xSemaphoreTake( mutex_engine_state, portMAX_DELAY );
            engine_state_ = (int)engine_state;
            xSemaphoreGive(mutex_engine_state);
            
            char* buffer = malloc(100);
            sprintf(buffer, "s%d&e%d&", speed_, engine_state_);
            uart_puts(UART_ID_1, buffer);
            free(buffer);
        }
        vTaskDelay(500);
    }
}

// Receive from Controller
void uart_task2_receive(void *pvParameters) {
    /* To avoid compiler warnings */
    (void) pvParameters;

    char* buffer = malloc(100);
    char current_char;
    int i = 0;
    while (true) {
        // Parse the following uart input s{%d}&  e{%d}&. IdLetter then the number and it ends with &.
        while (uart_is_readable(UART_ID_2))
        {
            current_char = uart_getc(UART_ID_2);
            if (current_char == 's' || current_char == 'd' || current_char == 'e' || current_char == 'b') {
                i = 0;
                buffer[i++] = current_char;
                continue;
            }
            if (current_char == '&') {
                buffer[i] = '\0';
                if (buffer[0] == 's') {
                    char* ptr;
                    int speed_ = strtol(buffer+1, &ptr, 10);
                    xSemaphoreTake(mutex_speed, portMAX_DELAY);
                    speed = speed_;
                    xSemaphoreGive(mutex_speed);
                }
                else if (buffer[0] == 'd') {
                    char* ptr;
                    int direction_ = strtol(buffer+1, &ptr, 10);
                    xSemaphoreTake(mutex_direction, portMAX_DELAY);
                    direction = direction_;
                    xSemaphoreGive(mutex_direction);
                }
                else if (buffer[0] == 'e') {
                    char* ptr;
                    int engine_state_ = strtol(buffer+1, &ptr, 10);
                    xSemaphoreTake(mutex_engine_state, portMAX_DELAY);
                    engine_state = engine_state_;
                    xSemaphoreGive(mutex_engine_state);
                }
                else if (buffer[0] == 'b') {
                    char* ptr;
                    int buttons_ = strtol(buffer+1, &ptr, 10);
                    xSemaphoreTake(mutex_buttons_state, portMAX_DELAY);
                    buttons = buttons_;
                    xSemaphoreGive(mutex_buttons_state);
                }

                i = 0;
                continue;
            }
            // If not numeric, reset
            if ( (current_char < '0' || current_char > '9') && current_char != '-') {
                i = 0;
                continue;
            }
            buffer[i++] = current_char;
        }
        vTaskDelay(20);
    }
    free(buffer);
}

// Send to Controller
void uart_task2_send(void *pvParameters) {
    /* To avoid compiler warnings */
    (void) pvParameters;

    int calculated_speed_ = 0;
    int mileage_ = 0;
    while (1) {
        if (uart_is_writable(UART_ID_2)) {
            xSemaphoreTake( mutex_calculated_speed, portMAX_DELAY );
            calculated_speed_ = calculated_speed;
            xSemaphoreGive(mutex_calculated_speed);
            xSemaphoreTake(mutex_mileage, portMAX_DELAY);
            mileage_ = mileage;
            xSemaphoreGive(mutex_mileage);
            char* buffer = malloc(100);
            sprintf(buffer, "s%d&m%d&", calculated_speed_, mileage_);
            uart_puts(UART_ID_2, buffer);
            free(buffer);
        }
        vTaskDelay(200);
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
    uart_set_fifo_enabled(UART_ID_1, xTaskGetCurrentTaskHandle);
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
    uart_set_fifo_enabled(UART_ID_2, true);
}

/*************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <math.h>
#include <time.h>
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "uart_task.h"
#include "semphr.h"

// PWM Definitions
#define PWM_PIN 16
#define SLICE_NUM 0
#define CHANNEL_NUM 0

// Engine Definitions
#define CTRLA 17
#define CTRLB 18
#define LOWEST_SPEED 0

// Masks for Pins
#define RED1_MASK 0x01
#define RED2_MASK 0x02
#define FRONT1_MASK 0x08
#define FRONT2_MASK 0x10
#define YELLOW1_MASK 0x04 // 0000 0010 
#define YELLOW2_MASK 0x20 // 0001 0000 
#define INIT_MASK 0x7F

// Output Pins
#define RED1 0
#define RED2 1
#define YELLOW1 2
#define FRONT1 3 
#define FRONT2 4 
#define YELLOW2 5
#define HORN 6

// Change
#define LEFT_LIGHTS_PIN 7
#define RIGHT_LIGHTS_PIN 14
#define HAZARD_PIN 15
#define FRONT_LIGHTS_PIN 10
#define HORN_PIN 11

/*
 * Configure the hardware as necessary
 */
static void prvSetupHardware();

void vBlinkTask()
{
	for(;;) 
	{
		gpio_put(PICO_DEFAULT_LED_PIN, 1);
		// printf("LED ON\n");
		vTaskDelay(200);
		gpio_put(PICO_DEFAULT_LED_PIN, 0);
		// printf("LED OFF\n");
		vTaskDelay(200);
	}
}

static void prvSetupHardware() {
    stdio_init_all();
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, 1);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    initUART1();
    initUART2();

    gpio_init(CTRLA);
	gpio_set_dir(CTRLA, GPIO_OUT);
	gpio_init(CTRLB);
	gpio_set_dir(CTRLB, GPIO_OUT);
	gpio_init_mask(INIT_MASK);
	gpio_set_dir_masked(INIT_MASK, INIT_MASK);

	// Change
	gpio_init(LEFT_LIGHTS_PIN);
	gpio_set_dir(LEFT_LIGHTS_PIN, GPIO_IN);
	gpio_init(RIGHT_LIGHTS_PIN);
	gpio_set_dir(RIGHT_LIGHTS_PIN, GPIO_IN);
	gpio_init(HAZARD_PIN);
	gpio_set_dir(HAZARD_PIN, GPIO_IN);
	gpio_init(FRONT_LIGHTS_PIN);
	gpio_set_dir(FRONT_LIGHTS_PIN, GPIO_IN);
	gpio_init(HORN_PIN);
	gpio_set_dir(HORN_PIN, GPIO_IN);

	// Set up the GPIO pin for PWM output
	gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);

	// Set up the PWM slice and channel
	pwm_set_wrap(SLICE_NUM, 500);				   // Set the period of the PWM signal (e.g., 1000 cycles)
	pwm_set_chan_level(SLICE_NUM, CHANNEL_NUM, 0); // Set the initial duty cycle to 0

	// Enable the PWM slice
	pwm_set_enabled(SLICE_NUM, true);
}

void steeringLogic()
{
	int steering_ = 0; // between -100 and 100
	// change
	int count = 0; bool right = true;
	for (;;)
	{
		xSemaphoreTake(mutex_direction, portMAX_DELAY);
        steering_ = direction;
        xSemaphoreGive(mutex_direction);

		// change
		count++;
		// Direction of steering_
		if (steering_ < 0) {
			gpio_put(CTRLA, 1);
			gpio_put(CTRLB, 0);
		} else {
			gpio_put(CTRLA, 0);
			gpio_put(CTRLB, 1);
		}

		// Write to PWM
		// printf("count: %d\tright: %d\n", count, right);
        if (abs(steering_) < 15) {
            steering_ = 0;
        }  else {
            steering_ = abs(steering_) + 400;
        }
		pwm_set_chan_level(SLICE_NUM, CHANNEL_NUM, steering_);
		pwm_set_enabled(SLICE_NUM, true);

		vTaskDelay(10);
	}
}

void setLights(uint8_t val, char side)
{
	if (side == 'R' || side == 'B')
	{
		gpio_put(RED2, val);
		gpio_put(YELLOW2, val);
	}
	if (side == 'L' || side == 'B')
	{
		gpio_put(RED1, val);
		gpio_put(YELLOW1, val);
	}
}

void rearLightLogic()
{
	int curr_time = 0;
	int prev_time = 0;
	// Commands
	bool hazard = false;
	bool right_dir = true;
	bool directionals = false;
	// Current
    bool beam_curr = false;
    bool horn_curr = false;
	bool hazard_curr = false;
	bool right_curr = false;
	bool left_curr = false;
	// Previous
	bool hazard_prev = false;
	bool right_prev = false;
	bool left_prev = false;
	// Toggle
	bool hazard_toggle = false;
	bool right_toggle = false;
	bool left_toggle = false;
    // Steering
    int steering_ = 0;

	setLights(0, 'B');
    int buttons_ = 0;
	for (;;)
	{

        xSemaphoreTake(mutex_buttons_state, portMAX_DELAY);
        buttons_ = buttons;
        xSemaphoreGive(mutex_buttons_state);
		xSemaphoreTake(mutex_direction, portMAX_DELAY);
        steering_ = direction;
        xSemaphoreGive(mutex_direction);

        right_curr = buttons_ & 0x01;
        left_curr = (buttons_ & 0x02) >> 1;
        hazard_curr = (buttons_ & 0x04) >> 2;
        horn_curr = (buttons_ & 0x08) >> 3;
        beam_curr = (buttons_ & 0x10) >> 4;

		// hazard_curr = gpio_get(HAZARD_PIN);
		// right_curr = gpio_get(RIGHT_LIGHTS_PIN);
		// left_curr = gpio_get(LEFT_LIGHTS_PIN);

		
		// Check changes in inputs
		if (hazard_curr != hazard_prev && hazard_curr == 1)
			hazard_toggle = true;
		hazard_prev = hazard_curr;

		if (right_curr != right_prev && right_curr == 1)
			right_toggle = true;
		right_prev = right_curr;

		if (left_curr != left_prev && left_curr == 1)
			left_toggle = true;
		left_prev = left_curr;

		// Toggle hazards if required
		if (hazard_toggle)
		{
			setLights(0, 'B');
			hazard = !hazard;
			hazard_toggle = false;
		}

		// Turn on directionals if hazard is off
		if (!hazard)
		{
			if (right_toggle)
			{
				setLights(0, 'L');
				if (directionals)
					directionals = (right_dir) ? false : true;
				else
					directionals = true;
				right_dir = true;
				right_toggle = false;
			}
			if (left_toggle)
			{
				setLights(0, 'R');
				if (directionals)
					directionals = (right_dir) ? true : false;
				else
					directionals = true;
				right_dir = false;
				left_toggle = false;
			}
		}
		else
		{
			directionals = false;
		}

        // If making a turn, turn off hazards
        if (directionals && right_dir && steering_ < 0)
            directionals = false;
        if (directionals && !right_dir && steering_ > 0)
            directionals = false;

		// Check if 500ms has passed
		curr_time = to_ms_since_boot(get_absolute_time());
		if (curr_time - prev_time >= 500)
		{
			if (hazard)
			{
				gpio_xor_mask(RED2_MASK | YELLOW2_MASK | RED1_MASK | YELLOW1_MASK);
			}
			else if (directionals)
			{
				if (right_dir)
				{
					gpio_xor_mask(RED2_MASK | YELLOW2_MASK);
				}
				else
				{
					gpio_xor_mask(RED1_MASK | YELLOW1_MASK);
				}
			}
			else
			{
				setLights(0, 'B');
			}
			prev_time = curr_time;
		}
		vTaskDelay(50);
	}

	// xQueueSend(xCommandQueue, (void*)&command, (TickType_t)0);
	// xQueueSend(xGearQueue, (void*)&gear_value, (TickType_t)0);
}

void frontLightLogic()
{
	uint8_t toggle = 0;
	gpio_put(FRONT1, 0);
	gpio_put(FRONT2, 0);

    int buttons_ = 0;
    uint8_t lights_toggle = 0;
	for (;;)
	{
		xSemaphoreTake(mutex_buttons_state, portMAX_DELAY);
        buttons_ = buttons;
        xSemaphoreGive(mutex_buttons_state);
        lights_toggle = (buttons_ & 0x10) >> 4;
		
        // uint8_t lights_toggle = gpio_get(FRONT_LIGHTS_PIN);
		// printf("Lights toggle: %d\n", lights_toggle);

		if (toggle == 0 && lights_toggle == 1)
		{
			gpio_xor_mask(FRONT1_MASK | FRONT2_MASK);
			toggle = 1;
		}
		else if (toggle == 1 && lights_toggle == 0)
		{
			toggle = 0;
		}
		vTaskDelay(50);
	}
}

void hornLogic()
{
	uint8_t horn = 0;
	gpio_put(HORN, 0);
    int buttons_ = 0;
	for (;;)
	{
        xSemaphoreTake(mutex_buttons_state, portMAX_DELAY);
        buttons_ = buttons;
        xSemaphoreGive(mutex_buttons_state);
        horn = (buttons_ & 0x08) >> 3;
		// horn = gpio_get(HORN_PIN);
		// printf("Horn: %d\n", horn);

		if (horn == 0)
		{
			gpio_put(HORN, 0);
		}
		else
		{
			gpio_put(HORN, 1);
		}
		vTaskDelay(50);
	}
}


void main()
{
	/* Configure the hardware */
	prvSetupHardware();

	
    mutex_calculated_speed = xSemaphoreCreateMutex();
    mutex_speed = xSemaphoreCreateMutex();
    mutex_direction = xSemaphoreCreateMutex();
    mutex_engine_state = xSemaphoreCreateMutex();
    mutex_buttons_state = xSemaphoreCreateMutex();
    mutex_mileage = xSemaphoreCreateMutex();

	// Create Tasks
	xTaskCreate(vBlinkTask, "Blink Task", 128, NULL, 3, NULL);
	/* Create the UART task. */
	xTaskCreate(uart_task1_receive, "UART Engine Receive task", UART_TASK_STACK_SIZE,
			NULL, UART_TASK_PRIORITY, NULL);
    xTaskCreate(uart_task1_send, "UART Engine Send task", UART_TASK_STACK_SIZE,
			NULL, UART_TASK_PRIORITY, NULL);
	/* Create the UART task. */
	xTaskCreate(uart_task2_receive, "UART Controller Receive task", UART_TASK_STACK_SIZE,
			NULL, UART_TASK_PRIORITY, NULL);
    xTaskCreate(uart_task2_send, "UART Controller Send task", UART_TASK_STACK_SIZE,
			NULL, UART_TASK_PRIORITY, NULL);
    
    xTaskCreate(steeringLogic, "Steering Logic", 128, NULL, 1, NULL);
	xTaskCreate(rearLightLogic, "Rear Light Logic", 1024, NULL, 1, NULL);
	xTaskCreate(frontLightLogic, "Front Lights Logic", 128, NULL, 1, NULL);
	xTaskCreate(hornLogic, "Horn Logic", 128, NULL, 1, NULL);

	// Start scheduler
	vTaskStartScheduler();

	for (;;) {}
}

void vApplicationMallocFailedHook() {
    /* Called if a call to pvPortMalloc() fails because there is insufficient
    free memory available in the FreeRTOS heap.  pvPortMalloc() is called
    internally by FreeRTOS API functions that create tasks, queues, software
    timers, and semaphores.  The size of the FreeRTOS heap is set by the
    configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

    /* Force an assert. */
    configASSERT((volatile void *)NULL);
}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) {
    (void)pcTaskName;
    (void)pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */

    /* Force an assert. */
    configASSERT((volatile void *)NULL);
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook() {
    volatile size_t xFreeHeapSpace;

    /* This is just a trivial example of an idle hook.  It is called on each
    cycle of the idle task.  It must *NOT* attempt to block.  In this case the
    idle task just queries the amount of FreeRTOS heap that remains.  See the
    memory management section on the http://www.FreeRTOS.org web site for memory
    management options.  If there is a lot of heap memory free then the
    configTOTAL_HEAP_SIZE value in FreeRTOSConfig.h can be reduced to free up
    RAM. */
    xFreeHeapSpace = xPortGetFreeHeapSize();

    /* Remove compiler warning about xFreeHeapSpace being set but never used. */
    ( void ) xFreeHeapSpace;
}



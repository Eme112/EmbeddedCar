#include <stdio.h>
#include <stdlib.h> 
#include <math.h>
#include <time.h>
#include "FreeRTOS.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "pico/stdlib.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"

// PWM Definitions
#define PWM_PIN 16
#define SLICE_NUM 0
#define CHANNEL_NUM 0

// Engine Definitions
#define CTRLA 17
#define CTRLB 18
#define LOWEST_SPEED 650 // Might need to be adjusted for better performance
#define NEUTRAL_RATE (1000.0 - LOWEST_SPEED) / 3000.0
#define FS2_RATE (1000.0 - LOWEST_SPEED) / 3000.0
#define FS1_RATE (1000.0 - LOWEST_SPEED) / 5000.0
#define REVERSE_RATE (1000.0 - LOWEST_SPEED) / 5000.0
#define FS2_MAX_SPEED 1000
#define FS1_MAX_SPEED (FS2_MAX_SPEED + LOWEST_SPEED) / 2.0
#define REVERSE_MAX_SPEED (FS2_MAX_SPEED + LOWEST_SPEED) / 3.0

// Encoder Definitions
#define SENSOR_PIN  2     // GPIO pin for the LM393 speed sensor
#define WHEEL_RADIUS  0.062 // Radius of the wheel in meters
#define PI 3.14159265358979323846
#define MS_TO_MH 3600.0

// UART Definitions
#define UART_TASK_PRIORITY (2)
#define UART_TASK_STACK_SIZE (1024 * 3)
#define UART_BUFFER_SIZE 26
#define UART_ID_1 uart0
#define UART_ID_2 uart1
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

#define UART_TX_PIN_2 8
#define UART_RX_PIN_2 9

// Variables
volatile uint32_t pulse_counter = 0;
SemaphoreHandle_t pulse_counter_mutex;

// Data to be sent and received
static SemaphoreHandle_t mutex_calculated_speed;
static int calculated_speed = 0.0;
static SemaphoreHandle_t mutex_mileage;
static int mileage = 0.0;
static SemaphoreHandle_t mutex_speed;
static int speed = 0.0;
static SemaphoreHandle_t mutex_filtered_speed;
static int filtered_speed = 0.0;
static SemaphoreHandle_t mutex_engine_state;
static char engine_state = 'N';
static void prvSetupHardware();

// Queue Definitions
// QueueHandle_t xCommandQueue = NULL;
// QueueHandle_t xCommandFilteredQueue = NULL;
// QueueHandle_t xGearQueue = NULL;
// QueueHandle_t xVelocityQueue = NULL;
// QueueHandle_t xDistanceQueue = NULL;

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

static void prvSetupHardware() {
    // stdio_init_all();
    // gpio_init(PICO_DEFAULT_LED_PIN);
    // gpio_set_dir(PICO_DEFAULT_LED_PIN, 1);
    // gpio_put(PICO_DEFAULT_LED_PIN, 0);
    initUART2();
}

void vBlinkTask()
{
	for (;;)
	{
		gpio_put(PICO_DEFAULT_LED_PIN, 1);
		// printf("LED ON\n");
		vTaskDelay(500);
		gpio_put(PICO_DEFAULT_LED_PIN, 0);
		// printf("LED OFF\n");
		vTaskDelay(500);
	}
}

void inputFiltering()
{
	int command = 0;	// Between -100 and 100
	int last_commands[10] = {0};
	int command_count = 0;
	int command_filtered = 0;
	int command_accum = 0;

	for (;;)
	{
		// Filter command value
		last_commands[command_count] = command;
		if(command_count == 9) {
			for(int i = 0; i < 10; i++) {
				command_accum += last_commands[i];
			}
			command_count = 0;
			command_filtered = command_accum / 10;
			command_accum = 0;
		} else {
			command_count++;
		}

		xSemaphoreTake(mutex_filtered_speed, portMAX_DELAY);
		filtered_speed = command_filtered;
		xSemaphoreGive(mutex_filtered_speed);
		
		vTaskDelay(100);
	}
}

void engineControl()
{
	int8_t command = 0;	// Between -100 and 100
	uint8_t gear_value = 0; // 0 = neutral, 1 = FS1, 2 = FS2, 3 = Reverse
	float speed_ = 1000;		// Between LOWEST_SPEED and 1000.0
	bool forward = true;	// True = forward, False = reverse

	for (;;)
	{
		// if( xCommandFilteredQueue != NULL )
		// 	xQueueReceive( xCommandFilteredQueue, &command, (TickType_t)10);
		// if( xGearQueue != NULL )
		// 	xQueueReceive( xGearQueue, &gear_value, (TickType_t)10);

		// Recieve command value and gear value
		xSemaphoreTake(mutex_filtered_speed, portMAX_DELAY);
		command = filtered_speed;
		xSemaphoreGive(mutex_filtered_speed);
		xSemaphoreTake(mutex_engine_state, portMAX_DELAY);
		gear_value = engine_state;
		xSemaphoreGive(mutex_engine_state);

		if((char)gear_value == 'N') gear_value = 0;
		else if((char)gear_value == '1') gear_value = 1;
		else if((char)gear_value == '2') gear_value = 2;
		else if((char)gear_value == 'R') gear_value = 3;
		else gear_value = 0;

		// printf("Command: %d\n", command);
		// printf("Gear: %d\n", gear_value);

		switch (gear_value)
		{
		case 1: // FS1
			if (!forward) { // If in reverse, stop and then go forward
				if (speed_ > LOWEST_SPEED)
					speed_ -= FS1_RATE*100;
				else
					forward = true;
			} else {
				if (speed_ > FS1_MAX_SPEED) // If going faster than max speed_, slow down
					speed_ -= FS1_RATE*100;
				else {
					if (speed_ < FS1_MAX_SPEED)
						speed_ += (command >= 0) ? FS1_RATE*abs(command) : -FS1_RATE*abs(command); // If braking, slow down
					if (speed_ >= FS1_MAX_SPEED) {
						speed_ = FS1_MAX_SPEED;
					}
				}
			}
			break;
		case 2: // FS2
			if (!forward) { // If in reverse, stop and then go forward
				if (speed_ > LOWEST_SPEED)
					speed_ -= FS2_RATE*100;
				else
					forward = true;
			} else {
				if (speed_ < FS2_MAX_SPEED)
					speed_ += (command >= 0) ? FS2_RATE*abs(command) : -FS2_RATE*abs(command); // If braking, slow down
				if (speed_ >= FS2_MAX_SPEED) {
					speed_ = FS2_MAX_SPEED;
				}
			}
			break;
		case 3: // Reverse
			if (forward) { // If going forward, stop and then go reverse
				if (speed_ > LOWEST_SPEED)
					speed_ -= REVERSE_RATE*100;
				else
					forward = false;
			} else {
				if (speed_ < REVERSE_MAX_SPEED)
					speed_ += (command >= 0) ? REVERSE_RATE*abs(command) : -REVERSE_RATE*abs(command); // If braking, slow down
				if (speed_ >= REVERSE_MAX_SPEED) {
					speed_ = REVERSE_MAX_SPEED;
				}
			}
			break;
		default: // Neutral
			if (speed_ > LOWEST_SPEED)
			{
				speed_ -= NEUTRAL_RATE*100;
			}
			break;
		}

		if (abs(speed_) < LOWEST_SPEED) { // If speed_ is below lowest speed_, stop
			speed_ = LOWEST_SPEED;
			pwm_set_chan_level(SLICE_NUM, CHANNEL_NUM, 0); // Set speed_
			pwm_set_enabled(SLICE_NUM, true);
		} else { // Otherwise, set speed_
			pwm_set_chan_level(SLICE_NUM, CHANNEL_NUM, (int)speed_); // Set speed_
			pwm_set_enabled(SLICE_NUM, true);
		}

		if (forward) { // Set direction
			gpio_put(CTRLA, 1);
			gpio_put(CTRLB, 0);
		} else {
			gpio_put(CTRLA, 0);
			gpio_put(CTRLB, 1);
		}

		// Print gear and speed_
		// int speed_int = (int)speed_;
		// int speed_dec = (int)((speed_ - speed_int) * 100);
		// printf("F:%d G:%d C:%d S:%d.%d t=%d\n", forward, gear_value, command, speed_int, speed_dec, time(NULL));

		vTaskDelay(100);
	}
}

// LM393 speed sensor interrupt handler
void sensorInterruptHandler()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Increment the pulse counter
    xSemaphoreTakeFromISR(pulse_counter_mutex, &xHigherPriorityTaskWoken);
    pulse_counter++;
    xSemaphoreGiveFromISR(pulse_counter_mutex, &xHigherPriorityTaskWoken);

	//printf("Counter: %d \n", pulse_counter);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// FreeRTOS task to calculate linear velocity
void velocityTask(void *parameter)
{
    (void)parameter; // Unused parameter

	int current_pulse_count = 0;
    int previous_pulse_count = 0;
    float linear_velocity = 0.0;
	float distance = 0.0;
	float total_distance = 0.0;
	int u_linear_velocity = 0;
	int u_total_distance = 0;

    for (;;)
    {
        // Read pulse count
        xSemaphoreTake(pulse_counter_mutex, portMAX_DELAY);
        current_pulse_count = pulse_counter;
        xSemaphoreGive(pulse_counter_mutex);

		//printf("Counter from Task: %d \n", current_pulse_count);

        // Calculate linear velocity and distance travelled
        distance = (current_pulse_count - previous_pulse_count)/20.0* (PI* WHEEL_RADIUS);
		// printf("Delta distance: %.2f ")
		total_distance += distance;
		linear_velocity = distance/0.05 * MS_TO_MH;

        // Store the current pulse count for the next iteration
        previous_pulse_count = current_pulse_count;

        // Print Linear Velocity
        // printf("Linear Velocity: %.2f m/h\n", linear_velocity);

		// Convert to uint8_t
		u_linear_velocity = linear_velocity;
		u_total_distance = total_distance;

		// Send to queues
		// xQueueSendToBack(xVelocityQueue, &u_linear_velocity, portMAX_DELAY);
		// xQueueSendToBack(xDistanceQueue, &u_total_distance, portMAX_DELAY);

		// Send variables
		xSemaphoreTake(mutex_calculated_speed, portMAX_DELAY);
		calculated_speed = u_linear_velocity;
		xSemaphoreGive(mutex_calculated_speed);
		xSemaphoreTake(mutex_mileage, portMAX_DELAY);
		mileage = u_total_distance;
		xSemaphoreGive(mutex_mileage);

        vTaskDelay(50);
    }
}

// Receive from Chasis
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
            if (current_char == 's' || current_char == 'e') {
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
                    printf("Speed: %d\n", speed_);
                } else if (buffer[0] == 'e') {
                    char* ptr;
                    int engine_state_ = strtol(buffer+1, &ptr, 10);
                    xSemaphoreTake(mutex_engine_state, portMAX_DELAY);
                    engine_state = (char)engine_state_;
                    xSemaphoreGive(mutex_engine_state);
                    printf("Engine State: %c\n", engine_state_);
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
        vTaskDelay(10);
    }
    free(buffer);
}

// Send to Chasis
void uart_task2_send(void *pvParameters) {
    /* To avoid compiler warnings */
    (void) pvParameters;

    int calculated_speed_ = 0;
    int mileage_ = 0;
    while (1) {
		// Receive data from queue
		
		
        if (uart_is_writable(UART_ID_2)) {
            // xQueueReceive(xVelocityQueue, &calculated_speed_, portMAX_DELAY);
            // xQueueReceive(xDistanceQueue, &mileage_, portMAX_DELAY);
			xSemaphoreTake(mutex_calculated_speed, portMAX_DELAY);
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
        vTaskDelay(50);
    }
}

void main()
{
	// Initialize the standard I/O library
	stdio_init_all();

	// Initialize I/O
	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
	gpio_init(CTRLA);
	gpio_set_dir(CTRLA, GPIO_OUT);
	gpio_init(CTRLB);
	gpio_set_dir(CTRLB, GPIO_OUT);
	stdio_init_all();

	/* Configure the hardware */
	prvSetupHardware();

	mutex_calculated_speed = xSemaphoreCreateMutex();
    mutex_speed = xSemaphoreCreateMutex();
    mutex_engine_state = xSemaphoreCreateMutex();
    mutex_mileage = xSemaphoreCreateMutex();
	mutex_filtered_speed = xSemaphoreCreateMutex();

	// Set up the GPIO pin for PWM output
	gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);

	// Set up the PWM slice and channel
	pwm_set_wrap(SLICE_NUM, 1000);				   // Set the period of the PWM signal (e.g., 1000 cycles)
	pwm_set_chan_level(SLICE_NUM, CHANNEL_NUM, 0); // Set the initial duty cycle to 0

	// Enable the PWM slice
	pwm_set_enabled(SLICE_NUM, true);

	// Initialize pulse counter mutex
    pulse_counter_mutex = xSemaphoreCreateMutex();

    // Set up LM393 speed sensor interrupt
    gpio_init(SENSOR_PIN);
    gpio_set_dir(SENSOR_PIN, GPIO_IN);

	// Initialization of Queues
	// xCommandQueue = xQueueCreate(100, sizeof(int8_t));
	// xCommandFilteredQueue = xQueueCreate(20, sizeof(int8_t));
	// xGearQueue = xQueueCreate(20, sizeof(uint8_t));
	// xVelocityQueue = xQueueCreate(20, sizeof(uint8_t));
	// xDistanceQueue = xQueueCreate(20, sizeof(uint8_t));

	gpio_set_irq_enabled_with_callback(SENSOR_PIN, GPIO_IRQ_EDGE_RISE, true, &sensorInterruptHandler);

	// Create Tasks
	xTaskCreate(vBlinkTask, "Blink Task", 128, NULL, 1, NULL);
	xTaskCreate(inputFiltering, "Input Filtering", 512, NULL, 1, NULL);
	xTaskCreate(engineControl, "Engine Control", 511, NULL, 1, NULL);
	xTaskCreate(velocityTask, "VelocityTask", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(uart_task2_receive, "UART Chasis Receive task", UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY + 1, NULL);
    xTaskCreate(uart_task2_send, "UART Chasis Send task", UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, NULL);

	// Start scheduler
	vTaskStartScheduler();
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
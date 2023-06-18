#include <stdio.h>
#include <stdlib.h> 
#include <math.h>
#include <time.h>
#include "FreeRTOS.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "semphr.h"
#include "task.h"

// PWM Definitions
#define PWM_PIN 16
#define SLICE_NUM 0
#define CHANNEL_NUM 0

// Engine Definitions
#define CTRLA 17
#define CTRLB 18
#define LOWEST_SPEED 0
#define NEUTRAL_RATE (1000.0 - LOWEST_SPEED) / 3000.0
#define FS2_RATE (1000.0 - LOWEST_SPEED) / 3000.0
#define FS1_RATE (1000.0 - LOWEST_SPEED) / 5000.0
#define REVERSE_RATE (1000.0 - LOWEST_SPEED) / 5000.0
#define FS2_MAX_SPEED 1000
#define FS1_MAX_SPEED (FS2_MAX_SPEED + LOWEST_SPEED) / 2.0
#define REVERSE_MAX_SPEED (FS2_MAX_SPEED + LOWEST_SPEED) / 3.0

// Encoder Definitions
#define WHEEL_RADIUS  0.062 // Radius of the wheel in meters
#define SENSOR_PIN  2     // GPIO pin for the LM393 speed sensor
#define PI 3.14159265358979323846
#define MS_TO_KMH 3.6

// Variables
volatile uint32_t pulse_counter = 0;
SemaphoreHandle_t pulse_counter_mutex;

// Queue Definitions
QueueHandle_t xCommandQueue = NULL;
QueueHandle_t xGearQueue = NULL;

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

void serialCom()
{
	for (;;)
	{
		printf("Hello from SerialCom\n");
		vTaskDelay(1000);
	}
}

void inputFiltering()
{
	for (;;)
	{
		int8_t command = rand() % 200 - 100;
		uint8_t gear_value = rand() % 4;

		xQueueSend(xCommandQueue, (void*)&command, (TickType_t)0);
		xQueueSend(xGearQueue, (void*)&gear_value, (TickType_t)0);

		vTaskDelay(100);
	}
}

void engineControl()
{
	// Change
	srand(time(NULL));
	// Change
	int8_t command = 0;	// Between -100 and 100
	// Change
	uint8_t gear_value = 0; // 0 = neutral, 1 = FS1, 2 = FS2, 3 = Reverse
	float speed = 1000;		// Between LOWEST_SPEED and 1000.0
	bool forward = true;	// True = forward, False = reverse

	for (;;)
	{
		if( xCommandQueue != NULL )
			xQueueReceive( xCommandQueue, &command, (TickType_t)10);
		if( xGearQueue != NULL )
			xQueueReceive( xGearQueue, &gear_value, (TickType_t)10);


		// change
		gpio_put(CTRLA, 1);
		gpio_put(CTRLB, 0);
		for(int i = 0; i < 500; i++) {
			pwm_set_gpio_level(PWM_PIN, i);
			vTaskDelay(50);
			printf("PWM: %d\n", i);
		}

		printf("Command: %d\n", command);
		printf("Gear: %d\n", gear_value);


		switch (gear_value)
		{
		case 1: // FS1
			if (!forward) { // If in reverse, stop and then go forward
				if (speed > LOWEST_SPEED)
					speed -= FS1_RATE*100;
				else
					forward = true;
			} else {
				if (speed > FS1_MAX_SPEED) // If going faster than max speed, slow down
					speed -= FS1_RATE*100;
				else {
					if (speed < FS1_MAX_SPEED)
						speed += (command >= 0) ? FS1_RATE*abs(command) : -FS1_RATE*abs(command); // If braking, slow down
					if (speed >= FS1_MAX_SPEED) {
						// Change
						// random number between 0 and 3
						gear_value = rand() % 4;
						speed = FS1_MAX_SPEED;
					}
				}
			}
			break;
		case 2: // FS2
			if (!forward) { // If in reverse, stop and then go forward
				if (speed > LOWEST_SPEED)
					speed -= FS2_RATE*100;
				else
					forward = true;
			} else {
				if (speed < FS2_MAX_SPEED)
					speed += (command >= 0) ? FS2_RATE*abs(command) : -FS2_RATE*abs(command); // If braking, slow down
				if (speed >= FS2_MAX_SPEED) {
					// Change
					// random number between 0 and 3
					gear_value = rand() % 4;
					speed = FS2_MAX_SPEED;
				}
			}
			break;
		case 3: // Reverse
			if (forward) { // If going forward, stop and then go reverse
				if (speed > LOWEST_SPEED)
					speed -= REVERSE_RATE*100;
				else
					forward = false;
			} else {
				if (speed < REVERSE_MAX_SPEED)
					speed += (command >= 0) ? REVERSE_RATE*abs(command) : -REVERSE_RATE*abs(command); // If braking, slow down
				if (speed >= REVERSE_MAX_SPEED) {
					// Change
					// random number between 0 and 3
					gear_value = rand() % 4;
					speed = REVERSE_MAX_SPEED;
				}
			}
			break;
		default: // Neutral
			if (speed > LOWEST_SPEED)
			{
				speed -= NEUTRAL_RATE*100;
			}
			break;
		}

		if (abs(speed) < LOWEST_SPEED) { // If speed is below lowest speed, stop
			speed = LOWEST_SPEED;
			pwm_set_chan_level(SLICE_NUM, CHANNEL_NUM, 0); // Set speed
		} else { // Otherwise, set speed
			pwm_set_chan_level(SLICE_NUM, CHANNEL_NUM, (int)speed); // Set speed
		}

		if (forward) { // Set direction
			gpio_put(CTRLA, 1);
			gpio_put(CTRLB, 0);
		} else {
			gpio_put(CTRLA, 0);
			gpio_put(CTRLB, 1);
		}

		// Print gear and speed
		int speed_int = (int)speed;
		int speed_dec = (int)((speed - speed_int) * 100);
		//printf("F:%d G:%d C:%d S:%d.%d t=%d\n", forward, gear_value, command, speed_int, speed_dec, time(NULL));

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

	uint32_t current_pulse_count = 0;
    uint32_t previous_pulse_count = 0;
    float linear_velocity = 0;
	float distance = 0; 

    for (;;)
    {
        // Read pulse count
        xSemaphoreTake(pulse_counter_mutex, portMAX_DELAY);
        current_pulse_count = pulse_counter;
        xSemaphoreGive(pulse_counter_mutex);

		//printf("Counter from Task: %d \n", current_pulse_count);

        // Calculate linear velocity
        distance = (current_pulse_count - previous_pulse_count)/20.0* (PI* WHEEL_RADIUS);
		// printf("Delta distance: %.2f ")
		linear_velocity = distance/0.05 * MS_TO_KMH;

        // Store the current pulse count for the next iteration
        previous_pulse_count = current_pulse_count;

        // Print Linear Velocity
        printf("Linear Velocity: %.2f km/h\n", linear_velocity);

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

	// Set up the GPIO pin for PWM output
	gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);

	// Set up the PWM slice and channel
	pwm_set_wrap(SLICE_NUM, 100);				   // Set the period of the PWM signal (e.g., 1000 cycles)
	pwm_set_chan_level(SLICE_NUM, CHANNEL_NUM, 0); // Set the initial duty cycle to 0

	// Enable the PWM slice
	pwm_set_enabled(SLICE_NUM, true);

	// Initialize pulse counter mutex
    pulse_counter_mutex = xSemaphoreCreateMutex();

    // Set up LM393 speed sensor interrupt
    gpio_init(SENSOR_PIN);
    gpio_set_dir(SENSOR_PIN, GPIO_IN);

	// Initialization of Queues
	xCommandQueue = xQueueCreate(2, sizeof(int8_t));
	xGearQueue = xQueueCreate(2, sizeof(uint8_t));

	gpio_set_irq_enabled_with_callback(SENSOR_PIN, GPIO_IRQ_EDGE_RISE, true, &sensorInterruptHandler);

	// Create Tasks
	xTaskCreate(vBlinkTask, "Blink Task", 128, NULL, 1, NULL);
	//xTaskCreate(serialCom, "Serial Com", 128, NULL, 1, NULL);
	xTaskCreate(inputFiltering, "Input Filtering", 128, NULL, 1, NULL);
	xTaskCreate(engineControl, "Engine Control", 1024, NULL, 1, NULL);
	xTaskCreate(velocityTask, "VelocityTask", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	// Start scheduler
	vTaskStartScheduler();
}
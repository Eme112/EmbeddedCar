#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

void vBlinkTask()
{
	for(;;) 
	{
		gpio_put(PICO_DEFAULT_LED_PIN, 1);
		printf("LED ON\n");
		vTaskDelay(500);
		gpio_put(PICO_DEFAULT_LED_PIN, 0);
		printf("LED OFF\n");
		vTaskDelay(500);
	}
}

void main()
{
	// Initialize I/O
	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
	stdio_init_all();
	
	// Create Tasks
	xTaskCreate(vBlinkTask, "Blink Task", 128, NULL, 1, NULL);
	
	// Start scheduler
	vTaskStartScheduler();
}

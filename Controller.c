//*****************************************************************************
// SERIAL


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

// Chasis
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
                    xSemaphoreTake(mutex_calculated_speed, portMAX_DELAY);
                    calculated_speed = calculated_speed_;
                    xSemaphoreGive(mutex_calculated_speed);
                    // printf("Speed: %d\n", calculated_speed_);
                }
                else if (buffer[0] == 'm') {
                    char* ptr;
                    int mileage_ = strtol(buffer+1, &ptr, 10);
                    xSemaphoreTake(mutex_mileage, portMAX_DELAY);
                    mileage = mileage_;
                    xSemaphoreGive(mutex_mileage);
                    // printf("Mileage: %d\n", mileage_);
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
        vTaskDelay(100);
    }
    free(buffer);
}

// Send to Chasis
void uart_task2_send(void *pvParameters) {
    /* To avoid compiler warnings */
    (void) pvParameters;


    int count = 0;
    int count2 = 0;
    int calculated_speed_;
    int speed_;
    int direction_;
    int engine_state_;
    int buttons_;
    while (1) {
        // Handle Rate 10ms   
        xSemaphoreTake(mutex_direction, portMAX_DELAY);
        direction_ = direction;
        xSemaphoreGive(mutex_direction);
        xSemaphoreTake(mutex_speed, portMAX_DELAY);
        speed_ = speed;
        xSemaphoreGive(mutex_speed);
        
        if (count != 0 && uart_is_writable(UART_ID_2))
        {
            char* buffer = malloc(100);
            sprintf(buffer, "s%d&d%d&", speed_, direction_);
            uart_puts(UART_ID_2, buffer);
            free(buffer);
        }

        // Handle Rate 50ms
        if (count == 0 && uart_is_writable(UART_ID_2)) {
            xSemaphoreTake(mutex_buttons_state, portMAX_DELAY);
            buttons_ = buttons;
            xSemaphoreGive(mutex_buttons_state);
            xSemaphoreTake(mutex_engine_state, portMAX_DELAY);
            engine_state_ = (int)engine_state;
            xSemaphoreGive(mutex_engine_state);
            char* buffer = malloc(100);
            sprintf(buffer, "s%d&d%d&e%d&b%d&", speed_, direction_, engine_state_, buttons_);
            if (count2 == 0) {
                // printf("Sending: %s\n", buffer);
                // printf("Right: %d\n", buttons_ & 0x01);
                // printf("Left: %d\n", (buttons_ & 0x02) >> 1);
                // printf("Hazard: %d\n", (buttons_ & 0x04) >> 2);
                // printf("Horn: %d\n", (buttons_ & 0x08) >> 3);
                // printf("Beam: %d\n", (buttons_ & 0x10) >> 4);
            }
            uart_puts(UART_ID_2, buffer);
            free(buffer);
        }
        vTaskDelay(10);
        count = (count + 1) % 5;
        count2 = (count2 + 1) % 100;
    }
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

//*****************************************************************************

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "ssd1306.h"

#include "uart_task_controller.h"

const int width = 128;
const int height = 64;

char* current_state = "N";

int pin_sda = 2;
int pin_scl = 3;
int pin_joystick_speed = 26;
int pin_joystick_direction = 27;
int pin_engine_state = 28;

int pin_beam = 18;
int pin_horn = 19;
int pin_hazard = 20;
int pin_left_dir = 21;
int pin_right_dir = 22;

SemaphoreHandle_t mutex;
ssd1306_t disp;

void send_data_to_oled(ssd1306_t* disp, char* data) {
    ssd1306_draw_string(disp, width/2 - 20, height/2, 2, data);
    ssd1306_draw_string(disp, 10, 10, 2, current_state);        
    ssd1306_show(disp);
    ssd1306_clear(disp);
}

char* int_to_string(double value) {
    char* string_value = (char*)malloc(5);
    double decimalPart = (value - floor(value)) * 10.0;
    snprintf(
        string_value, 5, "%02d.%d", (int)floor(value), (int)decimalPart);
    return string_value;
}

int map(float x) {
  return (int)((200.0 * x)/3.3) - 100;
}

void vUpdateOledTask(void* arg) {
    char* task_name = (char*) arg;

    for(;;) {
        char* string_value = int_to_string(2.5);
        if (mutex_calculated_speed != NULL) {
            if (xSemaphoreTake(mutex_calculated_speed, portMAX_DELAY) == pdTRUE) {
                string_value = int_to_string(calculated_speed);
                xSemaphoreGive(mutex_calculated_speed);
            }
        }
        xSemaphoreTake(mutex, portMAX_DELAY);
        send_data_to_oled(&disp, string_value);
        xSemaphoreGive(mutex);
        
        ssd1306_draw_empty_square(
            &disp, 0, 0, width - 1, height - 1);
        free(string_value);
        vTaskDelay(200);       
	}
}

void vUpdateSpeedTask(void* arg) {
    char* task_name = (char*) arg;
    for(;;) {
        adc_select_input(0);
        uint16_t voltage_ac = adc_read();
        float conversion_factor = 3.3f / (1 << 12);
        float voltage_dc = (float)voltage_ac * conversion_factor;
        int unit = voltage_dc / 1;
        float decimal = (float)voltage_dc - (float)unit;

        int speed_ = map(voltage_dc);
        if (speed_ >= -15 && speed_ <= 15) {
            speed_ = 0;
        }
        // printf("Speed_: %d\n", speed_);

        if (mutex_speed != NULL) {
            if (xSemaphoreTake(mutex_speed, portMAX_DELAY) == pdTRUE) {
                speed = speed_;
                xSemaphoreGive(mutex_speed);
            }
        }
        vTaskDelay(10);
	}
}

void vUpdateDirectionTask(void* arg) {
    char* task_name = (char*) arg;
    for(;;) {
        adc_select_input(1);
        uint16_t voltage_ac = adc_read();
        float conversion_factor = 3.3f / (1 << 12);
        float voltage_dc = (float)voltage_ac * conversion_factor;
        int unit = voltage_dc / 1;
        float decimal = (float)voltage_dc - (float)unit;

        int direction_ = map(voltage_dc);
        if (direction_ >= -15 && direction_ <= 15) {
            direction_ = 0;
        }

        if (mutex_direction != NULL) {
            if (xSemaphoreTake(mutex_direction, portMAX_DELAY) == pdTRUE) {
                direction = direction_;
                xSemaphoreGive(mutex_direction);
            }
        }
        vTaskDelay(10);
	}
}

void vUpdateLeverStateTask(void* arg) {
    char* task_name = (char*) arg;
    int value_per_state = 1023;
    char* states[] = {"R", "N", "1", "2"};
    for(;;) {
        adc_select_input(2);
        int temp_state = adc_read() / value_per_state;
        temp_state = temp_state > 3 ? 3 : temp_state;
        temp_state = temp_state < 0 ? 0 : temp_state;
        xSemaphoreTake(mutex, portMAX_DELAY);
        current_state = states[temp_state];
        // printf("Oled: %d \n", current_state);
        xSemaphoreGive(mutex);
        
        xSemaphoreTake(mutex_engine_state, portMAX_DELAY);
        engine_state = states[temp_state][0];
        xSemaphoreGive(mutex_engine_state);

        vTaskDelay(50);
	}
}

void vUpdateButtonsStateTask(void* arg) {
    char* task_name = (char*) arg;
    for(;;) {
        int lights_state = 0;
        for (int i = pin_beam; i <= pin_right_dir; ++i) {
            lights_state = lights_state << 1;
            lights_state += gpio_get(i);
        }
        // printf("state of pin %d\n", lights_state);

        if (mutex_buttons_state != NULL) {
            if (xSemaphoreTake(mutex_buttons_state, portMAX_DELAY) == pdTRUE) {
                buttons = lights_state;
                xSemaphoreGive(mutex_buttons_state);
            }
        }
        vTaskDelay(50);
	}
}

void vLedBlinkTask(void* arg) {
    char* task_name = (char*) arg;
    for(;;) {
		gpio_put(PICO_DEFAULT_LED_PIN, 1);
		// printf("LED ON\n");
		vTaskDelay(500);
		gpio_put(PICO_DEFAULT_LED_PIN, 0);
		// printf("LED OFF\n");
		vTaskDelay(500);
	}
}


void setup_gpios(void) {
    // I2C pin setup
    i2c_init(i2c1, 400000);
    gpio_set_function(pin_sda, GPIO_FUNC_I2C);
    gpio_set_function(pin_scl, GPIO_FUNC_I2C);
    gpio_pull_up(pin_sda);
    gpio_pull_up(pin_scl);

    // ADC pin setup (joysticks and state lever)
    adc_init();
    adc_gpio_init(pin_joystick_speed);
    adc_gpio_init(pin_joystick_direction);
    adc_gpio_init(pin_engine_state);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // Lights pin setup
    gpio_init(pin_beam);
    gpio_init(pin_horn);
    gpio_init(pin_hazard);
    gpio_init(pin_left_dir);
    gpio_init(pin_right_dir);
    
    gpio_set_dir(pin_beam, GPIO_IN);
    gpio_set_dir(pin_horn, GPIO_IN);
    gpio_set_dir(pin_hazard, GPIO_IN);
    gpio_set_dir(pin_left_dir, GPIO_IN);
    gpio_set_dir(pin_right_dir, GPIO_IN);
    
    initUART2();
}

void main() {
    stdio_init_all();
    setup_gpios();

    disp.external_vcc=false;
    mutex = xSemaphoreCreateMutex();
    mutex_mileage = xSemaphoreCreateMutex();
    mutex_calculated_speed = xSemaphoreCreateMutex();
    mutex_speed = xSemaphoreCreateMutex();
    mutex_direction = xSemaphoreCreateMutex();
    mutex_engine_state = xSemaphoreCreateMutex();
    mutex_buttons_state = xSemaphoreCreateMutex();

    ssd1306_init(&disp, 128, 64, 0x3C, i2c1);
    ssd1306_clear(&disp);
    ssd1306_draw_empty_square(&disp, 0, 0, width - 1, height - 1);

    xTaskCreate(vLedBlinkTask,
        "Blink", 128, (void*)"Blink", 1, NULL);
    xTaskCreate(vUpdateLeverStateTask,
        "Engine State", configMINIMAL_STACK_SIZE, (void*)"Handle Engine State Task", 5, NULL);
    xTaskCreate(vUpdateSpeedTask,
        "Handle Speed", configMINIMAL_STACK_SIZE, (void*)"Handle Speed Task", 3, NULL);
    xTaskCreate(vUpdateDirectionTask,
        "Handle Direction", configMINIMAL_STACK_SIZE, (void*)"Handle Direction Task", 4, NULL);
    xTaskCreate(vUpdateOledTask,
        "Handle Buttons", configMINIMAL_STACK_SIZE, (void*)"Handle Oled Task", 6, NULL);
    xTaskCreate(vUpdateButtonsStateTask,
        "Engine State", configMINIMAL_STACK_SIZE, (void*)"Handle Lights State Task", 2, NULL);
    xTaskCreate(uart_task2_receive, "UART Chasis Receiving task", UART_TASK_STACK_SIZE,
			NULL, UART_TASK_PRIORITY + 6, NULL);
    xTaskCreate(uart_task2_send, "UART Chasis Sending task", UART_TASK_STACK_SIZE,
			NULL, UART_TASK_PRIORITY + 5, NULL);

    vTaskStartScheduler();

    for (;;) {}
}

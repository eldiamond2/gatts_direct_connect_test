//
// Created by Egor Kryndach on 2020-06-12.
// Modified by Dmitry Petrov (eldiamond) on 2023-apr-10
//

//MARK: Import common headers
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <esp_sleep.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"

//MARK: Import component header
#include "button.h"

//MARK: Private macros and constants
#define TAG "BUTTON"

#define BUTTON_PIN_BIT(x) (1ULL<<x)
#define ESP_INTR_FLAG_DEFAULT 0

//MARK: Private types
typedef struct {
    xQueueHandle queue;
    xQueueHandle hold_buttons;
    xTaskHandle task;
    // timeout for long press in ms
    // bool wake_up[GPIO_NUM_MAX];
    uint32_t long_press_timeout[GPIO_NUM_MAX]; 
    button_evt_handler_t on_button_press_cb[GPIO_NUM_MAX];
    button_evt_handler_t on_button_release_cb[GPIO_NUM_MAX];
    button_evt_handler_t on_button_long_press_cb[GPIO_NUM_MAX];
    int levels[GPIO_NUM_MAX];
} component_t;

//MARK: Declaration of the private opaque structs

//MARK: Public global variables

//MARK: Private global variables


static component_t component = {
        .queue = NULL,
        .hold_buttons = NULL,
};

//MARK: Private functions
static void IRAM_ATTR gpio_isr_handler(void *arg) {
    uint32_t gpio_num = (uint32_t) arg;

    // to avoid multiply interupts 
    gpio_set_intr_type(gpio_num, GPIO_INTR_DISABLE);
    // gpio_wakeup_disable(gpio_num);

    xQueueSendFromISR(component.queue, &gpio_num, NULL);
}

static void hold_task(void *arg) {
    uint32_t io_num = 0;
    uint64_t press_start = esp_timer_get_time(); 

    if(xQueueReceive(component.hold_buttons, &io_num, 100 / portTICK_PERIOD_MS) == pdFALSE) vTaskDelete(NULL);
    while(gpio_get_level(io_num) == 0) {
        if(esp_timer_get_time() - press_start > component.long_press_timeout[io_num] * 1000) {
            ESP_LOGI(TAG, "%d LONG PRESS", io_num);
            if(component.on_button_long_press_cb[io_num] != NULL) {
                component.on_button_long_press_cb[io_num]();
            }
            vTaskDelete(NULL);
        }
    }
    vTaskDelete(NULL);
}

static void gpio_task(void *arg) {
    uint32_t io_num;
    bool inf_loop = true;
    
    uint64_t dt = 0;
    while (inf_loop) {
        if (xQueueReceive(component.queue, &io_num, portMAX_DELAY)) {
            // debounce timer 
            if (esp_timer_get_time() - dt < (DEBOUNCE_TIMEOUT_MS * 1000)) {
                // return interrupts
                if(component.levels[io_num]) {
                    gpio_set_intr_type(io_num, GPIO_INTR_LOW_LEVEL);
                } else {
                    gpio_set_intr_type(io_num, GPIO_INTR_HIGH_LEVEL);
                }
                continue;
            }
            dt = esp_timer_get_time();

            int new_level = gpio_get_level(io_num);
            component.levels[io_num] = new_level;

            button_event_type_t event = new_level ? BUTTON_UP : BUTTON_DOWN;

            if (event == BUTTON_UP) {
                ESP_LOGI(TAG, "%d UP", io_num);
                gpio_set_intr_type(io_num, GPIO_INTR_LOW_LEVEL);
                if(component.on_button_release_cb[io_num] != NULL) {
                    component.on_button_release_cb[io_num]();
                }
            } else {
                ESP_LOGI(TAG, "%d DOWN", io_num);
                gpio_set_intr_type(io_num, GPIO_INTR_HIGH_LEVEL);
                if (component.on_button_press_cb[io_num] != NULL) {
                    component.on_button_press_cb[io_num]();
                }
                // so let's catch long press, not optimal, blocks multiple pressed buttons, TBD: migrate to independent task?
                xQueueSend(component.hold_buttons, &io_num, 5 / portTICK_PERIOD_MS);
                xTaskCreate(hold_task, "btn hld tsk", 2048, NULL, tskIDLE_PRIORITY, NULL);                
            }
        }
    }
}

//MARK: Implementation of the public functions
extern esp_err_t buttons_handler_init() {
    component.queue = xQueueCreate(10, sizeof(uint32_t));
    component.hold_buttons = xQueueCreate(10, sizeof(uint32_t));

    xTaskCreate(&gpio_task, "gpio_task", 4096, NULL, 10, &component.task);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    esp_sleep_enable_gpio_wakeup();
    return ESP_OK;
}

extern esp_err_t button_component_init (gpio_num_t pin, button_evt_handler_t on_press_button_event_cb, button_evt_handler_t on_release_button_event_cb, button_evt_handler_t on_long_press_button_event_cb) {
    component.on_button_press_cb[pin] = on_press_button_event_cb;
    component.on_button_release_cb[pin] = on_release_button_event_cb;
    component.on_button_long_press_cb[pin] = on_long_press_button_event_cb;
    return ESP_OK;
}

extern esp_err_t button_init(gpio_num_t pin, uint32_t long_press_timeout) {

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = BUTTON_PIN_BIT(pin);
    // pull-up and pull-down tied on PCB
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

    gpio_isr_handler_add(pin, gpio_isr_handler, (void *) pin);

    int level = gpio_get_level(pin);
    component.levels[pin] = level;
    component.long_press_timeout[pin] = long_press_timeout;
    if (level) {
        gpio_set_intr_type(pin, GPIO_INTR_LOW_LEVEL);
    } else {
        gpio_set_intr_type(pin, GPIO_INTR_HIGH_LEVEL);
    }

    return ESP_OK;
}
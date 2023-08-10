//
// Created by Egor Kryndach on 2020-06-12.
// Modified by Dmitry Petrov (eldiamond) on 2023-apr-10
//

#ifndef BLACK_BRICKS_ESP_BASE_BUTTON_H
#define BLACK_BRICKS_ESP_BASE_BUTTON_H

#define DEBOUNCE_TIMEOUT_MS 20

//MARK: Import common headers
#include "esp_err.h"
#include "driver/gpio.h"

//MARK: Macros and constants

//MARK: Types
typedef enum {
    BUTTON_DOWN = 0,
    BUTTON_UP = 1
} button_event_type_t;

typedef esp_err_t (*on_button_event_cb_t)(gpio_num_t pin, button_event_type_t event);
typedef void (*button_evt_handler_t)();

//MARK: Function prototypes
extern esp_err_t button_component_init (gpio_num_t pin, button_evt_handler_t on_press_button_event_cb, button_evt_handler_t on_release_button_event_cb, button_evt_handler_t on_long_press_button_event_cb);
extern esp_err_t button_init(gpio_num_t pin, uint32_t long_press_timeout);
extern esp_err_t buttons_handler_init();
#endif //BLACK_BRICKS_ESP_BASE_BUTTON_H
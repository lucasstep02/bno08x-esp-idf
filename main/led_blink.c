//
// Created by bohm on 4/15/23.
//
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "mini-car-esp-led";

void blink_led(void *pvParameters) {
  ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
  gpio_reset_pin(CONFIG_BLINK_GPIO);
  gpio_set_direction(CONFIG_BLINK_GPIO, GPIO_MODE_OUTPUT);
  uint8_t pattern[8] = {0, 2, 2, 2, 0, 2, 1, 2};

  while (1) {
    // ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
    for (uint8_t i = 0; i < 8; ++i) {
      uint8_t delay = pattern[i];
      if (delay != 0) {
        gpio_set_level(CONFIG_BLINK_GPIO, 1);
        if (delay == 2) {
          vTaskDelay(300 / portTICK_PERIOD_MS);
        } else if (delay == 1) {
          vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        gpio_set_level(CONFIG_BLINK_GPIO, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);
      } else {
        gpio_set_level(CONFIG_BLINK_GPIO, 0);
        vTaskDelay(200 / portTICK_PERIOD_MS);
      }
    }
  }
}

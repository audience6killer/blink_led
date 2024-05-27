/*#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "freertos/timers.h"
#include "driver/ledc.h"


#define led GPIO_NUM_2

static const char *TAG = "MAIN";

TimerHandle_t xTimer;

uint8_t led_level = 0;

const uint8_t timerInterval = 100;
const int timerID = 1;

int dutyR = 0;
int dutyG = 300;
int dutyB = 600;

esp_err_t init_led(void);
esp_err_t blink_led(void);
esp_err_t set_timer(void);
esp_err_t set_pwm(void);
esp_err_t set_pwm_duty(void);

void vTimerCallback(TimerHandle_t pxTimer);

void app_main(void)
{
    init_led();
    set_pwm();
    set_timer();
}

esp_err_t init_led(void)
{
    gpio_reset_pin(led);
    gpio_set_direction(led, GPIO_MODE_OUTPUT);

    return ESP_OK;
}
esp_err_t blink_led(void)
{
    led_level = !led_level;
    gpio_set_level(led, led_level);
    ESP_LOGI(TAG, "Value %u", led_level);
    return ESP_OK;
}
esp_err_t set_timer(void)
{
    ESP_LOGI(TAG, "Setting up the timer");
    xTimer = xTimerCreate("Timer",                        // Just a text name, not used by the kernel.
                           (pdMS_TO_TICKS(timerInterval)), // The timer period in ticks.
                           pdTRUE,                         // The timers will auto-reload themselves when they expire.
                           (void *)timerID,                // Assign each timer a unique id equal to its array index.
                           vTimerCallback                  // Each timer calls the same callback when it expires.
    );

    if (xTimer == NULL)
    {
        // The timer was not created.
        ESP_LOGE(TAG, "Timer was not created");
    }
    else
    {
        // Start the timer.  No block time is specified, and even if one was
        // it would be ignored because the scheduler has not yet been
        // started.
        if (xTimerStart(xTimer, 0) != pdPASS)
        {
            // The timer could not be set into the Active state.
            ESP_LOGE(TAG, "The timer could not be set into the Active state.");
        }
    }
    return ESP_OK;
}

void vTimerCallback(TimerHandle_t pxTimer)
{
    dutyR += 10;
    if (dutyR > 1023) dutyR = 0;

    dutyB += 10;
    if (dutyB > 1023) dutyB = 0;

    dutyG += 10;
    if (dutyG > 1023) dutyG = 0;

    set_pwm_duty();
    blink_led();
}

esp_err_t set_pwm(void)
{
    ledc_channel_config_t channelConfigL1 = {
        .channel = LEDC_CHANNEL_0,
        .gpio_num = 17,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .intr_type = LEDC_INTR_DISABLE
    };
    ledc_channel_config_t channelConfigL2 = {
        .channel = LEDC_CHANNEL_1,
        .gpio_num = 4,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .intr_type = LEDC_INTR_DISABLE
    };
    ledc_channel_config_t channelConfigL3 = {
        .channel = LEDC_CHANNEL_2,
        .gpio_num = 16,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .intr_type = LEDC_INTR_DISABLE
    };
    ledc_channel_config(&channelConfigL1);
    ledc_channel_config(&channelConfigL2);
    ledc_channel_config(&channelConfigL3);

    ledc_timer_config_t timerConfig = {
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 20000,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0

    };

    ledc_timer_config(&timerConfig);
    return ESP_OK;
}

esp_err_t set_pwm_duty(void)
{
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, dutyR);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, dutyG);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, dutyB);

    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);
    return ESP_OK;
}*/

/**
 * @brief Application entry point
 * 
 */

#include "nvs_flash.h"

#include "wifi_app.h"

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
         ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Start wifi app
    wifi_app_start();

}


/**
 * @file gps_app.c
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-05-27
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "sys/param.h"

#include "gps_app.h"
#include "gps_parser.h"
#include "tasks_common.h"

#define UART_TX_PIN 17
#define UART_RX_PIN 16

static const char *tag = "gps";

static const uart_port_t UART_NUM_PORT = UART_NUM_2;

static const size_t BUFF_SIZE = 1024;

// Parser object
static GPSData gpsData1;

// Uart handle
static QueueHandle_t uart_queue;

esp_err_t gps_init_serial(void)
{
    ESP_LOGI(tag, "gps_uart_app: Configuring UART communication");

    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_PORT, &uart_config));

    // Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Setup UART buffered IO with event queue
    // const int uart_buffer_size = (1024 * 2);

    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_PORT, BUFF_SIZE * 2,
                                        BUFF_SIZE * 2, 10, &uart_queue, 0));

    return ESP_OK;
}

int gps_read_data(char *data, size_t max_size)
{
    // Read data from UART.

    int length = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_PORT, (size_t *)&length));
    length = uart_read_bytes(UART_NUM_PORT, data, MIN(length, max_size), pdMS_TO_TICKS(100));

    return length;
}

static void gps_app_task(void *pvParameter)
{
    // Configure the port
    gps_init_serial();

    char data[BUFF_SIZE];

    for (;;)
    {
        int length = gps_read_data(data, BUFF_SIZE);

        if (length > 0)
        {
            data[length] = '\0';
            char *line = strtok(data, "\n");
            while (line != NULL)
            {
                const char *nmea_code = "$GPGGA,";
                if (strncmp(line, nmea_code, 7) == 0)
                {

                    //GPSData gpsData1;
                    parse_gps_data(line, &gpsData1);

                    /*printf("Packet 1 - Valid\n");
                    printf("Packet: %s\n", line);
                    printf("Latitude: %.9f %c\n", gpsData1.latitude, gpsData1.latitude_dir);
                    printf("Longitude: %.9f %c\n", gpsData1.longitude, gpsData1.longitude_dir);
                    printf("Altitude: %.2f %c\n", gpsData1.altitude, gpsData1.altitude_dir);
                    printf("Number of Satellites: %d\n", gpsData1.num_satellites);
                    printf("Fix Quality: %d\n", gpsData1.fix_quality);*/

                    // ESP_LOGI(tag, "Read %d bytes: %s", length, line);
                }
                // Get the next line
                line = strtok(NULL, "\n");
            }

            //
        }

        vTaskDelay(pdMS_TO_TICKS(1200));
    }
}
double get_latitude(void)
{
    return gpsData1.latitude;
}

float get_longitude(void)
{
    return gpsData1.longitude;
}

double get_altitude(void)
{
    return gpsData1.altitude;
}

void gps_task_start(void)
{
    ESP_LOGI(tag, "gps_app: Creating Task");
    xTaskCreatePinnedToCore(&gps_app_task, "gps_task", GPS_UART_TASK_STACK_SIZE, NULL, GPS_UART_TASK_PRIORITY, NULL, GPS_UART_CORE_ID);
    // xTaskCreatePinnedToCore(&gps_app_task, "gps_task", GPS_UART_TASK_STACK_SIZE, NULL, GPS_UART_TASK_PRIORITY, NULL, GPS_UART_CORE_ID);
}

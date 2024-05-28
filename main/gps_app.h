/**
 * @file gps_app.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-05-27
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef MAIN_GPS_APP_H
#define MAIN_GPS_APP_H

/**
 * @brief Task start
 * 
 */
void gps_task_start(void);

/**
 * @brief Initializes gps serial configuration
 * 
 * @return esp_err_t 
 */
esp_err_t gps_init_serial(void); 

/**
 * @brief Begins serial communication
 * 
 * @return esp_err_t 
 */
//esp_err_t gps_begin_serial_comm(void);


/**
 * @brief Reads data from the serial port
 * 
 * @return esp_err_t 
 */
int gps_read_data(char* data, size_t max_size);

/**
 * @brief Get the latitude object
 * 
 * @return double 
 */
double get_latitude(void);

float get_longitude(void);

double get_altitude(void);


#endif
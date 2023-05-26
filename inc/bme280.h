/******************************************************************************
 * Project:         Library file                                              *
 * Filename:        bme280.h                                                  *
 * Author:          Craig Hemingway                                           *
 * Company:         Dynament Ltd.                                             *
 *                  Status Scientific Controls Ltd.                           *
 * Date:            25/05/2023                                                *
 * File Version:   	1.0.0                                                     *
 * Version history: 1.0.0 - 25/05/2023 - Craig Hemingway                      *
 *                      Initial release                                       *
 * Hardware:        RP2040                                                    *
 * Tools Used:      Visual Studio Code -> 1.78.2                              *
 *                  Compiler           -> GCC 11.3.1 arm-none-eabi            *
 ******************************************************************************/

#ifndef __BME280_H
#define __BME280_H

/* Includes ------------------------------------------------------------------*/
#include <hardware/i2c.h>

/* Exported types ------------------------------------------------------------*/
struct bme280_calib_param
{
    // Temperature parameters
    uint16_t dig_t1;
    int16_t  dig_t2;
    int16_t  dig_t3;

    // Pressure parameters
    uint16_t dig_p1;
    int16_t  dig_p2;
    int16_t  dig_p3;
    int16_t  dig_p4;
    int16_t  dig_p5;
    int16_t  dig_p6;
    int16_t  dig_p7;
    int16_t  dig_p8;
    int16_t  dig_p9;

    // Humidity parameters
    uint8_t dig_h1;
    int16_t dig_h2;
    uint8_t dig_h3;
    int16_t dig_h4;
    int16_t dig_h5;
    int8_t  dig_h6;

    // Intermediate temperature coefficient
    int32_t t_fine;
};

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported function prototypes ----------------------------------------------*/
int32_t  compensate_temperature  ( int32_t temperature_raw , struct bme280_calib_param* calib_data );
uint16_t compensate_humidity     ( uint16_t humidity_raw , struct bme280_calib_param* calib_data );
uint32_t compensate_pressure     ( int32_t pressure_raw , struct bme280_calib_param* calib_data );
void     bme280_get_calib_params ( i2c_inst_t *i2c , struct bme280_calib_param* calib_data );
void     bme280_init             ( i2c_inst_t *i2c );
void     bme280_read_raw         ( i2c_inst_t *i2c , int32_t* temperature , int32_t* pressure , uint16_t* humidity );
void     bme280_reset            ( i2c_inst_t *i2c );

/* Exported defines ----------------------------------------------------------*/

#endif /* __BME280_H */

/* End of file ---------------------------------------------------------------*/

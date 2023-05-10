/******************************************************************************
 * Project:         AKM LED test                                              *
 * Filename:        main.h                                                    *
 * Author:          Craig Hemingway                                           *
 * Company:         Dynament Ltd.                                             *
 *                  Status Scientific Controls Ltd.                           *
 * Date:            10/05/2023                                                *
 * File Version:   	1.0.0                                                     *
 * Version history: 1.0.0 - 10/05/2023 - Craig Hemingway                      *
 *                      Initial release                                       *
 * Hardware:        RP2040                                                    *
 * Tools Used:      Visual Studio Code -> 1.78.1                              *
 *                  Compiler           -> GCC 11.3.1 arm-none-eabi            *
 *                                                                            *
 ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include <hardware/pio_instructions.h>
#include <pico/stdlib.h>

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported function prototypes ----------------------------------------------*/

/* Exported defines ----------------------------------------------------------*/
// System setup
#define NOP                     pio_encode_nop ( )
#define WATCHDOG_MILLISECONDS   8000    // Maximum 8 300 ms

// ADC
#define ADC_CHANNEL_0_ACT       0
#define ADC_CHANNEL_1_REF       1
#define ADC_CHANNEL_2_VMON      2
#define ADC_CHANNEL_3_AN_DAC    3

// GPIO
#define LED_PICO_PIN            25
#define PWM_OUT_PIN             14
#define SENSOR_CO2_I2C_SCL_PIN  13
#define SENSOR_CO2_I2C_SDA_PIN  12
#define SENSOR_CO2_PDN_PIN      15
#define SENSOR_HC_I2C_SCL_PIN   11
#define SENSOR_HC_I2C_SDA_PIN   10
#define SENSOR_HC_PDN_PIN        9

#define LED_PICO_OFF            gpio_put ( LED_PICO_PIN       , 0 )
#define LED_PICO_ON             gpio_put ( LED_PICO_PIN       , 1 )
#define SENSOR_CO2_PDN_HIGH     gpio_put ( SENSOR_CO2_PDN_PIN , 1 )
#define SENSOR_CO2_PDN_LOW      gpio_put ( SENSOR_CO2_PDN_PIN , 0 )
#define SENSOR_HC_PDN_HIGH      gpio_put ( SENSOR_HC_PDN_PIN  , 1 )
#define SENSOR_HC_PDN_LOW       gpio_put ( SENSOR_HC_PDN_PIN  , 0 )

// I2C
#define I2C_BAUD_RATE           100 // kHz
#define I2C_BUFFER_LENGTH       10
#define SENSOR_CO2_I2C          i2c0
#define SENSOR_HC_I2C           i2c1

#endif /* __MAIN_H */

/* End of file ---------------------------------------------------------------*/

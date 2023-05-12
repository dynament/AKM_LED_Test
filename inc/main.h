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

// AK9723 addresses
const uint8_t WIA1   = 0x00;    // Company code - 0x48
const uint8_t WIA2   = 0x01;    // Device ID - 0x23
const uint8_t STATUS = 0x04;    // Flags - [7:3] <NOT USED> ; [2] OVCUR_DET ; [1] ERR_FLAG ; [0] DRDY
const uint8_t IR1L   = 0x05;    // IR1 ( DET ) - Lower 8-bits ( 2's complement )
const uint8_t IR1M   = 0x06;    // IR1 ( DET ) - Middle 8-bits
const uint8_t IR1H   = 0x07;    // IR1 ( DET ) - Upper 8-bits
const uint8_t IR2L   = 0x08;    // IR2 ( REF ) - Lower 8-bits ( 2's complement )
const uint8_t IR2M   = 0x09;    // IR2 ( REF ) - Middle 8-bits
const uint8_t IR2H   = 0x0A;    // IR2 ( REF ) - Upper 8-bits
const uint8_t TMPL   = 0x0B;    // Temperature data lower 8-bits ( 2's complement )
const uint8_t TMPH   = 0x0C;    // Temperature data upper 8-bits
const uint8_t VFL    = 0x0D;    // LED forward Voltage measurement lower 8-bits ( 2's complement )
const uint8_t VFH    = 0x0E;    // LED forward Voltage measurement upper 8-bits
const uint8_t CNTL1  = 0x0F;    // Measurement frequency - [7:4] <NOT USED> ; [3:0] MLOOP3-0
const uint8_t CNTL2  = 0x10;    // Measurement time
const uint8_t CNTL3  = 0x11;    // LED emission time ( ( time * 2 ) + 26 us )
const uint8_t CNTL4  = 0x12;    // IR2 ( REF ) integration time adjustment
const uint8_t CNTL5  = 0x13;    // Interrupt settings
const uint8_t CNTL6  = 0x14;    // Measurement mode
const uint8_t CNTL7  = 0x15;    // ADC settings
const uint8_t CNTL8  = 0x16;    // LED current setting
const uint8_t CNTL9  = 0x17;    // Operating mode
const uint8_t CNTL10 = 0x18;    // Soft reset

// GPIO
#define LED_PICO_PIN            25
#define PWM_OUT_PIN             14
#define SENSOR_CO2_I2C_SCL_PIN  13
#define SENSOR_CO2_I2C_SDA_PIN  12
#define SENSOR_CO2_PDN_PIN      15
#define SENSOR_HC_I2C_SCL_PIN   11
#define SENSOR_HC_I2C_SDA_PIN   10
#define SENSOR_HC_PDN_PIN        9
#define UART_PC_RX_PIN           1
#define UART_PC_TX_PIN           0

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
#define SENSOR_ADDRESS          0x65

// UART
#define DATA_BITS           8
#define PARITY              UART_PARITY_NONE
#define STOP_BITS           1
#define UART_PC             uart0
#define UART_BAUD_RATE      38400
#define UART_BUFFER_LENGTH  500
#define UART_TIMEOUT        1000

#endif /* __MAIN_H */

/* End of file ---------------------------------------------------------------*/

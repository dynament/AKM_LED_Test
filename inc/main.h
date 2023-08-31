/******************************************************************************
 * Project:         AKM LED test                                              *
 * Filename:        main.h                                                    *
 * Author:          Craig Hemingway                                           *
 * Company:         Dynament Ltd.                                             *
 *                  Status Scientific Controls Ltd.                           *
 * Date:            26/05/2023                                                *
 * File Version:   	1.0.0                                                     *
 * Version history: 1.0.0 - 26/05/2023 - Craig Hemingway                      *
 *                      Initial release                                       *
 * Hardware:        RP2040                                                    *
 * Tools Used:      Visual Studio Code -> 1.78.2                              *
 *                  Compiler           -> GCC 11.3.1 arm-none-eabi            *
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
// 0x05 to 0x0E - Burst read , 2's complement 16-bit signed integer
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

const uint8_t NUM_MEAS_1    = 0x00;
const uint8_t NUM_MEAS_2    = 0x01;
const uint8_t NUM_MEAS_4    = 0x02;
const uint8_t NUM_MEAS_8    = 0x03;
const uint8_t NUM_MEAS_16   = 0x04;
const uint8_t NUM_MEAS_32   = 0x05;
const uint8_t NUM_MEAS_64   = 0x06;
const uint8_t NUM_MEAS_128  = 0x07;
const uint8_t NUM_MEAS_256  = 0x08;
const uint8_t NUM_MEAS_512  = 0x09;
const uint8_t NUM_MEAS_1024 = 0x0A;

// GPIO
// #define LED_PICO_PIN        25
#define PWM_OUT_PIN         14
#define SENSOR_I2C_SCL_PIN  27
#define SENSOR_I2C_SDA_PIN  26
#define SENSOR_PDN_PIN      25
#define UART_PC_RX_PIN      17
#define UART_PC_TX_PIN      16

#define LED_PICO_OFF        gpio_put ( LED_PICO_PIN   , 0 )
#define LED_PICO_ON         gpio_put ( LED_PICO_PIN   , 1 )
#define SENSOR_PDN_HIGH     gpio_put ( SENSOR_PDN_PIN , 1 )
#define SENSOR_PDN_LOW      gpio_put ( SENSOR_PDN_PIN , 0 )

// I2C
#define I2C_BAUD_RATE       100 // kHz
#define I2C_BUFFER_LENGTH   10
#define SENSOR_I2C_PORT     i2c1
#define SENSOR_ADDRESS      0x65

// UART
#define DATA_BITS           8
#define PARITY              UART_PARITY_NONE
#define STOP_BITS           1
#define UART_PC             uart0
#define UART_BAUD_RATE      38400
#define UART_BUFFER_LENGTH  500
#define UART_TIMEOUT        1000

const uint8_t TEST1  = 0b10000100;
const uint8_t TEST2  = 0b10001100;
const uint8_t TEST3  = 0b10000000;
const uint8_t TEST4  = 0b10001000;
const uint8_t TEST5  = 0b10000101;
const uint8_t TEST6  = 0b10001101;
const uint8_t TEST7  = 0b10000001;
const uint8_t TEST8  = 0b10001001;
const uint8_t TEST9  = 0b10000110;
const uint8_t TEST10 = 0b10001110;
const uint8_t TEST11 = 0b10000010;
const uint8_t TEST12 = 0b10001010;
const uint8_t TEST13 = 0b10000111;
const uint8_t TEST14 = 0b10001111;
const uint8_t TEST15 = 0b10000011;
const uint8_t TEST16 = 0b10001011;

#endif /* __MAIN_H */

/* End of file ---------------------------------------------------------------*/

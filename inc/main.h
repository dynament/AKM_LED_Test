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

// BME280 addresses
// 0xF7 to 0xFE - Burst read , 20bit unsigned pressure & temperature , 16-bit unsigned humidity
const uint8_t MID        = 0xD0;    // Manufacturer ID - 0x60
const uint8_t RESET      = 0xE0;    // Soft reset
const uint8_t CTRL_HUM   = 0xF2;    // [7:3] NOT USED , [2:0] Humidity oversampling
const uint8_t STATUS_BME = 0xF3;    // [7:4] NOT USED , [3] Conversion running flag , [2:1] NOT USED , [0] NVM data copy in progrss flag
const uint8_t CTRL_MEAS  = 0xF4;    // [7:5] Pressure oversampling , [4:2] Temperature oversampling , [1:0] Mode ( 00 sleep , 01 & 10 forced , 11 normal )
const uint8_t CONFIG     = 0xF5;    // [7:5] Inatctive duration ( Tstandby ) , [4:2] IIR filter coefficient , [1] NOT USED , [0] 3-wire SPI enable
const uint8_t PRESS_MSB  = 0xF7;    // MSB  - [19:12] Raw pressure measurement
const uint8_t PRESS_LSB  = 0xF8;    // LSB  - [11: 4] Raw pressure measurement
const uint8_t PRESS_XLSB = 0xF9;    // XLSB - [ 3: 0] Raw pressure measurement - [7:4] Data , [3:0] NOT USED
const uint8_t TEMP_MSB   = 0xFA;    // MSB  - [19:12] Raw temperature measurement
const uint8_t TEMP_LSB   = 0xFB;    // LSB  - [11: 4] Raw temperature measurement
const uint8_t TEMP_XLSB  = 0xFC;    // XLSB - [ 3: 0] Raw temperature measurement - [7:4] Data , [3:0] NOT USED
const uint8_t HUM_MSB    = 0xFD;    // MSB  - [15: 8] Raw humidity measurement
const uint8_t HUM_LSB    = 0xFE;    // LSB  - [ 7: 0] Raw humidity measurement

#define ADDR _u(0x76)

// hardware registers
#define REG_CONFIG _u(0xF5)
#define REG_CTRL_MEAS _u(0xF4)
#define REG_RESET _u(0xE0)

#define REG_TEMP_XLSB _u(0xFC)
#define REG_TEMP_LSB _u(0xFB)
#define REG_TEMP_MSB _u(0xFA)

#define REG_PRESSURE_XLSB _u(0xF9)
#define REG_PRESSURE_LSB _u(0xF8)
#define REG_PRESSURE_MSB _u(0xF7)

// calibration registers
#define REG_DIG_T1_LSB _u(0x88)
#define REG_DIG_T1_MSB _u(0x89)
#define REG_DIG_T2_LSB _u(0x8A)
#define REG_DIG_T2_MSB _u(0x8B)
#define REG_DIG_T3_LSB _u(0x8C)
#define REG_DIG_T3_MSB _u(0x8D)
#define REG_DIG_P1_LSB _u(0x8E)
#define REG_DIG_P1_MSB _u(0x8F)
#define REG_DIG_P2_LSB _u(0x90)
#define REG_DIG_P2_MSB _u(0x91)
#define REG_DIG_P3_LSB _u(0x92)
#define REG_DIG_P3_MSB _u(0x93)
#define REG_DIG_P4_LSB _u(0x94)
#define REG_DIG_P4_MSB _u(0x95)
#define REG_DIG_P5_LSB _u(0x96)
#define REG_DIG_P5_MSB _u(0x97)
#define REG_DIG_P6_LSB _u(0x98)
#define REG_DIG_P6_MSB _u(0x99)
#define REG_DIG_P7_LSB _u(0x9A)
#define REG_DIG_P7_MSB _u(0x9B)
#define REG_DIG_P8_LSB _u(0x9C)
#define REG_DIG_P8_MSB _u(0x9D)
#define REG_DIG_P9_LSB _u(0x9E)
#define REG_DIG_P9_MSB _u(0x9F)
#define REG_DIG_H1     _u(0xA1)
#define REG_DIG_H2_LSB _u(0xE1)
#define REG_DIG_H2_MSB _u(0xE2)
#define REG_DIG_H3     _u(0xE3)
#define REG_DIG_H4_MSB _u(0xE4)
#define REG_DIG_H4_LSB_H5_MSB _u(0xE5) // H4_LSB 0xE5[3:0],H5_MSB 0xE5[7:4]
#define REG_DIG_H5_LSB _u(0xE6)
#define REG_DIS_H6     _u(0xE7)

// number of calibration registers to be read
#define NUM_CALIB_PARAMS 32


// GPIO
#define LED_PICO_PIN                        25
#define PWM_OUT_PIN                         14
#define SENSOR_CO2_ANGLED_I2C_SCL_PIN       11
#define SENSOR_CO2_ANGLED_I2C_SDA_PIN       10
#define SENSOR_CO2_ANGLED_PDN_PIN            9
#define SENSOR_CO2_STRAIGHT_I2C_SCL_PIN     13
#define SENSOR_CO2_STRAIGHT_I2C_SDA_PIN     12
#define SENSOR_CO2_STRAIGHT_PDN_PIN         15
#define UART_PC_RX_PIN                       1
#define UART_PC_TX_PIN                       0

#define LED_PICO_OFF                    gpio_put ( LED_PICO_PIN                , 0 )
#define LED_PICO_ON                     gpio_put ( LED_PICO_PIN                , 1 )
#define SENSOR_CO2_ANGLED_PDN_HIGH      gpio_put ( SENSOR_CO2_ANGLED_PDN_PIN   , 1 )
#define SENSOR_CO2_ANGLED_PDN_LOW       gpio_put ( SENSOR_CO2_ANGLED_PDN_PIN   , 0 )
#define SENSOR_CO2_STRAIGHT_PDN_HIGH    gpio_put ( SENSOR_CO2_STRAIGHT_PDN_PIN , 1 )
#define SENSOR_CO2_STRAIGHT_PDN_LOW     gpio_put ( SENSOR_CO2_STRAIGHT_PDN_PIN , 0 )

// I2C
#define BME_ADDRESS                 0x76
#define I2C_BAUD_RATE               100 // kHz
#define I2C_BUFFER_LENGTH           10
#define SENSOR_CO2_STRAIGHT_I2C     i2c0
#define SENSOR_CO2_ANGLED_I2C       i2c1
#define SENSOR_ADDRESS              0x65

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

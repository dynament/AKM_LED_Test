/******************************************************************************
 * Project:         Library file                                              *
 * Filename:        bme280.c                                                  *
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

/* Includes ------------------------------------------------------------------*/
#include <bme280.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define BME_ADDR                0x76

// Hardware registers
// 0xF7 to 0xFE - Burst read , 20bit unsigned pressure & temperature , 16-bit unsigned humidity
#define REG_MID                 0xD0    // Manufacturer ID - 0x60
#define REG_RESET               0xE0    // Soft reset
#define REG_CTRL_HUM            0xF2    // [7:3] NOT USED , [2:0] Humidity oversampling
#define REG_STATUS_BME          0xF3    // [7:4] NOT USED , [3] Conversion running flag , [2:1] NOT USED , [0] NVM data copy in progrss flag
#define REG_CTRL_MEAS           0xF4    // [7:5] Pressure oversampling , [4:2] Temperature oversampling , [1:0] Mode ( 00 sleep , 01 & 10 forced , 11 normal )
#define REG_CONFIG              0xF5    // [7:5] Inatctive duration ( Tstandby ) , [4:2] IIR filter coefficient , [1] NOT USED , [0] 3-wire SPI enable
#define REG_PRESS_MSB           0xF7    // MSB  - [19:12] Raw pressure measurement
#define REG_PRESS_LSB           0xF8    // LSB  - [11: 4] Raw pressure measurement
#define REG_PRESS_XLSB          0xF9    // XLSB - [ 3: 0] Raw pressure measurement - [7:4] Data , [3:0] NOT USED
#define REG_TEMP_MSB            0xFA    // MSB  - [19:12] Raw temperature measurement
#define REG_TEMP_LSB            0xFB    // LSB  - [11: 4] Raw temperature measurement
#define REG_TEMP_XLSB           0xFC    // XLSB - [ 3: 0] Raw temperature measurement - [7:4] Data , [3:0] NOT USED
#define REG_HUM_MSB             0xFD    // MSB  - [15: 8] Raw humidity measurement
#define REG_HUM_LSB             0xFE    // LSB  - [ 7: 0] Raw humidity measurement

// Calibration registers
#define REG_DIG_T1_LSB          0x88
#define REG_DIG_T1_MSB          0x89
#define REG_DIG_T2_LSB          0x8A
#define REG_DIG_T2_MSB          0x8B
#define REG_DIG_T3_LSB          0x8C
#define REG_DIG_T3_MSB          0x8D
#define REG_DIG_P1_LSB          0x8E
#define REG_DIG_P1_MSB          0x8F
#define REG_DIG_P2_LSB          0x90
#define REG_DIG_P2_MSB          0x91
#define REG_DIG_P3_LSB          0x92
#define REG_DIG_P3_MSB          0x93
#define REG_DIG_P4_LSB          0x94
#define REG_DIG_P4_MSB          0x95
#define REG_DIG_P5_LSB          0x96
#define REG_DIG_P5_MSB          0x97
#define REG_DIG_P6_LSB          0x98
#define REG_DIG_P6_MSB          0x99
#define REG_DIG_P7_LSB          0x9A
#define REG_DIG_P7_MSB          0x9B
#define REG_DIG_P8_LSB          0x9C
#define REG_DIG_P8_MSB          0x9D
#define REG_DIG_P9_LSB          0x9E
#define REG_DIG_P9_MSB          0x9F
#define REG_DIG_H1              0xA1
#define REG_DIG_H2_LSB          0xE1
#define REG_DIG_H2_MSB          0xE2
#define REG_DIG_H3              0xE3
#define REG_DIG_H4_MSB          0xE4
#define REG_DIG_H4_LSB_H5_MSB   0xE5    // H4_LSB 0xE5[3:0],H5_MSB 0xE5[7:4]
#define REG_DIG_H5_LSB          0xE6
#define REG_DIS_H6              0xE7

// Number of calibration registers to be read
#define NUM_CALIB_PARAMS        32

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* User code -----------------------------------------------------------------*/
int32_t compensate_temperature ( int32_t temperature_raw , struct bme280_calib_param* calib_data )
{
    int32_t var1;
    int32_t var2;
    int32_t temperature;
    int32_t temperature_min = -4000;
    int32_t temperature_max = 8500;

    var1 = ( int32_t ) ( ( temperature_raw / 8 ) - ( ( int32_t ) calib_data->dig_t1 * 2 ) );
    var1 = ( var1 * ( ( int32_t ) calib_data->dig_t2 ) ) / 0x800;
    var2 = ( int32_t ) ( ( temperature_raw / 16 ) - ( ( int32_t ) calib_data->dig_t1 ) );
    var2 = ( ( ( var2 * var2 ) / 0x1000 ) * ( ( int32_t ) calib_data->dig_t3 ) ) / 0x4000;
    calib_data->t_fine = var1 + var2;
    temperature = ( ( calib_data->t_fine * 5 ) + 0x80 ) / 0x100;

    if ( temperature < temperature_min )
    {
        temperature = temperature_min;
    }
    else if ( temperature > temperature_max )
    {
        temperature = temperature_max;
    }
    else
    {
        // Nothing to do
    }

    return temperature;
}

uint16_t compensate_humidity ( uint16_t humidity_raw , struct bme280_calib_param* calib_data )
{
    int32_t  var1;
    int32_t  var2;
    int32_t  var3;
    int32_t  var4;
    int32_t  var5;
    uint32_t humidity;
    uint32_t humidity_max = 102400;

    var1 = calib_data->t_fine - 76800;
    var2 = ( int32_t ) ( humidity_raw * 0x4000 );
    var3 = ( int32_t ) ( ( ( int32_t ) calib_data->dig_h4 ) * 0x100000 );
    var4 = ( ( int32_t ) calib_data->dig_h5 ) * var1;
    var5 = ( ( var2 - var3 - var4 ) + 0x4000 ) / 0x8000;
    var2 = ( var1 * ( ( int32_t ) calib_data->dig_h6 ) ) / 0x400;
    var3 = ( var1 * ( ( int32_t ) calib_data->dig_h3 ) ) / 0x800;
    var4 = ( ( var2 * ( var3 + 0x8000 ) ) / 0x400 ) + 0x200000;
    var2 = ( ( var4 * ( ( int32_t ) calib_data->dig_h2 ) ) + 0x2000 ) / 0x4000 ;
    var3 = var5 * var2;
    var4 = ( ( var3 / 0x8000 ) * ( var3 / 0x8000 ) ) / 0x80;
    var5 = var3 - ( ( var4 * ( ( int32_t ) calib_data->dig_h1 ) ) / 16 );

    if ( var5 < 0 )
    {
        var5 = 0;
    }
    else if ( var5 > 0x19000000 )
    {
        var5 = 0x19000000;
    }
    else
    {
        // Nothing to do
    }

    humidity = ( uint32_t ) ( var5 / 0x1000 );

    if ( humidity > humidity_max )
    {
        humidity = humidity_max;
    }
    else
    {
        // Nothing to do
    }

    return humidity;
}

uint32_t compensate_pressure ( int32_t pressure_raw , struct bme280_calib_param* calib_data )
{
    int32_t  var1;
    int32_t  var2;
    int32_t  var3;
    int32_t  var4;
    uint32_t var5;
    uint32_t pressure;
    uint32_t pressure_min = 30000;
    uint32_t pressure_max = 110000;

    var1 = ( ( ( int32_t ) calib_data->t_fine ) / 2 ) - ( int32_t ) 0xFA00;
    var2 = ( ( ( var1 / 4 ) * ( var1 / 4 ) ) / 0x800 ) * ( ( int32_t ) calib_data->dig_p6 );
    var2 = var2 + ( ( var1 * ( ( int32_t ) calib_data->dig_p5 ) ) * 2 );
    var2 = ( var2 / 4 ) + ( ( ( int32_t ) calib_data->dig_p4 ) * 0x10000 );
    var3 = ( calib_data->dig_p3 * ( ( ( var1 / 4 ) * ( var1 / 4 ) ) / 0x2000 ) ) / 8;
    var4 = ( ( ( int32_t ) calib_data->dig_p2 ) * var1 ) / 2;
    var1 = ( var3 + var4 ) / 0x40000;
    var1 = ( ( ( 0x8000 + var1 ) ) * ( ( int32_t ) calib_data->dig_p1 ) ) / 0x8000;

    // Avoid exception caused by division by zero
    if ( var1 )
    {
        var5     = ( uint32_t ) ( 0x00100000 - pressure_raw );
        pressure = ( ( uint32_t ) ( var5 - ( uint32_t ) ( var2 / 0x1000 ) ) ) * 3125;

        if ( pressure < 0x80000000 )
        {
            pressure = ( pressure << 1 ) / ( ( uint32_t ) var1 ) ;
        }
        else
        {
            pressure = ( pressure / ( uint32_t ) var1 ) * 2;
        }

        var1     = ( ( ( int32_t ) calib_data->dig_p9 ) * ( ( int32_t ) ( ( ( pressure / 8 ) * ( pressure / 8 ) ) / 0x2000 ) ) ) / 0x1000;
        var2     = ( ( ( int32_t ) ( pressure / 4 ) ) * ( ( int32_t ) calib_data->dig_p8 ) ) / 0x2000;
        pressure = ( uint32_t ) ( ( int32_t ) pressure + ( ( var1 + var2 + calib_data->dig_p7 ) / 16 ) );

        if ( pressure < pressure_min )
        {
            pressure = pressure_min;
        }
        else if ( pressure > pressure_max )
        {
            pressure = pressure_max;
        }
        else
        {
            // Nothing to do
        }
    }
    else
    {
        pressure = pressure_min;
    }

    return pressure;
}

void bme280_get_calib_params ( i2c_inst_t *i2c , struct bme280_calib_param* calib_data )
{
    uint8_t buf [ NUM_CALIB_PARAMS ] = { 0 };
    uint8_t reg = REG_DIG_T1_LSB;
    i2c_write_blocking ( i2c , BME_ADDR , &reg ,  1 , true  );
    i2c_read_blocking  ( i2c , BME_ADDR , buf  , 24 , false );

    reg = REG_DIG_H1;
    i2c_write_blocking ( i2c , BME_ADDR , &reg     , 1 , true  );
    i2c_read_blocking  ( i2c , BME_ADDR , buf + 24 , 1 , false );
    
    reg = REG_DIG_H2_LSB;
    i2c_write_blocking ( i2c , BME_ADDR , &reg     , 1 , true  );
    i2c_read_blocking  ( i2c , BME_ADDR , buf + 25 , 7 , false );

    calib_data->dig_t1 = ( uint16_t ) ( buf [  1 ] << 8 ) | buf [ 0 ];
    calib_data->dig_t2 = ( int16_t  ) ( buf [  3 ] << 8 ) | buf [ 2 ];
    calib_data->dig_t3 = ( int16_t  ) ( buf [  5 ] << 8 ) | buf [ 4 ] ;

    calib_data->dig_p1 = ( uint16_t ) ( buf [  7 ] << 8 ) | buf [  6 ];
    calib_data->dig_p2 = ( int16_t  ) ( buf [  9 ] << 8 ) | buf [  8 ];
    calib_data->dig_p3 = ( int16_t  ) ( buf [ 11 ] << 8 ) | buf [ 10 ];
    calib_data->dig_p4 = ( int16_t  ) ( buf [ 13 ] << 8 ) | buf [ 12 ];
    calib_data->dig_p5 = ( int16_t  ) ( buf [ 15 ] << 8 ) | buf [ 14 ];
    calib_data->dig_p6 = ( int16_t  ) ( buf [ 17 ] << 8 ) | buf [ 16 ];
    calib_data->dig_p7 = ( int16_t  ) ( buf [ 19 ] << 8 ) | buf [ 18 ];
    calib_data->dig_p8 = ( int16_t  ) ( buf [ 21 ] << 8 ) | buf [ 20 ];
    calib_data->dig_p9 = ( int16_t  ) ( buf [ 23 ] << 8 ) | buf [ 22 ];

    calib_data->dig_h1 = ( uint8_t  ) ( buf [ 24 ] );
    calib_data->dig_h2 = ( int16_t  ) ( buf [ 26 ] <<8 ) | buf [ 25 ];
    calib_data->dig_h3 = ( uint8_t  ) ( buf [ 27 ] );
    calib_data->dig_h4 = ( int16_t  ) ( buf [ 28 ] <<4 ) + (   buf [ 29 ] & 0x0F );
    calib_data->dig_h5 = ( int16_t  ) ( buf [ 30 ] <<4 ) + ( ( buf [ 29 ] & 0xF0 ) >> 4 );
    calib_data->dig_h6 = ( uint8_t  ) ( buf [ 31 ] );
}

void bme280_init ( i2c_inst_t *i2c )
{
    uint8_t  I2C_RX_Buffer [ 2 ];

    I2C_RX_Buffer [ 0 ] = REG_CONFIG;
    I2C_RX_Buffer [ 1 ] = 0b01000000;
    i2c_write_blocking ( i2c , BME_ADDR , I2C_RX_Buffer , 2 , false );

    I2C_RX_Buffer [ 0 ] = REG_CTRL_HUM;
    I2C_RX_Buffer [ 1 ] = 0b00000101;
    i2c_write_blocking ( i2c , BME_ADDR , I2C_RX_Buffer , 2 , false );

    I2C_RX_Buffer [ 0 ] = REG_CTRL_MEAS;
    I2C_RX_Buffer [ 1 ] = 0b10110111;
    i2c_write_blocking ( i2c , BME_ADDR , I2C_RX_Buffer , 2 , false );
}

void bme280_read_raw ( i2c_inst_t *i2c , int32_t* temperature , int32_t* pressure , uint16_t* humidity )
{
    uint8_t buf [ 8 ];
    uint8_t reg = REG_PRESS_MSB;

    i2c_write_blocking ( i2c , BME_ADDR , &reg , 1 , true  );
    i2c_read_blocking  ( i2c , BME_ADDR , buf  , 8 , false );

    // Store the 20 bit read in a 32 bit signed integer for conversion
    *pressure    = ( buf [ 0 ] << 12) | ( buf [ 1 ] << 4 ) | ( buf [ 2 ] >> 4);
    *temperature = ( buf [ 3 ] << 12) | ( buf [ 4 ] << 4 ) | ( buf [ 5 ] >> 4);
    *humidity    = ( buf [ 6 ] <<  8) | ( buf [ 7 ] );
}

void bme280_reset ( i2c_inst_t *i2c )
{
    uint8_t buf [ 2 ] = { REG_RESET, 0xB6 };

    i2c_write_blocking ( i2c , BME_ADDR , buf , 2 , false );
    sleep_ms ( 100 );
}

/* End of file ---------------------------------------------------------------*/

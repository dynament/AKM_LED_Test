/******************************************************************************
 * Project:         AKM LED test                                              *
 * Filename:        main.c                                                    *
 * Author:          Craig Hemingway                                           *
 * Company:         Dynament Ltd.                                             *
 *                  Status Scientific Controls Ltd.                           *
 * Date:            26/05/2023                                                *
 * File Version:   	1.0.0                                                     *
 * Version history: 1.0.0 - 26/05/2023 - Craig Hemingway                      *
 *                      Initial release                                       *
 * Hardware:        RP2040                                                    *
 * Tools Used:      Visual Studio Code -> 1.78.2                              *
 *                  Compiler           ->  GCC 11.3.1 arm-none-eabi           *
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <main.h>
#include <bme280.h>
#include <stdio.h>
#include <string.h>
#include <pico/binary_info.h>
#include <hardware/clocks.h>
#include <hardware/pwm.h>

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    float    Sensor_LED_Voltage_Display;
    int16_t  Sensor_LED_Voltage;
    int16_t  Sensor_Temperature;
    int32_t  Sensor_DET_Reading;
    int32_t  Sensor_REF_Reading;
} datastruct_t;

struct repeating_timer timer_millisec;

datastruct_t Sensor;

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
const uint8_t  NUM_SAMPLES = 32;
const uint16_t TIMEOUT     = 1000;
uint16_t Timeout = 0;

/* Private function prototypes -----------------------------------------------*/
uint8_t LED_OnTime              ( uint16_t led_on_time );
uint8_t LED_SetCurrent          ( uint8_t  led_current );
uint8_t Measurement_SetTime     ( uint8_t  measurement_set_time );
uint8_t Sensor_DataReady        ( void );
void    Sensor_GetData          ( void );
void    Sensor_GetLED_Voltage   ( void );
void    Sensor_GetTemperature   ( void );
void    Sensor_Init             ( void );
void    Sensor_SoftReset        ( void );
void    Sensor_StartMeasurement ( void );

/* User code -----------------------------------------------------------------*/
static bool timer_millisec_isr ( struct repeating_timer *t )
{
    if ( Timeout )
    {
        Timeout--;
    }
    else
    {
        // Nothing to do
    }
}

void main ( void )
{
    int32_t  Sensor_DET_ResultsArray [ 100 ];
    uint8_t  DataCount = 0;
    uint32_t Average   = 0;

    // Useful information for picotool
    bi_decl ( bi_program_description ( "RP2040 Premier" ) );
    
    stdio_init_all();

    // GPIO
    gpio_init    ( SENSOR_PDN_PIN            );
    gpio_set_dir ( SENSOR_PDN_PIN , GPIO_OUT );

    add_repeating_timer_ms ( 1 , timer_millisec_isr , NULL , &timer_millisec  );

    // I2C
    gpio_set_function ( SENSOR_I2C_SDA_PIN , GPIO_FUNC_I2C );
    gpio_set_function ( SENSOR_I2C_SCL_PIN , GPIO_FUNC_I2C );
    gpio_pull_up      ( SENSOR_I2C_SDA_PIN );
    gpio_pull_up      ( SENSOR_I2C_SCL_PIN );
    i2c_init          ( SENSOR_I2C_PORT , I2C_BAUD_RATE * 1000 );

    // UART
    uart_init         ( UART_PC         , UART_BAUD_RATE );
    gpio_set_function ( UART_PC_RX_PIN  , GPIO_FUNC_UART );
    gpio_set_function ( UART_PC_TX_PIN  , GPIO_FUNC_UART );
    // Set UART parameters
    uart_set_hw_flow      ( UART_PC  , false     , false              );    // Disable CTS / RTS
    uart_set_format       ( UART_PC  , DATA_BITS , STOP_BITS , PARITY );    // Data format
    uart_set_fifo_enabled ( UART_PC  , false                          );    // Turn off FIFO ( handled character by character )
    // Set up UART_RX interrupt
    // irq_set_exclusive_handler ( UART0_IRQ , uart_rx_isr  );
    // irq_set_enabled           ( UART0_IRQ , true         );
    // uart_set_irq_enables      ( UART_PC   , true , false );  // Enable UART interrupt ( RX only )

    SENSOR_PDN_LOW;
	sleep_ms ( 1000 );
    SENSOR_PDN_HIGH;

    for ( ; ; )
    {
        Average = 0;
        Timeout = TIMEOUT;
        memset ( Sensor_DET_ResultsArray , 0 , sizeof ( Sensor_DET_ResultsArray ) );
        memset ( &Sensor                 , 0 , sizeof ( Sensor )                  );

// Manually calculated average
/*
        for ( DataCount = 0 ; DataCount < NUM_SAMPLES ; DataCount++ )
        {
            Sensor_SoftReset ( );
            Sensor_Init ( );
            Sensor_StartMeasurement ( );
            sleep_ms ( 18 );   // ( Number of measurements * Time per measurement ) + minimum 10%

            Sensor_GetData ( );

            if ( 0x800000 > Sensor.Sensor_DET_Reading )
            {
                Sensor_DET_ResultsArray [ DataCount ] = Sensor.Sensor_DET_Reading;
                Average += ( uint32_t ) ( Sensor.Sensor_DET_Reading );
                printf ( "DET reading [ %2u ]  = %lu mV\n" , DataCount , ( uint32_t ) ( Sensor_DET_ResultsArray [ DataCount ] / 11185 ) );
            }
            else
            {
                Sensor_DET_ResultsArray [ DataCount ] = 0;
                printf ( "DET reading [ %2u ]  = 0.000 mV\n" , DataCount );
            }
        }

        Average = ( uint32_t ) ( ( Average / 11185 ) / NUM_SAMPLES );
        printf ( "DET reading average = %3lu mV\n" , ( uint32_t ) ( Average ) );
        while ( Timeout );
*/

// Automatically calculated average

        Sensor_SoftReset ( );
        Sensor_Init ( );
        Sensor_StartMeasurement ( );
        sleep_ms ( 550 );   // ( Number of measurements * Time per measurement ) + minimum 10%

        Sensor_GetData ( );

        printf ( "DET reading = %ld mV\n" , ( Sensor.Sensor_DET_Reading / 11184 ) );

    }
}

uint8_t LED_OnTime ( uint16_t led_on_time )
{
    uint16_t LED_IntTime = 0;
    uint8_t  LED_Time    = 0;

    if ( 60 > led_on_time )
    {
        LED_Time = 0x00;
    }
    else if ( 1066 < led_on_time )
    {
        LED_Time = 0xFF;
    }
    else
    {
        LED_IntTime = ( uint16_t ) ( ( led_on_time - 26 ) / 2 );
        LED_Time =    ( uint8_t )  ( ( LED_IntTime * 0.5061 ) - 8.614 );
    }

    return LED_Time;
}

uint8_t LED_SetCurrent   ( uint8_t  led_current )
{
    uint8_t LED_Current = 0;

    if ( 121 < led_current )
    {
        LED_Current = 0x1F;
    }
    else if ( 28 > led_current )
    {
        LED_Current = 0x00;
    }
    else
    {
        LED_Current = ( uint8_t ) ( ( led_current - 28 ) / 3 );
    }

    return LED_Current;
}

uint8_t Measurement_SetTime ( uint8_t measurement_set_time )
{
    uint8_t Measurement_Time = 0;

    if ( measurement_set_time > 515 )
    {
        Measurement_Time = 0xFF;
    }
    else if ( measurement_set_time < 6 )
    {
        Measurement_Time = 0x00;
    }
    else
    {
        Measurement_Time = ( uint8_t ) ( ( measurement_set_time - 5 ) / 2 );
    }

    return Measurement_Time;
}

uint8_t Sensor_DataReady ( void )
{
    uint8_t DataReady = 0;
    uint8_t RX_Buffer [ 2 ] = { 0 , 0 };

    i2c_write_blocking ( SENSOR_I2C_PORT , SENSOR_ADDRESS , &STATUS   , 1 , true  );
    i2c_read_blocking  ( SENSOR_I2C_PORT , SENSOR_ADDRESS , RX_Buffer , 1 , false );

    if ( RX_Buffer [ 0 ] & ( 1 << 0 ) )
    {
        DataReady = 1;
    }
    else
    {
        // Nothing to do
    }

    if ( RX_Buffer [ 1 ] & ( 1 << 0 ) )
    {
        DataReady |= 1 << 1;
    }
    else
    {
        // Nothing to do
    }

    return DataReady;
}

void Sensor_GetData ( void )
{
    uint8_t RX_Buffer [ 6 ] = { 0 , 0 , 0 , 0 , 0 , 0 };

    i2c_write_blocking ( SENSOR_I2C_PORT , SENSOR_ADDRESS , &IR1L     , 1 , true  );
    i2c_read_blocking  ( SENSOR_I2C_PORT , SENSOR_ADDRESS , RX_Buffer , 6 , false );

    Sensor.Sensor_DET_Reading = ( int32_t ) ( ( RX_Buffer [ 2 ] << 16 ) + ( RX_Buffer [ 1 ] << 8 ) + RX_Buffer [ 0 ] );
    Sensor.Sensor_REF_Reading = ( int32_t ) ( ( RX_Buffer [ 5 ] << 16 ) + ( RX_Buffer [ 4 ] << 8 ) + RX_Buffer [ 3 ] );
}

void Sensor_GetLED_Voltage ( void )
{
    uint8_t RX_Buffer [ 2 ] = { 0 , 0 };

    i2c_write_blocking ( SENSOR_I2C_PORT , SENSOR_ADDRESS , &VFL      , 1 , true  );
    i2c_read_blocking  ( SENSOR_I2C_PORT , SENSOR_ADDRESS , RX_Buffer , 2 , false );

    Sensor.Sensor_LED_Voltage = ( int16_t ) ( ( RX_Buffer [ 1 ] << 8 ) + RX_Buffer [ 0 ] );
}

void Sensor_GetTemperature ( void )
{
    uint8_t RX_Buffer [ 2 ] = { 0 , 0 };

    i2c_write_blocking ( SENSOR_I2C_PORT , SENSOR_ADDRESS , &TMPL     , 1 , true  );
    i2c_read_blocking  ( SENSOR_I2C_PORT , SENSOR_ADDRESS , RX_Buffer , 2 , false );

    Sensor.Sensor_Temperature = ( int16_t ) ( ( RX_Buffer [ 1 ] << 8 ) + RX_Buffer [ 0 ] );
}

void Sensor_Init ( void )
{
    uint8_t I2C_TX_Buffer [ 2 ] = { 0x00 , 0x00 };

    // Number of measurements per cycle
    I2C_TX_Buffer [ 0 ] = CNTL1;
    I2C_TX_Buffer [ 1 ] = NUM_MEAS_32;
    i2c_write_blocking ( SENSOR_I2C_PORT , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );

    // Measurement time ( ms )
    I2C_TX_Buffer [ 0 ] = CNTL2;
    I2C_TX_Buffer [ 1 ] = Measurement_SetTime ( 15 );
    i2c_write_blocking ( SENSOR_I2C_PORT , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );

    // LED integration ( pulse width ( 'on' ) ) time ( us )
    I2C_TX_Buffer [ 0 ] = CNTL3;
    I2C_TX_Buffer [ 1 ] = LED_OnTime ( 1000 );
    i2c_write_blocking ( SENSOR_I2C_PORT , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );

    // Analogue setup
    I2C_TX_Buffer [ 0 ] = CNTL7;
    // I2C_TX_Buffer [ 1 ] = 0b1001100;    // [7] <NOT USED> , [6] IR2 resolution = x1 , [5] IR1 resolution = x1 , [4] IR2 SH gain = x0.5 , [3] IR1 range = 500 mV , [2] IR1 SH gain = x2, [1:0] IR1 AFE gain = x2
    I2C_TX_Buffer [ 1 ] = TEST15;
    i2c_write_blocking ( SENSOR_I2C_PORT , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );

    // LED current ( mA )
    I2C_TX_Buffer [ 0 ] = CNTL8;
    I2C_TX_Buffer [ 1 ] = LED_SetCurrent ( 100 );  // mA
    i2c_write_blocking ( SENSOR_I2C_PORT , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );

    // Operating mode ( normal or test )
    I2C_TX_Buffer [ 0 ] = CNTL9;
    I2C_TX_Buffer [ 1 ] = 0x00; // Normal operating mode
    i2c_write_blocking ( SENSOR_I2C_PORT , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );
}

void Sensor_SoftReset ( void )
{
    uint8_t I2C_TX_Buffer [ 2 ] = { CNTL10 , 0x01 };

    i2c_write_blocking ( SENSOR_I2C_PORT , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );
}

void Sensor_StartMeasurement ( void )
{
    uint8_t I2C_TX_Buffer [ 2 ] = { 0x00 , 0x00 };

    I2C_TX_Buffer [ 0 ] = CNTL6;
    I2C_TX_Buffer [ 1 ] = 0x02; // Single measurement mode
    i2c_write_blocking ( SENSOR_I2C_PORT , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );
}

/* End of file ---------------------------------------------------------------*/

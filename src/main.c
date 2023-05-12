/* 
    This code originates from the Getting started with Raspberry Pi Pico document
    https://datasheets.raspberrypi.org/pico/getting-started-with-pico.pdf
    CC BY-ND Raspberry Pi (Trading) Ltd
*/

#include <main.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include <hardware/pwm.h>


const uint16_t TIMEOUT = 5000;

uint8_t LED_OnTime              ( uint16_t led_on_time );
uint8_t LED_SetCurrent          ( uint8_t  led_current );
uint8_t Sensor_DataReady        ( void );
uint8_t Set_NumberMeasurements  ( uint8_t num_measurements );   // Number of measurements per cycle
void    Sensor_GetData          ( void );
void    Sensor_GetLED_Voltage   ( void );
void    Sensor_GetTemperature   ( void );
void    Sensor_Init             ( void );
void    Sensor_SoftReset        ( void );
void    Sensor_StartMeasurement ( void );

int16_t  CO2_LED_Voltage = 0;
int16_t  CO2_Temperature = 0;
int16_t  HC_LED_Voltage  = 0;
int16_t  HC_Temperature  = 0;
int32_t  CO2_DET_Reading = 0;
int32_t  CO2_REF_Reading = 0;
int32_t  HC_DET_Reading  = 0;
int32_t  HC_REF_Reading  = 0;
uint16_t Timeout         = 0;

struct repeating_timer timer_heartbeat;
struct repeating_timer timer_millisec;

static bool timer_heartbeat_isr ( struct repeating_timer *t )
{
    if ( gpio_get ( LED_PICO_PIN ) )
    {
        LED_PICO_OFF;
    }
    else
    {
        LED_PICO_ON;
    }
}

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
    uint8_t  I2C_RX_Buffer [ 10 ];
    uint8_t  DataReady = 0;
    uint32_t PWM_DC    = 0xFFFF;

    // Useful information for picotool
    bi_decl ( bi_program_description ( "RP2040 Premier" ) );
    
    stdio_init_all();

    // GPIO
    gpio_init    ( LED_PICO_PIN       );
    gpio_init    ( PWM_OUT_PIN        );
    gpio_init    ( SENSOR_CO2_PDN_PIN );
    gpio_init    ( SENSOR_HC_PDN_PIN  );
    gpio_set_dir ( LED_PICO_PIN       , GPIO_OUT );
    gpio_set_dir ( PWM_OUT_PIN        , GPIO_OUT );
    gpio_set_dir ( SENSOR_CO2_PDN_PIN , GPIO_OUT );
    gpio_set_dir ( SENSOR_HC_PDN_PIN  , GPIO_OUT );

    LED_PICO_OFF;
    SENSOR_CO2_PDN_LOW;
    SENSOR_HC_PDN_LOW;
    gpio_put ( PWM_OUT_PIN , 0 );

    add_repeating_timer_ms ( 500 , timer_heartbeat_isr , NULL , &timer_heartbeat );
    add_repeating_timer_ms (   1 , timer_millisec_isr  , NULL , &timer_millisec  );

    // I2C
    gpio_set_function ( SENSOR_CO2_I2C_SDA_PIN , GPIO_FUNC_I2C );
    gpio_set_function ( SENSOR_CO2_I2C_SCL_PIN , GPIO_FUNC_I2C );
    gpio_pull_up      ( SENSOR_CO2_I2C_SDA_PIN );
    gpio_pull_up      ( SENSOR_CO2_I2C_SCL_PIN );
    i2c_init          ( SENSOR_CO2_I2C , I2C_BAUD_RATE * 1000 );
    gpio_set_function ( SENSOR_HC_I2C_SDA_PIN , GPIO_FUNC_I2C );
    gpio_set_function ( SENSOR_HC_I2C_SCL_PIN , GPIO_FUNC_I2C );
    gpio_pull_up      ( SENSOR_HC_I2C_SDA_PIN );
    gpio_pull_up      ( SENSOR_HC_I2C_SCL_PIN );
    i2c_init          ( SENSOR_HC_I2C , I2C_BAUD_RATE * 1000 );

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

    gpio_set_function ( PWM_OUT_PIN , GPIO_FUNC_PWM );
    uint8_t slice_num = pwm_gpio_to_slice_num ( PWM_OUT_PIN );
    uint8_t channel   = pwm_gpio_to_channel   ( PWM_OUT_PIN );
    pwm_set_wrap       ( slice_num , 0xFFFF );
    pwm_set_chan_level ( slice_num , channel , ( uint16_t ) PWM_DC );
    pwm_set_enabled    ( slice_num , true );

    PWM_DC = 0;
	sleep_ms ( 1000 );

    SENSOR_CO2_PDN_HIGH;
    SENSOR_HC_PDN_HIGH;

    Sensor_SoftReset ( );
    Sensor_Init ( );

    for ( ; ; )
    {
        DataReady = 0;
        Timeout   = TIMEOUT;
        memset ( I2C_RX_Buffer , 0 , sizeof ( I2C_RX_Buffer ) );

        // Sensor_SoftReset ( );
        // Sensor_Init ( );
        Sensor_StartMeasurement ( );
        sleep_ms ( 500 );

        while ( Timeout && !DataReady )
        {
            DataReady = Sensor_DataReady ( );
        }

        Sensor_GetData          ( );
        Sensor_GetLED_Voltage   ( );
        Sensor_GetTemperature   ( );

        if ( PWM_DC == 0 )
        {
            PWM_DC = 70000;
        }
        else
        {
            // Nothing to do
        }

        PWM_DC -= 10000;
        pwm_set_chan_level ( slice_num , channel , ( uint16_t ) PWM_DC );
        printf ( "PWM_DC            = %lu\n"    , PWM_DC            );
        printf ( "Time taken        = %i\n"     , TIMEOUT - Timeout );
        printf ( "Timeout remaining = %i\n"     , Timeout           );
        printf ( "CO2_DET_Reading   = 0x%06x\n" , CO2_DET_Reading   );
        printf ( "CO2_REF_Reading   = 0x%06x\n" , CO2_REF_Reading   );
        printf ( "CO2_Temperature   = 0x%04x\n" , CO2_Temperature   );
        printf ( "CO2_LED_Voltage   = 0x%04x\n" , CO2_LED_Voltage   );
        printf ( "HC_DET_Reading    = 0x%06x\n" , HC_DET_Reading    );
        printf ( "HC_REF_Reading    = 0x%06x\n" , HC_REF_Reading    );
        printf ( "HC_Temperature    = 0x%04x\n" , HC_Temperature    );
        printf ( "HC_LED_Voltage    = 0x%04x\n" , HC_LED_Voltage    );

        sleep_ms ( 100 );
    }
}

uint8_t LED_OnTime ( uint16_t led_on_time )
{
    uint8_t LED_Time = 0;

    LED_Time = ( uint8_t ) ( ( led_on_time - 26 ) / 2 );

    return LED_Time;
}

uint8_t LED_SetCurrent   ( uint8_t  led_current )
{
    uint8_t LED_Current = 0;

    LED_Current = ( uint8_t ) ( ( led_current - 28 ) / 3 );

    return LED_Current;
}

uint8_t Sensor_DataReady ( void )
{
    uint8_t DataReady = 0;
    uint8_t RX_Buffer [ 2 ] = { 0 , 0 };

    i2c_write_blocking ( SENSOR_CO2_I2C , SENSOR_ADDRESS , &STATUS       , 1 , true  );
    i2c_read_blocking  ( SENSOR_CO2_I2C , SENSOR_ADDRESS , RX_Buffer     , 1 , false );
    i2c_write_blocking ( SENSOR_HC_I2C  , SENSOR_ADDRESS , &STATUS       , 1 , true  );
    i2c_read_blocking  ( SENSOR_HC_I2C  , SENSOR_ADDRESS , RX_Buffer + 1 , 1 , false );

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
    uint8_t RX_Buffer_CO2 [ 6 ] = { 0 , 0 , 0 , 0 , 0 , 0 };
    uint8_t RX_Buffer_HC  [ 6 ] = { 0 , 0 , 0 , 0 , 0 , 0 };

    i2c_write_blocking ( SENSOR_HC_I2C  , SENSOR_ADDRESS , &IR1L         , 1 , true  );
    i2c_read_blocking  ( SENSOR_HC_I2C  , SENSOR_ADDRESS , RX_Buffer_CO2 , 6 , false );
    i2c_write_blocking ( SENSOR_CO2_I2C , SENSOR_ADDRESS , &IR1L         , 1 , true  );
    i2c_read_blocking  ( SENSOR_CO2_I2C , SENSOR_ADDRESS , RX_Buffer_HC  , 6 , false );

    CO2_DET_Reading = ( int32_t ) ( ( RX_Buffer_CO2 [ 2 ] << 16 ) + ( RX_Buffer_CO2 [ 1 ] << 8 ) + RX_Buffer_CO2 [ 0 ] );
    CO2_REF_Reading = ( int32_t ) ( ( RX_Buffer_CO2 [ 5 ] << 16 ) + ( RX_Buffer_CO2 [ 4 ] << 8 ) + RX_Buffer_CO2 [ 3 ] );
    HC_DET_Reading  = ( int32_t ) ( ( RX_Buffer_HC  [ 2 ] << 16 ) + ( RX_Buffer_HC  [ 1 ] << 8 ) + RX_Buffer_HC  [ 0 ] );
    HC_REF_Reading  = ( int32_t ) ( ( RX_Buffer_HC  [ 5 ] << 16 ) + ( RX_Buffer_HC  [ 4 ] << 8 ) + RX_Buffer_HC  [ 3 ] );
}

void Sensor_GetLED_Voltage ( void )
{
    uint8_t RX_Buffer_CO2 [ 2 ] = { 0 , 0 };
    uint8_t RX_Buffer_HC  [ 2 ] = { 0 , 0 };

    i2c_write_blocking ( SENSOR_HC_I2C  , SENSOR_ADDRESS , &VFL          , 1 , true  );
    i2c_read_blocking  ( SENSOR_HC_I2C  , SENSOR_ADDRESS , RX_Buffer_CO2 , 2 , false );
    i2c_write_blocking ( SENSOR_CO2_I2C , SENSOR_ADDRESS , &VFL          , 1 , true  );
    i2c_read_blocking  ( SENSOR_CO2_I2C , SENSOR_ADDRESS , RX_Buffer_HC  , 2 , false );

    CO2_LED_Voltage = ( int16_t ) ( ( RX_Buffer_CO2 [ 1 ] << 8 ) + RX_Buffer_CO2 [ 0 ] );
    HC_LED_Voltage  = ( int16_t ) ( ( RX_Buffer_HC  [ 1 ] << 8 ) + RX_Buffer_HC  [ 0 ] );
}

void Sensor_GetTemperature ( void )
{
    uint8_t RX_Buffer_CO2 [ 2 ] = { 0 , 0 };
    uint8_t RX_Buffer_HC  [ 2 ] = { 0 , 0 };

    i2c_write_blocking ( SENSOR_HC_I2C  , SENSOR_ADDRESS , &TMPL         , 1 , true  );
    i2c_read_blocking  ( SENSOR_HC_I2C  , SENSOR_ADDRESS , RX_Buffer_CO2 , 2 , false );
    i2c_write_blocking ( SENSOR_CO2_I2C , SENSOR_ADDRESS , &TMPL         , 1 , true  );
    i2c_read_blocking  ( SENSOR_CO2_I2C , SENSOR_ADDRESS , RX_Buffer_HC  , 2 , false );

    CO2_Temperature = ( int16_t ) ( ( RX_Buffer_CO2 [ 1 ] << 8 ) + RX_Buffer_CO2 [ 0 ] );
    HC_Temperature  = ( int16_t ) ( ( RX_Buffer_HC  [ 1 ] << 8 ) + RX_Buffer_HC  [ 0 ] );
}

void Sensor_Init ( void )
{
    uint8_t I2C_TX_Buffer [ 2 ] = { 0x00 , 0x00 };

    // Number of measurements per cycle
    I2C_TX_Buffer [ 0 ] = CNTL1;
    I2C_TX_Buffer [ 1 ] = 0x00; // 1 measurement
    i2c_write_blocking ( SENSOR_CO2_I2C , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );
    i2c_write_blocking ( SENSOR_HC_I2C  , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );

    // Measurement time
    I2C_TX_Buffer [ 0 ] = CNTL2;
    I2C_TX_Buffer [ 1 ] = 0xFF; // 515 ms
    i2c_write_blocking ( SENSOR_CO2_I2C , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );
    i2c_write_blocking ( SENSOR_HC_I2C  , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );

    // LED integration ( pulse width ( 'on' ) ) time ( us )
    I2C_TX_Buffer [ 0 ] = CNTL3;
    I2C_TX_Buffer [ 1 ] = LED_OnTime ( 500 );
    i2c_write_blocking ( SENSOR_CO2_I2C , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );
    i2c_write_blocking ( SENSOR_HC_I2C  , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );

    // Analogue setup
    // I2C_TX_Buffer [ 0 ] = CNTL7;
    // I2C_TX_Buffer [ 1 ] = 0b10000000;    // [7] <NOT USED> , [6] IR2 resolution , [5] IR1 resolution , [4] IR2 gain , [3] IR1 range , [2] IR1 gain , [1:0] IR1 AFE gain
    // i2c_write_blocking ( SENSOR_CO2_I2C , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );
    // i2c_write_blocking ( SENSOR_HC_I2C  , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );

    // LED current ( mA )
    I2C_TX_Buffer [ 0 ] = CNTL8;
    I2C_TX_Buffer [ 1 ] = LED_SetCurrent ( 100 );  // mA
    i2c_write_blocking ( SENSOR_CO2_I2C , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );
    i2c_write_blocking ( SENSOR_HC_I2C  , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );

    // Operating mode ( normal or test )
    I2C_TX_Buffer [ 0 ] = CNTL9;
    I2C_TX_Buffer [ 1 ] = 0x00; // Normal operating mode
    i2c_write_blocking ( SENSOR_CO2_I2C , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );
    i2c_write_blocking ( SENSOR_HC_I2C  , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );
}

void Sensor_SoftReset ( void )
{
    uint8_t I2C_TX_Buffer [ 2 ] = { CNTL10 , 0x01 };

    i2c_write_blocking ( SENSOR_CO2_I2C , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );
    i2c_write_blocking ( SENSOR_HC_I2C  , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );
}

void Sensor_StartMeasurement ( void )
{
    uint8_t I2C_TX_Buffer [ 2 ] = { 0x00 , 0x00 };

    // Measurement mode
    I2C_TX_Buffer [ 0 ] = CNTL6;
    I2C_TX_Buffer [ 1 ] = 0x02; // Single measurement mode
    i2c_write_blocking ( SENSOR_CO2_I2C , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );
    i2c_write_blocking ( SENSOR_HC_I2C  , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );
}
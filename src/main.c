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
void    BME_GetData             ( void );
void    Sensor_GetData          ( void );
void    Sensor_GetLED_Voltage   ( void );
void    Sensor_GetTemperature   ( void );
void    Sensor_Init             ( void );
void    Sensor_SoftReset        ( void );
void    Sensor_StartMeasurement ( void );

typedef struct
{
    float    Sensor_LED_Voltage_Display;
    int16_t  Sensor_LED_Voltage;
    int16_t  Sensor_Temperature;
    int32_t  Sensor_DET_Reading;
    int32_t  Sensor_REF_Reading;
    int32_t  BME_Pressure_Raw;
    int32_t  BME_Temperature_Raw;
    uint16_t BME_Humidity_Raw;
} datastruct_t;

uint16_t Timeout = 0;

datastruct_t Angled;
datastruct_t Straight;

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
    gpio_init    ( LED_PICO_PIN                );
    gpio_init    ( PWM_OUT_PIN                 );
    gpio_init    ( SENSOR_CO2_ANGLED_PDN_PIN   );
    gpio_init    ( SENSOR_CO2_STRAIGHT_PDN_PIN );
    gpio_set_dir ( LED_PICO_PIN                 , GPIO_OUT );
    gpio_set_dir ( PWM_OUT_PIN                  , GPIO_OUT );
    gpio_set_dir ( SENSOR_CO2_ANGLED_PDN_PIN    , GPIO_OUT );
    gpio_set_dir ( SENSOR_CO2_STRAIGHT_PDN_PIN  , GPIO_OUT );

    LED_PICO_OFF;
    SENSOR_CO2_ANGLED_PDN_LOW;
    SENSOR_CO2_STRAIGHT_PDN_LOW;
    gpio_put ( PWM_OUT_PIN , 0 );

    add_repeating_timer_ms ( 500 , timer_heartbeat_isr , NULL , &timer_heartbeat );
    add_repeating_timer_ms (   1 , timer_millisec_isr  , NULL , &timer_millisec  );

    // I2C
    gpio_set_function ( SENSOR_CO2_ANGLED_I2C_SDA_PIN , GPIO_FUNC_I2C );
    gpio_set_function ( SENSOR_CO2_ANGLED_I2C_SCL_PIN , GPIO_FUNC_I2C );
    gpio_pull_up      ( SENSOR_CO2_ANGLED_I2C_SDA_PIN );
    gpio_pull_up      ( SENSOR_CO2_ANGLED_I2C_SCL_PIN );
    i2c_init          ( SENSOR_CO2_ANGLED_I2C , I2C_BAUD_RATE * 1000 );
    gpio_set_function ( SENSOR_CO2_STRAIGHT_I2C_SDA_PIN , GPIO_FUNC_I2C );
    gpio_set_function ( SENSOR_CO2_STRAIGHT_I2C_SCL_PIN , GPIO_FUNC_I2C );
    gpio_pull_up      ( SENSOR_CO2_STRAIGHT_I2C_SDA_PIN );
    gpio_pull_up      ( SENSOR_CO2_STRAIGHT_I2C_SCL_PIN );
    i2c_init          ( SENSOR_CO2_STRAIGHT_I2C , I2C_BAUD_RATE * 1000 );

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

    SENSOR_CO2_ANGLED_PDN_HIGH;
    SENSOR_CO2_STRAIGHT_PDN_HIGH;

    // Sensor_SoftReset ( );
    // Sensor_Init ( );

    I2C_RX_Buffer [ 0 ] = RESET;
    I2C_RX_Buffer [ 1 ] = 0xB6;
    i2c_write_blocking ( SENSOR_CO2_ANGLED_I2C   , BME_ADDRESS , I2C_RX_Buffer , 2 , false );
    i2c_write_blocking ( SENSOR_CO2_STRAIGHT_I2C , BME_ADDRESS , I2C_RX_Buffer , 2 , false );
    
    sleep_ms ( 100 );

    I2C_RX_Buffer [ 0 ] = CONFIG;
    I2C_RX_Buffer [ 1 ] = 0b01000000;
    i2c_write_blocking ( SENSOR_CO2_ANGLED_I2C   , BME_ADDRESS , I2C_RX_Buffer , 2 , false );
    i2c_write_blocking ( SENSOR_CO2_STRAIGHT_I2C , BME_ADDRESS , I2C_RX_Buffer , 2 , false );

    I2C_RX_Buffer [ 0 ] = CTRL_HUM;
    I2C_RX_Buffer [ 1 ] = 0b00000101;
    i2c_write_blocking ( SENSOR_CO2_ANGLED_I2C   , BME_ADDRESS , I2C_RX_Buffer , 2 , false );
    i2c_write_blocking ( SENSOR_CO2_STRAIGHT_I2C , BME_ADDRESS , I2C_RX_Buffer , 2 , false );

    I2C_RX_Buffer [ 0 ] = CTRL_MEAS;
    I2C_RX_Buffer [ 1 ] = 0b10110111;
    i2c_write_blocking ( SENSOR_CO2_ANGLED_I2C   , BME_ADDRESS , I2C_RX_Buffer , 2 , false );
    i2c_write_blocking ( SENSOR_CO2_STRAIGHT_I2C , BME_ADDRESS , I2C_RX_Buffer , 2 , false );

    for ( ; ; )
    {
        DataReady = 0;
        Timeout   = TIMEOUT;
        memset ( &Angled       , 0 , sizeof ( Angled        ) );
        memset ( &Straight     , 0 , sizeof ( Straight      ) );
        memset ( I2C_RX_Buffer , 0 , sizeof ( I2C_RX_Buffer ) );

        sleep_ms ( 10 );
        Sensor_SoftReset ( );
        Sensor_Init ( );
        Sensor_StartMeasurement ( );
        sleep_ms ( 550 );

        DataReady = Sensor_DataReady ( );

        Sensor_GetData          ( );
        Sensor_GetLED_Voltage   ( );
        Sensor_GetTemperature   ( );

        BME_GetData ( );

        if ( PWM_DC == 0 )
        {
            PWM_DC = 70000;
        }
        else
        {
            // Nothing to do
        }

        Angled.Sensor_LED_Voltage_Display   = ( float ) ( ( Angled.Sensor_LED_Voltage    * 0.0458 ) + 1399.9 );
        Straight.Sensor_LED_Voltage_Display = ( float ) ( ( Straight.Sensor_LED_Voltage  * 0.0458 ) + 1399.9 );

        PWM_DC -= 10000;
        pwm_set_chan_level ( slice_num , channel , ( uint16_t ) PWM_DC );

        printf ( "Time taken                   = %i\n"     , TIMEOUT - Timeout            );
        printf ( "Timeout remaining            = %i\n"     , Timeout                      );
        printf ( "Angled   BME Humidity    raw = 0x%04x\n" , Angled.BME_Humidity_Raw      );
        printf ( "Angled   BME Pressure    raw = 0x%05x\n" , Angled.BME_Pressure_Raw      );
        printf ( "Angled   BME Temperature raw = 0x%05x\n" , Angled.BME_Temperature_Raw   );
        printf ( "Straight BME Humidity    raw = 0x%04x\n" , Straight.BME_Humidity_Raw    );
        printf ( "Straight BME Pressure    raw = 0x%05x\n" , Straight.BME_Pressure_Raw    );
        printf ( "Straight BME Temperature raw = 0x%05x\n" , Straight.BME_Temperature_Raw );
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

uint8_t Sensor_DataReady ( void )
{
    uint8_t DataReady = 0;
    uint8_t RX_Buffer [ 2 ] = { 0 , 0 };

    i2c_write_blocking ( SENSOR_CO2_ANGLED_I2C   , SENSOR_ADDRESS , &STATUS       , 1 , true  );
    i2c_read_blocking  ( SENSOR_CO2_ANGLED_I2C   , SENSOR_ADDRESS , RX_Buffer     , 1 , false );
    i2c_write_blocking ( SENSOR_CO2_STRAIGHT_I2C , SENSOR_ADDRESS , &STATUS       , 1 , true  );
    i2c_read_blocking  ( SENSOR_CO2_STRAIGHT_I2C , SENSOR_ADDRESS , RX_Buffer + 1 , 1 , false );

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

void BME_GetData ( void )
{
    uint8_t RX_Buffer_CO2_Angled   [ 8 ] = { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 };
    uint8_t RX_Buffer_CO2_Straight [ 8 ] = { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 };

    i2c_write_blocking ( SENSOR_CO2_ANGLED_I2C   , BME_ADDRESS , &PRESS_MSB             , 1 , true  );
    i2c_read_blocking  ( SENSOR_CO2_ANGLED_I2C   , BME_ADDRESS , RX_Buffer_CO2_Angled   , 8 , false );
    i2c_write_blocking ( SENSOR_CO2_STRAIGHT_I2C , BME_ADDRESS , &PRESS_MSB             , 1 , true  );
    i2c_read_blocking  ( SENSOR_CO2_STRAIGHT_I2C , BME_ADDRESS , RX_Buffer_CO2_Straight , 8 , false );

    Angled.BME_Humidity_Raw      = 0;
    Angled.BME_Pressure_Raw      = 0;
    Angled.BME_Temperature_Raw   = 0;
    Angled.BME_Humidity_Raw      = ( uint16_t ) ( ( RX_Buffer_CO2_Angled   [ 6 ] <<  8 ) +   RX_Buffer_CO2_Angled   [ 7 ] );
    Angled.BME_Pressure_Raw      = (  int32_t ) ( ( RX_Buffer_CO2_Angled   [ 0 ] << 12 ) + ( RX_Buffer_CO2_Angled   [ 1 ] << 4 ) + ( ( RX_Buffer_CO2_Angled   [ 2 ] & 0xF0 ) >> 4 ) );
    Angled.BME_Temperature_Raw   = (  int32_t ) ( ( RX_Buffer_CO2_Angled   [ 3 ] << 12 ) + ( RX_Buffer_CO2_Angled   [ 4 ] << 4 ) + ( ( RX_Buffer_CO2_Angled   [ 5 ] & 0xF0 ) >> 4 ) );
    Straight.BME_Humidity_Raw    = 0;
    Straight.BME_Pressure_Raw    = 0;
    Straight.BME_Temperature_Raw = 0;
    Straight.BME_Humidity_Raw    = ( uint16_t ) ( ( RX_Buffer_CO2_Straight [ 6 ] <<  8 ) +   RX_Buffer_CO2_Straight [ 7 ] );
    Straight.BME_Pressure_Raw    = (  int32_t ) ( ( RX_Buffer_CO2_Straight [ 0 ] << 12 ) + ( RX_Buffer_CO2_Straight [ 1 ] << 4 ) + ( ( RX_Buffer_CO2_Straight [ 2 ] & 0xF0 ) >> 4 ) );
    Straight.BME_Temperature_Raw = (  int32_t ) ( ( RX_Buffer_CO2_Straight [ 3 ] << 12 ) + ( RX_Buffer_CO2_Straight [ 4 ] << 4 ) + ( ( RX_Buffer_CO2_Straight [ 5 ] & 0xF0 ) >> 4 ) );
}

void Sensor_GetData ( void )
{
    uint8_t RX_Buffer_CO2_Angled   [ 6 ] = { 0 , 0 , 0 , 0 , 0 , 0 };
    uint8_t RX_Buffer_CO2_Straight [ 6 ] = { 0 , 0 , 0 , 0 , 0 , 0 };

    i2c_write_blocking ( SENSOR_CO2_ANGLED_I2C   , SENSOR_ADDRESS , &IR1L                  , 1 , true  );
    i2c_read_blocking  ( SENSOR_CO2_ANGLED_I2C   , SENSOR_ADDRESS , RX_Buffer_CO2_Angled   , 6 , false );
    i2c_write_blocking ( SENSOR_CO2_STRAIGHT_I2C , SENSOR_ADDRESS , &IR1L                  , 1 , true  );
    i2c_read_blocking  ( SENSOR_CO2_STRAIGHT_I2C , SENSOR_ADDRESS , RX_Buffer_CO2_Straight , 6 , false );

    Angled.Sensor_DET_Reading   = ( int32_t ) ( ( RX_Buffer_CO2_Angled   [ 2 ] << 16 ) + ( RX_Buffer_CO2_Angled   [ 1 ] << 8 ) + RX_Buffer_CO2_Angled   [ 0 ] );
    Angled.Sensor_REF_Reading   = ( int32_t ) ( ( RX_Buffer_CO2_Angled   [ 5 ] << 16 ) + ( RX_Buffer_CO2_Angled   [ 4 ] << 8 ) + RX_Buffer_CO2_Angled   [ 3 ] );
    Straight.Sensor_DET_Reading = ( int32_t ) ( ( RX_Buffer_CO2_Straight [ 2 ] << 16 ) + ( RX_Buffer_CO2_Straight [ 1 ] << 8 ) + RX_Buffer_CO2_Straight [ 0 ] );
    Straight.Sensor_REF_Reading = ( int32_t ) ( ( RX_Buffer_CO2_Straight [ 5 ] << 16 ) + ( RX_Buffer_CO2_Straight [ 4 ] << 8 ) + RX_Buffer_CO2_Straight [ 3 ] );
}

void Sensor_GetLED_Voltage ( void )
{
    uint8_t RX_Buffer_CO2_Angled   [ 2 ] = { 0 , 0 };
    uint8_t RX_Buffer_CO2_Straight [ 2 ] = { 0 , 0 };

    i2c_write_blocking ( SENSOR_CO2_ANGLED_I2C   , SENSOR_ADDRESS , &VFL                   , 1 , true  );
    i2c_read_blocking  ( SENSOR_CO2_ANGLED_I2C   , SENSOR_ADDRESS , RX_Buffer_CO2_Angled   , 2 , false );
    i2c_write_blocking ( SENSOR_CO2_STRAIGHT_I2C , SENSOR_ADDRESS , &VFL                   , 1 , true  );
    i2c_read_blocking  ( SENSOR_CO2_STRAIGHT_I2C , SENSOR_ADDRESS , RX_Buffer_CO2_Straight , 2 , false );

    Angled.Sensor_LED_Voltage   = ( int16_t ) ( ( RX_Buffer_CO2_Angled   [ 1 ] << 8 ) + RX_Buffer_CO2_Angled   [ 0 ] );
    Straight.Sensor_LED_Voltage = ( int16_t ) ( ( RX_Buffer_CO2_Straight [ 1 ] << 8 ) + RX_Buffer_CO2_Straight [ 0 ] );
}

void Sensor_GetTemperature ( void )
{
    uint8_t RX_Buffer_CO2_Angled   [ 2 ] = { 0 , 0 };
    uint8_t RX_Buffer_CO2_Straight [ 2 ] = { 0 , 0 };

    i2c_write_blocking ( SENSOR_CO2_ANGLED_I2C   , SENSOR_ADDRESS , &TMPL                  , 1 , true  );
    i2c_read_blocking  ( SENSOR_CO2_ANGLED_I2C   , SENSOR_ADDRESS , RX_Buffer_CO2_Angled   , 2 , false );
    i2c_write_blocking ( SENSOR_CO2_STRAIGHT_I2C , SENSOR_ADDRESS , &TMPL                  , 1 , true  );
    i2c_read_blocking  ( SENSOR_CO2_STRAIGHT_I2C , SENSOR_ADDRESS , RX_Buffer_CO2_Straight , 2 , false );

    Angled.Sensor_Temperature   = ( int16_t ) ( ( RX_Buffer_CO2_Angled   [ 1 ] << 8 ) + RX_Buffer_CO2_Angled   [ 0 ] );
    Straight.Sensor_Temperature = ( int16_t ) ( ( RX_Buffer_CO2_Straight [ 1 ] << 8 ) + RX_Buffer_CO2_Straight [ 0 ] );
}

void Sensor_Init ( void )
{
    uint8_t I2C_TX_Buffer [ 2 ] = { 0x00 , 0x00 };

    // Number of measurements per cycle
    I2C_TX_Buffer [ 0 ] = CNTL1;
    // I2C_TX_Buffer [ 1 ] = 0x03; // 8 measurements
    I2C_TX_Buffer [ 1 ] = 0x02; // 4 measurements
    // I2C_TX_Buffer [ 1 ] = 0x01; // 2 measurements
    // I2C_TX_Buffer [ 1 ] = 0x00; // 1 measurement
    i2c_write_blocking ( SENSOR_CO2_ANGLED_I2C   , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );
    i2c_write_blocking ( SENSOR_CO2_STRAIGHT_I2C , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );

    // Measurement time
    I2C_TX_Buffer [ 0 ] = CNTL2;
    // I2C_TX_Buffer [ 1 ] = 0xF7; // 500 ms
    // I2C_TX_Buffer [ 1 ] = 0x7A; // 250 ms
    I2C_TX_Buffer [ 1 ] = 0x3C; // 125 ms
    // I2C_TX_Buffer [ 1 ] = 0x1D; //  63 ms
    i2c_write_blocking ( SENSOR_CO2_ANGLED_I2C   , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );
    i2c_write_blocking ( SENSOR_CO2_STRAIGHT_I2C , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );

    // LED integration ( pulse width ( 'on' ) ) time ( us )
    I2C_TX_Buffer [ 0 ] = CNTL3;
    I2C_TX_Buffer [ 1 ] = LED_OnTime ( 1000 );
    i2c_write_blocking ( SENSOR_CO2_ANGLED_I2C   , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );
    i2c_write_blocking ( SENSOR_CO2_STRAIGHT_I2C , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );

    // Analogue setup
    I2C_TX_Buffer [ 0 ] = CNTL7;
    I2C_TX_Buffer [ 1 ] = 0b10000000;    // [7] <NOT USED> , [6] IR2 resolution = x1 , [5] IR1 resolution = x1 , [4] IR2 SH gain = x0.5 , [3] IR1 range = 1000 mV , [2] IR1 SH gain = x1, [1:0] IR1 AFE gain = x2
    i2c_write_blocking ( SENSOR_CO2_ANGLED_I2C   , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );
    i2c_write_blocking ( SENSOR_CO2_STRAIGHT_I2C , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );

    // LED current ( mA )
    I2C_TX_Buffer [ 0 ] = CNTL8;
    I2C_TX_Buffer [ 1 ] = LED_SetCurrent ( 100 );  // mA
    i2c_write_blocking ( SENSOR_CO2_ANGLED_I2C   , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );
    i2c_write_blocking ( SENSOR_CO2_STRAIGHT_I2C , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );

    // Operating mode ( normal or test )
    I2C_TX_Buffer [ 0 ] = CNTL9;
    I2C_TX_Buffer [ 1 ] = 0x00; // Normal operating mode
    i2c_write_blocking ( SENSOR_CO2_ANGLED_I2C   , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );
    i2c_write_blocking ( SENSOR_CO2_STRAIGHT_I2C , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );
}

void Sensor_SoftReset ( void )
{
    uint8_t I2C_TX_Buffer [ 2 ] = { CNTL10 , 0x01 };

    i2c_write_blocking ( SENSOR_CO2_ANGLED_I2C   , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );
    i2c_write_blocking ( SENSOR_CO2_STRAIGHT_I2C , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );
}

void Sensor_StartMeasurement ( void )
{
    uint8_t I2C_TX_Buffer [ 2 ] = { 0x00 , 0x00 };

    // Measurement mode
    I2C_TX_Buffer [ 0 ] = CNTL6;
    I2C_TX_Buffer [ 1 ] = 0x02; // Single measurement mode
    i2c_write_blocking ( SENSOR_CO2_ANGLED_I2C   , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );
    i2c_write_blocking ( SENSOR_CO2_STRAIGHT_I2C , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );
}
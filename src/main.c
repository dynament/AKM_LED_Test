/* 
    This code originates from the Getting started with Raspberry Pi Pico document
    https://datasheets.raspberrypi.org/pico/getting-started-with-pico.pdf
    CC BY-ND Raspberry Pi (Trading) Ltd
*/

#include <main.h>
#include <bme280.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include <hardware/pwm.h>

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

// struct bmp280_calib_param {
//     // temperature params
//     uint16_t dig_t1;
//     int16_t dig_t2;
//     int16_t dig_t3;

//     // pressure params
//     uint16_t dig_p1;
//     int16_t dig_p2;
//     int16_t dig_p3;
//     int16_t dig_p4;
//     int16_t dig_p5;
//     int16_t dig_p6;
//     int16_t dig_p7;
//     int16_t dig_p8;
//     int16_t dig_p9;

//     // humidity params
//     uint8_t dig_h1;
//     int16_t dig_h2;
//     uint8_t dig_h3;
//     int16_t dig_h4;
//     int16_t dig_h5;
//     int8_t dig_h6;

//     // Intermediate temperature coefficient
//     int32_t t_fine;

// };

const uint16_t TIMEOUT = 5000;

uint8_t LED_OnTime              ( uint16_t led_on_time );
uint8_t LED_SetCurrent          ( uint8_t  led_current );
uint8_t Sensor_DataReady        ( void );
uint8_t Set_NumberMeasurements  ( uint8_t num_measurements );   // Number of measurements per cycle
// void    BME_GetData             ( void );
void    Sensor_GetData          ( void );
void    Sensor_GetLED_Voltage   ( void );
void    Sensor_GetTemperature   ( void );
void    Sensor_Init             ( void );
void    Sensor_SoftReset        ( void );
void    Sensor_StartMeasurement ( void );

// void bmp280_init(i2c_inst_t *i2c);
// void bmp280_read_raw(i2c_inst_t *i2c,int32_t* temp, int32_t* pressure,uint16_t* humidity);
// void bmp280_reset(i2c_inst_t *i2c);
// int32_t bmp280_convert(int32_t temp, struct bmp280_calib_param* params);
// int32_t bmp280_convert_temp(int32_t temp, struct bmp280_calib_param* params);
// int32_t bmp280_convert_pressure(int32_t pressure, struct bmp280_calib_param* params);
// uint32_t bmp280_convert_humidity(uint16_t humidity, struct bmp280_calib_param* params);
// void bmp280_get_calib_params(i2c_inst_t *i2c,struct bmp280_calib_param* params);
// // int32_t compensate_humidity(int32_t humidity,struct bme280_calib_param* params);
// static int32_t compensate_temperature(int32_t temperature_raw,struct bmp280_calib_param* calib_data);
// static uint32_t compensate_pressure(int32_t pressure_raw,const struct bmp280_calib_param* calib_data);
// static uint16_t compensate_humidity(uint16_t humidity_raw,const struct bmp280_calib_param* calib_data);

uint16_t Timeout = 0;
// int32_t t_fine = 0;

datastruct_t Angled;
datastruct_t Straight;

// struct bme280_dev      BME_Struct;
struct bme280_calib_param Params_Angled;
struct bme280_calib_param Params_Straight;
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
    int32_t  Pressure_Angled        = 0;
    int32_t  Pressure_Straight        = 0;
    int32_t  Raw_Pressure_Angled    = 0;
    int32_t  Raw_Pressure_Straight    = 0;
    int32_t  Raw_Temperature_Angled = 0;
    int32_t  Raw_Temperature_Straight = 0;
    int32_t  Temperature_Angled     = 0;
    int32_t  Temperature_Straight     = 0;
    // uint8_t  I2C_RX_Buffer [ 10 ];
    uint8_t  DataReady       = 0;
    uint32_t PWM_DC          = 0xFFFF;
    uint16_t Humidity_Angled=0;
    uint16_t Humidity_Straight=0;
    uint16_t Raw_Humidity_Angled=0;
    uint16_t Raw_Humidity_Straight=0;
    

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
bme280_reset(SENSOR_CO2_ANGLED_I2C);
bme280_reset(SENSOR_CO2_STRAIGHT_I2C);
    // I2C_RX_Buffer [ 0 ] = RESET;
    // I2C_RX_Buffer [ 1 ] = 0xB6;
    // i2c_write_blocking ( SENSOR_CO2_ANGLED_I2C   , BME_ADDRESS , I2C_RX_Buffer , 2 , false );
    // i2c_write_blocking ( SENSOR_CO2_STRAIGHT_I2C , BME_ADDRESS , I2C_RX_Buffer , 2 , false );
    
    // sleep_ms ( 100 );

    // I2C_RX_Buffer [ 0 ] = CONFIG;
    // I2C_RX_Buffer [ 1 ] = 0b01000000;
    // i2c_write_blocking ( SENSOR_CO2_ANGLED_I2C   , BME_ADDRESS , I2C_RX_Buffer , 2 , false );
    // i2c_write_blocking ( SENSOR_CO2_STRAIGHT_I2C , BME_ADDRESS , I2C_RX_Buffer , 2 , false );

    // I2C_RX_Buffer [ 0 ] = CTRL_HUM;
    // I2C_RX_Buffer [ 1 ] = 0b00000101;
    // i2c_write_blocking ( SENSOR_CO2_ANGLED_I2C   , BME_ADDRESS , I2C_RX_Buffer , 2 , false );
    // i2c_write_blocking ( SENSOR_CO2_STRAIGHT_I2C , BME_ADDRESS , I2C_RX_Buffer , 2 , false );

    // I2C_RX_Buffer [ 0 ] = CTRL_MEAS;
    // I2C_RX_Buffer [ 1 ] = 0b10110111;
    // i2c_write_blocking ( SENSOR_CO2_ANGLED_I2C   , BME_ADDRESS , I2C_RX_Buffer , 2 , false );
    // i2c_write_blocking ( SENSOR_CO2_STRAIGHT_I2C , BME_ADDRESS , I2C_RX_Buffer , 2 , false );

    bme280_init(SENSOR_CO2_ANGLED_I2C);
    bme280_init(SENSOR_CO2_STRAIGHT_I2C);
    bme280_get_calib_params(SENSOR_CO2_ANGLED_I2C,&Params_Angled);
    bme280_get_calib_params(SENSOR_CO2_STRAIGHT_I2C,&Params_Straight);

    for ( ; ; )
    {
        DataReady = 0;
        Timeout   = TIMEOUT;
        memset ( &Angled       , 0 , sizeof ( Angled        ) );
        memset ( &Straight     , 0 , sizeof ( Straight      ) );
        // memset ( I2C_RX_Buffer , 0 , sizeof ( I2C_RX_Buffer ) );

        sleep_ms ( 10 );
        Sensor_SoftReset ( );
        Sensor_Init ( );
        Sensor_StartMeasurement ( );
        sleep_ms ( 550 );

        DataReady = Sensor_DataReady ( );

        Sensor_GetData          ( );
        Sensor_GetLED_Voltage   ( );
        Sensor_GetTemperature   ( );

        // BME_GetData ( );
        bme280_read_raw ( SENSOR_CO2_ANGLED_I2C , &Raw_Temperature_Angled , &Raw_Pressure_Angled ,&Raw_Humidity_Angled);
        bme280_read_raw ( SENSOR_CO2_STRAIGHT_I2C , &Raw_Temperature_Straight , &Raw_Pressure_Straight ,&Raw_Humidity_Straight);
        // Temperature_Angled = bmp280_convert_temp     ( Raw_Temperature_Angled , &Params_Angled );
        // Temperature_Straight = bmp280_convert_temp     ( Raw_Temperature_Straight , &Params_Straight );
        Temperature_Angled=compensate_temperature(Raw_Temperature_Angled,&Params_Angled);
        Temperature_Straight=compensate_temperature(Raw_Temperature_Straight,&Params_Straight);
        // Pressure_Angled    = bmp280_convert_pressure ( Raw_Pressure_Angled    ,  &Params_Angled );
        // Pressure_Straight    = bmp280_convert_pressure ( Raw_Pressure_Straight   , &Params_Straight );
        Pressure_Angled=compensate_pressure(Raw_Pressure_Angled,&Params_Angled);
        Pressure_Straight=compensate_pressure(Raw_Pressure_Straight,&Params_Straight);
        // Humidity_Angled = bmp280_convert_humidity     ( Raw_Humidity_Angled , &Params_Angled );
        // Humidity_Straight = bmp280_convert_humidity     ( Raw_Humidity_Straight , &Params_Straight );
        // Humidity_Angled = compensate_humidity     ( Raw_Humidity_Angled , &Params_Angled );
        // Humidity_Straight = bmp280_convert_humidity     ( Raw_Humidity_Straight , &Params_Straight );
        Humidity_Angled=compensate_humidity(Raw_Humidity_Angled,&Params_Angled);
        Humidity_Straight=compensate_humidity(Raw_Humidity_Straight,&Params_Straight);
        printf ( "Temperature angled   = %.2f degC\n" , Temperature_Angled / 100.f );
        printf ( "Temperature straight = %.2f degC\n" , Temperature_Straight / 100.f );
        printf ( "Pressure angled      = %.3f kPa\n"  , Pressure_Angled / 1000.f );
        printf ( "Pressure straight    = %.3f kPa\n"  , Pressure_Straight / 1000.f );
        printf ( "Humidity angled      = %.3f \%RH\n" , Humidity_Angled / 1024.f );
        printf ( "Humidity straight    = %.3f \%RH\n" , Humidity_Straight / 1024.f );

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

        // printf ( "Time taken                   = %i\n"     , TIMEOUT - Timeout            );
        // printf ( "Timeout remaining            = %i\n"     , Timeout                      );
        // printf ( "Angled   BME Humidity    raw = 0x%04x\n" , Angled.BME_Humidity_Raw      );
        // printf ( "Angled   BME Pressure    raw = 0x%05x\n" , Angled.BME_Pressure_Raw      );
        // printf ( "Angled   BME Temperature raw = 0x%05x\n" , Angled.BME_Temperature_Raw   );
        // printf ( "Straight BME Humidity    raw = 0x%04x\n" , Straight.BME_Humidity_Raw    );
        // printf ( "Straight BME Pressure    raw = 0x%05x\n" , Straight.BME_Pressure_Raw    );
        // printf ( "Straight BME Temperature raw = 0x%05x\n" , Straight.BME_Temperature_Raw );
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

// void BME_GetData ( void )
// {
//     uint8_t RX_Buffer_CO2_Angled   [ 8 ] = { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 };
//     uint8_t RX_Buffer_CO2_Straight [ 8 ] = { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 };

//     i2c_write_blocking ( SENSOR_CO2_ANGLED_I2C   , BME_ADDRESS , &PRESS_MSB             , 1 , true  );
//     i2c_read_blocking  ( SENSOR_CO2_ANGLED_I2C   , BME_ADDRESS , RX_Buffer_CO2_Angled   , 8 , false );
//     i2c_write_blocking ( SENSOR_CO2_STRAIGHT_I2C , BME_ADDRESS , &PRESS_MSB             , 1 , true  );
//     i2c_read_blocking  ( SENSOR_CO2_STRAIGHT_I2C , BME_ADDRESS , RX_Buffer_CO2_Straight , 8 , false );

//     Angled.BME_Humidity_Raw      = 0;
//     Angled.BME_Pressure_Raw      = 0;
//     Angled.BME_Temperature_Raw   = 0;
//     Angled.BME_Humidity_Raw      = ( uint16_t ) ( ( RX_Buffer_CO2_Angled   [ 6 ] <<  8 ) +   RX_Buffer_CO2_Angled   [ 7 ] );
//     Angled.BME_Pressure_Raw      = (  int32_t ) ( ( RX_Buffer_CO2_Angled   [ 0 ] << 12 ) + ( RX_Buffer_CO2_Angled   [ 1 ] << 4 ) + ( ( RX_Buffer_CO2_Angled   [ 2 ] & 0xF0 ) >> 4 ) );
//     Angled.BME_Temperature_Raw   = (  int32_t ) ( ( RX_Buffer_CO2_Angled   [ 3 ] << 12 ) + ( RX_Buffer_CO2_Angled   [ 4 ] << 4 ) + ( ( RX_Buffer_CO2_Angled   [ 5 ] & 0xF0 ) >> 4 ) );
//     Straight.BME_Humidity_Raw    = 0;
//     Straight.BME_Pressure_Raw    = 0;
//     Straight.BME_Temperature_Raw = 0;
//     Straight.BME_Humidity_Raw    = ( uint16_t ) ( ( RX_Buffer_CO2_Straight [ 6 ] <<  8 ) +   RX_Buffer_CO2_Straight [ 7 ] );
//     Straight.BME_Pressure_Raw    = (  int32_t ) ( ( RX_Buffer_CO2_Straight [ 0 ] << 12 ) + ( RX_Buffer_CO2_Straight [ 1 ] << 4 ) + ( ( RX_Buffer_CO2_Straight [ 2 ] & 0xF0 ) >> 4 ) );
//     Straight.BME_Temperature_Raw = (  int32_t ) ( ( RX_Buffer_CO2_Straight [ 3 ] << 12 ) + ( RX_Buffer_CO2_Straight [ 4 ] << 4 ) + ( ( RX_Buffer_CO2_Straight [ 5 ] & 0xF0 ) >> 4 ) );
// }

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









// void bmp280_init(i2c_inst_t *i2c) {
//     // use the "handheld device dynamic" optimal setting (see datasheet)
//     uint8_t buf[2];

//     // 500ms sampling time, x16 filter
//     const uint8_t reg_config_val = ((0x02 << 5) | (0x05 << 2)) & 0xFC;

//     // send register number followed by its corresponding value
//     buf[0] = REG_CONFIG;
//     buf[1] = reg_config_val;
//     i2c_write_blocking(i2c, ADDR, buf, 2, false);

//     // osrs_t x1, osrs_p x4, normal mode operation
//     const uint8_t reg_ctrl_meas_val = (0x01 << 5) | (0x03 << 2) | (0x03);
//     buf[0] = REG_CTRL_MEAS;
//     buf[1] = reg_ctrl_meas_val;
//     i2c_write_blocking(i2c, ADDR, buf, 2, false);

//     // osrs_h x4
//     const uint8_t reg_ctrl_hum_val = 0b0000011;//(0x01 << 5) | (0x03 << 2) | (0x03);
//     buf[0] = CTRL_HUM;
//     buf[1] = reg_ctrl_hum_val;
//     i2c_write_blocking(i2c, ADDR, buf, 2, false);
// }

// void bmp280_read_raw(i2c_inst_t *i2c, int32_t* temp, int32_t* pressure,uint16_t* humidity) {
//     // BMP280 data registers are auto-incrementing and we have 3 temperature and
//     // pressure registers each and 2 humidity registers so we start at 0xF7 and read 8 bytes to 0xFE
//     // note: normal mode does not require further ctrl_meas and config register writes

//     uint8_t buf[8];
//     uint8_t reg = REG_PRESSURE_MSB;
//     i2c_write_blocking(i2c, ADDR, &reg, 1, true);  // true to keep master control of bus
//     i2c_read_blocking(i2c, ADDR, buf, 8, false);  // false - finished with bus

//     // store the 20 bit read in a 32 bit signed integer for conversion
//     *pressure = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
//     *temp = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);
//     *humidity = (buf[6] << 8) | (buf[7]);
// }
/*
void bmp280_reset(i2c_inst_t *i2c) {
    // reset the device with the power-on-reset procedure
    uint8_t buf[2] = { REG_RESET, 0xB6 };
    i2c_write_blocking(i2c, ADDR, buf, 2, false);
}
*/

/*
// intermediate function that calculates the fine resolution temperature
// used for both pressure and temperature conversions
int32_t bmp280_convert(int32_t temp, struct bmp280_calib_param* params) {
    // use the 32-bit fixed point compensation implementation given in the
    // datasheet
    
    int32_t var1, var2;
    var1 = ((((temp >> 3) - ((int32_t)params->dig_t1 << 1))) * ((int32_t)params->dig_t2)) >> 11;
    var2 = (((((temp >> 4) - ((int32_t)params->dig_t1)) * ((temp >> 4) - ((int32_t)params->dig_t1))) >> 12) * ((int32_t)params->dig_t3)) >> 14;
    return var1 + var2;
}
*/

/*
int32_t bmp280_convert_temp(int32_t temp, struct bmp280_calib_param* params) {
    // uses the BMP280 calibration parameters to compensate the temperature value read from its registers
    params->t_fine = bmp280_convert(temp, params);
    return (params->t_fine * 5 + 128) >> 8;
}

int32_t bmp280_convert_pressure(int32_t pressure,  struct bmp280_calib_param* params) {
    // uses the BMP280 calibration parameters to compensate the pressure value read from its registers

    // t_fine = bmp280_convert(temp, params);

    int32_t var1, var2;
    uint32_t converted = 0.0;
    var1 = (((int32_t)params->t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)params->dig_p6);
    var2 += ((var1 * ((int32_t)params->dig_p5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)params->dig_p4) << 16);
    var1 = (((params->dig_p3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)params->dig_p2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)params->dig_p1)) >> 15);
    if (var1 == 0) {
        return 0;  // avoid exception caused by division by zero
    }
    converted = (((uint32_t)(((int32_t)1048576) - pressure) - (var2 >> 12))) * 3125;
    if (converted < 0x80000000) {
        converted = (converted << 1) / ((uint32_t)var1);
    } else {
        converted = (converted / (uint32_t)var1) * 2;
    }
    var1 = (((int32_t)params->dig_p9) * ((int32_t)(((converted >> 3) * (converted >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(converted >> 2)) * ((int32_t)params->dig_p8)) >> 13;
    converted = (uint32_t)((int32_t)converted + ((var1 + var2 + params->dig_p7) >> 4));
    return converted;
}
*/

/*
uint32_t bmp280_convert_humidity(uint16_t humidity, struct bmp280_calib_param* params)
{
    // int32_t humidity_min = 0.0;
    // int32_t humidity_max = 100.0;
    int32_t v_x1_u32r;
//     v_x1_u32r = (params->t_fine - ((int32_t)76800));
// v_x1_u32r = ((((((int32_t)humidity << 14) - (((int32_t)params->dig_h4) << 20) - (((int32_t)params->dig_h5) *
// v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r *
// ((int32_t)params->dig_h6)) >> 10) * (((v_x1_u32r * ((int32_t)params->dig_h3)) >> 11) +
// ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)params->dig_h2) +
// 8192) >> 14));
// v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
// ((int32_t)params->dig_h1)) >> 4));
// // v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
// // v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
// return (uint32_t)(v_x1_u32r>>12);

    double result;
    double humidity_min = 0.0;
    double humidity_max = 100.0;
    double var1;
    double var2;
    double var3;
    double var4;
    double var5;
    double var6;

    var1 = ((double)params->t_fine) - 76800.0;
    var2 = (((double)params->dig_h4) * 64.0 + (((double)params->dig_h5) / 16384.0) * var1);
    var3 = humidity - var2;
    var4 = ((double)params->dig_h2) / 65536.0;
    var5 = (1.0 + (((double)params->dig_h3) / 67108864.0) * var1);
    var6 = 1.0 + (((double)params->dig_h6) / 67108864.0) * var1 * var5;
    var6 = var3 * var4 * (var5 * var6);
    result = var6 * (1.0 - ((double)params->dig_h1) * var6 / 524288.0);

    if (result > humidity_max)
    {
        result = humidity_max;
    }
    else if (result < humidity_min)
    {
        result = humidity_min;
    }

    return result;



    if(var1<0)
    {
        var1=0;
    }
    else
    {
        // Nothing to do
    }
    if(var1>0x19000000)
    {
        var1=0x19000000;
    }
    else
    {
        // Nothing to do
    }

    return (uint32_t)(var1>>12);



    int32_t result;
    int32_t humidity_min = 0.0;
    int32_t humidity_max = 100.0;
    // int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    int32_t var6;

    var1 = ((int32_t)params->t_fine) - 76800.0;
    var2 = (((int32_t)params->dig_h4) * 64.0 + (((int32_t)params->dig_h5) / 16384.0) * var1);
    var3 = humidity - var2;
    var4 = ((int32_t)params->dig_h2) / 65536.0;
    var5 = (1.0 + (((int32_t)params->dig_h3) / 67108864.0) * var1);
    var6 = 1.0 + (((int32_t)params->dig_h6) / 67108864.0) * var1 * var5;
    var6 = var3 * var4 * (var5 * var6);
    result = var6 * (1.0 - ((int32_t)params->dig_h1) * var6 / 524288.0);

    if (result > humidity_max)
    {
        result = humidity_max;
    }
    else if (result < humidity_min)
    {
        result = humidity_min;
    }

    return result;
    
}
*/


// int32_t compensate_humidity(int32_t humidity,struct bme280_calib_param* params)
// {
    // int32_t var1=0;
    // return var1;

// }

// void bmp280_get_calib_params(i2c_inst_t *i2c,struct bmp280_calib_param* params) {
//     // raw temp and pressure values need to be calibrated according to
//     // parameters generated during the manufacturing of the sensor
//     // there are 3 temperature params, and 9 pressure params, each with a LSB
//     // and MSB register, so we read from 24 registers

//     uint8_t buf[NUM_CALIB_PARAMS] = { 0 };
//     uint8_t reg = REG_DIG_T1_LSB;
//     i2c_write_blocking(i2c, ADDR, &reg, 1, true);  // true to keep master control of bus
//     // read in one go as register addresses auto-increment
//     i2c_read_blocking(i2c, ADDR, buf, 24, false);  // false, we're done reading

//     reg=REG_DIG_H1;
//     i2c_write_blocking(i2c, ADDR, &reg, 1, true);  // true to keep master control of bus
//     i2c_read_blocking(i2c, ADDR, buf+24, 1, false);  // false, we're done reading

//     reg=REG_DIG_H2_LSB;
//     i2c_write_blocking(i2c, ADDR, &reg, 1, true);  // true to keep master control of bus
//     i2c_read_blocking(i2c, ADDR, buf+25, 7, false);  // false, we're done reading

//     // store these in a struct for later use
//     params->dig_t1 = (uint16_t)(buf[1] << 8) | buf[0];
//     params->dig_t2 = (int16_t)(buf[3] << 8) | buf[2];
//     params->dig_t3 = (int16_t)(buf[5] << 8) | buf[4];

//     params->dig_p1 = (uint16_t)(buf[7] << 8) | buf[6];
//     params->dig_p2 = (int16_t)(buf[9] << 8) | buf[8];
//     params->dig_p3 = (int16_t)(buf[11] << 8) | buf[10];
//     params->dig_p4 = (int16_t)(buf[13] << 8) | buf[12];
//     params->dig_p5 = (int16_t)(buf[15] << 8) | buf[14];
//     params->dig_p6 = (int16_t)(buf[17] << 8) | buf[16];
//     params->dig_p7 = (int16_t)(buf[19] << 8) | buf[18];
//     params->dig_p8 = (int16_t)(buf[21] << 8) | buf[20];
//     params->dig_p9 = (int16_t)(buf[23] << 8) | buf[22];

//     params->dig_h1 = (uint8_t)(buf[24]);
//     params->dig_h2 = (int16_t)(buf[26]<<8) | buf[25];
//     params->dig_h3 = (uint8_t)(buf[27]);
//     params->dig_h4 = (int16_t)(buf[28]<<4) + (buf[29]&0b00001111);
//     params->dig_h5 = (int16_t)(buf[30]<<4) + ((buf[29]&0b11110000)>>4);
//     params->dig_h6 = (uint8_t)(buf[31]);
// }

// static int32_t compensate_temperature(int32_t temperature_raw,struct bmp280_calib_param* calib_data)
// {
//     int32_t var1;
//     int32_t var2;
//     int32_t temperature;
//     int32_t temperature_min = -4000;
//     int32_t temperature_max = 8500;

//     var1 = (int32_t)((temperature_raw / 8) - ((int32_t)calib_data->dig_t1 * 2));
//     var1 = (var1 * ((int32_t)calib_data->dig_t2)) / 2048;
//     var2 = (int32_t)((temperature_raw / 16) - ((int32_t)calib_data->dig_t1));
//     var2 = (((var2 * var2) / 4096) * ((int32_t)calib_data->dig_t3)) / 16384;
//     calib_data->t_fine = var1 + var2;
//     temperature = (calib_data->t_fine * 5 + 128) / 256;

//     if (temperature < temperature_min)
//     {
//         temperature = temperature_min;
//     }
//     else if (temperature > temperature_max)
//     {
//         temperature = temperature_max;
//     }

//     return temperature;
// }

// static uint32_t compensate_pressure(int32_t pressure_raw,const struct bmp280_calib_param* calib_data)
// {
//     int32_t var1;
//     int32_t var2;
//     int32_t var3;
//     int32_t var4;
//     uint32_t var5;
//     uint32_t pressure;
//     uint32_t pressure_min = 30000;
//     uint32_t pressure_max = 110000;

//     var1 = (((int32_t)calib_data->t_fine) / 2) - (int32_t)64000;
//     var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t)calib_data->dig_p6);
//     var2 = var2 + ((var1 * ((int32_t)calib_data->dig_p5)) * 2);
//     var2 = (var2 / 4) + (((int32_t)calib_data->dig_p4) * 65536);
//     var3 = (calib_data->dig_p3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
//     var4 = (((int32_t)calib_data->dig_p2) * var1) / 2;
//     var1 = (var3 + var4) / 262144;
//     var1 = (((32768 + var1)) * ((int32_t)calib_data->dig_p1)) / 32768;

//     /* Avoid exception caused by division by zero */
//     if (var1)
//     {
//         var5 = (uint32_t)((uint32_t)1048576) - pressure_raw;
//         pressure = ((uint32_t)(var5 - (uint32_t)(var2 / 4096))) * 3125;

//         if (pressure < 0x80000000)
//         {
//             pressure = (pressure << 1) / ((uint32_t)var1);
//         }
//         else
//         {
//             pressure = (pressure / (uint32_t)var1) * 2;
//         }

//         var1 = (((int32_t)calib_data->dig_p9) * ((int32_t)(((pressure / 8) * (pressure / 8)) / 8192))) / 4096;
//         var2 = (((int32_t)(pressure / 4)) * ((int32_t)calib_data->dig_p8)) / 8192;
//         pressure = (uint32_t)((int32_t)pressure + ((var1 + var2 + calib_data->dig_p7) / 16));

//         if (pressure < pressure_min)
//         {
//             pressure = pressure_min;
//         }
//         else if (pressure > pressure_max)
//         {
//             pressure = pressure_max;
//         }
//     }
//     else
//     {
//         pressure = pressure_min;
//     }

//     return pressure;
// }

// static uint16_t compensate_humidity(uint16_t humidity_raw,const struct bmp280_calib_param* calib_data)
// {
//     int32_t var1;
//     int32_t var2;
//     int32_t var3;
//     int32_t var4;
//     int32_t var5;
//     uint32_t humidity;
//     uint32_t humidity_max = 102400;

//     var1 = calib_data->t_fine - ((int32_t)76800);
//     var2 = (int32_t)(humidity_raw * 16384);
//     var3 = (int32_t)(((int32_t)calib_data->dig_h4) * 1048576);
//     var4 = ((int32_t)calib_data->dig_h5) * var1;
//     var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
//     var2 = (var1 * ((int32_t)calib_data->dig_h6)) / 1024;
//     var3 = (var1 * ((int32_t)calib_data->dig_h3)) / 2048;
//     var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
//     var2 = ((var4 * ((int32_t)calib_data->dig_h2)) + 8192) / 16384;
//     var3 = var5 * var2;
//     var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
//     var5 = var3 - ((var4 * ((int32_t)calib_data->dig_h1)) / 16);
//     // var5 = (var5 < 0 ? 0 : var5);
//     // var5 = (var5 > 419430400 ? 419430400 : var5);
//     if(var5<0)
//     {
//         var5=0;
//     }
//     else if (var5>419430400)
//     {
//         var5=419430400;
//     }
//     humidity = (uint32_t)(var5 / 4096);

//     if (humidity > humidity_max)
//     {
//         humidity = humidity_max;
//     }

//     return humidity;

//     // double humidity;
//     // double humidity_min = 0.0;
//     // double humidity_max = 100.0;
//     // double var1;
//     // double var2;
//     // double var3;
//     // double var4;
//     // double var5;
//     // double var6;

//     // var1 = ((double)calib_data->t_fine) - 76800.0;
//     // var2 = (((double)calib_data->dig_h4) * 64.0 + (((double)calib_data->dig_h5) / 16384.0) * var1);
//     // var3 = humidity_raw - var2;
//     // var4 = ((double)calib_data->dig_h2) / 65536.0;
//     // var5 = (1.0 + (((double)calib_data->dig_h3) / 67108864.0) * var1);
//     // var6 = 1.0 + (((double)calib_data->dig_h6) / 67108864.0) * var1 * var5;
//     // var6 = var3 * var4 * (var5 * var6);
//     // humidity = var6 * (1.0 - ((double)calib_data->dig_h1) * var6 / 524288.0);

//     // if (humidity > humidity_max)
//     // {
//     //     humidity = humidity_max;
//     // }
//     // else if (humidity < humidity_min)
//     // {
//     //     humidity = humidity_min;
//     // }

//     // return humidity;
// }

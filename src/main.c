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

struct bme280_calib_param Params_Angled;
struct bme280_calib_param Params_Straight;
struct repeating_timer    timer_heartbeat;
struct repeating_timer    timer_millisec;

datastruct_t Angled;
datastruct_t Straight;

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
const uint16_t TIMEOUT = 5000;
uint16_t Timeout = 0;

/* Private function prototypes -----------------------------------------------*/
uint8_t Sensor_DataReady        ( void );
void    Sensor_GetData          ( void );
void    Sensor_GetLED_Voltage   ( void );
void    Sensor_GetTemperature   ( void );
void    Sensor_Init             ( void );
void    Sensor_SoftReset        ( void );
void    Sensor_StartMeasurement ( void );

/* User code -----------------------------------------------------------------*/
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
    int32_t  Pressure_Angled          = 0;
    int32_t  Pressure_Straight        = 0;
    int32_t  Raw_Pressure_Angled      = 0;
    int32_t  Raw_Pressure_Straight    = 0;
    int32_t  Raw_Temperature_Angled   = 0;
    int32_t  Raw_Temperature_Straight = 0;
    int32_t  Temperature_Angled       = 0;
    int32_t  Temperature_Straight     = 0;
    uint8_t  DataReady                = 0;
    uint32_t PWM_DC                   = 0xFFFF;
    uint16_t Humidity_Angled          = 0;
    uint16_t Humidity_Straight        = 0;
    uint16_t Raw_Humidity_Angled      = 0;
    uint16_t Raw_Humidity_Straight    = 0;
    

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

    bme280_reset            ( SENSOR_CO2_ANGLED_I2C   );
    bme280_reset            ( SENSOR_CO2_STRAIGHT_I2C );
    bme280_init             ( SENSOR_CO2_ANGLED_I2C   );
    bme280_init             ( SENSOR_CO2_STRAIGHT_I2C );
    bme280_get_calib_params ( SENSOR_CO2_ANGLED_I2C   , &Params_Angled   );
    bme280_get_calib_params ( SENSOR_CO2_STRAIGHT_I2C , &Params_Straight );

    for ( ; ; )
    {
        DataReady = 0;
        Timeout   = TIMEOUT;
        memset ( &Angled   , 0 , sizeof ( Angled   ) );
        memset ( &Straight , 0 , sizeof ( Straight ) );

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

        sleep_ms ( 10 );
        Sensor_SoftReset ( );
        Sensor_Init ( );
        Sensor_StartMeasurement ( );
        sleep_ms ( 550 );

        DataReady = Sensor_DataReady ( );

        Sensor_GetData          ( );
        Sensor_GetLED_Voltage   ( );
        Sensor_GetTemperature   ( );

        Angled.Sensor_LED_Voltage_Display   = ( float ) ( ( Angled.Sensor_LED_Voltage    * 0.0458 ) + 1399.9 );
        Straight.Sensor_LED_Voltage_Display = ( float ) ( ( Straight.Sensor_LED_Voltage  * 0.0458 ) + 1399.9 );

        bme280_read_raw ( SENSOR_CO2_ANGLED_I2C   , &Raw_Temperature_Angled   , &Raw_Pressure_Angled   , &Raw_Humidity_Angled   );
        bme280_read_raw ( SENSOR_CO2_STRAIGHT_I2C , &Raw_Temperature_Straight , &Raw_Pressure_Straight , &Raw_Humidity_Straight );
        Temperature_Angled   = compensate_temperature ( Raw_Temperature_Angled   , &Params_Angled   );
        Temperature_Straight = compensate_temperature ( Raw_Temperature_Straight , &Params_Straight );
        Pressure_Angled      = compensate_pressure    ( Raw_Pressure_Angled      , &Params_Angled   );
        Pressure_Straight    = compensate_pressure    ( Raw_Pressure_Straight    , &Params_Straight );
        Humidity_Angled      = compensate_humidity    ( Raw_Humidity_Angled      , &Params_Angled   );
        Humidity_Straight    = compensate_humidity    ( Raw_Humidity_Straight    , &Params_Straight );

        printf ( "Temperature angled   = %.2f degC\n" , Temperature_Angled   / 100.f  );
        printf ( "Temperature straight = %.2f degC\n" , Temperature_Straight / 100.f  );
        printf ( "Pressure angled      = %.3f kPa\n"  , Pressure_Angled      / 1000.f );
        printf ( "Pressure straight    = %.3f kPa\n"  , Pressure_Straight    / 1000.f );
        printf ( "Humidity angled      = %.3f \%RH\n" , Humidity_Angled      / 1024.f );
        printf ( "Humidity straight    = %.3f \%RH\n" , Humidity_Straight    / 1024.f );
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

    I2C_TX_Buffer [ 0 ] = CNTL6;
    I2C_TX_Buffer [ 1 ] = 0x02; // Single measurement mode
    i2c_write_blocking ( SENSOR_CO2_ANGLED_I2C   , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );
    i2c_write_blocking ( SENSOR_CO2_STRAIGHT_I2C , SENSOR_ADDRESS , I2C_TX_Buffer , 2 , false );
}

/* End of file ---------------------------------------------------------------*/

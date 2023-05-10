/* 
    This code originates from the Getting started with Raspberry Pi Pico document
    https://datasheets.raspberrypi.org/pico/getting-started-with-pico.pdf
    CC BY-ND Raspberry Pi (Trading) Ltd
*/

#include <main.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include <hardware/pwm.h>

struct repeating_timer timer_heartbeat;

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

void main ( void )
{
    uint32_t PWM_DC = 0xFFFF;

    // Useful information for picotool
    bi_decl ( bi_program_description ( "RP2040 Premier" ) );
    
    stdio_init_all();

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

    add_repeating_timer_ms (  500 , timer_heartbeat_isr , NULL , &timer_heartbeat );

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

    gpio_set_function ( PWM_OUT_PIN , GPIO_FUNC_PWM );
    uint8_t slice_num = pwm_gpio_to_slice_num ( PWM_OUT_PIN );
    uint8_t channel   = pwm_gpio_to_channel   ( PWM_OUT_PIN );
    pwm_set_wrap       ( slice_num , 0xFFFF );
    pwm_set_chan_level ( slice_num , channel , ( uint16_t ) PWM_DC );
    pwm_set_enabled    ( slice_num , true );

    PWM_DC = 0;
	sleep_ms ( 1000 );

    for ( ; ; )
    {
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

        sleep_ms ( 1000 );
    }
}
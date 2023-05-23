
add_executable(src
        main.c
        bme280.c
        )

pico_enable_stdio_uart(src 1)

# pull in common dependencies
target_link_libraries(src
        hardware_adc
        hardware_dma
        hardware_i2c
        hardware_irq
        hardware_pio
        hardware_pwm
        hardware_spi
        hardware_sync
        hardware_uart
        hardware_watchdog
        pico_stdlib
        )

# create map/bin/hex file etc.
pico_add_extra_outputs(src)

include_directories(
	../inc
	)

## add url via pico_set_program_url
#//example_auto_set_url(adc)
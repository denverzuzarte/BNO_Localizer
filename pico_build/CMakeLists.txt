cmake_minimum_required(VERSION 3.13)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(blink C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(blink_test blink_test.c)
target_link_libraries(blink_test pico_stdlib)
pico_add_extra_outputs(blink_test)

add_executable(i2c_scanner i2c_scanner.c)
target_link_libraries(i2c_scanner pico_stdlib hardware_i2c)
pico_add_extra_outputs(i2c_scanner)

add_executable(shtp_1 shtp_1.c)
target_link_libraries(shtp_1 pico_stdlib hardware_i2c)
pico_add_extra_outputs(shtp_1)

add_executable(shtp_2 shtp_2.c)
target_link_libraries(shtp_2 pico_stdlib hardware_i2c)
pico_add_extra_outputs(shtp_2)

add_executable(pico_imu pico_imu.c)
target_link_libraries(pico_imu pico_stdlib hardware_i2c)
pico_add_extra_outputs(pico_imu)

add_executable(imu imu.c)
target_link_libraries(imu pico_stdlib hardware_i2c)
pico_add_extra_outputs(imu)

add_executable(imu_single_read imu_single_read.c)
target_link_libraries(imu_single_read pico_stdlib hardware_i2c)
pico_add_extra_outputs(imu_single_read)

pico_enable_stdio_usb(i2c_scanner 1)
pico_enable_stdio_uart(i2c_scanner 0)

pico_enable_stdio_usb(shtp_1 1)
pico_enable_stdio_uart(shtp_1 0)

pico_enable_stdio_usb(shtp_2 1)
pico_enable_stdio_uart(shtp_2 0)

pico_enable_stdio_usb(pico_imu 1)
pico_enable_stdio_uart(pico_imu 0)

pico_enable_stdio_usb(imu 1)
pico_enable_stdio_uart(imu 0)

pico_enable_stdio_usb(imu_single_read 1)
pico_enable_stdio_uart(imu_single_read 0)

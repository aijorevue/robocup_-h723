cmake_minimum_required(VERSION 3.20)

project(DM_MC02_Gyro_3m C ASM)

set(MCU_FLAGS
    -mcpu=cortex-m7
    -mthumb
    -mfpu=fpv5-d16
    -mfloat-abi=hard
)

set(HAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/vendor/stm32/Drivers/STM32H7xx_HAL_Driver)
set(CMSIS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/vendor/stm32/Drivers/CMSIS)
set(USB_DEVICE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/usb_device)
set(USB_MW_DIR ${CMAKE_CURRENT_SOURCE_DIR}/vendor/stm32/Middlewares/ST/STM32_USB_Device_Library)

add_executable(${CMAKE_PROJECT_NAME}.elf
    startup_stm32h723xx.s
    src/exit_run0_mode.c
    src/route_controller.c
    src/main.c
    src/board.c
    src/mecanum.c
    src/motor_output.c
    src/run_log.c
    usb_device/app/usb_device.c
    usb_device/app/usbd_cdc_if.c
    usb_device/app/usbd_desc.c
    usb_device/target/usbd_conf.c
    bmi088/BMI088driver.c
    bmi088/BMI088Middleware.c
    vendor/dm_motor/dm_motor_protocol.c
    vendor/stm32/system_stm32h7xx.c
    ${HAL_DIR}/Src/stm32h7xx_hal.c
    ${HAL_DIR}/Src/stm32h7xx_hal_cortex.c
    ${HAL_DIR}/Src/stm32h7xx_hal_fdcan.c
    ${HAL_DIR}/Src/stm32h7xx_hal_flash.c
    ${HAL_DIR}/Src/stm32h7xx_hal_flash_ex.c
    ${HAL_DIR}/Src/stm32h7xx_hal_gpio.c
    ${HAL_DIR}/Src/stm32h7xx_hal_pwr.c
    ${HAL_DIR}/Src/stm32h7xx_hal_pwr_ex.c
    ${HAL_DIR}/Src/stm32h7xx_hal_pcd.c
    ${HAL_DIR}/Src/stm32h7xx_hal_pcd_ex.c
    ${HAL_DIR}/Src/stm32h7xx_hal_rcc.c
    ${HAL_DIR}/Src/stm32h7xx_hal_rcc_ex.c
    ${HAL_DIR}/Src/stm32h7xx_hal_spi.c
    ${HAL_DIR}/Src/stm32h7xx_hal_spi_ex.c
    ${HAL_DIR}/Src/stm32h7xx_hal_tim.c
    ${HAL_DIR}/Src/stm32h7xx_hal_tim_ex.c
    ${HAL_DIR}/Src/stm32h7xx_hal_uart.c
    ${HAL_DIR}/Src/stm32h7xx_hal_uart_ex.c
    ${HAL_DIR}/Src/stm32h7xx_ll_usb.c
    ${USB_MW_DIR}/Core/Src/usbd_core.c
    ${USB_MW_DIR}/Core/Src/usbd_ctlreq.c
    ${USB_MW_DIR}/Core/Src/usbd_ioreq.c
    ${USB_MW_DIR}/Class/CDC/Src/usbd_cdc.c
)

target_compile_definitions(${CMAKE_PROJECT_NAME}.elf PRIVATE
    USE_HAL_DRIVER
    STM32H723xx
    USE_PWR_LDO_SUPPLY
)

target_include_directories(${CMAKE_PROJECT_NAME}.elf PRIVATE
    include
    bmi088
    vendor/dm_motor
    ${USB_DEVICE_DIR}/app
    ${USB_DEVICE_DIR}/target
    ${USB_MW_DIR}/Core/Inc
    ${USB_MW_DIR}/Class/CDC/Inc
    ${HAL_DIR}/Inc
    ${HAL_DIR}/Inc/Legacy
    ${CMSIS_DIR}/Device/ST/STM32H7xx/Include
    ${CMSIS_DIR}/Include
)

target_compile_options(${CMAKE_PROJECT_NAME}.elf PRIVATE
    ${MCU_FLAGS}
    -ffunction-sections
    -fdata-sections
    -fno-common
    -Wall
    -Wextra
    -Wno-unused-parameter
    -g3
    $<$<CONFIG:Debug>:-O0>
    $<$<NOT:$<CONFIG:Debug>>:-O2>
)

target_link_options(${CMAKE_PROJECT_NAME}.elf PRIVATE
    ${MCU_FLAGS}
    -T${CMAKE_CURRENT_SOURCE_DIR}/STM32H723VG_FLASH.ld
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    --specs=nano.specs
    --specs=nosys.specs
)

target_link_libraries(${CMAKE_PROJECT_NAME}.elf PRIVATE m c nosys)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} ${CMAKE_PROJECT_NAME}.elf
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

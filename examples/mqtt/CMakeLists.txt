set(TARGET_NAME wizchip_mqtt)

add_executable(${TARGET_NAME}
        ${TARGET_NAME}.c
        )

target_include_directories(${TARGET_NAME} PUBLIC
        ${PORT_DIR}/FreeRTOS-Kernel/inc
)

target_link_libraries(${TARGET_NAME} PRIVATE
        pico_stdlib
        hardware_spi
        hardware_dma
        FreeRTOS-Kernel-Heap4
        ETHERNET_FILES
        IOLIBRARY_FILES
        MQTT_FILES
        TIMER_FILES
        )

pico_enable_stdio_usb(${TARGET_NAME} 1)
pico_enable_stdio_uart(${TARGET_NAME} 0)

pico_add_extra_outputs(wizchip_mqtt)

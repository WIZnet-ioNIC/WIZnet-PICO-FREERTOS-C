# CMake minimum required version
cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
if(NOT DEFINED PICO_SDK_PATH)
    set(PICO_SDK_PATH ${CMAKE_SOURCE_DIR}/libraries/pico-sdk)
    message(STATUS "PICO_SDK_PATH = ${PICO_SDK_PATH}")
endif()

include(pico_sdk_import.cmake)
include(wiznet_pico_freertos_c_sdk_version.cmake)

# Pull in FreeRTOS (must be before project)
if(NOT DEFINED FREERTOS_KERNEL_PATH)
set(FREERTOS_KERNEL_PATH ${CMAKE_SOURCE_DIR}/libraries/FreeRTOS-Kernel)
    message(STATUS "FREERTOS_KERNEL_PATH = ${FREERTOS_KERNEL_PATH}")
endif()
include(FreeRTOS_Kernel_import.cmake)

# Set project name
set(PROJECT_NAME WIZNET-PICO-FREERTOS-C)

# Set project informations
project(${PROJECT_NAME} C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# Set board
#set(BOARD_NAME WIZnet_Ethernet_HAT)
#set(BOARD_NAME W5100S_EVB_PICO)
set(BOARD_NAME W5500_EVB_PICO)
#set(BOARD_NAME W55RP20_EVB_PICO)

if(${BOARD_NAME} STREQUAL WIZnet_Ethernet_HAT)
    set(WIZNET_CHIP W5100S)
    add_definitions(-D_WIZCHIP_=W5100S)
    add_definitions(-DDEVICE_BOARD_NAME=WIZnet_Ethernet_HAT)
elseif(${BOARD_NAME} STREQUAL W5100S_EVB_PICO)
    set(WIZNET_CHIP W5100S)
    add_definitions(-D_WIZCHIP_=W5100S)
    add_definitions(-DDEVICE_BOARD_NAME=W5100S_EVB_PICO)
elseif(${BOARD_NAME} STREQUAL W5500_EVB_PICO)
    set(WIZNET_CHIP W5500)
    add_definitions(-D_WIZCHIP_=W5500)
    add_definitions(-DDEVICE_BOARD_NAME=W5500_EVB_PICO)
elseif(${BOARD_NAME} STREQUAL W55RP20_EVB_PICO)
    set(WIZNET_CHIP W5500)
    add_definitions(-D_WIZCHIP_=W5500)
    add_definitions(-DDEVICE_BOARD_NAME=W55RP20_EVB_PICO)
else() 
    message(FATAL_ERROR "BOARD_NAME is wrong = ${BOARD_NAME}")
endif()

message(STATUS "WIZNET_CHIP = ${WIZNET_CHIP}")

# Set the project root directory if it's not already defined, as may happen if
# the tests folder is included directly by a parent project, without including
# the top level CMakeLists.txt.
if(NOT DEFINED FREERTOS_DIR)
    set(FREERTOS_DIR ${CMAKE_SOURCE_DIR}/libraries/FreeRTOS-Kernel)
    message(STATUS "FREERTOS_DIR = ${FREERTOS_DIR}")
endif()

if(NOT DEFINED WIZNET_DIR)
    set(WIZNET_DIR ${CMAKE_SOURCE_DIR}/libraries/ioLibrary_Driver)
    message(STATUS "WIZNET_DIR = ${WIZNET_DIR}")
endif()

if(NOT DEFINED MBEDTLS_DIR)
    set(MBEDTLS_DIR ${CMAKE_SOURCE_DIR}/libraries/mbedtls)
    message(STATUS "MBEDTLS_DIR = ${MBEDTLS_DIR}")
endif()

if(NOT DEFINED PORT_DIR)
    set(PORT_DIR ${CMAKE_SOURCE_DIR}/port)
    message(STATUS "PORT_DIR = ${PORT_DIR}")
endif()

# Turn off mbedtls test mode 
set(ENABLE_PROGRAMS OFF CACHE BOOL "Build mbedtls programs")
set(ENABLE_TESTING OFF CACHE BOOL "Build mbedtls testing")
add_definitions(-DMBEDTLS_CONFIG_FILE="${PORT_DIR}/mbedtls/inc/ssl_config.h")
add_definitions(-DSET_TRUSTED_CERT_IN_SAMPLES)

# Hardware-specific examples in subdirectories:
add_subdirectory(examples)

# Add libraries in subdirectories
add_subdirectory(${CMAKE_SOURCE_DIR}/libraries)
#add_subdirectory(${FREERTOS_DIR})
add_subdirectory(${MBEDTLS_DIR})
add_subdirectory(${PORT_DIR})

# Set compile options
add_compile_options(
        -Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )

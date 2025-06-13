# Getting Started with FreeRTOS Examples

These sections will guide you through a series of steps from configuring development environment to running FreeRTOS examples using the **WIZnet's ethernet products**.

- [Getting Started with FreeRTOS Examples](#getting-started-with-freertos-examples)
  - [Development environment configuration](#development-environment-configuration)
  - [Hardware requirements](#hardware-requirements)
  - [FreeRTOS example structure](#freertos-example-structure)
  - [FreeRTOS example testing](#freertos-example-testing)
  - [How to use port directory](#how-to-use-port-directory)



<a name="development_environment_configuration"></a>
## Development environment configuration

To test the FreeRTOS examples, the development environment must be configured to Raspberry Pi Pico, W5100S-EVB-Pico, W5500-EVB-Pico, W55RP20-EVB-Pico, W6100-EVB-Pico, W6300-EVB-Pico, W5100S-EVB-Pico2, W5500-EVB-Pico2, W6100-EVB-Pico2 or W6300-EVB-Pico2.

These examples were tested after configuring the development environment on **Windows**. Please refer to '**Chapter 3: Installing the Raspberry Pi Pico VS Code Extension**' in the document below and configure accordingly.

- [**Getting started with Raspberry Pi Pico**][link-getting_started_with_raspberry_pi_pico]

**Visual Studio Code** was used during development and testing of FreeRTOS examples, the guide document in each directory was prepared also base on development with Visual Studio Code. Please refer to corresponding document.



<a name="hardware_requirements"></a>
## Hardware requirements

The Ethernet examples are compatible with the following Raspberry Pi-compatible WIZnet Ethernet I/O modules. These modules integrate [**WIZnet Ethernet chips**][link-wiznet_ethernet_chips] with either the [**RP2040**][link-rp2040] or [**RP2350**][link-rp2350] microcontrollers.

| Board/Module Name              | MCU      | Ethernet Chip  | Interface     | Socket # | TX/RX Buffer  | Notes                                  |
|--------------------------------|----------|----------------|---------------|----------|---------------|----------------------------------------|
| **[WIZnet Ethernet HAT][link-wiznet_ethernet_hat]** |  | W5100S | SPI | 4 | 16KB | RP Pico-compatible |
| **[W5100S-EVB-Pico][link-w5100s-evb-pico]** | RP2040 | W5100S | SPI | 4 | 16KB |  |
| **[W5500-EVB-Pico][link-w5500-evb-pico]** | RP2040 | W5500 | SPI | 8 | 32KB |  |
| **[W55RP20-EVB-Pico][link-w55rp20-evb-pico]** | RP2040 | W5500 | SPI (PIO) | 8 | 32KB | SiP: RP2040 + W5500 |
| **[W6100-EVB-Pico][link-w6100-evb-pico]** | RP2040 | W6100 | SPI | 8 | 32KB | Supports IPv4/IPv6 |
| **[W6300-EVB-Pico][link-w6300-evb-pico]** | RP2040 | W6300 | QSPI (PIO) | 8 | 64KB | Supports IPv4/IPv6 |
| **[W5100S-EVB-Pico2][link-w5100s-evb-pico2]** | RP2350 | W5100S | SPI | 4 | 16KB |  |
| **[W5500-EVB-Pico2][link-w5500-evb-pico2]** | RP2350 | W5500 | SPI | 8 | 32KB |  |
| **[W6100-EVB-Pico2][link-w6100-evb-pico2]** | RP2350 | W6100 | SPI | 8 | 32KB | Supports IPv4/IPv6 |
| **[W6300-EVB-Pico2][link-w6300-evb-pico2]** | RP2350 | W6300 | QSPI (PIO) | 8 | 64KB | Supports IPv4/IPv6 |


<a name="ethernet_example_structure"></a>


<a name="freertos_example_structure"></a>
## FreeRTOS example structure

Examples are available at '**WIZnet-PICO-FREERTOS-C/examples/**' directory. As of now, following examples are provided.

- [**DHCP & DNS**][link-dhcp_dns]
- [**MQTT**][link-mqtt]
- [**TCP Client over SSL**][link-tcp_client_over_ssl]

Note that **ioLibrary_Driver**, **FreeRTOS-Kernel**, **mbedtls**, **pico-sdk** are needed to run FreeRTOS examples.

- **ioLibrary_Driver** library is applicable to WIZnet's W5x00, W6x00 ethernet chip.
- **FreeRTOS-Kernel** is a real-time operating system kernel for embedded devices that contains FreeRTOS kernel source/header files and kernel ports only.
- **mbedtls** library supports additional algorithms and support related to SSL and TLS connections.
- **pico-sdk** is made available by Pico to enable developers to build software applications for the Pico platform.
- **pico-extras** has additional libraries that are not yet ready for inclusion the Pico SDK proper, or are just useful but don't necessarily belong in the Pico SDK.

Libraries are located in the '**WIZnet-PICO-FREERTOS-C/libraries/**' directory.

- [**ioLibrary_Driver**][link-iolibrary_driver]
- [**FreeRTOS-Kernel**][link-freertos_kernel]
- [**mbedtls**][link-mbedtls]
- [**pico-sdk**][link-pico_sdk]
- [**pico-extras**][link-pico_extras]

If you want to modify the code that MCU-dependent and use a MCU other than **RP2040** & **RP2350**, you can modify it in the **WIZnet-PICO-FREERTOS-C/port/** directory.

port is located in the '**WIZnet-PICO-FREERTOS-C/port/**' directory.

- [**ioLibrary_Driver**][link-port_iolibrary_driver]
- [**FreeRTOS-Kernel**][link-port_freertos_kernel]
- [**mbedtls**][link-port_mbedtls]
- [**timer**][link-port_timer]



<a name="freertos_example_testing"></a>
## FreeRTOS example testing

1. Download

If the FreeRTOS examples are cloned, the library set as a submodule is an empty directory. Therefore, if you want to download the library set as a submodule together, clone the FreeRTOS examples with the following Git command.

```cpp
/* Change directory */
// change to the directory to clone
cd [user path]

// e.g.
cd D:/WIZnet-Pico

/* Clone */
git clone --recurse-submodules https://github.com/WIZnet-ioNIC/WIZnet-PICO-FREERTOS-C.git
```

With Visual Studio Code, the library set as a submodule is automatically downloaded, so it doesn't matter whether the library set as a submodule is an empty directory or not, so refer to it.

2. Setup ethetnet chip

Setup the ethernet chip in '**CMakeLists.txt**' in '**WIZnet-PICO-FREERTOS-C/**' directory according to the evaluation board to be used referring to the following.

- **[WIZnet Ethernet HAT][link-wiznet_ethernet_hat]**
- **[W5100S-EVB-Pico][link-w5100s-evb-pico]**
- **[W5500-EVB-Pico][link-w5500-evb-pico]**
- **[W55RP20-EVB-Pico][link-w55rp20-evb-pico]**
- **[W6100-EVB-Pico][link-w6100-evb-pico]**
- **W6300-EVB-Pico**
- **[W5100S-EVB-Pico2][link-w5100s-evb-pico2]**
- **[W5500-EVB-Pico2][link-w5500-evb-pico2]**
- **[W6100-EVB-Pico2][link-w6100-evb-pico2]**
- **[W6300-EVB-Pico2][link-w6300-evb-pico2]**


For example, when using WIZnet Ethernet HAT :

```cpp
# Set board
set(BOARD_NAME WIZnet_Ethernet_HAT)
```

When using W5500-EVB-Pico :

```cpp
# Set board
set(BOARD_NAME W5500_EVB_PICO)
```

You can easily configure **SPI clock speed of the WIZnet chip** in the CMakeLists.txt file. Enter your desired clock speed in the code below and build.

```cpp
# Set WIZchip Clock Speed
add_definitions(-D_WIZCHIP_SPI_SCLK_SPEED=40) # SPEED MHz
```

**When using W6300**, **you can configure the QSPI mode** by modifying the board selection parameter.

For example, when using **QSPI QUAD MODE**:

```cpp
# Set QSPI MODE for W6300
    add_definitions(-D_WIZCHIP_QSPI_MODE_=QSPI_QUAD_MODE) # QSPI_QUAD_MODE
    # add_definitions(-D_WIZCHIP_QSPI_MODE_=QSPI_DUAL_MODE) # QSPI_DUAL_MODE 
    # add_definitions(-D_WIZCHIP_QSPI_MODE_=QSPI_SINGLE_MODE) # QSPI_SINGLE_MODE 
```

3. Test

Please refer to 'README.md' in each example directory to find detail guide for testing FreeRTOS examples.

> ※ If the board pauses when rebooting using W55RP20-EVB-Pico, patch it as follows.
>
> ```cpp
> // Patch
> git apply ./patches/0001_pico_sdk_clocks.patch
> ```

> ※ To test the TFTP example, please apply the following patch.
> 
> ```cpp
> cd libraries/ioLibrary_Driver
> git apply ../../patches/0002_iolibrary_driver_tftp.patch
> ```

> ※ To test the FTP client example, please apply the following patch.
> 
> ```cpp
> cd libraries/ioLibrary_Driver
> git apply ../../patches/0003_iolibrary_driver_ftp_client.patch
> ```

<a name="how_to_use_port_directory"></a>
## How to use port directory

We moved the MCU dependent code to the port directory. The tree of port is shown below.

```
WIZNET-PICO-FREERTOS-C
┣ port
    ┣ FreeRTOS-Kernel
    ┃   ┗ inc
    ┃   ┃   ┗ FreeRTOSConfig.h
    ┣ ioLibrary_Driver
    ┃   ┣ inc
    ┃   ┃   ┣ wizchip_gpio_irq.h
    ┃   ┃   ┣ wizchip_spi.h
    ┃   ┃   ┣ wizchip_qspi_pio.h
    ┃   ┃   ┗ wiznet_spi.h
    ┃   ┗ src
    ┃   ┃   ┣ wizchip_gpio_irq.c
    ┃   ┃   ┣ wizchip_spi.c
    ┃   ┃   ┣ wizchip_qspi_pio.c
    ┃   ┃   ┗ wizchip_spi_pio.pio
    ┣ mbedtls
    ┃   ┗ inc
    ┃   ┃   ┗ ssl_config.h
    ┣ timer
    ┃   ┣ timer.c
    ┃   ┗ timer.h
    ┣ CMakeLists.txt
    ┗ port_common.h
```

- **ioLibrary_Driver**

If you want to change things related to **SPI**, such as the SPI port number and SPI read/write function, or GPIO port number and function related to **interrupt** or use a different MCU without using the RP2040 & RP2350, you need to change the code in the '**WIZnet-PICO-FREERTOS-C/port/ioLibrary_Driver/**' directory. Here is information about functions.

```cpp
/* wizchip_spi */
/*! \brief Set CS pin
 *  \ingroup wizchip_spi
 *
 *  Set chip select pin of spi0 to low(Active low).
 *
 *  \param none
 */
static inline void wizchip_select(void);

/*! \brief Set CS pin
 *  \ingroup wizchip_spi
 *
 *  Set chip select pin of spi0 to high(Inactive high).
 *
 *  \param none
 */
static inline void wizchip_deselect(void);

/*! \brief Read from an SPI device, blocking
 *  \ingroup wizchip_spi
 *
 *  Set spi_read_blocking function.
 *  Read byte from SPI to rx_data buffer.
 *  Blocks until all data is transferred. No timeout, as SPI hardware always transfers at a known data rate.
 *
 *  \param none
 */
static uint8_t wizchip_read(void);

/*! \brief Write to an SPI device, blocking
 *  \ingroup wizchip_spi
 *
 *  Set spi_write_blocking function.
 *  Write byte from tx_data buffer to SPI device.
 *  Blocks until all data is transferred. No timeout, as SPI hardware always transfers at a known data rate.
 *
 *  \param tx_data Buffer of data to write
 */
static void wizchip_write(uint8_t tx_data);

#ifdef USE_SPI_DMA
/*! \brief Configure all DMA parameters and optionally start transfer
 *  \ingroup wizchip_spi
 *
 *  Configure all DMA parameters and read from DMA
 *
 *  \param pBuf Buffer of data to read
 *  \param len element count (each element is of size transfer_data_size)
 */
static void wizchip_read_burst(uint8_t *pBuf, uint16_t len);

/*! \brief Configure all DMA parameters and optionally start transfer
 *  \ingroup wizchip_spi
 *
 *  Configure all DMA parameters and write to DMA
 *
 *  \param pBuf Buffer of data to write
 *  \param len element count (each element is of size transfer_data_size)
 */
static void wizchip_write_burst(uint8_t *pBuf, uint16_t len);
#endif

/*! \brief Enter a critical section
 *  \ingroup wizchip_spi
 *
 *  Set ciritical section enter blocking function.
 *  If the spin lock associated with this critical section is in use, then this
 *  method will block until it is released.
 *
 *  \param none
 */
static void wizchip_critical_section_lock(void);

/*! \brief Release a critical section
 *  \ingroup wizchip_spi
 *
 *  Set ciritical section exit function.
 *  Release a critical section.
 *
 *  \param none
 */
static void wizchip_critical_section_unlock(void);

/*! \brief Initialize SPI instances and Set DMA channel
 *  \ingroup wizchip_spi
 *
 *  Set GPIO to spi0.
 *  Puts the SPI into a known state, and enable it.
 *  Set DMA channel completion channel.
 *
 *  \param none
 */
void wizchip_spi_initialize(void);

/*! \brief Initialize a critical section structure
 *  \ingroup wizchip_spi
 *
 *  The critical section is initialized ready for use.
 *  Registers callback function for critical section for WIZchip.
 *
 *  \param none
 */
void wizchip_cris_initialize(void);

/*! \brief wizchip chip reset
 *  \ingroup wizchip_spi
 *
 *  Set a reset pin and reset.
 *
 *  \param none
 */
void wizchip_reset(void);

/*! \brief Initialize WIZchip
 *  \ingroup wizchip_spi
 *
 *  Set callback function to read/write byte using SPI.
 *  Set callback function for WIZchip select/deselect.
 *  Set memory size of wizchip chip and monitor PHY link status.
 *
 *  \param none
 */
void wizchip_initialize(void);

/*! \brief Check chip version
 *  \ingroup wizchip_spi
 *
 *  Get version information.
 *
 *  \param none
 */
void wizchip_check(void);

/* Network */
/*! \brief Initialize network
 *  \ingroup wizchip_spi
 *
 *  Set network information.
 *
 *  \param net_info network information.
 */
void network_initialize(wiz_NetInfo net_info);

/*! \brief Print network information
 *  \ingroup wizchip_spi
 *
 *  Print network information about MAC address, IP address, Subnet mask, Gateway, DHCP and DNS address.
 *
 *  \param net_info network information.
 */
void print_network_information(wiz_NetInfo net_info);
```

```cpp
/* GPIO */
/*! \brief Initialize wizchip gpio interrupt callback function
 *  \ingroup wizchip_gpio_irq
 *
 *  Add a wizchip interrupt callback.
 *
 *  \param socket socket number
 *  \param callback the gpio interrupt callback function
 */
void wizchip_gpio_interrupt_initialize(uint8_t socket, void (*callback)(void));

/*! \brief Assign gpio interrupt callback function
 *  \ingroup wizchip_gpio_irq
 *
 *  GPIO interrupt callback function.
 *
 *  \param gpio Which GPIO caused this interrupt
 *  \param events Which events caused this interrupt. See \ref gpio_set_irq_enabled for details.
 */
static void wizchip_gpio_interrupt_callback(uint gpio, uint32_t events);
```

- **timer**

If you want to change things related to the **timer**. Also, if you use a different MCU without using the  & RP2350, you need to change the code in the '**WIZnet-PICO-FREERTOS-C/port/timer/**' directory. Here is information about functions.

```cpp
/* Timer */
/*! \brief Initialize timer callback function
 *  \ingroup timer
 *
 *  Add a repeating timer that is called repeatedly at the specified interval in microseconds.
 *
 *  \param callback the repeating timer callback function
 */
void wizchip_1ms_timer_initialize(void (*callback)(void));

/*! \brief Assign timer callback function
 *  \ingroup timer
 *
 *  1ms timer callback function.
 *
 *  \param t Information about a repeating timer
 */
bool wizchip_1ms_timer_callback(struct repeating_timer *t);

/* Delay */
/*! \brief Wait for the given number of milliseconds before returning
 *  \ingroup timer
 *
 *  This method attempts to perform a lower power sleep (using WFE) as much as possible.
 *
 *  \param ms the number of milliseconds to sleep
 */
void wizchip_delay_ms(uint32_t ms);
```



<!--
Link
-->

[link-getting_started_with_raspberry_pi_pico]: https://datasheets.raspberrypi.org/pico/getting-started-with-pico.pdf
[link-rp2040]: https://www.raspberrypi.org/products/rp2040/
[link-w5100s]: https://docs.wiznet.io/Product/iEthernet/W5100S/overview
[link-w5500]: https://docs.wiznet.io/Product/iEthernet/W5500/overview
[link-w55rp20-evb-pico]: https://docs.wiznet.io/Product/ioNIC/W55RP20/w55rp20-evb-pico#overview
[link-raspberry_pi_pico]: https://www.raspberrypi.org/products/raspberry-pi-pico/
[link-wiznet_ethernet_hat]: https://docs.wiznet.io/Product/Open-Source-Hardware/wiznet_ethernet_hat
[link-w5100s-evb-pico]: https://docs.wiznet.io/Product/iEthernet/W5100S/w5100s-evb-pico
[link-w5500-evb-pico]: https://docs.wiznet.io/Product/iEthernet/W5500/w5500-evb-pico
[link-dhcp_dns]: https://github.com/WIZnet-ioNIC/WIZnet-PICO-FREERTOS-C/tree/main/examples/dhcp_dns
[link-mqtt]: https://github.com/WIZnet-ioNIC/WIZnet-PICO-FREERTOS-C/tree/main/examples/mqtt
[link-tcp_client_over_ssl]: https://github.com/WIZnet-ioNIC/WIZnet-PICO-FREERTOS-C/tree/main/examples/tcp_client_over_ssl
[link-iolibrary_driver]: https://github.com/Wiznet/ioLibrary_Driver
[link-freertos_kernel]: https://github.com/FreeRTOS/FreeRTOS-Kernel
[link-mbedtls]: https://github.com/ARMmbed/mbedtls
[link-pico_sdk]: https://github.com/raspberrypi/pico-sdk
[link-pico_extras]: https://github.com/raspberrypi/pico-extras
[link-port_iolibrary_driver]: https://github.com/WIZnet-ioNIC/WIZnet-PICO-FREERTOS-C/tree/main/port/ioLibrary_Driver
[link-port_freertos_kernel]: https://github.com/WIZnet-ioNIC/WIZnet-PICO-FREERTOS-C/tree/main/port/FreeRTOS-Kernel
[link-port_mbedtls]: https://github.com/WIZnet-ioNIC/WIZnet-PICO-FREERTOS-C/tree/main/port/mbedtls
[link-port_timer]: https://github.com/WIZnet-ioNIC/WIZnet-PICO-FREERTOS-C/tree/main/port/timer
  
[link-rp2350]: https://www.raspberrypi.com/products/rp2350/
[link-w6100]: https://docs.wiznet.io/Product/iEthernet/W6100/overview
[link-w6300]: https://docs.wiznet.io/Product/iEthernet/W6300/overview
[link-wiznet_ethernet_chips]: https://docs.wiznet.io/Product/iEthernet#product-family
[link-raspberry_pi_pico]: https://www.raspberrypi.com/products/raspberry-pi-pico/
[link-wiznet_ethernet_hat]: https://docs.wiznet.io/Product/Open-Source-Hardware/wiznet_ethernet_hat
[link-w6100-evb-pico]: https://docs.wiznet.io/Product/iEthernet/W6100/w6100-evb-pico
[link-w6300-evb-pico]: https://docs.wiznet.io/Product/iEthernet/W6300/w6300-evb-pico
[link-w5100s-evb-pico2]: https://docs.wiznet.io/Product/iEthernet/W5100S/w5100s-evb-pico2
[link-w5500-evb-pico2]: https://docs.wiznet.io/Product/iEthernet/W5500/w5500-evb-pico2
[link-w6100-evb-pico2]: https://docs.wiznet.io/Product/iEthernet/W6100/w6100-evb-pico2
[link-w6300-evb-pico2]: https://docs.wiznet.io/Product/iEthernet/W6300/w6300-evb-pico2

[link-w5100s]: https://docs.wiznet.io/Product/iEthernet/W5100S/overview
[link-w5500]: https://docs.wiznet.io/Product/iEthernet/W5500/overview
[link-w6100]: https://docs.wiznet.io/Product/iEthernet/W6100
[link-w6300]: https://docs.wiznet.io/Product/iEthernet/W6300

/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "bus_pal_hardware.h"
#include "fpga_clock_registers.h"
#include "fsl_device_registers.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_iopctl.h"
#include "microseconds/microseconds.h"
#include "fsl_flexcomm.h"
#include "fsl_spi.h"
#include "fsl_usart.h"

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

/*!
 * @brief init uart functions.
 */
static void init_uarts(void);

/*!
 * @brief dspi initialization.
 */
static void init_spi(void);

/*!
 * @brief i2c initialization.
 */
static void init_i2c(uint32_t instance);

/*!
 * @brief i2c de-initialization.
 */
static void deinit_i2c(uint32_t instance);

/*!
 * @brief uart rx callback function.
 */
static void uart_rx_callback(uint8_t byte);

/*!
 * @brief get GPIO base address function.
 */
static GPIO_Type *getGpioBaseAddrFromAscii(uint8_t port);

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! @brief Variable for spi host configuration information
static spi_user_config_t s_spiUserConfig = {.polarity     = kSPI_ClockPolarityActiveLow, /*!< Clock polarity */
                                            .phase        = kSPI_ClockPhaseSecondEdge,   /*!< Clock phase */
                                            .direction    = kSPI_MsbFirst,               /*!< MSB or LSB */
                                            .baudRate_Bps = 100000,                        /*!< Baud Rate for SPI in Hz */
                                            .clock_Hz     = 0 };

static spi_master_handle_t s_spiHandle;

//! @brief Variable for host data receiving
static uint8_t *s_rxData;
static uint32_t s_bytesRx;

const static uint32_t g_spiBaseAddr[]  = SPI_BASE_ADDRS;
const static uint32_t g_uartBaseAddr[] = USART_BASE_ADDRS;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

/*FUNCTION**********************************************************************
 *
 * Function Name : get_bus_clock
 * Description   : Gets bus clock
 *
 *END**************************************************************************/
uint32_t get_bus_clock(void)
{
    return CLOCK_GetFreq(kCLOCK_BusClk);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : init_hardware
 * Description   : hardware initialization
 *
 *END**************************************************************************/
void init_hardware(void)
{
    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();

    // Enable pins for Flexcomm0 USART.
    const uint32_t port0_pin1_config = (/* Pin is configured as FC0_TXD_SCL_MISO_WS */
                                        IOPCTL_PIO_FUNC1 |
                                        /* Disable pull-up / pull-down function */
                                        IOPCTL_PIO_PUPD_DI |
                                        /* Enable pull-down function */
                                        IOPCTL_PIO_PULLDOWN_EN |
                                        /* Disable input buffer function */
                                        IOPCTL_PIO_INBUF_DI |
                                        /* Normal mode */
                                        IOPCTL_PIO_SLEW_RATE_NORMAL |
                                        /* Normal drive */
                                        IOPCTL_PIO_FULLDRIVE_DI |
                                        /* Analog mux is disabled */
                                        IOPCTL_PIO_ANAMUX_DI |
                                        /* Pseudo Output Drain is disabled */
                                        IOPCTL_PIO_PSEDRAIN_DI |
                                        /* Input function is not inverted */
                                        IOPCTL_PIO_INV_DI);
    /* PORT0 PIN1 (coords: G2) is configured as FC0_TXD_SCL_MISO_WS */
    IOPCTL_PinMuxSet(IOPCTL, 0U, 1U, port0_pin1_config);

    const uint32_t port0_pin2_config = (/* Pin is configured as FC0_RXD_SDA_MOSI_DATA */
                                        IOPCTL_PIO_FUNC1 |
                                        /* Disable pull-up / pull-down function */
                                        IOPCTL_PIO_PUPD_DI |
                                        /* Enable pull-down function */
                                        IOPCTL_PIO_PULLDOWN_EN |
                                        /* Enables input buffer function */
                                        IOPCTL_PIO_INBUF_EN |
                                        /* Normal mode */
                                        IOPCTL_PIO_SLEW_RATE_NORMAL |
                                        /* Normal drive */
                                        IOPCTL_PIO_FULLDRIVE_DI |
                                        /* Analog mux is disabled */
                                        IOPCTL_PIO_ANAMUX_DI |
                                        /* Pseudo Output Drain is disabled */
                                        IOPCTL_PIO_PSEDRAIN_DI |
                                        /* Input function is not inverted */
                                        IOPCTL_PIO_INV_DI);
    /* PORT0 PIN2 (coords: G4) is configured as FC0_RXD_SDA_MOSI_DATA */
    IOPCTL_PinMuxSet(IOPCTL, 0U, 2U, port0_pin2_config);

    // Enable pins for Flexcomm5 SPI
    // RT685-EVK Rev.Ex - J28.3/4/5/6
    const uint32_t port1_pin3_config = (/* Pin is configured as FC5_SCK */
                                        IOPCTL_PIO_FUNC1 |
                                        /* Disable pull-up / pull-down function */
                                        IOPCTL_PIO_PUPD_DI |
                                        /* Enable pull-down function */
                                        IOPCTL_PIO_PULLDOWN_EN |
                                        /* Enables input buffer function */
                                        IOPCTL_PIO_INBUF_EN |
                                        /* Normal mode */
                                        IOPCTL_PIO_SLEW_RATE_NORMAL |
                                        /* Normal drive */
                                        IOPCTL_PIO_FULLDRIVE_DI |
                                        /* Analog mux is disabled */
                                        IOPCTL_PIO_ANAMUX_DI |
                                        /* Pseudo Output Drain is disabled */
                                        IOPCTL_PIO_PSEDRAIN_DI |
                                        /* Input function is not inverted */
                                        IOPCTL_PIO_INV_DI);
    /* PORT1 PIN3 (coords: G16) is configured as FC5_SCK */
    IOPCTL_PinMuxSet(IOPCTL, 1U, 3U, port1_pin3_config);

    const uint32_t port1_pin4_config = (/* Pin is configured as FC5_TXD_SCL_MISO_WS */
                                        IOPCTL_PIO_FUNC1 |
                                        /* Disable pull-up / pull-down function */
                                        IOPCTL_PIO_PUPD_DI |
                                        /* Enable pull-down function */
                                        IOPCTL_PIO_PULLDOWN_EN |
                                        /* Enables input buffer function */
                                        IOPCTL_PIO_INBUF_EN |
                                        /* Normal mode */
                                        IOPCTL_PIO_SLEW_RATE_NORMAL |
                                        /* Normal drive */
                                        IOPCTL_PIO_FULLDRIVE_DI |
                                        /* Analog mux is disabled */
                                        IOPCTL_PIO_ANAMUX_DI |
                                        /* Pseudo Output Drain is disabled */
                                        IOPCTL_PIO_PSEDRAIN_DI |
                                        /* Input function is not inverted */
                                        IOPCTL_PIO_INV_DI);
    /* PORT1 PIN4 (coords: G17) is configured as FC5_TXD_SCL_MISO_WS */
    IOPCTL_PinMuxSet(IOPCTL, 1U, 4U, port1_pin4_config);

    const uint32_t port1_pin5_config = (/* Pin is configured as FC5_RXD_SDA_MOSI_DATA */
                                        IOPCTL_PIO_FUNC1 |
                                        /* Disable pull-up / pull-down function */
                                        IOPCTL_PIO_PUPD_DI |
                                        /* Enable pull-down function */
                                        IOPCTL_PIO_PULLDOWN_EN |
                                        /* Enables input buffer function */
                                        IOPCTL_PIO_INBUF_EN |
                                        /* Normal mode */
                                        IOPCTL_PIO_SLEW_RATE_NORMAL |
                                        /* Normal drive */
                                        IOPCTL_PIO_FULLDRIVE_DI |
                                        /* Analog mux is disabled */
                                        IOPCTL_PIO_ANAMUX_DI |
                                        /* Pseudo Output Drain is disabled */
                                        IOPCTL_PIO_PSEDRAIN_DI |
                                        /* Input function is not inverted */
                                        IOPCTL_PIO_INV_DI);
    /* PORT1 PIN5 (coords: J16) is configured as FC5_RXD_SDA_MOSI_DATA */
    IOPCTL_PinMuxSet(IOPCTL, 1U, 5U, port1_pin5_config);

    const uint32_t port1_pin6_config = (/* Pin is configured as FC5_CTS_SDA_SSEL0 */
                                        IOPCTL_PIO_FUNC1 |
                                        /* Disable pull-up / pull-down function */
                                        IOPCTL_PIO_PUPD_DI |
                                        /* Enable pull-down function */
                                        IOPCTL_PIO_PULLDOWN_EN |
                                        /* Enables input buffer function */
                                        IOPCTL_PIO_INBUF_EN |
                                        /* Normal mode */
                                        IOPCTL_PIO_SLEW_RATE_NORMAL |
                                        /* Normal drive */
                                        IOPCTL_PIO_FULLDRIVE_DI |
                                        /* Analog mux is disabled */
                                        IOPCTL_PIO_ANAMUX_DI |
                                        /* Pseudo Output Drain is disabled */
                                        IOPCTL_PIO_PSEDRAIN_DI |
                                        /* Input function is not inverted */
                                        IOPCTL_PIO_INV_DI);
    /* PORT1 PIN6 (coords: J17) is configured as FC5_CTS_SDA_SSEL0 */
    IOPCTL_PinMuxSet(IOPCTL, 1U, 6U, port1_pin6_config);

    microseconds_init();

    init_uarts();
    init_spi();
}

/*FUNCTION**********************************************************************
 *
 * Function Name : uart_get_clock
 * Description   : get usart clock
 *
 *END**************************************************************************/
uint32_t uart_get_clock(uint32_t instance)
{
    return CLOCK_GetFlexCommClkFreq(instance);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : init_uarts
 * Description   : Initialize UART ports
 *
 *END**************************************************************************/
static void init_uarts(void)
{
    usart_config_t config;
    uint32_t baseAddr = g_uartBaseAddr[0];

    /* attach FRG0 clock to FLEXCOMM0 */
    CLOCK_SetFRGClock((&(const clock_frg_clk_config_t){0, kCLOCK_FrgPllDiv, 255, 0}));
    CLOCK_AttachClk(kFRG_to_FLEXCOMM0);

    /*
     * config.baudRate_Bps = 115200U;
     * config.parityMode = kUSART_ParityDisabled;
     * config.stopBitCount = kUSART_OneStopBit;
     * config.loopback = false;
     * config.enableTxFifo = false;
     * config.enableRxFifo = false;
     */
    USART_GetDefaultConfig(&config);
    config.baudRate_Bps = 57600;
    config.enableTx     = true;
    config.enableRx     = true;

    USART_Init((USART_Type *)baseAddr, &config, uart_get_clock(0));
    USART_EnableInterrupts((USART_Type *)baseAddr, kUSART_RxLevelInterruptEnable);
    EnableIRQ(FLEXCOMM0_IRQn);
}

/********************************************************************/
/*
 * USART IRQ Handler
 *
 */
void FLEXCOMM0_IRQHandler(void)
{
    uint32_t baseAddr = g_uartBaseAddr[0];
    uart_rx_callback(USART_ReadByte((USART_Type *)baseAddr));
}

/*FUNCTION**********************************************************************
 *
 * Function Name : set_fpga_clock
 * Description   : fpga clock set function
 *
 *END**************************************************************************/
void set_fpga_clock(uint32_t clock)
{
}

/*FUNCTION**********************************************************************
 *
 * Function Name : init_spi
 * Description   : spi init function
 *
 *END**************************************************************************/

void init_spi(void)
{
    spi_master_config_t config;
    uint32_t baseAddr = g_spiBaseAddr[5];

    /*Set clock source for FLEXCOMM5 */
    CLOCK_AttachClk(kSFRO_to_FLEXCOMM5);

    /*
     * config.enableLoopback = false;
     * config.enableMaster = true;
     * config.polarity = kSPI_ClockPolarityActiveHigh;
     * config.phase = kSPI_ClockPhaseFirstEdge;
     * config.direction = kSPI_MsbFirst;
     * config.baudRate_Bps = 500000U;
     */
    SPI_MasterGetDefaultConfig(&config);
    config.polarity = s_spiUserConfig.polarity;
    config.phase = s_spiUserConfig.phase;
    config.baudRate_Bps = s_spiUserConfig.baudRate_Bps;
    config.sselNum = (spi_ssel_t)0;
    config.sselPol = (spi_spol_t)kSPI_SpolActiveAllLow;

    s_spiUserConfig.clock_Hz = CLOCK_GetFlexCommClkFreq(5U);

    SPI_MasterInit((SPI_Type *)baseAddr, &config, s_spiUserConfig.clock_Hz);
    SPI_MasterTransferCreateHandle((SPI_Type *)baseAddr, &s_spiHandle, NULL, NULL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : init_i2c
 * Description   : I2C init function
 *
 *END**************************************************************************/

void init_i2c(uint32_t instance)
{

}

/*FUNCTION**********************************************************************
 *
 * Function Name : deinit_i2c
 * Description   : I2C de-init function
 *
 *END**************************************************************************/
void deinit_i2c(uint32_t instance)
{

}

/*FUNCTION**********************************************************************
 *
 * Function Name : host_start_command_rx
 * Description   : receiving host start command process
 *
 *END**************************************************************************/
void host_start_command_rx(uint8_t *dest, uint32_t length)
{
    s_rxData = dest;
    s_bytesRx = 0;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : host_stop_command_rx
 * Description   : receiving host stop command process
 *
 *END**************************************************************************/
void host_stop_command_rx(void)
{
    s_rxData = 0;
    s_bytesRx = 0;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : get_bytes_received_from_host
 * Description   : receiving host get bytes command process
 *
 *END**************************************************************************/
uint32_t get_bytes_received_from_host(void)
{
    return s_bytesRx;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : write_bytes_to_host
 * Description   : sending host bytes command process
 *
 *END**************************************************************************/
void write_bytes_to_host(uint8_t *src, uint32_t length)
{
    uint32_t baseAddr = g_uartBaseAddr[0];

    USART_WriteBlocking((USART_Type *)baseAddr, src, length);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : configure_i2c_address
 * Description   : i2c config address process
 *
 *END**************************************************************************/
void configure_i2c_address(uint8_t address)
{
}

/*FUNCTION**********************************************************************
 *
 * Function Name : configure_i2c_speed
 * Description   : i2c config speed process
 *
 *END**************************************************************************/
void configure_i2c_speed(uint32_t speedkhz)
{
}

/*FUNCTION**********************************************************************
 *
 * Function Name : send_spi_data
 * Description   : spi send data proces
 *
 *END**************************************************************************/
void send_spi_data(uint8_t *src, uint32_t writeLength)
{
    spi_transfer_t send_data;
    uint32_t baseAddr = g_spiBaseAddr[5];

    send_data.txData = src;
    send_data.dataSize = writeLength;
    send_data.rxData = NULL;
    send_data.configFlags = kSPI_FrameAssert;
    SPI_MasterTransferBlocking((SPI_Type *)baseAddr, &send_data);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : receive_spi_data
 * Description   : spi receiving data process
 *
 *END**************************************************************************/
void receive_spi_data(uint8_t *dest, uint32_t readLength)
{
    spi_transfer_t receive_data;
    uint32_t baseAddr = g_spiBaseAddr[5];

    receive_data.rxData = dest;
    receive_data.dataSize = readLength;
    receive_data.txData = NULL;
    receive_data.configFlags = kSPI_FrameAssert;
    SPI_MasterTransferBlocking((SPI_Type *)baseAddr, &receive_data);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : configure_spi_speed
 * Description   : spi config speed process
 *
 *END**************************************************************************/
void configure_spi_speed(uint32_t speedkhz)
{
    s_spiUserConfig.baudRate_Bps = speedkhz * 1000;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : configure_spi_settings
 * Description   : spi config settings process
 *
 *END**************************************************************************/
void configure_spi_settings(spi_clock_polarity_t polarity, spi_clock_phase_t phase, spi_shift_direction_t direction)
{
    s_spiUserConfig.polarity = polarity;
    s_spiUserConfig.phase = phase;
    s_spiUserConfig.direction = direction;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : send_i2c_data
 * Description   : i2c sending data process
 *
 *END**************************************************************************/
status_t send_i2c_data(uint8_t *src, uint32_t writeLength)
{
    return kStatus_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : receive_i2c_data
 * Description   : i2c receiving data process
 *
 *END**************************************************************************/
status_t receive_i2c_data(uint8_t *dest, uint32_t readLength)
{
    return kStatus_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : uart_rx_callback
 * Description   : uart callback function
 *
 *END**************************************************************************/
void uart_rx_callback(uint8_t byte)
{
    if (s_rxData)
    {
        s_rxData[s_bytesRx++] = byte;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : getGpioBaseAddrFromAscii
 * Description   : GPIO get base address function
 *
 *END**************************************************************************/
GPIO_Type *getGpioBaseAddrFromAscii(uint8_t port)
{
    return GPIO;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : configure_gpio
 * Description   : GPIO config processing
 *
 *END**************************************************************************/
void configure_gpio(uint8_t port, uint8_t pinNum, uint8_t muxVal)
{
    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput,
        0,
    };
    GPIO_PortInit(GPIO, port);
    GPIO_PinInit(GPIO, port, pinNum, &led_config);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : set_gpio
 * Description   : GPIO set up function
 *
 *END**************************************************************************/
void set_gpio(uint8_t port, uint8_t pinNum, uint8_t level)
{
    GPIO_PinWrite(GPIO, port, pinNum, level);
}

#if __ICCARM__
/*FUNCTION**********************************************************************
 *
 * Function Name : __write
 * Description   : ICCARM write function implementation
 *
 *END**************************************************************************/
size_t __write(int handle, const unsigned char *buf, size_t size)
{
    return size;
}

#endif // __ICCARM__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

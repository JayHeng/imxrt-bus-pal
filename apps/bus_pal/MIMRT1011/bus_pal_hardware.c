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
#include "fsl_iomuxc.h"
#include "microseconds/microseconds.h"
#include "fsl_lpspi.h"
#include "fsl_lpuart.h"

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
static spi_user_config_t s_spiUserConfig = {.polarity     = kLPSPI_ClockPolarityActiveLow, /*!< Clock polarity */
                                            .phase        = kLPSPI_ClockPhaseSecondEdge,   /*!< Clock phase */
                                            .direction    = kLPSPI_MsbFirst,               /*!< MSB or LSB */
                                            .baudRate_Bps = 100000,                        /*!< Baud Rate for SPI in Hz */
                                            .clock_Hz     = 0 };

static lpspi_master_handle_t s_spiHandle;

//! @brief Variable for host data receiving
static uint8_t *s_rxData;
static uint32_t s_bytesRx;

const static uint32_t g_spiBaseAddr[]  = LPSPI_BASE_ADDRS;
const static uint32_t g_uartBaseAddr[] = LPUART_BASE_ADDRS;

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
    return CLOCK_GetFreq(kCLOCK_IpgClk);
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
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();

    // Enable pins for LPUART1.
    IOMUXC_SetPinMux(IOMUXC_GPIO_09_LPUART1_RXD, 0U); 
    IOMUXC_SetPinMux(IOMUXC_GPIO_10_LPUART1_TXD, 0U); 
    IOMUXC_SetPinConfig(IOMUXC_GPIO_09_LPUART1_RXD, 0x10A0U); 
    IOMUXC_SetPinConfig(IOMUXC_GPIO_10_LPUART1_TXD, 0x10A0U); 

    // Enable pins for LPSPI1
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_03_LPSPI1_SDI, 0U); 
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_04_LPSPI1_SDO, 0U); 
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_05_LPSPI1_PCS0, 0U); 
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_06_LPSPI1_SCK, 0U); 
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_03_LPSPI1_SDI, 0x10A0U); 
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_04_LPSPI1_SDO, 0x10A0U); 
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_05_LPSPI1_PCS0, 0x10A0U); 
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_06_LPSPI1_SCK, 0x10A0U); 

    microseconds_init();

    init_uarts();
    init_spi();
}

/*FUNCTION**********************************************************************
 *
 * Function Name : uart_get_clock
 * Description   : get lpuart clock
 *
 *END**************************************************************************/
uint32_t uart_get_clock(uint32_t instance)
{
    switch (instance)
    {
        case 1:
        {
            return BOARD_DebugConsoleSrcFreq();
        }
        default:
            return 0;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : init_uarts
 * Description   : Initialize UART ports
 *
 *END**************************************************************************/
static void init_uarts(void)
{
    lpuart_config_t config;
    uint32_t baseAddr = g_uartBaseAddr[1];

    /*
     * config.baudRate_Bps = 115200U;
     * config.parityMode = kLPUART_ParityDisabled;
     * config.stopBitCount = kLPUART_OneStopBit;
     * config.txFifoWatermark = 0;
     * config.rxFifoWatermark = 0;
     * config.enableTx = false;
     * config.enableRx = false;
     */
    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = 57600;
    config.enableTx     = true;
    config.enableRx     = true;

    LPUART_Init((LPUART_Type *)baseAddr, &config, uart_get_clock(1));
    LPUART_EnableInterrupts((LPUART_Type *)baseAddr, kLPUART_RxDataRegFullInterruptEnable);
    EnableIRQ(LPUART1_IRQn);
}

/********************************************************************/
/*
 * UART IRQ Handler
 *
 */
void LPUART1_IRQHandler(void)
{
    uint32_t baseAddr = g_uartBaseAddr[1];
    uart_rx_callback(LPUART_ReadByte((LPUART_Type *)baseAddr));
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

/* Select USB1 PLL PFD0 (720 MHz) as lpspi clock source */
#define APP_LPSPI_CLOCK_SOURCE_SELECT (1U)
/* Clock divider for master lpspi clock source */
#define APP_LPSPI_CLOCK_SOURCE_DIVIDER (7U)

#define LPSPI_MASTER_CLK_FREQ (CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk) / (APP_LPSPI_CLOCK_SOURCE_DIVIDER + 1U))

void init_spi(void)
{
    lpspi_master_config_t config;
    uint32_t baseAddr = g_spiBaseAddr[1];

    /*Set clock source for LPSPI*/
    CLOCK_SetMux(kCLOCK_LpspiMux, APP_LPSPI_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_LpspiDiv, APP_LPSPI_CLOCK_SOURCE_DIVIDER);

    LPSPI_MasterGetDefaultConfig(&config);

    config.cpol = s_spiUserConfig.polarity;
    config.cpha = s_spiUserConfig.phase;
    config.baudRate = s_spiUserConfig.baudRate_Bps;

    config.whichPcs = kLPSPI_Pcs0;
    config.pcsToSckDelayInNanoSec        = 1000000000U / (config.baudRate * 2U);
    config.lastSckToPcsDelayInNanoSec    = 1000000000U / (config.baudRate * 2U);
    config.betweenTransferDelayInNanoSec = 1000000000U / (config.baudRate * 2U);

    s_spiUserConfig.clock_Hz = LPSPI_MASTER_CLK_FREQ;

    LPSPI_MasterInit((LPSPI_Type *)baseAddr, &config, s_spiUserConfig.clock_Hz);
    LPSPI_MasterTransferCreateHandle((LPSPI_Type *)baseAddr, &s_spiHandle, NULL, NULL);
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
    uint32_t baseAddr = g_uartBaseAddr[1];

    LPUART_WriteBlocking((LPUART_Type *)baseAddr, src, length);
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
    lpspi_transfer_t send_data;
    uint32_t baseAddr = g_spiBaseAddr[1];

    send_data.txData = src;
    send_data.dataSize = writeLength;
    send_data.rxData = NULL;
    send_data.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;
    LPSPI_MasterTransferBlocking((LPSPI_Type *)baseAddr, &send_data);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : receive_spi_data
 * Description   : spi receiving data process
 *
 *END**************************************************************************/
void receive_spi_data(uint8_t *dest, uint32_t readLength)
{
    lpspi_transfer_t receive_data;
    uint32_t baseAddr = g_spiBaseAddr[1];

    receive_data.rxData = dest;
    receive_data.dataSize = readLength;
    receive_data.txData = NULL;
    receive_data.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;
    LPSPI_MasterTransferBlocking((LPSPI_Type *)baseAddr, &receive_data);
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
void configure_spi_settings(lpspi_clock_polarity_t polarity, lpspi_clock_phase_t phase, lpspi_shift_direction_t direction)
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
    if ((port >= '1') && (port <= '5'))
    {
        port = port - '1';
    }

    switch (port)
    {
        default:
        case 1:
            return GPIO1;
        case 2:
            return GPIO2;
        case 5:
            return GPIO5;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : configure_gpio
 * Description   : GPIO config processing
 *
 *END**************************************************************************/
void configure_gpio(uint8_t port, uint8_t pinNum, uint8_t muxVal)
{

}

/*FUNCTION**********************************************************************
 *
 * Function Name : set_gpio
 * Description   : GPIO set up function
 *
 *END**************************************************************************/
void set_gpio(uint8_t port, uint8_t pinNum, uint8_t level)
{
    GPIO_Type *realPort = getGpioBaseAddrFromAscii(port);

    realPort->GDIR |= 1 << pinNum;

    if (level)
    {
        realPort->DR |= 1 << pinNum;
    }
    else
    {
        realPort->DR &= ~(1 << pinNum);
    }
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

/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>
#include "target.h"

#ifdef USE_SPI

#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/rcc.h"
#include "nvic.h"
#ifdef AURIX

#ifndef SPI1_SCK_PIN
#define SPI1_NSS_PIN    PA4
#define SPI1_SCK_PIN    PA5
#define SPI1_MISO_PIN   PA6
#define SPI1_MOSI_PIN   PA7
#endif

#else

#ifndef SPI1_SCK_PIN
#define SPI1_NSS_PIN    PA4
#define SPI1_SCK_PIN    PA5
#define SPI1_MISO_PIN   PA6
#define SPI1_MOSI_PIN   PA7
#endif

#ifndef SPI2_SCK_PIN
#define SPI2_NSS_PIN    PB12
#define SPI2_SCK_PIN    PB13
#define SPI2_MISO_PIN   PB14
#define SPI2_MOSI_PIN   PB15
#endif

#ifndef SPI3_SCK_PIN
#define SPI3_NSS_PIN    PA15
#define SPI3_SCK_PIN    PB3
#define SPI3_MISO_PIN   PB4
#define SPI3_MOSI_PIN   PB5
#endif

#ifndef SPI1_NSS_PIN
#define SPI1_NSS_PIN NONE
#endif
#ifndef SPI2_NSS_PIN
#define SPI2_NSS_PIN NONE
#endif
#ifndef SPI3_NSS_PIN
#define SPI3_NSS_PIN NONE
#endif

#if defined(STM32F4)
#if defined(USE_SPI_DEVICE_1)
static const uint32_t spiDivisorMapFast[] = {
    SPI_BaudRatePrescaler_256,    // SPI_CLOCK_INITIALIZATON      328.125 KBits/s
    SPI_BaudRatePrescaler_128,    // SPI_CLOCK_SLOW               656.25 KBits/s
    SPI_BaudRatePrescaler_8,      // SPI_CLOCK_STANDARD           10.5 MBits/s
    SPI_BaudRatePrescaler_4,      // SPI_CLOCK_FAST               21.0 MBits/s
    SPI_BaudRatePrescaler_2       // SPI_CLOCK_ULTRAFAST          42.0 MBits/s
};
#endif

#if defined(USE_SPI_DEVICE_2) || defined(USE_SPI_DEVICE_3)
static const uint32_t spiDivisorMapSlow[] = {
    SPI_BaudRatePrescaler_256,    // SPI_CLOCK_INITIALIZATON      164.062 KBits/s
    SPI_BaudRatePrescaler_64,     // SPI_CLOCK_SLOW               656.25 KBits/s
    SPI_BaudRatePrescaler_4,      // SPI_CLOCK_STANDARD           10.5 MBits/s
    SPI_BaudRatePrescaler_2,      // SPI_CLOCK_FAST               21.0 MBits/s
    SPI_BaudRatePrescaler_2       // SPI_CLOCK_ULTRAFAST          21.0 MBits/s
};
#endif
#endif
#endif

#if defined(AURIX)
static const uint32_t spiDivisorMapFast[] = {
	  100000,    	// SPI_CLOCK_INITIALIZATON
	  500000,     	// SPI_CLOCK_SLOW
	 1000000,      	// SPI_CLOCK_STANDARD
	 5000000,      	// SPI_CLOCK_FAST
	10000000       	// SPI_CLOCK_ULTRAFAST
};
static const uint32_t spiDivisorMapSlow[] = {
	   50000,    	// SPI_CLOCK_INITIALIZATON
	  100000,     	// SPI_CLOCK_SLOW
	  500000,      	// SPI_CLOCK_STANDARD
	 1000000,      	// SPI_CLOCK_FAST
	 5000000       	// SPI_CLOCK_ULTRAFAST
};

IfxQspi_SpiMaster_Channel SpiMasterChannels[4];
#else
#error "Invalid CPU"
#endif

static spiDevice_t spiHardwareMap[] = {
#ifdef AURIX
#ifdef USE_SPI_DEVICE_1
	{
		.dev = &SpiMasterChannels[0],
		.qspi = SPI1,
		.sck = IO_TAG(SPI1_SCK_PIN),
		.miso = IO_TAG(SPI1_MISO_PIN),
		.mosi = IO_TAG(SPI1_MOSI_PIN),
		.divisorMap = spiDivisorMapFast,
		.leadingEdge = false,
		.txPrio = IFX_INTPRIO_QSPI_SPI1_TX,
		.rxPrio = IFX_INTPRIO_QSPI_SPI1_RX,
		.rxDmaChannelId = IfxDma_ChannelId_0,
		.txDmaChannelId = IfxDma_ChannelId_1
	},
#endif
#ifdef USE_SPI_DEVICE_2
	{
		.dev = &SpiMasterChannels[1],
		.qspi = SPI2,
		.sck = IO_TAG(SPI2_SCK_PIN),
		.miso = IO_TAG(SPI2_MISO_PIN),
		.mosi = IO_TAG(SPI2_MOSI_PIN),
		.divisorMap = spiDivisorMapFast,
		.leadingEdge = false,
		.txPrio = IFX_INTPRIO_QSPI_SPI2_TX,
		.rxPrio = IFX_INTPRIO_QSPI_SPI2_RX,
		.rxDmaChannelId = IfxDma_ChannelId_2,
		.txDmaChannelId = IfxDma_ChannelId_3
	},
#endif
#ifdef USE_SPI_DEVICE_3
	{
		.dev = &SpiMasterChannels[2],
		.qspi = SPI3,
		.sck = IO_TAG(SPI3_SCK_PIN),
		.miso = IO_TAG(SPI3_MISO_PIN),
		.mosi = IO_TAG(SPI3_MOSI_PIN),
		.divisorMap = spiDivisorMapFast,
		.leadingEdge = false,
		.txPrio = IFX_INTPRIO_QSPI_SPI3_TX,
		.rxPrio = IFX_INTPRIO_QSPI_SPI3_RX,
		.rxDmaChannelId = IfxDma_ChannelId_4,
		.txDmaChannelId = IfxDma_ChannelId_5
	},
#endif
#else
#ifdef USE_SPI_DEVICE_1
    { .dev = SPI1, .nss = IO_TAG(SPI1_NSS_PIN), .sck = IO_TAG(SPI1_SCK_PIN), .miso = IO_TAG(SPI1_MISO_PIN), .mosi = IO_TAG(SPI1_MOSI_PIN), .rcc = RCC_APB2(SPI1), .af = GPIO_AF_SPI1, .divisorMap = spiDivisorMapFast },
#else
    { .dev = NULL },    // No SPI1
#endif
#ifdef USE_SPI_DEVICE_2
    { .dev = SPI2, .nss = IO_TAG(SPI2_NSS_PIN), .sck = IO_TAG(SPI2_SCK_PIN), .miso = IO_TAG(SPI2_MISO_PIN), .mosi = IO_TAG(SPI2_MOSI_PIN), .rcc = RCC_APB1(SPI2), .af = GPIO_AF_SPI2, .divisorMap = spiDivisorMapSlow },
#else
    { .dev = NULL },    // No SPI2
#endif
#ifdef USE_SPI_DEVICE_3
    { .dev = SPI3, .nss = IO_TAG(SPI3_NSS_PIN), .sck = IO_TAG(SPI3_SCK_PIN), .miso = IO_TAG(SPI3_MISO_PIN), .mosi = IO_TAG(SPI3_MOSI_PIN), .rcc = RCC_APB1(SPI3), .af = GPIO_AF_SPI3, .divisorMap = spiDivisorMapSlow },
#else
    { .dev = NULL },    // No SPI3
#endif
    { .dev = NULL },    // No SPI4
#endif
};



SPIDevice spiDeviceByInstance(SPI_TypeDef *instance)
{
#ifndef AURIX
    if (instance == SPI1)
        return SPIDEV_1;

    if (instance == SPI2)
        return SPIDEV_2;

    if (instance == SPI3)
        return SPIDEV_3;
#else

    IfxQspi_SpiMaster* handle = (IfxQspi_SpiMaster*)instance->base.driver;

    if (handle->qspi == SPI1)
    	return SPIDEV_1;

    if (handle->qspi == SPI2)
        return SPIDEV_2;

    if (handle->qspi == SPI3)
        return SPIDEV_3;

    if (handle->qspi == SPI4)
        return SPIDEV_4;

#endif
    return SPIINVALID;
}

bool spiInitDevice(SPIDevice device, bool leadingEdge)
{
    spiDevice_t *spi = &(spiHardwareMap[device]);

    if (!spi->dev) {
        return false;
    }

    if (spi->initDone) {
        return true;
    }
#ifndef AURIX
    // Enable SPI clock
    RCC_ClockCmd(spi->rcc, ENABLE);
    RCC_ResetCmd(spi->rcc, DISABLE);
#endif
    IOInit(IOGetByTag(spi->sck),  OWNER_SPI, RESOURCE_SPI_SCK,  device + 1);
    IOInit(IOGetByTag(spi->miso), OWNER_SPI, RESOURCE_SPI_MISO, device + 1);
    IOInit(IOGetByTag(spi->mosi), OWNER_SPI, RESOURCE_SPI_MOSI, device + 1);
#ifdef AURIX

    IfxQspi_SpiMaster_Config config;
    IfxQspi_SpiMaster_initModuleConfig(&config, spi->qspi);

    config.base.mode			= SpiIf_Mode_master;
    config.base.maximumBaudrate	= 10000000;
    config.base.txPriority		= spi->txPrio;
    config.base.rxPriority		= spi->rxPrio;
    config.dma.txDmaChannelId	= spi->txDmaChannelId;
    config.dma.rxDmaChannelId	= spi->rxDmaChannelId;
    config.dma.useDma			= 1;

    GPIO_TypeDef* port;
    uint8_t pinIndex;

    IfxQspi_SpiMaster_Pins pins =
    {
    	NULL, IfxPort_OutputMode_pushPull,		// SCLK
		NULL, IfxPort_OutputMode_pushPull,
		NULL, IfxPort_InputMode_pullDown,
		IfxPort_PadDriver_cmosAutomotiveSpeed3
    };

    for (int i=0; i<IFXQSPI_PINMAP_NUM_MODULES; i++)
    {
    	port = IO_GPIO(IOGetByTag(spi->sck));
    	pinIndex = IO_Pin(IOGetByTag(spi->sck));

    	for (int j=0; j<IFXQSPI_PINMAP_SCLK_OUT_NUM_ITEMS; j++)
    		if (IfxQspi_Sclk_Out_pinTable[i][j] != NULL)
				if (IfxQspi_Sclk_Out_pinTable[i][j]->pin.port == port &&
					IfxQspi_Sclk_Out_pinTable[i][j]->pin.pinIndex == pinIndex &&
					IfxQspi_Sclk_Out_pinTable[i][j]->module == (Ifx_QSPI*)spi->qspi)
					pins.sclk = IfxQspi_Sclk_Out_pinTable[i][j];

    	port = IO_GPIO(IOGetByTag(spi->mosi));
    	pinIndex = IO_Pin(IOGetByTag(spi->mosi));

    	for (int j=0; j<IFXQSPI_PINMAP_MTSR_OUT_NUM_ITEMS; j++)
    		if (IfxQspi_Mtsr_Out_pinTable[i][j] != NULL)
				if (IfxQspi_Mtsr_Out_pinTable[i][j]->pin.port == port &&
					IfxQspi_Mtsr_Out_pinTable[i][j]->pin.pinIndex == pinIndex &&
					IfxQspi_Mtsr_Out_pinTable[i][j]->module == (Ifx_QSPI*)spi->qspi)
					pins.mtsr = IfxQspi_Mtsr_Out_pinTable[i][j];

    	port = IO_GPIO(IOGetByTag(spi->miso));
    	pinIndex = IO_Pin(IOGetByTag(spi->miso));

    	for (int j=0; j<IFXQSPI_PINMAP_MRST_IN_NUM_ITEMS; j++)
    		if (IfxQspi_Mrst_In_pinTable[i][j] != NULL)
				if (IfxQspi_Mrst_In_pinTable[i][j]->pin.port == port &&
					IfxQspi_Mrst_In_pinTable[i][j]->pin.pinIndex == pinIndex &&
					IfxQspi_Mrst_In_pinTable[i][j]->module == (Ifx_QSPI*)spi->qspi)
					pins.mrst = IfxQspi_Mrst_In_pinTable[i][j];
    }
//    if(device == SPIDEV_1)
//    {
//    	pins.sclk = IfxQspi_Sclk_Out_pinTable[2][4];
//
//    	pins.mtsr = IfxQspi_Mtsr_Out_pinTable[2][3];
//
//    	pins.mrst = IfxQspi_Mrst_In_pinTable[2][1];
//    }
    config.pins = &pins;

    IfxQspi_SpiMaster_initModule(&spi->master, &config);

#else
#if defined(STM32F4)
    if (leadingEdge) {
        IOConfigGPIOAF(IOGetByTag(spi->sck),  SPI_IO_AF_SCK_CFG, spi->af);
        IOConfigGPIOAF(IOGetByTag(spi->miso), SPI_IO_AF_MISO_CFG, spi->af);
        IOConfigGPIOAF(IOGetByTag(spi->mosi), SPI_IO_AF_CFG, spi->af);
    } else {
        IOConfigGPIOAF(IOGetByTag(spi->sck),  SPI_IO_AF_CFG, spi->af);
        IOConfigGPIOAF(IOGetByTag(spi->miso), SPI_IO_AF_CFG, spi->af);
        IOConfigGPIOAF(IOGetByTag(spi->mosi), SPI_IO_AF_CFG, spi->af);
    }

    if (spi->nss) {
        IOConfigGPIOAF(IOGetByTag(spi->nss), SPI_IO_CS_CFG, spi->af);
    }
#endif

    // Init SPI hardware
    SPI_I2S_DeInit(spi->dev);

    SPI_InitTypeDef spiInit;
    spiInit.SPI_Mode = SPI_Mode_Master;
    spiInit.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spiInit.SPI_DataSize = SPI_DataSize_8b;
    spiInit.SPI_NSS = SPI_NSS_Soft;
    spiInit.SPI_FirstBit = SPI_FirstBit_MSB;
    spiInit.SPI_CRCPolynomial = 7;
    spiInit.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;

    if (leadingEdge) {
        // SPI_MODE0
        spiInit.SPI_CPOL = SPI_CPOL_Low;
        spiInit.SPI_CPHA = SPI_CPHA_1Edge;
    } else {
        // SPI_MODE3
        spiInit.SPI_CPOL = SPI_CPOL_High;
        spiInit.SPI_CPHA = SPI_CPHA_2Edge;
    }

    SPI_Init(spi->dev, &spiInit);
    SPI_Cmd(spi->dev, ENABLE);

    if (spi->nss) {
        // Drive NSS high to disable connected SPI device.
        IOHi(IOGetByTag(spi->nss));
    }
#endif
    spi->initDone = true;
    return true;
}

uint32_t spiTimeoutUserCallback(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (device == SPIINVALID) {
        return -1;
    }
    spiHardwareMap[device].errorCount++;
    return spiHardwareMap[device].errorCount;
}

// return uint8_t value or -1 when failure
uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t data)
{
#ifdef AURIX

	uint8_t byte = -1;

	spiTransfer(instance, &byte, &data, 1);

	return byte;

#else
    uint16_t spiTimeout = 1000;

    while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_TXE) == RESET)
        if ((spiTimeout--) == 0)
            return spiTimeoutUserCallback(instance);

    SPI_I2S_SendData(instance, data);

    spiTimeout = 1000;
    while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_RXNE) == RESET)
        if ((spiTimeout--) == 0)
            return spiTimeoutUserCallback(instance);

    return ((uint8_t)SPI_I2S_ReceiveData(instance));
#endif
}

/**
 * Return true if the bus is currently in the middle of a transmission.
 */
bool spiIsBusBusy(SPI_TypeDef *instance)
{
#ifdef AURIX

	return (IfxQspi_SpiMaster_getStatus(instance) == SpiIf_Status_busy);
#else
    return SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_TXE) == RESET || SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_BSY) == SET;
#endif
}

bool spiTransfer(SPI_TypeDef *instance, uint8_t *out, const uint8_t *in, int len)
{
    uint16_t spiTimeout = 1000;
#ifndef AURIX
    instance->DR;
    while (len--) {
        uint8_t b = in ? *(in++) : 0xFF;
        while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_TXE) == RESET) {
            if ((spiTimeout--) == 0)
                return spiTimeoutUserCallback(instance);
        }
        SPI_I2S_SendData(instance, b);
        spiTimeout = 1000;
        while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_RXNE) == RESET) {
            if ((spiTimeout--) == 0)
                return spiTimeoutUserCallback(instance);
        }
        b = SPI_I2S_ReceiveData(instance);
        if (out)
            *(out++) = b;
    }
#else

    spiTimeout = 0xffff;

    while(IfxQspi_SpiMaster_getStatus(instance) == SpiIf_Status_busy)
    	if ((spiTimeout--) == 0)
    		return spiTimeoutUserCallback(instance);

    IfxQspi_SpiMaster_exchange(instance, (const void*)in, (void*)out, len);

    spiTimeout = 0xffff;

    while(IfxQspi_SpiMaster_getStatus(instance) == SpiIf_Status_busy)
    	if ((spiTimeout--) == 0)
    		return spiTimeoutUserCallback(instance);

#endif
    return true;
}

void spiSetSpeed(SPI_TypeDef *instance, SPIClockSpeed_e speed)
{
#define BR_CLEAR_MASK 0xFFC7
    SPIDevice device = spiDeviceByInstance(instance);
    if (device == SPIINVALID) {
        return;
    }
#ifndef AURIX
    SPI_Cmd(instance, DISABLE);

    uint16_t tempRegister = instance->CR1;
    tempRegister &= BR_CLEAR_MASK;
    tempRegister |= spiHardwareMap[device].divisorMap[speed];
    instance->CR1 = tempRegister;

    SPI_Cmd(instance, ENABLE);
#else
    IfxQspi_SpiMaster_setChannelBaudrate(instance, spiHardwareMap[device].divisorMap[speed]);
#endif
}

uint16_t spiGetErrorCounter(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (device == SPIINVALID) {
        return 0;
    }
    return spiHardwareMap[device].errorCount;
}

void spiResetErrorCounter(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (device != SPIINVALID) {
        spiHardwareMap[device].errorCount = 0;
    }
}

SPI_TypeDef * spiInstanceByDevice(SPIDevice device)
{
    return spiHardwareMap[device].dev;
}

#ifdef AURIX

IfxQspi_SpiMaster * spiMasterByDevice(SPIDevice device)
{
	return &spiHardwareMap[device].master;
}

#ifdef USE_SPI_DEVICE_1

IFX_INTERRUPT(qspiSPI1TxISR, 0, IFX_INTPRIO_QSPI_SPI1_TX);
IFX_INTERRUPT(qspiSPI1RxISR, 0, IFX_INTPRIO_QSPI_SPI1_RX);

void qspiSPI1TxISR(void)
{
	IfxQspi_SpiMaster_isrDmaTransmit(&spiHardwareMap[SPIDEV_1].master);
}
void qspiSPI1RxISR(void)
{
	IfxQspi_SpiMaster_isrDmaReceive(&spiHardwareMap[SPIDEV_1].master);
}

#endif

#ifdef USE_SPI_DEVICE_2

IFX_INTERRUPT(qspiSPI2TxISR, 0, IFX_INTPRIO_QSPI_SPI2_TX);
IFX_INTERRUPT(qspiSPI2RxISR, 0, IFX_INTPRIO_QSPI_SPI2_RX);

void qspiSPI2TxISR(void)
{
	IfxQspi_SpiMaster_isrDmaTransmit(&spiHardwareMap[SPIDEV_2].master);
}
void qspiSPI2RxISR(void)
{
	IfxQspi_SpiMaster_isrDmaReceive(&spiHardwareMap[SPIDEV_2].master);
}

#endif

#ifdef USE_SPI_DEVICE_3

IFX_INTERRUPT(qspiSPI3TxISR, 0, IFX_INTPRIO_QSPI_SPI3_TX);
IFX_INTERRUPT(qspiSPI3RxISR, 0, IFX_INTPRIO_QSPI_SPI3_RX);

void qspiSPI3TxISR(void)
{
	IfxQspi_SpiMaster_isrDmaTransmit(&spiHardwareMap[SPIDEV_3].master);
}
void qspiSPI3RxISR(void)
{
	IfxQspi_SpiMaster_isrDmaReceive(&spiHardwareMap[SPIDEV_3].master);
}

#endif

#ifdef USE_SPI_DEVICE_4

IFX_INTERRUPT(qspiSPI4TxISR, 0, IFX_INTPRIO_QSPI_SPI4_TX);
IFX_INTERRUPT(qspiSPI4RxISR, 0, IFX_INTPRIO_QSPI_SPI4_RX);

void qspiSPI4TxISR(void)
{
	IfxQspi_SpiMaster_isrDmaTransmit(&spiHardwareMap[SPIDEV_4].master);
}
void qspiSPI4RxISR(void)
{
	IfxQspi_SpiMaster_isrDmaReceive(&spiHardwareMap[SPIDEV_4].master);
}

#endif
#endif // USE_SPI
#endif

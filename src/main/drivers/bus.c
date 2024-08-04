/*
 * This file is part of INAV.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <string.h>

#include "platform.h"
#include "build/debug.h"

#include "common/memory.h"

#include "drivers/bus.h"
#include "drivers/io.h"
#ifdef AURIX
#include "drivers/io_impl.h"
#include "sensors/gyro.h"
#endif
#define BUSDEV_MAX_DEVICES 16

#ifdef USE_SPI
static void busDevPreInit_SPI(const busDeviceDescriptor_t * descriptor)
{
    // Pre-initialize SPI device chip-select line to input with weak pull-up
    IO_t io = IOGetByTag(descriptor->busdev.spi.csnPin);
    if (io) {
        IOInit(io, OWNER_SPI_PREINIT, RESOURCE_SPI_CS, 0);
        IOConfigGPIO(io, IOCFG_IPU);
    }
}
#endif

static void busDevPreInit(const busDeviceDescriptor_t * descriptor)
{
    switch (descriptor->busType) {
        default:
        case BUSTYPE_NONE:
            break;

        case BUSTYPE_I2C:
            break;

        case BUSTYPE_SPI:
#ifdef USE_SPI
            busDevPreInit_SPI(descriptor);
#endif
            break;
    }
}

void busInit(void)
{
#if !defined(SITL_BUILD)
    /* Pre-initialize bus devices */
    for (const busDeviceDescriptor_t * descriptor = __busdev_registry_start; (descriptor) < __busdev_registry_end; descriptor++) {
        busDevPreInit(descriptor);
    }
#endif
}

#ifdef USE_I2C
static bool busDevInit_I2C(busDevice_t * dev, const busDeviceDescriptor_t * descriptor)
{
    dev->busType = descriptor->busType;
    dev->irqPin = IOGetByTag(descriptor->irqPin);
    dev->busdev.i2c.i2cBus = descriptor->busdev.i2c.i2cBus;
    dev->busdev.i2c.address = descriptor->busdev.i2c.address;
    return true;
}
#endif

#ifdef USE_SPI
static bool busDevInit_SPI(busDevice_t * dev, const busDeviceDescriptor_t * descriptor, resourceOwner_e owner)
{
    dev->busType = descriptor->busType;
    dev->irqPin = IOGetByTag(descriptor->irqPin);
    dev->busdev.spi.spiBus = descriptor->busdev.spi.spiBus;
    if(descriptor->busdev.spi.spiBus == BMI160_SPI_BUS)
    {
    	dev->busdev.spi.csnPin = IOGetByTag(IO_TAG(BMI160_CS_PIN));
//    	dev->busdev.spi.csnPin = IfxQspi_Slso_Out_pinTable[2][4][0];
    }
    if(descriptor->busdev.spi.spiBus == SDCARD_SPI_BUS)
    {
    	dev->busdev.spi.csnPin = IOGetByTag(IO_TAG(SDCARD_CS_PIN));
    	//dev->busdev.spi.csnPin = IfxQspi_Slso_Out_pinTable[3][2][0];
    }
    if (dev->busdev.spi.csnPin && spiBusInitHost(dev)) {
        // Init CSN pin
        IOInit(dev->busdev.spi.csnPin, owner, RESOURCE_SPI_CS, 0);
#ifdef AURIX
        if(descriptor->busdev.spi.spiBus == BMI160_SPI_BUS)
        {
        	IfxQspi_SpiMaster_ChannelConfig chConfig;
        	IfxQspi_SpiMaster* master = spiMasterByDevice(BMI160_SPI_BUS);
        	IfxQspi_SpiMaster_initChannelConfig(&chConfig, master);

        	chConfig.base.baudrate = 1000000;
        	chConfig.base.mode.shiftClock = SpiIf_ShiftClock_shiftTransmitDataOnTrailingEdge;

        	IfxQspi_SpiMaster_Output slsOutput =
        	{
        			NULL,
					IfxPort_OutputMode_pushPull,
					IfxPort_PadDriver_cmosAutomotiveSpeed1
        	};

        	GPIO_TypeDef* port = IO_GPIO(dev->busdev.spi.csnPin);
        	uint8_t pinIndex = IO_Pin(dev->busdev.spi.csnPin);

        	for (int i=0; i<IFXQSPI_PINMAP_NUM_MODULES; i++)
        		for (int j=0; j<IFXQSPI_PINMAP_NUM_SLAVESELECTS; j++)
        			for (int k=0; k<IFXQSPI_PINMAP_SLSO_OUT_NUM_ITEMS; k++)
        				if (IfxQspi_Slso_Out_pinTable[i][j][k] != NULL)
        					if (IfxQspi_Slso_Out_pinTable[i][j][k]->pin.port == port &&
        							IfxQspi_Slso_Out_pinTable[i][j][k]->pin.pinIndex == pinIndex &&
									IfxQspi_Slso_Out_pinTable[i][j][k]->module == (Ifx_QSPI*)master->qspi)
        						slsOutput.pin = IfxQspi_Slso_Out_pinTable[i][j][k];

//        	slsOutput.pin = IfxQspi_Slso_Out_pinTable[2][4][0];
        	chConfig.sls.output = (IfxQspi_SpiMaster_Output)slsOutput;

        	IfxQspi_SpiMaster_Channel* spiChannel = spiInstanceByDevice(dev->busdev.spi.spiBus);
        	IfxQspi_SpiMaster_initChannel(spiChannel, &chConfig);
        }
        if(descriptor->busdev.spi.spiBus == SDCARD_SPI_BUS)
        {
        	IfxQspi_SpiMaster_ChannelConfig chConfig;
        	IfxQspi_SpiMaster* master = spiMasterByDevice(SDCARD_SPI_BUS);
        	IfxQspi_SpiMaster_initChannelConfig(&chConfig, master);

        	chConfig.base.baudrate = 1000000;
        	chConfig.base.mode.shiftClock = SpiIf_ShiftClock_shiftTransmitDataOnTrailingEdge;
        	chConfig.base.mode.autoCS = false;

        	IfxQspi_SpiMaster_Output slsOutput =
        	{
        			NULL,
					IfxPort_OutputMode_pushPull,
					IfxPort_PadDriver_cmosAutomotiveSpeed1
        	};

        	GPIO_TypeDef* port = IO_GPIO(IOGetByTag(IO_TAG(SDCARD_CS_PIN)));
        	uint8_t pinIndex = IO_Pin(IOGetByTag(IO_TAG(SDCARD_CS_PIN)));

        	for (int i=0; i<IFXQSPI_PINMAP_NUM_MODULES; i++)
        		for (int j=0; j<IFXQSPI_PINMAP_NUM_SLAVESELECTS; j++)
        			for (int k=0; k<IFXQSPI_PINMAP_SLSO_OUT_NUM_ITEMS; k++)
        				if (IfxQspi_Slso_Out_pinTable[i][j][k] != NULL)
        					if (IfxQspi_Slso_Out_pinTable[i][j][k]->pin.port == port &&
        							IfxQspi_Slso_Out_pinTable[i][j][k]->pin.pinIndex == pinIndex &&
									IfxQspi_Slso_Out_pinTable[i][j][k]->module == master->qspi)
        						slsOutput.pin = IfxQspi_Slso_Out_pinTable[i][j][k];

//        	slsOutput.pin = IfxQspi_Slso_Out_pinTable[3][2][0];
        	chConfig.sls.output = (IfxQspi_SpiMaster_Output)slsOutput;

        	IfxQspi_SpiMaster_Channel* spiChannel = spiInstanceByDevice(dev->busdev.spi.spiBus);
        	IfxQspi_SpiMaster_initChannel(spiChannel, &chConfig);
        	spiChannel->activateSlso = NULL;
        	spiChannel->deactivateSlso = NULL;


            IOHi(dev->busdev.spi.csnPin);
        }
#else
    dev->busdev.spi.csnPin = IOGetByTag(descriptor->busdev.spi.csnPin);
#endif

#ifndef AURIX
        IOConfigGPIO(dev->busdev.spi.csnPin, SPI_IO_CS_CFG);
        IOHi(dev->busdev.spi.csnPin);
#endif

        return true;
    }

    return false;
}
#endif

void busDeviceDeInit(busDevice_t * dev)
{
    busDevPreInit(dev->descriptorPtr);
    dev->descriptorPtr = NULL;
    dev->busType = BUSTYPE_NONE;
}

busDevice_t * busDeviceInit(busType_e bus, devHardwareType_e hw, uint8_t tag, resourceOwner_e owner)
{
    UNUSED(owner);
#if defined(SITL_BUILD)
    UNUSED(bus);
    UNUSED(hw);
    UNUSED(tag);
#else       

    for (const busDeviceDescriptor_t * descriptor = __busdev_registry_start; (descriptor) < __busdev_registry_end; descriptor++) {
        if (hw == descriptor->devHwType && (bus == descriptor->busType || bus == BUSTYPE_ANY) && (tag == descriptor->tag)) {
            // We have a candidate - initialize device context memory
            busDevice_t * dev = descriptor->devicePtr;
            if (dev) {
                memset(dev, 0, sizeof(busDevice_t));

                dev->descriptorPtr = descriptor;
                dev->busType = descriptor->busType;
                dev->flags = descriptor->flags;
                dev->param = descriptor->param;

                switch (descriptor->busType) {
                    default:
                    case BUSTYPE_NONE:
                        return NULL;

                    case BUSTYPE_I2C:
#ifdef USE_I2C
                        if (!busDevInit_I2C(dev, descriptor)) {
                            busDeviceDeInit(dev);
                            return NULL;
                        }
                        break;
#else
                        busDeviceDeInit(dev);
                        return NULL;
#endif

                    case BUSTYPE_SPI:
#ifdef USE_SPI
                        if (!busDevInit_SPI(dev, descriptor, owner)) {
                            busDeviceDeInit(dev);
                            return NULL;
                        }
                        break;
#else
                        busDeviceDeInit(dev);
                        return NULL;
#endif
                }

                return dev;
            }
            else {
                return NULL;
            }
        }
    }
#endif
    return NULL;
}

busDevice_t * busDeviceOpen(busType_e bus, devHardwareType_e hw, uint8_t tag)
{
#if defined(SITL_BUILD)
    UNUSED(bus);
    UNUSED(hw);
    UNUSED(tag);
#else
    for (const busDeviceDescriptor_t * descriptor = __busdev_registry_start; (descriptor) < __busdev_registry_end; descriptor++) {
        if (hw == descriptor->devHwType && (bus == descriptor->busType || bus == BUSTYPE_ANY) && (tag == descriptor->tag)) {
            // Found a hardware descriptor. Now check if device context is valid
            busDevice_t * dev = descriptor->devicePtr;
            if (dev->busType == descriptor->busType && dev->descriptorPtr == descriptor) {
                return dev;
            }
        }
    }
#endif
    return NULL;
}

void busSetSpeed(const busDevice_t * dev, busSpeed_e speed)
{
    UNUSED(speed);

    switch (dev->busType) {
        default:
        case BUSTYPE_NONE:
            // Not available
            break;
        case BUSTYPE_SPI:
#ifdef USE_SPI
            spiBusSetSpeed(dev, speed);
#endif
            break;
        case BUSTYPE_I2C:
            // Do nothing for I2C
            break;
    }
}

uint32_t busDeviceReadScratchpad(const busDevice_t * dev)
{
    uint32_t * mem = busDeviceGetScratchpadMemory(dev);
    return (mem != NULL) ? mem[0] : 0;
}

void busDeviceWriteScratchpad(busDevice_t * dev, uint32_t value)
{
    uint32_t * mem = busDeviceGetScratchpadMemory(dev);

    if (mem != NULL) {
        mem[0] = value;
    }
}

void * busDeviceGetScratchpadMemory(const busDevice_t * dev)
{
    if (dev->scratchpad == NULL) {
        ((busDevice_t *)dev)->scratchpad = memAllocate(BUS_SCRATCHPAD_MEMORY_SIZE, OWNER_SYSTEM);
    }

    return (void *)dev->scratchpad;
}

bool busTransfer(const busDevice_t * dev, uint8_t * rxBuf, const uint8_t * txBuf, int length)
{
#ifdef USE_SPI
    return spiBusTransfer(dev, rxBuf, txBuf, length);
#else
    UNUSED(dev);
    UNUSED(rxBuf);
    UNUSED(txBuf);
    UNUSED(length);
#endif

    return false;
}

bool busTransferMultiple(const busDevice_t * dev, busTransferDescriptor_t * dsc, int count)
{
#ifdef USE_SPI
    // busTransfer function is only supported on SPI bus
    if (dev->busType == BUSTYPE_SPI) {
        return spiBusTransferMultiple(dev, dsc, count);
    }
#else
    UNUSED(dev);
    UNUSED(dsc);
    UNUSED(count);
#endif

    return false;
}

bool busWriteBuf(const busDevice_t * dev, uint8_t reg, const uint8_t * data, uint8_t length)
{
#if !defined(USE_SPI) && !defined(USE_I2C)
    UNUSED(reg);
    UNUSED(data);
    UNUSED(length);
#endif
    
    switch (dev->busType) {
        case BUSTYPE_SPI:
#ifdef USE_SPI
            if (dev->flags & DEVFLAGS_USE_RAW_REGISTERS) {
                return spiBusWriteBuffer(dev, reg, data, length);
            }
            else {
                return spiBusWriteBuffer(dev, reg | 0x80, data, length);
            }
#else
            return false;
#endif

        case BUSTYPE_I2C:
#ifdef USE_I2C
            return i2cBusWriteBuffer(dev, reg, data, length);
#else
            return false;
#endif

        default:
            return false;
    }
}

bool busWrite(const busDevice_t * dev, uint8_t reg, uint8_t data)
{
#if !defined(USE_SPI) && !defined(USE_I2C)
    UNUSED(reg);
    UNUSED(data);
#endif    
    
    switch (dev->busType) {
        case BUSTYPE_SPI:
#ifdef USE_SPI
            if (dev->flags & DEVFLAGS_USE_RAW_REGISTERS) {
                return spiBusWriteRegister(dev, reg, data);
            }
            else {
                return spiBusWriteRegister(dev, reg & 0x7F, data);
            }
#else
            return false;
#endif

        case BUSTYPE_I2C:
#ifdef USE_I2C
            return i2cBusWriteRegister(dev, reg, data);
#else
            return false;
#endif

        default:
            return false;
    }
}

bool busReadBuf(const busDevice_t * dev, uint8_t reg, uint8_t * data, uint8_t length)
{
#if !defined(USE_SPI) && !defined(USE_I2C)
    UNUSED(reg);
    UNUSED(data);
    UNUSED(length);
#endif
    switch (dev->busType) {
        case BUSTYPE_SPI:
#ifdef USE_SPI
            if (dev->flags & DEVFLAGS_USE_RAW_REGISTERS) {
                return spiBusReadBuffer(dev, reg, data, length);
            }
            else {
                return spiBusReadBuffer(dev, reg | 0x80, data, length);
            }
#else
            return false;
#endif

        case BUSTYPE_I2C:
#ifdef USE_I2C
            return i2cBusReadBuffer(dev, reg, data, length);
#else
            return false;
#endif

        default:
            return false;
    }
}

bool busRead(const busDevice_t * dev, uint8_t reg, uint8_t * data)
{
#if !defined(USE_SPI) && !defined(USE_I2C)
    UNUSED(reg);
    UNUSED(data);
#endif  

    switch (dev->busType) {
        case BUSTYPE_SPI:
#ifdef USE_SPI
            if (dev->flags & DEVFLAGS_USE_RAW_REGISTERS) {
                return spiBusReadRegister(dev, reg, data);
            }
            else {
                return spiBusReadRegister(dev, reg | 0x80, data);
            }
#else
            return false;
#endif

        case BUSTYPE_I2C:
#ifdef USE_I2C
            return i2cBusReadRegister(dev, reg, data);
#else
            return false;
#endif
        default:
            return false;
    }
}

void busSelectDevice(const busDevice_t * dev)
{
#ifdef USE_SPI
    if (dev->busType == BUSTYPE_SPI && (dev->flags & DEVFLAGS_USE_MANUAL_DEVICE_SELECT)) {
        spiBusSelectDevice(dev);
    }
#else
    UNUSED(dev);
#endif
}

void busDeselectDevice(const busDevice_t * dev)
{
#ifdef USE_SPI
    if (dev->busType == BUSTYPE_SPI && (dev->flags & DEVFLAGS_USE_MANUAL_DEVICE_SELECT)) {
        spiBusDeselectDevice(dev);
    }
#else
    UNUSED(dev);
#endif
}

bool busIsBusy(const busDevice_t * dev)
{
    switch (dev->busType) {
        case BUSTYPE_SPI:
#ifdef USE_SPI
            return spiBusIsBusy(dev);
#else
            UNUSED(dev);
            return false;
#endif
        case BUSTYPE_I2C:
#ifdef USE_I2C
            return i2cBusBusy(dev,NULL);
#else
            UNUSED(dev);
            return false;
#endif
        default:
            return false;
    }
}

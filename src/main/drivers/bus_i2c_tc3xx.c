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
#include <stdlib.h>

//#include <uart.h>
#include <platform.h>

#include "build/atomic.h"

#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/time.h"

#include "drivers/bus_i2c.h"
#include "drivers/nvic.h"
#include "io_impl.h"
#include "rcc.h"
#include "drivers/light_led.h"

#ifdef USE_I2C
#ifndef SOFT_I2C

#ifndef I2C1_SCL
#define I2C1_SCL P131
#endif
#ifndef I2C1_SDA
#define I2C1_SDA P132
#endif

#ifndef I2C_TIMEOUT
#define I2C_TIMEOUT 1000
#endif

static uint32_t i2cTimeout;
static volatile uint16_t i2cErrorCount = 0;

static i2cDevice_t i2cHardwareMap[] =
{
    { .dev = I2C1, .scl = IO_TAG(I2C1_SCL), .sda = IO_TAG(I2C1_SDA), .speed = I2C_SPEED_400KHZ },
};

void i2cSetSpeed(uint8_t speed)
{
    for (unsigned int i = 0; i < sizeof(i2cHardwareMap) / sizeof(i2cHardwareMap[0]); i++) {
        i2cHardwareMap[i].speed = speed;
    }
}

uint32_t i2cTimeoutUserCallback(void)
{
    i2cErrorCount++;
    return false;
}
#define SIZE_MSG 50
void i2cInit(I2CDevice device)
{
//	uint8 msg[SIZE_MSG] = {0};
//	initUART();


    if (device == I2CINVALID)
        return;

    i2cDevice_t *i2c = &(i2cHardwareMap[device]);

//    snprintf(msg,sizeof(msg),"INi2c %X\n",i2c);
//	uart_sendMessage((uint8*)msg, sizeof(msg));
//	memset(msg,'\0',SIZE_MSG);
//
//    snprintf(msg,sizeof(msg),"i2c %X\n",*i2c);
//    uart_sendMessage((uint8*)msg, sizeof(msg));
//    memset(msg,'\0',SIZE_MSG);

    IO_t scl = IOGetByTag(i2c->scl);
    IO_t sda = IOGetByTag(i2c->sda);

//    snprintf(msg,sizeof(msg),"SCL %d\n",i2c->scl);
//    uart_sendMessage((uint8*)msg, sizeof(msg));
//    memset(msg,'\0',SIZE_MSG);
//    snprintf(msg,sizeof(msg),"SDA %d\n",i2c->sda);
//    uart_sendMessage((uint8*)msg, sizeof(msg));
//    memset(msg,'\0',SIZE_MSG);
//
//    snprintf(msg,sizeof(msg),"SCL_TAG %X\n",scl);
//    uart_sendMessage((uint8*)msg, sizeof(msg));
//    memset(msg,'\0',SIZE_MSG);
//    snprintf(msg,sizeof(msg),"SDA_TAG %X\n",sda);
//    uart_sendMessage((uint8*)msg, sizeof(msg));
//    memset(msg,'\0',SIZE_MSG);

    IOInit(scl, OWNER_I2C, RESOURCE_I2C_SCL, RESOURCE_INDEX(device));
    IOInit(sda, OWNER_I2C, RESOURCE_I2C_SDA, RESOURCE_INDEX(device));


    IfxI2c_I2c_Config config;
    IfxI2c_I2c_initConfig(&config, i2c->dev);

    IfxI2c_Pins pins;
    pins.padDriver = IfxPort_PadDriver_ttlSpeed1;

    for (int i=0; i<IFXI2C_PINMAP_NUM_MODULES; i++)
    	for (int j=0; j<IFXI2C_PINMAP_SCL_INOUT_NUM_ITEMS; j++)
    		if (IfxI2c_Scl_InOut_pinTable[i][j]->pin.port == IO_GPIO(scl) && IfxI2c_Scl_InOut_pinTable[i][j]->pin.pinIndex == IO_Pin(scl))
    			pins.scl = IfxI2c_Scl_InOut_pinTable[i][j];

    for (int i=0; i<IFXI2C_PINMAP_NUM_MODULES; i++)
    	for (int j=0; j<IFXI2C_PINMAP_SCL_INOUT_NUM_ITEMS; j++)
    		if (IfxI2c_Sda_InOut_pinTable[i][j]->pin.port == IO_GPIO(sda) && IfxI2c_Sda_InOut_pinTable[i][j]->pin.pinIndex == IO_Pin(sda))
    			pins.sda = IfxI2c_Sda_InOut_pinTable[i][j];

//    memset(msg,'\0',SIZE_MSG);
//    snprintf(msg,sizeof(msg),"HEXPP1 %X\n",IfxI2c_Sda_InOut_pinTable[0][0]->pin.port);
//    uart_sendMessage((uint8*)msg, sizeof(msg));
//    memset(msg,'\0',SIZE_MSG);
//    snprintf(msg,sizeof(msg),"HEXPP2 %X\n",IfxI2c_Sda_InOut_pinTable[0][1]->pin.port);
//	uart_sendMessage((uint8*)msg, sizeof(msg));
//	memset(msg,'\0',SIZE_MSG);
//    snprintf(msg,sizeof(msg),"HEXPP3 %X\n",IfxI2c_Sda_InOut_pinTable[0][2]->pin.port);
//	uart_sendMessage((uint8*)msg, sizeof(msg));
//	memset(msg,'\0',SIZE_MSG);
//
//    snprintf(msg,sizeof(msg),"HEXPPGP %X\n",IO_GPIO(sda));
//    uart_sendMessage((uint8*)msg, sizeof(msg));
//    memset(msg,'\0',SIZE_MSG);

//    pins.scl = IfxI2c_Scl_InOut_pinTable[0][1];
//    pins.sda = IfxI2c_Sda_InOut_pinTable[0][1];

    config.pins = &pins;

    switch (i2c->speed)
    {
		case I2C_SPEED_100KHZ:
			config.baudrate = 100000;
			break;
		case I2C_SPEED_200KHZ:
			config.baudrate = 200000;
			break;
		case I2C_SPEED_400KHZ:
			config.baudrate = 400000;
			break;
		case I2C_SPEED_800KHZ:
			config.baudrate = 800000;
			break;
	}

    IfxI2c_I2c_initModule(&i2c->i2c, &config);

    IfxI2c_I2c_deviceConfig deviceConfig;
    IfxI2c_I2c_initDeviceConfig(&deviceConfig, &i2c->i2c);
    IfxI2c_I2c_initDevice(&i2c->i2cDev, &deviceConfig);
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

bool i2cWriteBuffer(I2CDevice device, uint8_t addr, uint8_t reg, uint8_t len, const uint8_t * data, bool allowRawAccess)
{
	uint8_t txData[2];

    if (device == I2CINVALID || device > I2CDEV_COUNT) {
        return false;
    }

    i2cDevice_t *pDev = &i2cHardwareMap[device];

    if (!pDev) {
        return false;
    }

    addr <<= 1;

	pDev->i2cDev.deviceAddress = addr;

	txData[0] = reg;
	txData[1] = *data;

	i2cTimeout = I2C_TIMEOUT;

	/* Send Register address */
	while (IfxI2c_I2c_write(&pDev->i2cDev, txData, 2))
		if ((i2cTimeout--) == 0)
			return i2cTimeoutUserCallback();

    return true;
}

bool i2cWrite(I2CDevice device, uint8_t addr, uint8_t reg, uint8_t data, bool allowRawAccess)
{
    return i2cWriteBuffer(device, addr, reg, 1, &data, allowRawAccess);
}

bool i2cRead(I2CDevice device, uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf, bool allowRawAccess)
{
	if (device == I2CINVALID || device > I2CDEV_COUNT) {
		return false;
	}

	i2cDevice_t *pDev = &i2cHardwareMap[device];

	if (!pDev) {
		return false;
	}

	addr <<= 1;

	pDev->i2cDev.deviceAddress = addr;

	i2cTimeout = I2C_TIMEOUT;

	/* Send Register address */
	while (IfxI2c_I2c_write(&pDev->i2cDev, &reg, 1))
		if ((i2cTimeout--) == 0)
			return i2cTimeoutUserCallback();

	i2cTimeout = I2C_TIMEOUT;

	while (IfxI2c_I2c_read(&pDev->i2cDev, buf, len))
		if ((i2cTimeout--) == 0)
			return i2cTimeoutUserCallback();

	/* If all operations OK */
	return true;
}

#endif //AURIX
#endif

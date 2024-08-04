/*
 * compass_bmx160.c
 *
 *  Created on: 12.03.2020
 *      Author: roman
 */
/* FHTW */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "drivers/accgyro/accgyro.h"
//#include "drivers/accgyro/accgyro_bmx.h"

#include "drivers/compass/compass.h"
#include "drivers/compass/compass_bmx160.h"

#if defined(USE_MAG_BMX160)

#define DETECTION_MAX_RETRY_COUNT   5

static int16_t magGain[3];
static int16_t cachedMagData[3];

static bool mpu9250CompassInit(magDev_t * mag)
{
    uint8_t status = 0;

    // Do init at low speed
    busSetSpeed(mag->busDev, BUS_SPEED_INITIALIZATION);

    busWrite(mag->busDev,BMX160_REG_CMD,BMX160_PMU_CMD_PMU_MAG_NORMAL);
    delay(10);

    busWrite(mag->busDev,BMX160_REG_MAG_IF_0,BMX160_REG_MAG_MAN_MODE);

    busWrite(mag->busDev,BMX160_REG_MAG_IF_3,BMX160_REG_MAG_SLEEP);
    busWrite(mag->busDev,BMX160_REG_MAG_IF_2,BMX160_REG_INDIR_MAG_0);
    do{
    	busRead(mag->busDev,BMX160_REG_STATUS,&status);
    }while(!(status&BMX160_REG_STAT_MAG_MAN_OP));
    status = 0x0;

    busWrite(mag->busDev,BMX160_REG_MAG_IF_3,BMX160_REG_MAG_REPXY_REGU_PRES);
    busWrite(mag->busDev,BMX160_REG_MAG_IF_2,BMX160_REG_INDIR_MAG_1);
    do{
    	busRead(mag->busDev,BMX160_REG_STATUS,&status);
    }while(!(status&BMX160_REG_STAT_MAG_MAN_OP));
    status = 0x0;

    busWrite(mag->busDev,BMX160_REG_MAG_IF_3,BMX160_REG_MAG_REPZ_REGU_PRES);
    busWrite(mag->busDev,BMX160_REG_MAG_IF_2,BMX160_REG_INDIR_MAG_2);
    do{
    	busRead(mag->busDev,BMX160_REG_STATUS,&status);
    }while(!(status&BMX160_REG_STAT_MAG_MAN_OP));
    status = 0x0;

    //Prepare MAG_IF[1-3] for mag_if data mode
    busWrite(mag->busDev,BMX160_REG_MAG_IF_3,BMX160_REG_MAG_DATA_MODE);
    busWrite(mag->busDev,BMX160_REG_MAG_IF_2,BMX160_REG_MAG_IF_2_DEF_VAL);
    busWrite(mag->busDev,BMX160_REG_MAG_IF_1,BMX160_REG_MAG_IF_1_DEF_VAL);

    busWrite(mag->busDev,BMX160_REG_MAG_CONF,BMX160_MAG_ODR_100_Hz);

    busWrite(mag->busDev,BMX160_REG_MAG_IF_0,BMX160_REG_MAG_CONF_DEF_VAL);

    busWrite(mag->busDev,BMX160_REG_CMD,BMX160_PMU_CMD_PMU_MAG_NORMAL);
    delay(10);

    busSetSpeed(mag->busDev, BUS_SPEED_FAST);
    return true;
}

static int16_t parseMag(uint8_t *raw, int16_t gain) {
  int ret = (int16_t)(raw[1] << 8 | raw[0]) * gain / 256;
  return constrain(ret, INT16_MIN, INT16_MAX);
}

static bool mpu9250CompassRead(magDev_t * mag)
{
    uint8_t buf[BMX160_MAG_DATA_REG_CNT+1];
    uint8_t cnt = 0;

    // set magData to latest cached value
    memcpy(&mag->magADCRaw, cachedMagData, sizeof(cachedMagData));

    for(cnt=0;cnt<=BMX160_MAG_DATA_REG_CNT;cnt++){
    	busRead(mag->busDev,(BMX160_REG_MAG_DATA_X_LSB+cnt),&buf[cnt]);
    }

    mag->magADCRaw[X] = parseMag(buf + 0, magGain[X]);
    mag->magADCRaw[Y] = parseMag(buf + 2, magGain[Y]);
    mag->magADCRaw[Z] = parseMag(buf + 4, magGain[Z]);

    memcpy(cachedMagData, &mag->magADCRaw, sizeof(cachedMagData));

    return true;
}

bool bmx160CompassDetect(magDev_t * mag)
{
    // Compass on BMX160 is only supported if BMX160 is connected to SPI bus
    // FIXME: We need to use gyro_to_use here, not mag_to_use
    mag->busDev = busDeviceOpen(BUSTYPE_SPI, DEVHW_BMI160, mag->magSensorToUse);
    if (mag->busDev == NULL) {
        return false;
    }

    // Check if Gyro driver initialized the chip
    bmx160ContextData_t *ctx = busDeviceGetScratchpadMemory(mag->busDev);
    if (ctx->chipMagicNumber != 0xB160) {
        return false;
    }

    busSetSpeed(mag->busDev, BUS_SPEED_INITIALIZATION);
    for (int retryCount = 0; retryCount < DETECTION_MAX_RETRY_COUNT; retryCount++) {
        uint8_t sig = 0;

        busRead(mag->busDev,BMX160_REG_CHIPID,&sig);

        if (sig == BMX160_DEVICE_ID) {
            mag->init = mpu9250CompassInit;
            mag->read = mpu9250CompassRead;
            magGain[X]=256;
            magGain[Y]=256;
            magGain[Z]=256;
            return true;
        }
    }

    busSetSpeed(mag->busDev, BUS_SPEED_FAST);
    return false;
}

#endif

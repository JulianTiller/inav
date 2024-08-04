/*
 * compass_bmx160.h
 *
 *  Created on: 12.03.2020
 *      Author: roman
 */

/* FHTW */

#pragma once

bool bmx160CompassDetect(magDev_t *mag);

/* FHTW */

#define BMX160_DEVICE_ID			0xD8

/* BMX160 Registers */
#define BMX160_REG_CHIPID           0x00
#define BMX160_REG_ERR		        0x02
#define BMX160_REG_PMU_STAT         0x03
#define BMX160_REG_MAG_DATA_X_LSB   0x04
#define BMX160_REG_MAG_DATA_X_MSB   0x05
#define BMX160_REG_MAG_DATA_Y_LSB   0x06
#define BMX160_REG_MAG_DATA_Y_MSB   0x07
#define BMX160_REG_MAG_DATA_Z_LSB   0x08
#define BMX160_REG_MAG_DATA_Z_MSB   0x09
#define BMX160_REG_MAG_DATA_R_HALL_LSB   0x0A
#define BMX160_REG_MAG_DATA_R_HALL_MSB   0x0B
#define BMX160_REG_GYR_DATA_X_LSB   0x0C
#define BMX160_REG_ACC_DATA_X_LSB   0x12
#define BMX160_REG_STATUS           0x1B
#define BMX160_REG_TEMPERATURE_0    0x20
#define BMX160_REG_ACC_CONF         0x40
#define BMX160_REG_ACC_RANGE        0x41
#define BMX160_REG_GYR_CONF         0x42
#define BMX160_REG_GYR_RANGE        0x43
#define BMX160_REG_MAG_CONF			0x44
#define BMX160_REG_MAG_IF_0			0x4C
#define BMX160_REG_MAG_IF_1			0x4D
#define BMX160_REG_MAG_IF_2			0x4E
#define BMX160_REG_MAG_IF_3			0x4F
#define BMX160_REG_INT_EN1          0x51
#define BMX160_REG_INT_OUT_CTRL     0x53
#define BMX160_REG_INT_MAP1         0x56
#define BMX160_REG_FOC_CONF         0x69
#define BMX160_REG_CONF             0x6A
#define BMX160_REG_OFFSET_0         0x77
#define BMX160_REG_CMD              0x7E
#define BMX160_REG_INDIR_MAG_0		0x4B
#define BMX160_REG_INDIR_MAG_1		0x51
#define BMX160_REG_INDIR_MAG_2		0x52

/* Register values */
#define BMX160_PMU_CMD_PMU_ACC_NORMAL   0x11
#define BMX160_PMU_CMD_PMU_GYR_NORMAL   0x15
#define BMX160_PMU_CMD_PMU_MAG_NORMAL   0x19
#define BMX160_INT_EN1_DRDY             0x10
#define BMX160_INT_OUT_CTRL_INT1_CONFIG 0x0A
#define BMX160_REG_INT_MAP1_INT1_DRDY   0x80
#define BMX160_CMD_START_FOC            0x03
#define BMX160_CMD_PROG_NVM             0xA0
#define BMX160_REG_STATUS_NVM_RDY       0x10
#define BMX160_REG_STATUS_FOC_RDY       0x08
#define BMX160_REG_CONF_NVM_PROG_EN     0x02
#define BMX160_REG_MAG_MAN_MODE			0x80
#define BMX160_REG_MAG_SLEEP			0x01
#define BMX160_REG_MAG_DATA_MODE		0x02
#define BMX160_REG_MAG_REPXY_LOW_PWR_PRES	0x01
#define BMX160_REG_MAG_REPXY_REGU_PRES		0x04
#define BMX160_REG_MAG_REPXY_ENHA_REGU_PRES	0x07
#define BMX160_REG_MAG_REPXY_HIGH_ACC_PRES	0x17
#define BMX160_REG_MAG_REPZ_LOW_PWR_PRES	0x02
#define BMX160_REG_MAG_REPZ_REGU_PRES		0x0E
#define BMX160_REG_MAG_REPZ_ENHA_REGU_PRES	0x1A
#define BMX160_REG_MAG_REPZ_HIGH_ACC_PRES	0x52
#define BMX160_REG_STAT_MAG_MAN_OP		0x40
#define BMX160_REG_MAG_IF_1_DEF_VAL		0x42
#define BMX160_REG_MAG_IF_2_DEF_VAL		0x4C
#define BMX160_REG_MAG_IF_3_DEF_VAL		0x00
#define BMX160_REG_MAG_CONF_DEF_VAL		0x00

#define BMX160_MAG_ODR_25_Hz			0x06
#define BMX160_MAG_ODR_100_Hz			0x08
#define BMX160_MAG_ODR_800_Hz			0x0B

#define BMX160_BWP_NORMAL               0x20
#define BMX160_BWP_OSR2                 0x10
#define BMX160_BWP_OSR4                 0x00

#define BMX160_ODR_400_Hz               0x0A
#define BMX160_ODR_800_Hz               0x0B
#define BMX160_ODR_1600_Hz              0x0C
#define BMX160_ODR_3200_Hz              0x0D

#define BMX160_RANGE_2G                 0x03
#define BMX160_RANGE_4G                 0x05
#define BMX160_RANGE_8G                 0x08
#define BMX160_RANGE_16G                0x0C

#define BMX160_RANGE_125DPS             0x04
#define BMX160_RANGE_250DPS             0x03
#define BMX160_RANGE_500DPS             0x02
#define BMX160_RANGE_1000DPS            0x01
#define BMX160_RANGE_2000DPS            0x00

#define BMX160_MAG_DATA_REG_CNT			0x07

typedef struct __attribute__ ((__packed__)) bmx160ContextData_s {
    uint16_t    chipMagicNumber;
    uint8_t     lastReadStatus;
    uint8_t     __padding;
    uint8_t     gyroRaw[6];
    uint8_t     accRaw[6];
} bmx160ContextData_t;

/* FHTW */

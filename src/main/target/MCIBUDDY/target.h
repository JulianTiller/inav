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

#pragma once

#define TARGET_BOARD_IDENTIFIER "BUDDY"

#define LED0                    P005
#define LED1                    P006
#define LED2                    P006

#define LED0_INVERTED
#define LED1_INVERTED
#define LED2_INVERTED

#define FLYINGSHIELD_V2 0
#define FLYINGSHIELD_V3 0
#define FLYINGSHIELD_V3_1 1
#define EXT_GPS_MAG 0

#if FLYINGSHIELD_V2
#define USE_FAKE_GPS
#define USE_RADAR_COORD_GPS

#define USE_GYRO
#define USE_DUAL_GYRO
#define USE_GYRO_MPU9250
#define GYRO_MPU9250_ALIGN      CW270_DEG
#define MPU9250_SPI_BUS			SPIDEV_1
#define GYRO_0_CS_PIN			P105
#define GYRO_1_CS_PIN			P104

#define USE_ACC
#define USE_ACC_MPU9250
#define ACC_MPU9250_ALIGN       CW270_DEG

// option to use MPU9150 or MPU9250 integrated AK89xx Mag
#define USE_MAG
#define MAG_SPI_BUS             SPIDEV_1
#define USE_MAG_MPU9250
#define MAG_MPU9250_ALIGN       CW0_DEG

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN 			P102
#define SPI1_MISO_PIN   		P101
#define SPI1_MOSI_PIN   		P103
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN 			P108
#define SPI2_MISO_PIN   		P107
#define SPI2_MOSI_PIN   		P026
#endif

#if FLYINGSHIELD_V3
#if EXT_GPS_MAG
#define USE_MAG
#define USE_MAG_HMC5883
#define MAG_I2C_BUS
#define MAG_I2C_BUS BUS_I2C1
#else
//#define USE_FAKE_GPS
//#define USE_RADAR_COORD_GPS
#define USE_MAG
#define USE_FAKE_MAG
#endif

#define USE_GYRO
#define USE_GYRO_BMX160
#define GYRO_BMX160_ALIGN		CW90_DEG
#define BMX160_SPI_BUS			SPIDEV_1
#define BMX160_CS_PIN			P105

#define USE_ACC
#define USE_ACC_BMX160
#define ACC_BMX160_ALIGN		CW90_DEG

#define USE_BARO
#define BARO_I2C_BUS			BUS_I2C1
#define USE_BARO_DPS310

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN 			P102
#define SPI1_MISO_PIN   		P101
#define SPI1_MOSI_PIN   		P103
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN 			P3311
#define SPI2_MISO_PIN   		P107
#define SPI2_MOSI_PIN   		P026

#define USE_CAN
#endif

#if FLYINGSHIELD_V3_1
#if EXT_GPS_MAG
#define USE_MAG
#define USE_MAG_HMC5883
#define MAG_I2C_BUS
#define MAG_I2C_BUS BUS_I2C1
#else
#define USE_FAKE_GPS
#define USE_MAG
#define USE_MAG_BMX160
#define MAG_BMX160_ALIGN	CW90_DEG
#endif


//#define USE_IMU_BMI160
//#define USE_GYRO_BMX160
//#define BMX160_ALIGN			CW90_DEG
//#define BMX160_SPI_BUS			SPIDEV_1
//#define BMX160_CS_PIN			P105

//#define USE_ACC
//#define USE_ACC_BMX160
#define ACC_BMX160_ALIGN		CW90_DEG

#define USE_BARO
#define BARO_I2C_BUS			BUS_I2C1
#define USE_BARO_DPS310


#define USE_IMU_BMI160
#define BMI160_SPI_BUS SPIDEV_1
#define BMI160_CS_PIN P147
#define IMU_BMI160_ALIGN CW90_DEG

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN 			P158
#define SPI1_MISO_PIN   		P157
#define SPI1_MOSI_PIN   		P156
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN 			P148
#define SPI2_MISO_PIN   		P107
#define SPI2_MOSI_PIN   		P026

//#define USE_CAN
//#define CAN_EN            P335
#endif

#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_SPI_BUS 			SPIDEV_2
#define SDCARD_CS_PIN			P021
#define SDCARD_DETECT_PIN		P008
//#define SDCARD_DETECT_INVERTED

#define USE_BLACKBOX
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2
#define SERIAL_PORT_COUNT       6

#define UART1_RX_PIN            P141
#define UART1_TX_PIN            P140

#define UART2_RX_PIN			P1110
#define UART2_TX_PIN			P1112

#define UART3_RX_PIN			P322
#define UART3_TX_PIN			P157

#define UART4_RX_PIN			P338
#define UART4_TX_PIN			P339

#define SOFTSERIAL_1_RX_PIN     P000
#define SOFTSERIAL_1_TX_PIN     P001

#define SOFTSERIAL_2_RX_PIN     P002
#define SOFTSERIAL_2_TX_PIN     P003

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL 				P131
#define I2C1_SDA 				P132

#define USE_ADC
#define ADC_INSTANCE            ADC1
#define ADC_CHANNEL_1_PIN       P323
#define VBAT_ADC_CHANNEL        ADC_CHN_1
#define VBAT_SCALE_DEFAULT      1585

//#define ADC_CHANNEL_2_PIN         P408
//#define CURRENT_METER_ADC_CHANNEL ADC_CHN_2
//#define CURRENT_METER_SCALE       400
//#define CURRENT_METER_OFFSET      0

#define DEFAULT_RX_TYPE         RX_TYPE_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_IBUS
#define SERIALRX_UART           SERIAL_PORT_USART2

// Number of available PWM outputs
#define MAX_PWM_OUTPUT_PORTS    20

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORT00        0x1fff
#define TARGET_IO_PORT01        0x00f8
#define TARGET_IO_PORT02        0x0fff
#define TARGET_IO_PORT10        0x01ff
#define TARGET_IO_PORT11        0xffff
#define TARGET_IO_PORT12        0x0003
#define TARGET_IO_PORT13        0x000f
#define TARGET_IO_PORT14        0x07ff
#define TARGET_IO_PORT15        0x01ff
#define TARGET_IO_PORT20        0x7fcf
#define TARGET_IO_PORT21        0x00ff
#define TARGET_IO_PORT22        0x0fff
#define TARGET_IO_PORT23        0x00ff
#define TARGET_IO_PORT32        0x00fd
#define TARGET_IO_PORT33        0xffff
#define TARGET_IO_PORT34        0x001e
#define TARGET_IO_PORT40        0x03ff
#define TARGET_IO_PORTAN        0xffff


#define USART1 &MODULE_ASCLIN0
#define USART2 &MODULE_ASCLIN1
#define USART3 &MODULE_ASCLIN3
#define UART4 &MODULE_ASCLIN2

#define SPI1 &MODULE_QSPI2
#define SPI2 &MODULE_QSPI3
#define SPI3 &MODULE_QSPI2
#define SPI4 &MODULE_QSPI0

#define I2C1 &MODULE_I2C0
#define ADC1 &MODULE_EVADC

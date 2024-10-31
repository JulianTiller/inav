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

#if defined(STM32F7)
#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_ll_spi.h"
#include "stm32f7xx_ll_gpio.h"
#include "stm32f7xx_ll_dma.h"
#include "stm32f7xx_ll_rcc.h"
#include "stm32f7xx_ll_bus.h"
#include "stm32f7xx_ll_tim.h"

// Chip Unique ID on F7
#if defined(STM32F722xx)
#define U_ID_0 (*(uint32_t*)0x1ff07a10)
#define U_ID_1 (*(uint32_t*)0x1ff07a14)
#define U_ID_2 (*(uint32_t*)0x1ff07a18)
#else
#define U_ID_0 (*(uint32_t*)0x1ff0f420)
#define U_ID_1 (*(uint32_t*)0x1ff0f424)
#define U_ID_2 (*(uint32_t*)0x1ff0f428)
#endif

#elif defined(STM32F4)
#include "stm32f4xx.h"

// Chip Unique ID on F405
#define U_ID_0 (*(uint32_t*)0x1fff7a10)
#define U_ID_1 (*(uint32_t*)0x1fff7a14)
#define U_ID_2 (*(uint32_t*)0x1fff7a18)

#elif defined(STM32F3)
#include "stm32f30x_conf.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_gpio.h"
#include "core_cm4.h"

// Chip Unique ID on F303
#define U_ID_0 (*(uint32_t*)0x1FFFF7AC)
#define U_ID_1 (*(uint32_t*)0x1FFFF7B0)
#define U_ID_2 (*(uint32_t*)0x1FFFF7B4)

#elif defined(AURIX)

#include <stdint.h>
#include "Ifx_Types.h"

#include "IfxPort.h"
#include "IfxFlash.h"
#include "IfxAsclin_Asc.h"
#include "IfxQspi_SpiMaster.h"
#include "IfxGtm_Tom_Pwm.h"
#include "IfxEvadc_Adc.h"

#ifdef MCI_BUDDY
#include "IfxI2c_I2c.h"
#endif

#define SystemCoreClock 64000000U

#define U_ID_0 (((*(uint32_t*)&SCU_CHIPID) & IFX_SCU_CHIPID_CHREV_MSK) >> IFX_SCU_CHIPID_CHREV_OFF)
#define U_ID_1 (((*(uint32_t*)&SCU_CHIPID) & IFX_SCU_CHIPID_CHTEC_MSK) >> IFX_SCU_CHIPID_CHTEC_OFF)
#define U_ID_2 (((*(uint32_t*)&SCU_CHIPID) & IFX_SCU_CHIPID_CHID_MSK) >> IFX_SCU_CHIPID_CHID_OFF)

typedef uint32_t EXTITrigger_TypeDef;
typedef uint32_t IRQn_Type;
typedef Ifx_ASCLIN USART_TypeDef;
typedef IfxQspi_SpiMaster_Channel SPI_TypeDef;
typedef uint32_t DMA_TypeDef;
typedef uint32_t DMA_Channel_TypeDef;
typedef Ifx_P GPIO_TypeDef;
typedef Ifx_EVADC ADC_TypeDef;
typedef uint32_t RCC_ClocksTypeDef;
typedef uint32_t FunctionalState;

#ifdef MCI_OCTO
typedef uint32_t I2C_TypeDef;
#else
typedef Ifx_I2C I2C_TypeDef;
#endif

typedef union
{
	Ifx_GTM_TOM_CH tom;
	Ifx_GTM_TIM_CH tim;
}TIM_TypeDef;

typedef enum
{
  Bit_RESET = 0,
  Bit_SET
}BitAction;

#endif

#include "target/common.h"
#include "target.h"
#include "target/common_post.h"

// Remove the unaligned packed structure member pointer access warning
// The compiler guarantees that unaligned access is safe for packed structures.

#if (__GNUC__ >= 9)
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#endif

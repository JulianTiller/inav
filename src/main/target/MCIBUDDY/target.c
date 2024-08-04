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

#include <stdint.h>

#include <platform.h>
#include "drivers/io.h"
#include "drivers/pwm_mapping.h"
#include "drivers/timer.h"

#define GPIO_AF_P023_TOM0_CH11 		IfxPort_OutputIdx_alt1
#define GPIO_AF_P024_TOM0_CH12 		IfxPort_OutputIdx_alt1
#define GPIO_AF_P025_TOM0_CH13 		IfxPort_OutputIdx_alt1
#define GPIO_AF_P027_TOM0_CH15 		IfxPort_OutputIdx_alt1
#define GPIO_AF_P007_TOM0_CH14    IfxPort_OutputIdx_alt1
#define GPIO_AF_P333_TOM0_CH7     IfxPort_OutputIdx_alt1
#define GPIO_AF_P112_TOM0_CH8     IfxPort_OutputIdx_alt1
#define GPIO_AF_P156_TOM0_CH0     IfxPort_OutputIdx_alt1
#define GPIO_AF_P141_TOM0_CH4     IfxPort_OutputIdx_alt1
#define GPIO_AF_P331_TOM0_CH5     IfxPort_OutputIdx_alt1
#define GPIO_AF_P001_TOM0_CH1 		IfxPort_OutputIdx_general
#define GPIO_AF_P003_TOM0_CH10 		IfxPort_OutputIdx_general

#define DEF_TOM(_tom, _ch, _pin, _usage, _flags, _sel, _outn) \
	{ (TIM_TypeDef*)((uint32)&GTM_TOM##_tom##_CH##_ch##_CTRL), \
	  IO_TAG(_pin), _ch, _flags, IOCFG_AF_PP, GPIO_AF_##_pin##_TOM##_tom##_CH##_ch, _usage, _sel, _outn }

#define DEF_TIM(_tim, _ch, _pin, _usage, _flags, _sel) \
	{ (TIM_TypeDef*)((uint32)&GTM_TIM##_tim##_CH##_ch##_GPR0), \
	   IO_TAG(_pin), _ch, _flags, IOCFG_IPU, IOCFG_IPU, _usage, _sel, 0 }

const timerHardware_t timerHardware[] = {
	DEF_TOM(0, 11, P023, TIM_USE_MC_MOTOR, 1, 0, 3),      //inav-configurator Motor 1
	DEF_TOM(0, 12, P024, TIM_USE_MC_MOTOR, 1, 0, 4),      //inav-configurator Motor 2
	DEF_TOM(0, 13, P025, TIM_USE_MC_MOTOR, 1, 0, 5),      //inav-configurator Motor 3
	DEF_TOM(0, 15, P027, TIM_USE_MC_MOTOR, 1, 0, 7),      //inav-configurator Motor 4
	DEF_TIM(2, 4, P151, TIM_USE_PPM, 1, 4),
	DEF_TIM(2, 0, P000, TIM_USE_PWM, 1, 1),
	DEF_TOM(0, 1, P001, TIM_USE_ANY, 1, 0, 19),
	DEF_TIM(2, 1, P001, TIM_USE_PWM, 1, 1),
	DEF_TIM(3, 1, P002, TIM_USE_PWM, 1, 2),
	DEF_TOM(0, 10, P003, TIM_USE_ANY, 1, 0, 12),
	DEF_TIM(3, 2, P003, TIM_USE_PWM, 1, 1),
	DEF_TIM(3, 3, P004, TIM_USE_PWM, 1, 1),
	DEF_TIM(3, 4, P005, TIM_USE_PWM, 1, 1),
};

const int timerHardwareCount = sizeof(timerHardware) / sizeof(timerHardware[0]);

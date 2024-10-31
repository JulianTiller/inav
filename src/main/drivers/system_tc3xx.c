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
#include <string.h>

#include "platform.h"
#include "IfxSmu.h"
#include "IfxSrc.h"

#include "drivers/system.h"
#include "drivers/time.h"

#include "Bsp.h"

uint32_t STMFreqMhz = 0;

void systemReset(void)
{
	IfxCpu_disableInterrupts();

	uint16 endinitPw = IfxScuWdt_getCpuWatchdogPassword();
	IfxScuWdt_clearCpuEndinit(endinitPw);
	IfxScuWdt_clearSafetyEndinit(endinitPw);

	SCU_RSTCON.B.SW = 1u;
	SCU_SWRSTCON.B.SWRSTREQ = 1u;

	IfxScuWdt_setSafetyEndinit(endinitPw);
	IfxScuWdt_setCpuEndinit(endinitPw);
}

void systemResetToBootloader(void)
{

}

void enableGPIOPowerUsageAndNoiseReductions(void)
{

}

bool isMPUSoftReset(void)
{
	return false;
}

void systemInit(void)
{
	IfxCpu_enableInterrupts();

    /*
     * !!WATCHDOG0 AND SAFETY WATCHDOG ARE DISABLED HERE!!
     * Enable the watchdog in the demo if it is required and also service the watchdog periodically
     * */
    IfxScuWdt_disableCpuWatchdog (IfxScuWdt_getCpuWatchdogPassword ());
    IfxScuWdt_disableSafetyWatchdog (IfxScuWdt_getSafetyWatchdogPassword ());

	STMFreqMhz = IfxStm_getFrequency(BSP_DEFAULT_TIMER) / 1000000;

}

timeUs_t micros(void)
{
	return (IfxStm_get(BSP_DEFAULT_TIMER) & TIME_INFINITE) / STMFreqMhz;
}

timeMs_t millis(void)
{
    return micros() / 1000;
}


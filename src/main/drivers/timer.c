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
#include <math.h>

#include "platform.h"

#include "build/atomic.h"

#include "common/log.h"
#include "common/memory.h"
#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/rcc.h"
#include "drivers/time.h"
#include "drivers/nvic.h"

#include "drivers/timer.h"
#include "drivers/timer_impl.h"

#ifdef AURIX
#define TIM_N(n) (1 << (n))
#endif

#ifdef AURIX
timerConfig_t * timerConfig[HARDWARE_TIMER_DEFINITION_COUNT];
#else
timHardwareContext_t * timerCtx[HARDWARE_TIMER_DEFINITION_COUNT];
#endif

#if defined(AT32F43x)
uint8_t lookupTimerIndex(const tmr_type *tim)
{
    int i;

    // let gcc do the work, switch should be quite optimized
    for (i = 0; i < HARDWARE_TIMER_DEFINITION_COUNT; i++) {
        if (tim == timerDefinitions[i].tim) {
            return i;
        }
    }
    // make sure final index is out of range
    return ~1;
}
#else
// return index of timer in timer table. Lowest timer has index 0
uint8_t lookupTimerIndex(const TIM_TypeDef *tim)
{
    int i;

    // let gcc do the work, switch should be quite optimized
    for (i = 0; i < HARDWARE_TIMER_DEFINITION_COUNT; i++) {
        if (tim == timerDefinitions[i].tim) {
            return i;
        }
    }

    // make sure final index is out of range
    return ~1;
}
#endif

#ifdef AURIX
static inline uint8_t lookupChannelIndex(const uint16_t channel)
{
	return 0;
}

void timerNVICConfigure(volatile Ifx_SRC_SRCR* src, int irqPriority)
{
    impl_timerNVICConfigure(src, irqPriority);
}

void timerConfigBase(TIM_TypeDef *tim, uint16_t period, uint8_t mhz)
{
    impl_timerConfigBase(tim, period, mhz);
}

// old interface for PWM inputs. It should be replaced
void timerConfigure(const timerHardware_t *timerHardwarePtr, uint16_t period, uint8_t mhz)
{
    unsigned timer = lookupTimerIndex(timerHardwarePtr->tim);

    impl_timerConfigBase(timerDefinitions[timer].tim, period, mhz);
    impl_enableTimer(timerHardwarePtr, timerHardwarePtr->channel);

    if (timerDefinitions[timer].src != 0) {
        impl_timerNVICConfigure(timerDefinitions[timer].src, timerDefinitions[timer].irqPrio);
    }
}

void timerChCCHandlerInit(timerCCHandlerRec_t *self, timerCCHandlerCallback *fn)
{
    self->fn = fn;
}

void timerChOvrHandlerInit(timerOvrHandlerRec_t *self, timerOvrHandlerCallback *fn)
{
    self->fn = fn;
    self->next = NULL;
}

// update overflow callback list
// some synchronization mechanism is neccesary to avoid disturbing other channels (BASEPRI used now)
static void timerChConfig_UpdateOverflow(timerConfig_t *cfg, TIM_TypeDef *tim) {
    timerOvrHandlerRec_t **chain = &cfg->overflowCallbackActive;
	for (int i = 0; i < CC_CHANNELS_PER_TIMER; i++)
		if (cfg->overflowCallback[i])
		{
			*chain = cfg->overflowCallback[i];
			chain = &cfg->overflowCallback[i]->next;
		}
	*chain = NULL;

    // enable or disable IRQ
    if (cfg->overflowCallbackActive) {
        impl_timerEnableIT(tim, OVER_IRQ);
    }
    else {
        impl_timerDisableIT(tim, OVER_IRQ);
    }
}

timerConfig_t * timerGetConfigContext(int timerIndex)
{
    // If timer context does not exist - allocate memory
    if (timerConfig[timerIndex] == NULL) {
        timerConfig[timerIndex] = memAllocate(sizeof(timerConfig_t), OWNER_TIMER);
    }

    return timerConfig[timerIndex];
}

// config edge and overflow callback for channel. Try to avoid overflowCallback, it is a bit expensive
void timerChConfigCallbacks(const timerHardware_t *timHw, timerCCHandlerRec_t *edgeCallback, timerOvrHandlerRec_t *overflowCallback)
{
    uint8_t timerIndex = lookupTimerIndex(timHw->tim);

    if (timerIndex >= HARDWARE_TIMER_DEFINITION_COUNT) {
        return;
    }

    uint8_t channelIndex = lookupChannelIndex(timHw->channel);
    if (edgeCallback == NULL)   // disable irq before changing callback to NULL
    {
    	impl_timerDisableIT(timHw->tim, EDGE_IRQ);
    }

    timerConfig_t * cfg = timerGetConfigContext(timerIndex);

    if (cfg) {
        // setup callback info
        cfg->edgeCallback[channelIndex] = edgeCallback;
        cfg->overflowCallback[channelIndex] = overflowCallback;

        // enable channel IRQ
        if (edgeCallback)
        {
        	impl_timerEnableIT(timHw->tim, EDGE_IRQ);
        }

        timerChConfig_UpdateOverflow(cfg, timHw->tim);
    }
    else {
        // OOM error, disable IRQs for this timer
    	impl_timerDisableIT(timHw->tim, EDGE_IRQ);
    }
}

// Configure input captupre
void timerChConfigIC(const timerHardware_t *timHw, bool polarityRising, unsigned inputFilterTicks)
{
    impl_timerChConfigIC(timHw, polarityRising, inputFilterTicks);
}

uint16_t timerGetPeriod(const timerHardware_t *timHw)
{
	return impl_timerGetPeriod(timHw);
}

void timerInit(void)
{
    memset(timerConfig, 0, sizeof (timerConfig));

	IfxGtm_enable(&MODULE_GTM);
	//IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK0);
	IfxGtm_Cmu_setGclkFrequency(&MODULE_GTM, SystemCoreClock);
	IfxGtm_Cmu_setClkFrequency(&MODULE_GTM, IfxGtm_Cmu_Clk_0, SystemCoreClock);				//Source TIM
	IfxGtm_Cmu_setClkFrequency(&MODULE_GTM, IfxGtm_Cmu_Clk_1, 10000);						//Source Timeout TIM
	IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_FXCLK | IFXGTM_CMU_CLKEN_CLK0 | IFXGTM_CMU_CLKEN_CLK1);

	IfxGtm_Tbu_enableChannel(&MODULE_GTM, IfxGtm_Tbu_Ts_0);

    /* Before 2.0 timer outputs were initialized to IOCFG_AF_PP_PD even if not used */
    /* To keep compatibility make sure all timer output pins are mapped to INPUT with weak pull-down */
    for (int i = 0; i < timerHardwareCount; i++) {
        const timerHardware_t *timerHardwarePtr = &timerHardware[i];
        IOConfigGPIO(IOGetByTag(timerHardwarePtr->tag), IOCFG_IPD);
    }
}

const timerHardware_t *timerGetByTag(ioTag_t tag, timerUsageFlag_e flag)
{
    if (!tag) {
        return NULL;
    }

    for (int i = 0; i < timerHardwareCount; i++) {
        if (timerHardware[i].tag == tag) {
            if (timerHardware[i].usageFlags & flag || flag == 0) {
                return &timerHardware[i];
            }
        }
    }
    return NULL;
}

void timerPWMConfigChannel(TIM_TypeDef * tim, uint8_t channel, bool isNChannel, bool inverted, uint16_t value)
{
    impl_timerPWMConfigChannel(tim, channel, isNChannel, inverted, value);
}

void timerEnable(const timerHardware_t *timHw, uint8_t channel)
{
	impl_enableTimer(timHw, channel);
}

void timerPWMStart(TIM_TypeDef * tim, uint8_t channel, bool isNChannel)
{
    impl_timerPWMStart(tim, channel, isNChannel);
}

volatile timCCR_t * timerCCR(TIM_TypeDef *tim, uint8_t channel)
{
    return impl_timerCCR(tim, channel);
}
#else
void timerConfigBase(TCH_t * tch, uint16_t period, uint32_t hz)
{
    if (tch == NULL) {
        return;
    }

    impl_timerConfigBase(tch, period, hz);
}

// old interface for PWM inputs. It should be replaced
void timerConfigure(TCH_t * tch, uint16_t period, uint32_t hz)
{
    if (tch == NULL) {
        return;
    }

    impl_timerConfigBase(tch, period, hz);
    impl_timerNVICConfigure(tch, NVIC_PRIO_TIMER);
    impl_enableTimer(tch);
}

TCH_t * timerGetTCH(const timerHardware_t * timHw)
{
    const int timerIndex = lookupTimerIndex(timHw->tim);
    
    if (timerIndex >= HARDWARE_TIMER_DEFINITION_COUNT) {
        LOG_ERROR(TIMER, "Can't find hardware timer definition");
        return NULL;
    }

    // If timer context does not exist - allocate memory
    if (timerCtx[timerIndex] == NULL) {
        timerCtx[timerIndex] = memAllocate(sizeof(timHardwareContext_t), OWNER_TIMER);
        
        // Check for OOM
        if (timerCtx[timerIndex] == NULL) {
            LOG_ERROR(TIMER, "Can't allocate TCH object");
            return NULL;
        }

        // Initialize parent object
        memset(timerCtx[timerIndex], 0, sizeof(timHardwareContext_t));
        timerCtx[timerIndex]->timDef = &timerDefinitions[timerIndex];
        timerCtx[timerIndex]->ch[0].timCtx = timerCtx[timerIndex];
        timerCtx[timerIndex]->ch[1].timCtx = timerCtx[timerIndex];
        timerCtx[timerIndex]->ch[2].timCtx = timerCtx[timerIndex];
        timerCtx[timerIndex]->ch[3].timCtx = timerCtx[timerIndex];

        // Implementation-specific init
        impl_timerInitContext(timerCtx[timerIndex]);
    }

    // Initialize timer channel object
    timerCtx[timerIndex]->ch[timHw->channelIndex].timHw = timHw;
    timerCtx[timerIndex]->ch[timHw->channelIndex].dma = NULL;
    timerCtx[timerIndex]->ch[timHw->channelIndex].cb = NULL;
    timerCtx[timerIndex]->ch[timHw->channelIndex].dmaState = TCH_DMA_IDLE;

    return &timerCtx[timerIndex]->ch[timHw->channelIndex];
}

// config edge and overflow callback for channel. Try to avoid overflowCallback, it is a bit expensive
void timerChInitCallbacks(timerCallbacks_t * cb, void * callbackParam, timerCallbackFn * edgeCallback, timerCallbackFn * overflowCallback)
{
    cb->callbackParam = callbackParam;
    cb->callbackEdge = edgeCallback;
    cb->callbackOvr = overflowCallback;
}

void timerChConfigCallbacks(TCH_t * tch, timerCallbacks_t * cb)
{
    if (tch == NULL) {
        return;
    }

    if (cb->callbackEdge == NULL) {
        impl_timerDisableIT(tch, TIM_IT_CCx(tch->timHw->channelIndex));
    }
    
    if (cb->callbackOvr == NULL) {
        impl_timerDisableIT(tch, IMPL_TIM_IT_UPDATE_INTERRUPT);
    }

    tch->cb = cb;

    if (cb->callbackEdge) {
        impl_timerEnableIT(tch, TIM_IT_CCx(tch->timHw->channelIndex));
    }

    if (cb->callbackOvr) {
        impl_timerEnableIT(tch, IMPL_TIM_IT_UPDATE_INTERRUPT);
    }
}

// Configure input captupre
void timerChConfigIC(TCH_t * tch, bool polarityRising, unsigned inputFilterSamples)
{
    impl_timerChConfigIC(tch, polarityRising, inputFilterSamples);
}

uint16_t timerGetPeriod(TCH_t * tch)
{
#if defined(AT32F43x)
    return tch->timHw->tim->pr;     //tmr pr registe
#else
    return tch->timHw->tim->ARR;
#endif
}
//timerHardware  target.c
void timerInit(void)
{
    memset(timerCtx, 0, sizeof (timerCtx));

    /* enable the timer peripherals */
    for (int i = 0; i < timerHardwareCount; i++) {
        unsigned timer = lookupTimerIndex(timerHardware[i].tim);
        RCC_ClockCmd(timerDefinitions[timer].rcc, ENABLE);
    }

    /* Before 2.0 timer outputs were initialized to IOCFG_AF_PP_PD even if not used */
    /* To keep compatibility make sure all timer output pins are mapped to INPUT with weak pull-down */
    for (int i = 0; i < timerHardwareCount; i++) {
        const timerHardware_t *timerHardwarePtr = &timerHardware[i];
        IOConfigGPIO(IOGetByTag(timerHardwarePtr->tag), IOCFG_IPD);
    }
}

const timerHardware_t * timerGetByTag(ioTag_t tag, timerUsageFlag_e flag)
{
    if (!tag) {
        return NULL;
    }

    for (int i = 0; i < timerHardwareCount; i++) {
        if (timerHardware[i].tag == tag) {
            if (timerHardware[i].usageFlags & flag || flag == 0) {
                return &timerHardware[i];
            }
        }
    }
    return NULL;
}

const timerHardware_t * timerGetByUsageFlag(timerUsageFlag_e flag)
{
    for (int i = 0; i < timerHardwareCount; i++) {
        if (timerHardware[i].usageFlags & flag) {
            return &timerHardware[i];
        }
    }
    return NULL;
}

void timerPWMConfigChannel(TCH_t * tch, uint16_t value)
{
    impl_timerPWMConfigChannel(tch, value);
}

void timerEnable(TCH_t * tch)
{
    impl_enableTimer(tch);
}

void timerPWMStart(TCH_t * tch)
{
    impl_timerPWMStart(tch);
}

volatile timCCR_t *timerCCR(TCH_t * tch)
{
    return impl_timerCCR(tch);
}

void timerChCaptureEnable(TCH_t * tch)
{
    impl_timerChCaptureCompareEnable(tch, true);
}

void timerChCaptureDisable(TCH_t * tch)
{
    impl_timerChCaptureCompareEnable(tch, false);
}

uint32_t timerGetBaseClock(TCH_t * tch)
{
    return timerGetBaseClockHW(tch->timHw);
}

uint32_t timerGetBaseClockHW(const timerHardware_t * timHw)
{
    return timerClock(timHw->tim);
}

bool timerPWMConfigChannelDMA(TCH_t * tch, void * dmaBuffer, uint8_t dmaBufferElementSize, uint32_t dmaBufferElementCount)
{
    return impl_timerPWMConfigChannelDMA(tch, dmaBuffer, dmaBufferElementSize, dmaBufferElementCount);
}

void timerPWMPrepareDMA(TCH_t * tch, uint32_t dmaBufferElementCount)
{
    impl_timerPWMPrepareDMA(tch, dmaBufferElementCount);
}

void timerPWMStartDMA(TCH_t * tch)
{
    impl_timerPWMStartDMA(tch);
}

void timerPWMStopDMA(TCH_t * tch)
{
    impl_timerPWMStopDMA(tch);
}

bool timerPWMDMAInProgress(TCH_t * tch)
{
    return tch->dmaState != TCH_DMA_IDLE;
}
#endif

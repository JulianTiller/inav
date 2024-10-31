/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#if defined(AT32F43x)
#define IMPL_TIM_IT_UPDATE_INTERRUPT      TMR_OVF_INT
#define TIM_IT_CCx(chIdx)                 (TMR_C1_INT << (chIdx))
// 0x2 0x4 0x8 0x10 0\1\2\3

#define _TIM_IRQ_HANDLER2(name, i, j)                                   \
    void name(void)                                                     \
    {                                                                   \s
        impl_timerCaptureCompareHandler(TMR ## i, timerCtx[i - 1]); \
        impl_timerCaptureCompareHandler(TMR ## j, timerCtx[j - 1]); \
    } struct dummy

#define _TIM_IRQ_HANDLER(name, i)                                       \
    void name(void)                                                     \
    {                                                                   \
        impl_timerCaptureCompareHandler(TMR ## i, timerCtx[i - 1]); \
    } struct dummy

uint8_t lookupTimerIndex(const tmr_type *tim);
void impl_timerCaptureCompareHandler(tmr_type *tim, timHardwareContext_t * timerCtx);

#endif// end at32

#if defined(USE_HAL_DRIVER)
# define IMPL_TIM_IT_UPDATE_INTERRUPT      TIM_IT_UPDATE
# define TIM_IT_CCx(chIdx)                 (TIM_IT_CC1 << (chIdx))
#elif defined(AURIX)
#define TIM_IT_CCx(ch)                    (TIM_IT_CC1 << ((ch) / 4))
#define IMPL_TIM_IT_UPDATE_INTERRUPT      TIM_IT_Update
#define CC_CHANNELS_PER_TIMER       1

#define EDGE_IRQ					1
#define OVER_IRQ					2
#else
#define IMPL_TIM_IT_UPDATE_INTERRUPT      TIM_IT_Update
#define TIM_IT_CCx(chIdx)                 (TIM_IT_CC1 << (chIdx))
#endif

#ifdef AURIX

#define _TIM_IRQ_HANDLER(_tim, _ch, _index)													\
	IFX_INTERRUPT(TIM##_tim##CH##_ch##_ISR, 0, IFX_INTPRIO_TIM##_tim##_CH##_ch)				\
	{																						\
		impl_timerCaptureCompareHandler(timerDefinitions[_index].tim, timerConfig[_index]);	\
	}

#define _TOM_IRQ_HANDLER(_tom, _ch1, _ch2, _index1, _index2)															\
	IFX_INTERRUPT(TOM##_tom##CH##_ch1##_##_ch2##_ISR, 0, IFX_INTPRIO_TOM##_tom##_CH##_ch1##_##_ch2)			\
	{																								\
		if(timerDefinitions[_index1].tim->tom.IRQ_NOTIFY.B.CCU0TC == 1)								\
			impl_timerCaptureCompareHandler(timerDefinitions[_index1].tim, timerConfig[_index1]);		\
		if(timerDefinitions[_index2].tim->tom.IRQ_NOTIFY.B.CCU0TC == 1)							\
			impl_timerCaptureCompareHandler(timerDefinitions[_index2].tim, timerConfig[_index2]);	\
	}

typedef struct timerConfig_s {
    timerCCHandlerRec_t *edgeCallback[CC_CHANNELS_PER_TIMER];
    timerOvrHandlerRec_t *overflowCallback[CC_CHANNELS_PER_TIMER];
    timerOvrHandlerRec_t *overflowCallbackActive; // null-terminated linkded list of active overflow callbacks
} timerConfig_t;

extern timerConfig_t * timerConfig[HARDWARE_TIMER_DEFINITION_COUNT];

#else
#define _TIM_IRQ_HANDLER2(name, i, j)                                   \
    void name(void)                                                     \
    {                                                                   \
        impl_timerCaptureCompareHandler(TIM ## i, timerCtx[i - 1]); \
        impl_timerCaptureCompareHandler(TIM ## j, timerCtx[j - 1]); \
    } struct dummy

#define _TIM_IRQ_HANDLER(name, i)                                       \
    void name(void)                                                     \
    {                                                                   \
        impl_timerCaptureCompareHandler(TIM ## i, timerCtx[i - 1]); \
    } struct dummy

uint8_t lookupTimerIndex(const TIM_TypeDef *tim);
void impl_timerCaptureCompareHandler(TIM_TypeDef *tim, timHardwareContext_t * timerCtx);

#endif //end of else (stm32)


uint8_t lookupTimerIndex(const TIM_TypeDef *tim);

#ifdef AURIX
void impl_timerNVICConfigure(volatile Ifx_SRC_SRCR* src, int irqPriority);
void impl_enableTimer(const timerHardware_t *timHw, uint8_t channel);
uint16_t impl_timerGetPeriod(const timerHardware_t *timHw);
void impl_timerConfigBase(TIM_TypeDef *tim, uint16_t period, uint8_t mhz);
void impl_timerEnableIT(TIM_TypeDef * tim, uint32_t interrupt);
void impl_timerDisableIT(TIM_TypeDef * tim, uint32_t interrupt);
void impl_timerClearFlag(TIM_TypeDef * tim, uint32_t flag);
void impl_timerChConfigIC(const timerHardware_t *timHw, bool polarityRising, unsigned inputFilterTicks);
void impl_timerCaptureCompareHandler(TIM_TypeDef *tim, timerConfig_t *timerConfig);
void impl_timerPWMConfigChannel(TIM_TypeDef * tim, uint8_t channel, bool isNChannel, bool inverted, uint16_t value);
void impl_timerPWMStart(TIM_TypeDef * tim, unsigned channel, bool isNChannel);
uint16_t impl_timerDmaSource(uint8_t channel);
volatile timCCR_t * impl_timerCCR(TIM_TypeDef *tim, uint8_t channel);
#else
void impl_timerInitContext(timHardwareContext_t * timCtx);

volatile timCCR_t * impl_timerCCR(TCH_t * tch);
void impl_timerNVICConfigure(TCH_t * tch, int irqPriority);
void impl_timerConfigBase(TCH_t * tch, uint16_t period, uint32_t hz);
void impl_enableTimer(TCH_t * tch);
void impl_timerEnableIT(TCH_t * tch, uint32_t interrupt);
void impl_timerDisableIT(TCH_t * tch, uint32_t interrupt);
void impl_timerClearFlag(TCH_t * tch, uint32_t flag);
void impl_timerChConfigIC(TCH_t * tch, bool polarityRising, unsigned inputFilterTicks);
void impl_timerChCaptureCompareEnable(TCH_t * tch, bool enable);

void impl_timerPWMConfigChannel(TCH_t * tch, uint16_t value);
void impl_timerPWMStart(TCH_t * tch);
bool impl_timerPWMConfigChannelDMA(TCH_t * tch, void * dmaBuffer, uint8_t dmaBufferElementSize, uint32_t dmaBufferElementCount);
void impl_timerPWMPrepareDMA(TCH_t * tch, uint32_t dmaBufferElementCount);
void impl_timerPWMStartDMA(TCH_t * tch);
void impl_timerPWMStopDMA(TCH_t * tch);
#endif

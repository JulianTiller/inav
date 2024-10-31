	/*
  modified version of StdPeriph function is located here.
  TODO - what license does apply here?
  original file was lincesed under MCD-ST Liberty SW License Agreement V2
  http://www.st.com/software_license_agreement_liberty_v2
*/

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/atomic.h"

#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/rcc.h"
#include "drivers/time.h"
#include "drivers/nvic.h"
#include "drivers/timer.h"
#include "drivers/timer_impl.h"
#include "drivers/pwm_mapping.h"
#include "drivers/light_led.h"

static inline bool impl_timerIsTom(TIM_TypeDef * tim)
{
	return  (uint32_t)tim >= (uint32_t)&GTM_TOM0_CH0_CTRL ? true : false;
}

static inline Ifx_GTM_TOM_TGC* impl_timerGetTgcPointer(TIM_TypeDef * tim, uint8_t channel)
{
	Ifx_GTM_TOM_TGC* tgc;
	uint32 tgcIndex = channel < 8 ? 0 : 1;

#if defined(GTM_TOM2_CH0_CTRL)
	if ((uint32)tim >= (uint32)&GTM_TOM2_CH0_CTRL)
		tgc = IfxGtm_Tom_Ch_getTgcPointer(&MODULE_GTM.TOM[2], tgcIndex);
	else if ((uint32)tim >= (uint32)&GTM_TOM1_CH0_CTRL)
		tgc = IfxGtm_Tom_Ch_getTgcPointer(&MODULE_GTM.TOM[1], tgcIndex);
	else
		tgc = IfxGtm_Tom_Ch_getTgcPointer(&MODULE_GTM.TOM[0], tgcIndex);
#elif defined(GTM_TOM1_CH0_CTRL)
	if ((uint32)tim >= (uint32)&GTM_TOM1_CH0_CTRL)
		tgc = IfxGtm_Tom_Ch_getTgcPointer(&MODULE_GTM.TOM[1], tgcIndex);
	else
		tgc = IfxGtm_Tom_Ch_getTgcPointer(&MODULE_GTM.TOM[0], tgcIndex);
#endif

	return tgc;
}

volatile timCCR_t * impl_timerCCR(TIM_TypeDef *tim, uint8_t channel)
{
	return (volatile timCCR_t *)&((Ifx_GTM_TOM_CH*)tim)->CM1;

//	volatile timCCR_t *timCCRPointer = (volatile timCCR_t *)&((Ifx_GTM_TOM_CH *)tim)->CM1;
//	return timCCRPointer;
}

void impl_timerConfigBase(TIM_TypeDef *tim, uint16_t period, uint8_t mhz)
{
	if (impl_timerIsTom(tim))
	{
		uint32_t freq = mhz * 1000000 / period;

		uint32_t cr0 = 0;
		IfxGtm_Tom_Ch_ClkSrc clk = -1;

		do
		{
			clk++;
			cr0 = (SystemCoreClock >> (4*clk)) / freq;
		}while(cr0 > 0xFFFF);

		tim->tom.CTRL.B.CLK_SRC_SR = clk;
		tim->tom.SR0.U = cr0;
	}
	else //TIM
	{
		tim->tim.CTRL.B.CLK_SEL = IfxGtm_Cmu_Clk_0;
		tim->tim.TDUV.B.TCS = IfxGtm_Cmu_Clk_1;

		if (mhz == (PWM_TIMER_HZ/1000000.0f))
		{
			tim->tim.CTRL.B.TIM_MODE =  IfxGtm_Tim_Mode_pwmMeasurement;
			tim->tim.TDUV.B.TOV = 0xff;
			tim->tim.CTRL.B.TOCTRL = 0x3;
		}
		else
			tim->tim.CTRL.B.TIM_MODE = IfxGtm_Tim_Mode_inputEvent;
	}
}

void impl_enableTimer(const timerHardware_t *timHw, uint8_t channel)
{
	if (impl_timerIsTom(timHw->tim))
	{
        uint32 outselReg = (timHw->toutn >> 4);
        uint32 shift = (timHw->toutn & 0xFU) * 2;
        uint32 outsel = (uint32)timHw->select << shift;
        uint32 mask = 0x3U << shift;

        __ldmst_c(&(MODULE_GTM.TOUTSEL), mask, outsel);

		Ifx_GTM_TOM_TGC* tgc = impl_timerGetTgcPointer(timHw->tim, channel);

		IfxGtm_Tom_Tgc_setChannelForceUpdate(tgc, channel, TRUE, TRUE);
		IfxGtm_Tom_Tgc_trigger(tgc);
		IfxGtm_Tom_Tgc_enableChannel(tgc, channel, TRUE, FALSE);
		IfxGtm_Tom_Tgc_trigger(tgc);
	}
	else
	{
		timHw->tim->tim.CTRL.B.CNTS_SEL = IfxGtm_Tim_CntsSel_cntReg;
		timHw->tim->tim.CTRL.B.GPR0_SEL = IfxGtm_Tim_GprSel_cnts;
		timHw->tim->tim.CTRL.B.GPR1_SEL = IfxGtm_Tim_GprSel_cnts;

		uint32 shift = timHw->channel * 4;
#if defined(GTM_TIM3_CH0_GPR0)
		if ((int)timHw->tim >= (int)&GTM_TIM3_CH0_GPR0)
			__ldmst_c(&(MODULE_GTM.TIMINSEL[3].U), (0xFU << shift), ((uint32)timHw->select) << shift);
		else if ((int)timHw->tim >= (int)&GTM_TIM2_CH0_GPR0)
			__ldmst_c(&(MODULE_GTM.TIMINSEL[3].U), (0xFU << shift), ((uint32)timHw->select) << shift);
		else if ((int)timHw->tim >= (int)&GTM_TIM1_CH0_GPR0)
			__ldmst_c(&(MODULE_GTM.TIMINSEL[3].U), (0xFU << shift), ((uint32)timHw->select) << shift);
		else
			__ldmst_c(&(MODULE_GTM.TIMINSEL[0].U), (0xFU << shift), ((uint32)timHw->select) << shift);
#elif defined(GTM_TIM2_CH0_GPR0)
		if ((int)timHw->tim >= (int)&GTM_TIM2_CH0_GPR0)
			__ldmst_c(&(MODULE_GTM.INOUTSEL.TIM[2].INSEL.U), (0xFU << shift), ((uint32)timHw->select) << shift);
		else if ((int)timHw->tim >= (int)&GTM_TIM1_CH0_GPR0)
			__ldmst_c(&(MODULE_GTM.INOUTSEL.TIM[1].INSEL.U), (0xFU << shift), ((uint32)timHw->select) << shift);
		else
			__ldmst_c(&(MODULE_GTM.INOUTSEL.TIM[0].INSEL.U), (0xFU << shift), ((uint32)timHw->select) << shift);
#elif defined(GTM_TIM1_CH0_GPR0)
		if ((int)timHw->tim >= (int)&GTM_TIM1_CH0_GPR0)
			__ldmst_c(&(MODULE_GTM.INOUTSEL.TIM[1].INSEL.U), (0xFU << shift), ((uint32)timHw->select) << shift);
		else
			__ldmst_c(&(MODULE_GTM.INOUTSEL.TIM[0].INSEL.U), (0xFU << shift), ((uint32)timHw->select) << shift);
#else
		__ldmst_c(&(MODULE_GTM.INOUTSEL.TIM[0].INSEL.U), (0xFU << shift), ((uint32)timHw->select) << shift);
#endif
	}
}

void impl_timerChConfigIC(const timerHardware_t *timHw, bool polarityRising, unsigned inputFilterTicks)
{
	if (!impl_timerIsTom(timHw->tim))
	{
		timHw->tim->tim.CTRL.B.DSL = polarityRising ? 1 : 0;
		timHw->tim->tim.CTRL.B.ISL = 1;
		timHw->tim->tim.CTRL.B.TIM_EN = 1;
	}
}

void impl_timerPWMConfigChannel(TIM_TypeDef * tim, uint8_t channel, bool isNChannel, bool inverted, uint16_t value)
{
	if (impl_timerIsTom(tim))
	{
		tim->tom.CTRL.B.TRIGOUT = inverted ? Ifx_ActiveState_low : Ifx_ActiveState_high;
		tim->tom.SR1.U = 0;
	}
}

void impl_timerPWMStart(TIM_TypeDef * tim, unsigned channel, bool isNChannel)
{
	if (impl_timerIsTom(tim))
	{
		Ifx_GTM_TOM_TGC* tgc = impl_timerGetTgcPointer(tim, channel);

		IfxGtm_Tom_Tgc_enableChannelOutput(tgc, channel, TRUE, FALSE);
	}
}

uint16_t impl_timerGetPeriod(const timerHardware_t *timHw)
{
	if (impl_timerIsTom(timHw->tim))
		return timHw->tim->tom.CM0.U;
	else
		return 0;
}

void impl_timerNVICConfigure(volatile Ifx_SRC_SRCR* src, int irqPriority)
{
	IfxSrc_init(src, IfxSrc_Tos_cpu0, irqPriority);
	IfxSrc_enable(src);
}

void impl_timerDisableIT(TIM_TypeDef * tim, uint32_t interrupt)
{
	if (impl_timerIsTom(tim))
	{
		Ifx_GTM_TOM_CH_IRQ_EN en;
		en.U = tim->tom.IRQ.EN.U;

		if (interrupt & OVER_IRQ)
			en.B.CCU0TC_IRQ_EN = 0;

		tim->tom.IRQ.EN.U = 0;
		tim->tom.IRQ.MODE.B.IRQ_MODE = IfxGtm_IrqMode_pulseNotify;
		tim->tom.IRQ.EN.U = en.U;
	}
	else
	{
		Ifx_GTM_TIM_CH_IRQ_EN en;
		en.U =  tim->tim.IRQ.EN.U;

		if (interrupt & EDGE_IRQ)
			en.B.NEWVAL_IRQ_EN = 0;
		if (interrupt & OVER_IRQ)
			en.B.TODET_IRQ_EN = 0;

		tim->tim.IRQ.EN.U   = en.U;
	}
}

void impl_timerEnableIT(TIM_TypeDef * tim, uint32_t interrupt)
{
	if (impl_timerIsTom(tim))
	{
		Ifx_GTM_TOM_CH_IRQ_EN en;
		en.U = tim->tom.IRQ.EN.U;

		if (interrupt & OVER_IRQ)
			en.B.CCU0TC_IRQ_EN = 1;

		tim->tom.IRQ.EN.U = 0;
		tim->tom.IRQ.MODE.B.IRQ_MODE = IfxGtm_IrqMode_pulseNotify;
		tim->tom.IRQ.EN.U = en.U;
	}
	else
	{
		Ifx_GTM_TIM_CH_IRQ_EN en;
		en.U = tim->tim.IRQ.EN.U;

		tim->tim.IRQ.EN.U            = 0;
		tim->tim.IRQ.MODE.B.IRQ_MODE = IfxGtm_IrqMode_pulseNotify;

		if (interrupt & EDGE_IRQ)
			en.B.NEWVAL_IRQ_EN = 1;
		if (interrupt & OVER_IRQ)
			en.B.TODET_IRQ_EN = 1;

		tim->tim.IRQ.EN.U = en.U;
	}
}

void impl_timerCaptureCompareHandler(TIM_TypeDef *tim, timerConfig_t *timerConfig)
{
	if (impl_timerIsTom(tim))
	{
		if (tim->tom.IRQ.NOTIFY.B.CCU0TC == 1)
		{
			if (timerConfig)
			{
				if (timerConfig->overflowCallback[0])
					timerConfig->overflowCallback[0]->fn(timerConfig->overflowCallback[0], tim->tom.CN0.U);
			}

			tim->tom.IRQ.NOTIFY.U = 0x1 << IFX_GTM_TOM_CH_IRQ_NOTIFY_CCU0TC_OFF;
		}
	}
	else
	{
		if (tim->tim.IRQ.NOTIFY.B.NEWVAL == 1)
		{
			if (timerConfig)
			{
				if (timerConfig->edgeCallback[0])
					timerConfig->edgeCallback[0]->fn(timerConfig->edgeCallback[0], tim->tim.GPR1.B.GPR1, tim->tim.GPR0.B.GPR0);
			}

			tim->tim.IRQ.NOTIFY.U = 0x1 << IFX_GTM_TIM_CH_IRQ_NOTIFY_NEWVAL_OFF;
		}

		if (tim->tim.IRQ.NOTIFY.B.TODET == 1)
		{
			if (timerConfig)
			{
				if (timerConfig->overflowCallback[0])
					timerConfig->overflowCallback[0]->fn(timerConfig->overflowCallback[0], tim->tim.GPR0.B.GPR0);
			}

			tim->tim.IRQ.NOTIFY.U = 0x1 << IFX_GTM_TIM_CH_IRQ_NOTIFY_TODET_OFF;
		}
	}
}

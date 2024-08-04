/*
  modified version of StdPeriph function is located here.
  TODO - what license does apply here?
  original file was lincesed under MCD-ST Liberty SW License Agreement V2
  http://www.st.com/software_license_agreement_liberty_v2
*/

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/rcc.h"
#include "drivers/time.h"
#include "drivers/nvic.h"
#include "drivers/timer.h"
#include "drivers/timer_impl.h"


#define DEF_TIM(_tim, _ch) (TIM_TypeDef*)&GTM_TIM##_tim##_CH##_ch##_GPR0
#define DEF_TOM(_tom, _ch) (TIM_TypeDef*)&GTM_TOM##_tom##_CH##_ch##_CTRL

const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {
	[0] = { .tim = DEF_TIM(0,0), .src = &MODULE_SRC.GTM_TIM[0][0], .irqPrio = IFX_INTPRIO_TIM0_CH0 },
	[1] = { .tim = DEF_TIM(0,1), .src = &MODULE_SRC.GTM_TIM[0][1], .irqPrio = IFX_INTPRIO_TIM0_CH1 },
	[2] = { .tim = DEF_TIM(0,2), .src = &MODULE_SRC.GTM_TIM[0][2], .irqPrio = IFX_INTPRIO_TIM0_CH2 },
	[3] = { .tim = DEF_TIM(0,3), .src = &MODULE_SRC.GTM_TIM[0][3], .irqPrio = IFX_INTPRIO_TIM0_CH3 },
	[4] = { .tim = DEF_TIM(0,4), .src = &MODULE_SRC.GTM_TIM[0][4], .irqPrio = IFX_INTPRIO_TIM0_CH4 },
	[5] = { .tim = DEF_TIM(0,5), .src = &MODULE_SRC.GTM_TIM[0][5], .irqPrio = IFX_INTPRIO_TIM0_CH5 },
	[6] = { .tim = DEF_TIM(0,6), .src = &MODULE_SRC.GTM_TIM[0][6], .irqPrio = IFX_INTPRIO_TIM0_CH6 },
	[7] = { .tim = DEF_TIM(0,7), .src = &MODULE_SRC.GTM_TIM[0][7], .irqPrio = IFX_INTPRIO_TIM0_CH7 },
#ifdef GTM_TIM1_CH0_GPR0
	[8] = { .tim = DEF_TIM(1,0), .src = &MODULE_SRC.GTM_TIM[1][0], .irqPrio = IFX_INTPRIO_TIM1_CH0 },
	[9] = { .tim = DEF_TIM(1,1), .src = &MODULE_SRC.GTM_TIM[1][1], .irqPrio = IFX_INTPRIO_TIM1_CH1 },
	[10] = { .tim = DEF_TIM(1,2), .src = &MODULE_SRC.GTM_TIM[1][2], .irqPrio = IFX_INTPRIO_TIM1_CH2 },
	[11] = { .tim = DEF_TIM(1,3), .src = &MODULE_SRC.GTM_TIM[1][3], .irqPrio = IFX_INTPRIO_TIM1_CH3 },
	[12] = { .tim = DEF_TIM(1,4), .src = &MODULE_SRC.GTM_TIM[1][4], .irqPrio = IFX_INTPRIO_TIM1_CH4 },
	[13] = { .tim = DEF_TIM(1,5), .src = &MODULE_SRC.GTM_TIM[1][5], .irqPrio = IFX_INTPRIO_TIM1_CH5 },
	[14] = { .tim = DEF_TIM(1,6), .src = &MODULE_SRC.GTM_TIM[1][6], .irqPrio = IFX_INTPRIO_TIM1_CH6 },
	[15] = { .tim = DEF_TIM(1,7), .src = &MODULE_SRC.GTM_TIM[1][7], .irqPrio = IFX_INTPRIO_TIM1_CH7 },
#endif
#ifdef GTM_TIM2_CH0_GPR0
	[16] = { .tim = DEF_TIM(2,0), .src = &MODULE_SRC.GTM_TIM[2][0], .irqPrio = IFX_INTPRIO_TIM2_CH0 },
	[17] = { .tim = DEF_TIM(2,1), .src = &MODULE_SRC.GTM_TIM[2][1], .irqPrio = IFX_INTPRIO_TIM2_CH1 },
	[18] = { .tim = DEF_TIM(2,2), .src = &MODULE_SRC.GTM_TIM[2][2], .irqPrio = IFX_INTPRIO_TIM2_CH2 },
	[19] = { .tim = DEF_TIM(2,3), .src = &MODULE_SRC.GTM_TIM[2][3], .irqPrio = IFX_INTPRIO_TIM2_CH3 },
	[20] = { .tim = DEF_TIM(2,4), .src = &MODULE_SRC.GTM_TIM[2][4], .irqPrio = IFX_INTPRIO_TIM2_CH4 },
	[21] = { .tim = DEF_TIM(2,5), .src = &MODULE_SRC.GTM_TIM[2][5], .irqPrio = IFX_INTPRIO_TIM2_CH5 },
	[22] = { .tim = DEF_TIM(2,6), .src = &MODULE_SRC.GTM_TIM[2][6], .irqPrio = IFX_INTPRIO_TIM2_CH6 },
	[23] = { .tim = DEF_TIM(2,7), .src = &MODULE_SRC.GTM_TIM[2][7], .irqPrio = IFX_INTPRIO_TIM2_CH7 },
#endif
#ifdef GTM_TIM3_CH0_GPR0
	[24] = { .tim = DEF_TIM(3,0), .src = &MODULE_SRC.GTM_TIM[3][0], .irqPrio = IFX_INTPRIO_TIM3_CH0 },
	[25] = { .tim = DEF_TIM(3,1), .src = &MODULE_SRC.GTM_TIM[3][1], .irqPrio = IFX_INTPRIO_TIM3_CH1 },
	[26] = { .tim = DEF_TIM(3,2), .src = &MODULE_SRC.GTM_TIM[3][2], .irqPrio = IFX_INTPRIO_TIM3_CH2 },
	[27] = { .tim = DEF_TIM(3,3), .src = &MODULE_SRC.GTM_TIM[3][3], .irqPrio = IFX_INTPRIO_TIM3_CH3 },
	[28] = { .tim = DEF_TIM(3,4), .src = &MODULE_SRC.GTM_TIM[3][4], .irqPrio = IFX_INTPRIO_TIM3_CH4 },
	[29] = { .tim = DEF_TIM(3,5), .src = &MODULE_SRC.GTM_TIM[3][5], .irqPrio = IFX_INTPRIO_TIM3_CH5 },
	[30] = { .tim = DEF_TIM(3,6), .src = &MODULE_SRC.GTM_TIM[3][6], .irqPrio = IFX_INTPRIO_TIM3_CH6 },
	[31] = { .tim = DEF_TIM(3,7), .src = &MODULE_SRC.GTM_TIM[3][7], .irqPrio = IFX_INTPRIO_TIM3_CH7 },
#endif
	[32] = { .tim = DEF_TOM(0,0), .src = &MODULE_SRC.GTM_TOM[0][0], .irqPrio = IFX_INTPRIO_TOM0_CH0_1 },
	[33] = { .tim = DEF_TOM(0,1), .src = &MODULE_SRC.GTM_TOM[0][0], .irqPrio = IFX_INTPRIO_TOM0_CH0_1 },
	[34] = { .tim = DEF_TOM(0,2), .src = &MODULE_SRC.GTM_TOM[0][1], .irqPrio = IFX_INTPRIO_TOM0_CH2_3 },
	[35] = { .tim = DEF_TOM(0,3), .src = &MODULE_SRC.GTM_TOM[0][1], .irqPrio = IFX_INTPRIO_TOM0_CH2_3 },
	[36] = { .tim = DEF_TOM(0,4), .src = &MODULE_SRC.GTM_TOM[0][2], .irqPrio = IFX_INTPRIO_TOM0_CH4_5 },
	[37] = { .tim = DEF_TOM(0,5), .src = &MODULE_SRC.GTM_TOM[0][2], .irqPrio = IFX_INTPRIO_TOM0_CH4_5 },
	[38] = { .tim = DEF_TOM(0,6), .src = &MODULE_SRC.GTM_TOM[0][3], .irqPrio = IFX_INTPRIO_TOM0_CH6_7 },
	[39] = { .tim = DEF_TOM(0,7), .src = &MODULE_SRC.GTM_TOM[0][3], .irqPrio = IFX_INTPRIO_TOM0_CH6_7 },
	[40] = { .tim = DEF_TOM(0,8), .src = &MODULE_SRC.GTM_TOM[0][4], .irqPrio = IFX_INTPRIO_TOM0_CH8_9 },
	[41] = { .tim = DEF_TOM(0,9), .src = &MODULE_SRC.GTM_TOM[0][4], .irqPrio = IFX_INTPRIO_TOM0_CH8_9 },
	[42] = { .tim = DEF_TOM(0,10), .src = &MODULE_SRC.GTM_TOM[0][5], .irqPrio = IFX_INTPRIO_TOM0_CH10_11 },
	[43] = { .tim = DEF_TOM(0,11), .src = &MODULE_SRC.GTM_TOM[0][5], .irqPrio = IFX_INTPRIO_TOM0_CH10_11 },
	[44] = { .tim = DEF_TOM(0,12), .src = &MODULE_SRC.GTM_TOM[0][6], .irqPrio = IFX_INTPRIO_TOM0_CH12_13 },
	[45] = { .tim = DEF_TOM(0,13), .src = &MODULE_SRC.GTM_TOM[0][6], .irqPrio = IFX_INTPRIO_TOM0_CH12_13 },
	[46] = { .tim = DEF_TOM(0,14), .src = &MODULE_SRC.GTM_TOM[0][7], .irqPrio = IFX_INTPRIO_TOM0_CH14_15 },
	[47] = { .tim = DEF_TOM(0,15), .src = &MODULE_SRC.GTM_TOM[0][7], .irqPrio = IFX_INTPRIO_TOM0_CH14_15 },
	[48] = { .tim = DEF_TOM(1,0), .src = &MODULE_SRC.GTM_TOM[1][0], .irqPrio = IFX_INTPRIO_TOM1_CH0_1 },
	[49] = { .tim = DEF_TOM(1,1), .src = &MODULE_SRC.GTM_TOM[1][0], .irqPrio = IFX_INTPRIO_TOM1_CH0_1 },
	[50] = { .tim = DEF_TOM(1,2), .src = &MODULE_SRC.GTM_TOM[1][1], .irqPrio = IFX_INTPRIO_TOM1_CH2_3 },
	[51] = { .tim = DEF_TOM(1,3), .src = &MODULE_SRC.GTM_TOM[1][1], .irqPrio = IFX_INTPRIO_TOM1_CH2_3 },
	[52] = { .tim = DEF_TOM(1,4), .src = &MODULE_SRC.GTM_TOM[1][2], .irqPrio = IFX_INTPRIO_TOM1_CH4_5 },
	[53] = { .tim = DEF_TOM(1,5), .src = &MODULE_SRC.GTM_TOM[1][2], .irqPrio = IFX_INTPRIO_TOM1_CH4_5 },
	[54] = { .tim = DEF_TOM(1,6), .src = &MODULE_SRC.GTM_TOM[1][3], .irqPrio = IFX_INTPRIO_TOM1_CH6_7 },
	[55] = { .tim = DEF_TOM(1,7), .src = &MODULE_SRC.GTM_TOM[1][3], .irqPrio = IFX_INTPRIO_TOM1_CH6_7 },
	[56] = { .tim = DEF_TOM(1,8), .src = &MODULE_SRC.GTM_TOM[1][4], .irqPrio = IFX_INTPRIO_TOM1_CH8_9 },
	[57] = { .tim = DEF_TOM(1,9), .src = &MODULE_SRC.GTM_TOM[1][4], .irqPrio = IFX_INTPRIO_TOM1_CH8_9 },
	[58] = { .tim = DEF_TOM(1,10), .src = &MODULE_SRC.GTM_TOM[1][5], .irqPrio = IFX_INTPRIO_TOM1_CH10_11 },
	[59] = { .tim = DEF_TOM(1,11), .src = &MODULE_SRC.GTM_TOM[1][5], .irqPrio = IFX_INTPRIO_TOM1_CH10_11 },
	[60] = { .tim = DEF_TOM(1,12), .src = &MODULE_SRC.GTM_TOM[1][6], .irqPrio = IFX_INTPRIO_TOM1_CH12_13 },
	[61] = { .tim = DEF_TOM(1,13), .src = &MODULE_SRC.GTM_TOM[1][6], .irqPrio = IFX_INTPRIO_TOM1_CH12_13 },
	[62] = { .tim = DEF_TOM(1,14), .src = &MODULE_SRC.GTM_TOM[1][7], .irqPrio = IFX_INTPRIO_TOM1_CH14_15 },
	[63] = { .tim = DEF_TOM(1,15), .src = &MODULE_SRC.GTM_TOM[1][7], .irqPrio = IFX_INTPRIO_TOM1_CH14_15 },
};

uint8_t timerClockDivisor(TIM_TypeDef *tim)
{
    UNUSED(tim);
    return 1;
}


uint32_t timerClock(TIM_TypeDef *tim)
{
    UNUSED(tim);
    return SystemCoreClock;
}

_TIM_IRQ_HANDLER(0, 0, 0);
_TIM_IRQ_HANDLER(0, 1, 1);
_TIM_IRQ_HANDLER(0, 2, 2);
_TIM_IRQ_HANDLER(0, 3, 3);
_TIM_IRQ_HANDLER(0, 4, 4);
_TIM_IRQ_HANDLER(0, 5, 5);
_TIM_IRQ_HANDLER(0, 6, 6);
_TIM_IRQ_HANDLER(0, 7, 7);

#ifdef GTM_TIM1_CH0_GPR0
_TIM_IRQ_HANDLER(1, 0, 8);
_TIM_IRQ_HANDLER(1, 1, 9);
_TIM_IRQ_HANDLER(1, 2, 10);
_TIM_IRQ_HANDLER(1, 3, 11);
_TIM_IRQ_HANDLER(1, 4, 12);
_TIM_IRQ_HANDLER(1, 5, 13);
_TIM_IRQ_HANDLER(1, 6, 14);
_TIM_IRQ_HANDLER(1, 7, 15);
#endif

#ifdef GTM_TIM2_CH0_GPR0
_TIM_IRQ_HANDLER(2, 0, 16);
_TIM_IRQ_HANDLER(2, 1, 17);
_TIM_IRQ_HANDLER(2, 2, 18);
_TIM_IRQ_HANDLER(2, 3, 19);
_TIM_IRQ_HANDLER(2, 4, 20);
_TIM_IRQ_HANDLER(2, 5, 21);
_TIM_IRQ_HANDLER(2, 6, 22);
_TIM_IRQ_HANDLER(2, 7, 23);
#endif

#ifdef GTM_TIM3_CH0_GPR0
_TIM_IRQ_HANDLER(3, 0, 24);
_TIM_IRQ_HANDLER(3, 1, 25);
_TIM_IRQ_HANDLER(3, 2, 26);
_TIM_IRQ_HANDLER(3, 3, 27);
_TIM_IRQ_HANDLER(3, 4, 28);
_TIM_IRQ_HANDLER(3, 5, 29);
_TIM_IRQ_HANDLER(3, 6, 30);
_TIM_IRQ_HANDLER(3, 7, 31);
#endif
/*
_TOM_IRQ_HANDLER(0, 0, 1, 32, 33);
_TOM_IRQ_HANDLER(0, 2, 3, 34, 35);
_TOM_IRQ_HANDLER(0, 4, 5, 36, 37);
_TOM_IRQ_HANDLER(0, 6, 7, 38, 39);
_TOM_IRQ_HANDLER(0, 8, 9, 40, 41);
_TOM_IRQ_HANDLER(0, 10, 11, 42, 43);
_TOM_IRQ_HANDLER(0, 12, 13, 44, 45);
_TOM_IRQ_HANDLER(0, 14, 15, 46, 47);

_TOM_IRQ_HANDLER(1, 0, 1, 48, 49);
_TOM_IRQ_HANDLER(1, 2, 3, 50, 51);
_TOM_IRQ_HANDLER(1, 4, 5, 52, 53);
_TOM_IRQ_HANDLER(1, 6, 7, 54, 55);
_TOM_IRQ_HANDLER(1, 8, 9, 56, 57);
_TOM_IRQ_HANDLER(1, 10, 11, 58, 59);
_TOM_IRQ_HANDLER(1, 12, 13, 60, 61);
_TOM_IRQ_HANDLER(1, 14, 15, 62, 63);
*/

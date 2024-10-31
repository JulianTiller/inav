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
#include "drivers/time.h"

#include "drivers/io.h"
#include "io_impl.h"
#include "rcc.h"

#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"

#include "adc.h"
#include "adc_impl.h"

static adcDevice_t adcHardware[ADCDEV_COUNT] = {
    { .ADCx = ADC1, .enabled = false, .usedChannelCount = 0 },
};

/* note these could be packed up for saving space */
const adcTagMap_t adcTagMap[] = {
	{ DEFIO_TAG_E__PAN0, 0, 0 },
	{ DEFIO_TAG_E__PAN1, 0, 1 },
	{ DEFIO_TAG_E__PAN2, 0, 2 },
	{ DEFIO_TAG_E__PAN3, 0, 3 },
	{ DEFIO_TAG_E__PAN4, 0, 4 },
	{ DEFIO_TAG_E__PAN5, 0, 5 },
	{ DEFIO_TAG_E__PAN6, 0, 6 },
	{ DEFIO_TAG_E__PAN7, 0, 7 },
	{ DEFIO_TAG_E__P409, 4, 7 },
	{ DEFIO_TAG_E__P408, 4, 6 },
};

ADCDevice adcDeviceByInstance(ADC_TypeDef *instance)
{
    if (instance == ADC1)
        return ADCDEV_1;

    return ADCINVALID;
}

static void adcInstanceInit(ADCDevice adcDevice)
{
	IfxEvadc_Adc_Config ifxAdcConfig;
	IfxEvadc_Adc_GroupConfig ifxAdcGroupConfig;
	IfxEvadc_Adc_ChannelConfig ifxAdcChannelConfig;

    adcDevice_t * adc = &adcHardware[adcDevice];

    IfxEvadc_Adc_initModuleConfig(&ifxAdcConfig, adc->ADCx);
    IfxEvadc_Adc_initModule(&adc->ifxAdc, &ifxAdcConfig);

    for (int i = ADC_CHN_1; i < ADC_CHN_COUNT; i++) {
        if (!adcConfig[i].enabled || adcConfig[i].adcDevice != adcDevice) {
            continue;
        }

		IfxEvadc_Adc_initGroupConfig(&ifxAdcGroupConfig, &adc->ifxAdc);

		ifxAdcGroupConfig.groupId = adcGroupByTag(adcConfig[i].tag);
		ifxAdcGroupConfig.master = ifxAdcGroupConfig.groupId;

		// enable all arbiter request sources
		ifxAdcGroupConfig.arbiter.requestSlotQueue0Enabled           = TRUE; // enable Queue0 mode

		// enable all gates in "always" mode (no edge detection)
		ifxAdcGroupConfig.queueRequest[0].triggerConfig.gatingMode = IfxEvadc_GatingMode_always;

		/* Initialize the group */
		IfxEvadc_Adc_initGroup(&adcConfig[i].ifxAdcGroup, &ifxAdcGroupConfig);
	    /* Initialize the configuration with default values */
		IfxEvadc_Adc_initChannelConfig(&ifxAdcChannelConfig, &adcConfig[i].ifxAdcGroup);

		adcConfig[i].adcChannel = adcChannelByTag(adcConfig[i].tag);
		adcConfig[i].enabled = true;

	    /* Select the channel ID and the respective result register */
		ifxAdcChannelConfig.channelId = (IfxEvadc_ChannelId)adcConfig[i].adcChannel;
		ifxAdcChannelConfig.resultRegister = (IfxEvadc_ChannelResult)adcConfig[i].adcChannel;
	    /* Initialize the channel */
		IfxEvadc_Adc_initChannel(&adcConfig[i].ifxAdcChannel, &ifxAdcChannelConfig);

	    /* Add channel to queue with refill option enabled */
	    IfxEvadc_Adc_addToQueue(&adcConfig[i].ifxAdcChannel, IfxEvadc_RequestSource_queue0, IFXEVADC_QUEUE_REFILL);

	    /* Start the queue */
//	    IfxEvadc_Adc_startQueue(&adcConfig[i].ifxAdcChannel, IfxEvadc_RequestSource_queue0);
	    IfxEvadc_Adc_startQueue(&adcConfig[i].ifxAdcGroup, IfxEvadc_RequestSource_queue0);
    }

}

void adcHardwareInit(drv_adc_config_t *init)
{
    UNUSED(init);
    int configuredAdcChannels = 0;

    for (int i = ADC_CHN_1; i < ADC_CHN_COUNT; i++) {
        if (!adcConfig[i].tag)
            continue;

        adcDevice_t * adc = &adcHardware[adcConfig[i].adcDevice];

        IOInit(IOGetByTag(adcConfig[i].tag), OWNER_ADC, RESOURCE_ADC_CH1 + (i - ADC_CHN_1), 0);

        adcConfig[i].adcChannel = adcChannelByTag(adcConfig[i].tag);
        adcConfig[i].dmaIndex = adc->usedChannelCount++;
        adcConfig[i].enabled = true;

        adc->enabled = true;
        configuredAdcChannels++;
    }

    if (configuredAdcChannels == 0)
        return;

    for (int i = 0; i < ADCDEV_COUNT; i++) {
        if (adcHardware[i].enabled) {
            adcInstanceInit(i);
        }
    }
}

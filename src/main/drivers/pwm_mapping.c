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

#if !defined(SITL_BUILD)

#include "build/debug.h"
#include "common/log.h"
#include "common/memory.h"

#include "config/feature.h"

#include "fc/config.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/timer.h"
#include "drivers/pwm_output.h"
#include "drivers/pwm_mapping.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#ifdef AURIX
//#include "drivers/rx_pwm.h"
#endif
#include "sensors/rangefinder.h"

#include "io/serial.h"
#include "io/servo_sbus.h"
#ifdef AURIX //TODO outdated code duplication
enum {
    MAP_TO_NONE,
    MAP_TO_PPM_INPUT,
    MAP_TO_PWM_INPUT,
    MAP_TO_MOTOR_OUTPUT,
    MAP_TO_SERVO_OUTPUT,
};

static const char * pwmInitErrorMsg[] = {
    /* PWM_INIT_ERROR_NONE */                     "No error",
    /* PWM_INIT_ERROR_TOO_MANY_MOTORS */          "Mixer defines too many motors",
    /* PWM_INIT_ERROR_TOO_MANY_SERVOS */          "Mixer defines too many servos",
    /* PWM_INIT_ERROR_NOT_ENOUGH_MOTOR_OUTPUTS */ "Not enough motor outputs/timers",
    /* PWM_INIT_ERROR_NOT_ENOUGH_SERVO_OUTPUTS */ "Not enough servo outputs/timers",
    /* PWM_INIT_ERROR_TIMER_INIT_FAILED */        "Output timer init failed"
};

static pwmIOConfiguration_t pwmIOConfiguration;

static pwmInitError_e pwmInitError = PWM_INIT_ERROR_NONE;

pwmIOConfiguration_t *pwmGetOutputConfiguration(void)
{
    return &pwmIOConfiguration;
}

bool CheckGPIOPin(ioTag_t tag, GPIO_TypeDef *gpio, uint16_t pin)
{
    return IO_GPIOBYTAG(tag) == gpio && IO_PINBYTAG(tag) == pin;
}

bool CheckGPIOPinSource(ioTag_t tag, GPIO_TypeDef *gpio, uint16_t pin)
{
    return IO_GPIOBYTAG(tag) == gpio && IO_GPIO_PinSource(IOGetByTag(tag)) == pin;
}

pwmIOConfiguration_t *pwmInit(drv_pwm_config_t *init)
{
    memset(&pwmIOConfiguration, 0, sizeof(pwmIOConfiguration));
    pwmIOConfiguration.ioConfigurations = memAllocate(sizeof(pwmPortConfiguration_t) * timerHardwareCount, OWNER_PWMIO);

#if defined(USE_RX_PWM) || defined(USE_RX_PPM)
    int channelIndex = 0;
#endif

    for (int timerIndex = 0; timerIndex < timerHardwareCount; timerIndex++) {
        const timerHardware_t *timerHardwarePtr = &timerHardware[timerIndex];
        int type = MAP_TO_NONE;

#if defined(UART6_TX_PIN) || defined(UART6_RX_PIN)
        if (init->useUART6 && (timerHardwarePtr->tag == IO_TAG(UART6_TX_PIN) || timerHardwarePtr->tag == IO_TAG(UART6_RX_PIN))) {
            addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
            continue;
        }
#endif

#if defined(USE_SOFTSERIAL1)
        const timerHardware_t *ss1TimerHardware = timerGetByTag(IO_TAG(SOFTSERIAL_1_RX_PIN), TIM_USE_ANY);
        if (init->useSoftSerial && ss1TimerHardware != NULL && ss1TimerHardware->tim == timerHardwarePtr->tim) {
//            addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
            continue;
        }
#endif

#if defined(USE_SOFTSERIAL2)
        const timerHardware_t *ss2TimerHardware = timerGetByTag(IO_TAG(SOFTSERIAL_2_RX_PIN), TIM_USE_ANY);
        if (init->useSoftSerial && ss2TimerHardware != NULL && ss2TimerHardware->tim == timerHardwarePtr->tim) {
//            addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
            continue;
        }
#endif

#ifdef WS2811_TIMER
        // skip LED Strip output
        if (init->useLEDStrip) {
            if (timerHardwarePtr->tim == WS2811_TIMER) {
//                addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
                continue;
            }
#if defined(STM32F303xC) && defined(WS2811_PIN)
            if (timerHardwarePtr->tag == IO_TAG(WS2811_PIN)) {
//                addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
                continue;
            }
#endif
        }

#endif

#ifdef VBAT_ADC_PIN
        if (init->useVbat && timerHardwarePtr->tag == IO_TAG(VBAT_ADC_PIN)) {
//            addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
            continue;
        }
#endif

#ifdef RSSI_ADC_PIN
        if (init->useRSSIADC && timerHardwarePtr->tag == IO_TAG(RSSI_ADC_PIN)) {
//            addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
            continue;
        }
#endif

#ifdef CURRENT_METER_ADC_PIN
        if (init->useCurrentMeterADC && timerHardwarePtr->tag == IO_TAG(CURRENT_METER_ADC_PIN)) {
//            addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
            continue;
        }
#endif

#ifdef USE_RANGEFINDER_HCSR04
        if (init->useTriggerRangefinder &&
            (
                timerHardwarePtr->tag == init->rangefinderIOConfig.triggerTag ||
                timerHardwarePtr->tag == init->rangefinderIOConfig.echoTag
            )) {
//            addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
            continue;
        }
#endif

        // Handle timer mapping to PWM/PPM inputs
        if (init->useSerialRx) {
            type = MAP_TO_NONE;
        }
        else if (init->useParallelPWM && (timerHardwarePtr->usageFlags & TIM_USE_PWM)) {
            type = MAP_TO_PWM_INPUT;
        }
        else if (init->usePPM && (timerHardwarePtr->usageFlags & TIM_USE_PPM)) {
            type = MAP_TO_PPM_INPUT;
        }

        // Handle outputs - may override the PWM/PPM inputs
        if (init->flyingPlatformType == PLATFORM_MULTIROTOR || init->flyingPlatformType == PLATFORM_TRICOPTER) {
            // Multicopter
            if (init->useServoOutputs && (timerHardwarePtr->usageFlags & TIM_USE_MC_SERVO)) {
                type = MAP_TO_SERVO_OUTPUT;
            }
            else
            if (timerHardwarePtr->usageFlags & TIM_USE_MC_MOTOR) {
                type = MAP_TO_MOTOR_OUTPUT;
            }
        } else {
            // Fixed wing or HELI (one/two motors and a lot of servos
            if (timerHardwarePtr->usageFlags & TIM_USE_FW_SERVO) {
                type = MAP_TO_SERVO_OUTPUT;
            }
            else if (timerHardwarePtr->usageFlags & TIM_USE_FW_MOTOR) {
                type = MAP_TO_MOTOR_OUTPUT;
            }
        }

        // If timer not mapped - skip
        if (type == MAP_TO_NONE)
            continue;

        if (type == MAP_TO_PPM_INPUT) {
#if defined(USE_RX_PPM)
//            ppmInConfig(timerHardwarePtr, init->pwmProtocolType);
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_PPM;
            pwmIOConfiguration.ppmInputCount++;

//            addBootlogEvent6(BOOT_EVENT_TIMER_CH_MAPPED, BOOT_EVENT_FLAGS_NONE, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 0);
#endif
        } else if (type == MAP_TO_PWM_INPUT) {
#if defined(USE_RX_PWM)
//            pwmInConfig(timerHardwarePtr, channelIndex);
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_PWM;
            pwmIOConfiguration.pwmInputCount++;
            channelIndex++;

//            addBootlogEvent6(BOOT_EVENT_TIMER_CH_MAPPED, BOOT_EVENT_FLAGS_NONE, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 1);
#endif
        } else if (type == MAP_TO_MOTOR_OUTPUT) {
            /* Check if we already configured maximum supported number of motors and skip the rest */
            if (pwmIOConfiguration.motorCount >= MAX_MOTORS) {
//                addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 2);
                continue;
            }

            if (pwmMotorConfig(timerHardwarePtr, pwmIOConfiguration.motorCount, init->motorPwmRate, init->idlePulse, init->pwmProtocolType, init->enablePWMOutput)) {
                if (init->useFastPwm) {
                    pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_MOTOR | PWM_PF_OUTPUT_PROTOCOL_FASTPWM | PWM_PF_OUTPUT_PROTOCOL_PWM;
                } else if (init->pwmProtocolType == PWM_TYPE_BRUSHED) {
                    pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_MOTOR | PWM_PF_MOTOR_MODE_BRUSHED | PWM_PF_OUTPUT_PROTOCOL_PWM;
                } else {
                    pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_MOTOR | PWM_PF_OUTPUT_PROTOCOL_PWM ;
                }

                pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].index = pwmIOConfiguration.motorCount;
                pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].timerHardware = timerHardwarePtr;

                pwmIOConfiguration.motorCount++;

//                addBootlogEvent6(BOOT_EVENT_TIMER_CH_MAPPED, BOOT_EVENT_FLAGS_NONE, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 2);
            }
            else {
//                addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 2);
                continue;
            }
        } else if (type == MAP_TO_SERVO_OUTPUT) {
            if (pwmIOConfiguration.servoCount >=  MAX_SERVOS) {
//                addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
                continue;
            }

            if (pwmServoConfig(timerHardwarePtr, pwmIOConfiguration.servoCount, init->servoPwmRate, init->servoCenterPulse, init->enablePWMOutput)) {
                pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_SERVO | PWM_PF_OUTPUT_PROTOCOL_PWM;
                pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].index = pwmIOConfiguration.servoCount;
                pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].timerHardware = timerHardwarePtr;

                pwmIOConfiguration.servoCount++;

//                addBootlogEvent6(BOOT_EVENT_TIMER_CH_MAPPED, BOOT_EVENT_FLAGS_NONE, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
            }
            else {
//                addBootlogEvent6(BOOT_EVENT_TIMER_CH_SKIPPED, BOOT_EVENT_FLAGS_WARNING, timerIndex, pwmIOConfiguration.motorCount, pwmIOConfiguration.servoCount, 3);
                continue;
            }
        } else {
            continue;
        }

        pwmIOConfiguration.ioCount++;
    }

    return &pwmIOConfiguration;
}

pwmInitError_e getPwmInitError(void)
{
    return pwmInitError;
}

const char * getPwmInitErrorMessage(void)
{
    return pwmInitErrorMsg[pwmInitError];
}

#else //AURIX
enum {
    MAP_TO_NONE,
    MAP_TO_MOTOR_OUTPUT,
    MAP_TO_SERVO_OUTPUT,
    MAP_TO_LED_OUTPUT
};

typedef struct {
    int maxTimMotorCount;
    int maxTimServoCount;
    const timerHardware_t * timMotors[MAX_PWM_OUTPUTS];
    const timerHardware_t * timServos[MAX_PWM_OUTPUTS];
} timMotorServoHardware_t;

static pwmInitError_e pwmInitError = PWM_INIT_ERROR_NONE;

static const char * pwmInitErrorMsg[] = {
    /* PWM_INIT_ERROR_NONE */                     "No error",
    /* PWM_INIT_ERROR_TOO_MANY_MOTORS */          "Mixer defines too many motors",
    /* PWM_INIT_ERROR_TOO_MANY_SERVOS */          "Mixer defines too many servos",
    /* PWM_INIT_ERROR_NOT_ENOUGH_MOTOR_OUTPUTS */ "Not enough motor outputs/timers",
    /* PWM_INIT_ERROR_NOT_ENOUGH_SERVO_OUTPUTS */ "Not enough servo outputs/timers",
    /* PWM_INIT_ERROR_TIMER_INIT_FAILED */        "Output timer init failed"
};

static const motorProtocolProperties_t motorProtocolProperties[] = {
    [PWM_TYPE_STANDARD]     = { .usesHwTimer = true,    .isDSHOT = false },
    [PWM_TYPE_ONESHOT125]   = { .usesHwTimer = true,    .isDSHOT = false },
    [PWM_TYPE_MULTISHOT]    = { .usesHwTimer = true,    .isDSHOT = false },
    [PWM_TYPE_BRUSHED]      = { .usesHwTimer = true,    .isDSHOT = false },
    [PWM_TYPE_DSHOT150]     = { .usesHwTimer = true,    .isDSHOT = true },
    [PWM_TYPE_DSHOT300]     = { .usesHwTimer = true,    .isDSHOT = true },
    [PWM_TYPE_DSHOT600]     = { .usesHwTimer = true,    .isDSHOT = true },
};

pwmInitError_e getPwmInitError(void)
{
    return pwmInitError;
}

const char * getPwmInitErrorMessage(void)
{
    return pwmInitErrorMsg[pwmInitError];
}

const motorProtocolProperties_t * getMotorProtocolProperties(motorPwmProtocolTypes_e proto)
{
    return &motorProtocolProperties[proto];
}

static bool checkPwmTimerConflicts(const timerHardware_t *timHw)
{
    serialPortPins_t uartPins;

#if defined(USE_UART2)
    uartGetPortPins(UARTDEV_2, &uartPins);
    if (doesConfigurationUsePort(SERIAL_PORT_USART2) && (timHw->tag == uartPins.txPin || timHw->tag == uartPins.rxPin)) {
        return true;
    }
#endif

#if defined(USE_UART3)
    uartGetPortPins(UARTDEV_3, &uartPins);
    if (doesConfigurationUsePort(SERIAL_PORT_USART3) && (timHw->tag == uartPins.txPin || timHw->tag == uartPins.rxPin)) {
        return true;
    }
#endif

#if defined(USE_UART4)
    uartGetPortPins(UARTDEV_4, &uartPins);
    if (doesConfigurationUsePort(SERIAL_PORT_USART4) && (timHw->tag == uartPins.txPin || timHw->tag == uartPins.rxPin)) {
        return true;
    }
#endif

#if defined(USE_UART5)
    uartGetPortPins(UARTDEV_5, &uartPins);
    if (doesConfigurationUsePort(SERIAL_PORT_USART5) && (timHw->tag == uartPins.txPin || timHw->tag == uartPins.rxPin)) {
        return true;
    }
#endif

#if defined(USE_UART6)
    uartGetPortPins(UARTDEV_6, &uartPins);
    if (doesConfigurationUsePort(SERIAL_PORT_USART6) && (timHw->tag == uartPins.txPin || timHw->tag == uartPins.rxPin)) {
        return true;
    }
#endif

#if defined(USE_UART7)
    uartGetPortPins(UARTDEV_7, &uartPins);
    if (doesConfigurationUsePort(SERIAL_PORT_USART7) && (timHw->tag == uartPins.txPin || timHw->tag == uartPins.rxPin)) {
        return true;
    }
#endif

#if defined(USE_UART8)
    uartGetPortPins(UARTDEV_8, &uartPins);
    if (doesConfigurationUsePort(SERIAL_PORT_USART8) && (timHw->tag == uartPins.txPin || timHw->tag == uartPins.rxPin)) {
        return true;
    }
#endif

#if defined(USE_SOFTSERIAL1)
    if (feature(FEATURE_SOFTSERIAL)) {
        const timerHardware_t *ssrx = timerGetByTag(IO_TAG(SOFTSERIAL_1_RX_PIN), TIM_USE_ANY);
        const timerHardware_t *sstx = timerGetByTag(IO_TAG(SOFTSERIAL_1_TX_PIN), TIM_USE_ANY);
        if ((ssrx != NULL && ssrx->tim == timHw->tim) || (sstx != NULL && sstx->tim == timHw->tim)) {
            return true;
        }
    }
#endif

#if defined(USE_SOFTSERIAL2)
    if (feature(FEATURE_SOFTSERIAL)) {
        const timerHardware_t *ssrx = timerGetByTag(IO_TAG(SOFTSERIAL_2_RX_PIN), TIM_USE_ANY);
        const timerHardware_t *sstx = timerGetByTag(IO_TAG(SOFTSERIAL_2_TX_PIN), TIM_USE_ANY);
        if ((ssrx != NULL && ssrx->tim == timHw->tim) || (sstx != NULL && sstx->tim == timHw->tim)) {
            return true;
        }
    }
#endif

#if defined(USE_LED_STRIP)
    if (feature(FEATURE_LED_STRIP)) {
        for (int i = 0; i < timerHardwareCount; i++) {
            if (timHw->tim == timerHardware[i].tim && timerHardware[i].usageFlags & TIM_USE_LED) {
				return true;
            }
        }

        //const timerHardware_t * ledTimHw = timerGetByTag(IO_TAG(WS2811_PIN), TIM_USE_ANY);
        //if (ledTimHw != NULL && timHw->tim == ledTimHw->tim) {
        //    return true;
        //}
    }
#endif

#if defined(USE_ADC)
#if defined(ADC_CHANNEL_1_PIN)
    if (timHw->tag == IO_TAG(ADC_CHANNEL_1_PIN)) {
        return true;
    }
#endif
#if defined(ADC_CHANNEL_2_PIN)
    if (timHw->tag == IO_TAG(ADC_CHANNEL_2_PIN)) {
        return true;
    }
#endif
#if defined(ADC_CHANNEL_3_PIN)
    if (timHw->tag == IO_TAG(ADC_CHANNEL_3_PIN)) {
        return true;
    }
#endif
#if defined(ADC_CHANNEL_4_PIN)
    if (timHw->tag == IO_TAG(ADC_CHANNEL_4_PIN)) {
        return true;
    }
#endif
#if defined(ADC_CHANNEL_5_PIN)
    if (timHw->tag == IO_TAG(ADC_CHANNEL_5_PIN)) {
        return true;
    }
#endif
#if defined(ADC_CHANNEL_6_PIN)
    if (timHw->tag == IO_TAG(ADC_CHANNEL_6_PIN)) {
        return true;
    }
#endif
#endif

    return false;
}

static void timerHardwareOverride(timerHardware_t * timer) {
    if (currentMixerConfig.outputMode == OUTPUT_MODE_SERVOS) {
        
        //Motors are rewritten as servos
        if (timer->usageFlags & (TIM_USE_MC_MOTOR|TIM_USE_FW_MOTOR)) {
            timer->usageFlags &= ~(TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR);
            timer->usageFlags |= TIM_USE_MC_SERVO | TIM_USE_FW_SERVO;
        }
        
    } else if (currentMixerConfig.outputMode == OUTPUT_MODE_MOTORS) {
        
        // Servos are rewritten as motors
        if (timer->usageFlags & (TIM_USE_MC_SERVO | TIM_USE_FW_SERVO)) {
            timer->usageFlags &= ~(TIM_USE_MC_SERVO | TIM_USE_FW_SERVO);
            timer->usageFlags |= TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR;
        }
    }

    switch (timerOverrides(timer2id(timer->tim))->outputMode) {
        case OUTPUT_MODE_MOTORS:
            timer->usageFlags &= ~(TIM_USE_SERVO|TIM_USE_LED);
            timer->usageFlags |= TIM_USE_MOTOR;
            break;
        case OUTPUT_MODE_SERVOS:
            timer->usageFlags &= ~(TIM_USE_MOTOR|TIM_USE_LED);
            timer->usageFlags |= TIM_USE_SERVO;
            break;
        case OUTPUT_MODE_LED:
            timer->usageFlags &= ~(TIM_USE_MOTOR|TIM_USE_SERVO);
            timer->usageFlags |= TIM_USE_LED;
            break;
    }
}

bool checkPwmAssignedToMotorOrServo(void)
{  
    // Check TIM_USE_FW_* and TIM_USE_MC_* is consistent, If so, return true, means the pwm mapping will remain same between FW and MC
    bool pwm_assigned_to_motor_or_servo = true;
    for (int idx = 0; idx < timerHardwareCount; idx++) {
        timerHardware_t *timHw = &timerHardware[idx];
        if (timHw->usageFlags & (TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR | TIM_USE_MC_SERVO | TIM_USE_FW_SERVO)) {
            pwm_assigned_to_motor_or_servo &= (timHw->usageFlags == TIM_USE_VTOL_SERVO) | (timHw->usageFlags == TIM_USE_VTOL_MOTOR);
        }
    }
    return pwm_assigned_to_motor_or_servo;
}


bool pwmHasMotorOnTimer(timMotorServoHardware_t * timOutputs, HAL_Timer_t *tim)
{
    for (int i = 0; i < timOutputs->maxTimMotorCount; ++i) {
        if (timOutputs->timMotors[i]->tim == tim) {
            return true;
        }
    }

    return false;
}

bool pwmHasServoOnTimer(timMotorServoHardware_t * timOutputs, HAL_Timer_t *tim)
{
    for (int i = 0; i < timOutputs->maxTimServoCount; ++i) {
        if (timOutputs->timServos[i]->tim == tim) {
            return true;
        }
        
    } else if (currentMixerConfig.outputMode == OUTPUT_MODE_MOTORS) {
        
        // Servos are rewritten as motors
        if (timer->usageFlags & TIM_USE_MC_SERVO) {
            timer->usageFlags = timer->usageFlags & ~TIM_USE_MC_SERVO;
            timer->usageFlags = timer->usageFlags | TIM_USE_MC_MOTOR;
        }
    }

    return changed;
}

void pwmEnsureEnoughtMotors(uint8_t motorCount)
{
    uint8_t motorOnlyOutputs = 0;

    for (int idx = 0; idx < timerHardwareCount; idx++) {
        timerHardware_t *timHw = &timerHardware[idx];

        timerHardwareOverride(timHw);

        if (checkPwmTimerConflicts(timHw)) {
            continue;
        }

        if (TIM_IS_MOTOR_ONLY(timHw->usageFlags)) {
            motorOnlyOutputs++;
            motorOnlyOutputs += pwmClaimTimer(timHw->tim, timHw->usageFlags);
        }
    }

    for (int idx = 0; idx < timerHardwareCount; idx++) {
        timerHardware_t *timHw = &timerHardware[idx];

        if (checkPwmTimerConflicts(timHw)) {
            continue;
        }

        if (TIM_IS_MOTOR(timHw->usageFlags) && !TIM_IS_MOTOR_ONLY(timHw->usageFlags)) {
            if (motorOnlyOutputs < motorCount) {
                timHw->usageFlags &= ~TIM_USE_SERVO;
                timHw->usageFlags |= TIM_USE_MOTOR;
                motorOnlyOutputs++;
                motorOnlyOutputs += pwmClaimTimer(timHw->tim, timHw->usageFlags);
            } else {
                timHw->usageFlags &= ~TIM_USE_MOTOR;
                pwmClaimTimer(timHw->tim, timHw->usageFlags);
            }
        }
    }
}

void pwmBuildTimerOutputList(timMotorServoHardware_t * timOutputs, bool isMixerUsingServos)
{
    timOutputs->maxTimMotorCount = 0;
    timOutputs->maxTimServoCount = 0;

    uint8_t motorCount = getMotorCount();
    uint8_t motorIdx = 0;

    pwmEnsureEnoughtMotors(motorCount);

    for (int idx = 0; idx < timerHardwareCount; idx++) {
        timerHardware_t *timHw = &timerHardware[idx];

        int type = MAP_TO_NONE;

        // Check for known conflicts (i.e. UART, LEDSTRIP, Rangefinder and ADC)
        if (checkPwmTimerConflicts(timHw)) {
            LOG_WARNING(PWM, "Timer output %d skipped", idx);
            continue;
        }

        // Determine if timer belongs to motor/servo
        if (currentMixerConfig.platformType == PLATFORM_MULTIROTOR || currentMixerConfig.platformType == PLATFORM_TRICOPTER || currentMixerConfig.platformType == PLATFORM_TAILSITTER) {
            // Multicopter

        if (TIM_IS_SERVO(timHw->usageFlags) && !pwmHasMotorOnTimer(timOutputs, timHw->tim)) {
            type = MAP_TO_SERVO_OUTPUT;
        } else if (TIM_IS_MOTOR(timHw->usageFlags) && !pwmHasServoOnTimer(timOutputs, timHw->tim)) {
            type = MAP_TO_MOTOR_OUTPUT;
        } else if (TIM_IS_LED(timHw->usageFlags) && !pwmHasMotorOnTimer(timOutputs, timHw->tim) && !pwmHasServoOnTimer(timOutputs, timHw->tim)) {
            type = MAP_TO_LED_OUTPUT;
        }

        switch(type) {
            case MAP_TO_MOTOR_OUTPUT:
                timHw->usageFlags &= TIM_USE_MOTOR;
                timOutputs->timMotors[timOutputs->maxTimMotorCount++] = timHw;
                pwmClaimTimer(timHw->tim, timHw->usageFlags);
                break;
            case MAP_TO_SERVO_OUTPUT:
                timHw->usageFlags &= TIM_USE_SERVO;
                timOutputs->timServos[timOutputs->maxTimServoCount++] = timHw;
                pwmClaimTimer(timHw->tim, timHw->usageFlags);
                break;
            case MAP_TO_LED_OUTPUT:
                timHw->usageFlags &= TIM_USE_LED;
                pwmClaimTimer(timHw->tim, timHw->usageFlags);
                break;
            default:
                break;
        }
    }
}

static bool motorsUseHardwareTimers(void)
{
    return getMotorProtocolProperties(motorConfig()->motorPwmProtocol)->usesHwTimer;
}

static bool servosUseHardwareTimers(void)
{
    return servoConfig()->servo_protocol == SERVO_TYPE_PWM ||
        servoConfig()->servo_protocol == SERVO_TYPE_SBUS_PWM;
}

static void pwmInitMotors(timMotorServoHardware_t * timOutputs)
{
    const int motorCount = getMotorCount();

    // Check if too many motors
    if (motorCount > MAX_MOTORS) {
        pwmInitError = PWM_INIT_ERROR_TOO_MANY_MOTORS;
        LOG_ERROR(PWM, "Too many motors. Mixer requested %d, max %d", motorCount, MAX_MOTORS);
        return;
    }

    // Do the pre-configuration. For motors w/o hardware timers this should be sufficient
    pwmMotorPreconfigure();

    // Now if we need to configure individual motor outputs - do that
    if (!motorsUseHardwareTimers()) {
        LOG_INFO(PWM, "Skipped timer init for motors");
        return;
    }

    // If mixer requests more motors than we have timer outputs - throw an error
    if (motorCount > timOutputs->maxTimMotorCount) {
        pwmInitError = PWM_INIT_ERROR_NOT_ENOUGH_MOTOR_OUTPUTS;
        LOG_ERROR(PWM, "Not enough motor outputs. Mixer requested %d, outputs %d", motorCount, timOutputs->maxTimMotorCount);
        return;
    }

    // Finally initialize individual motor outputs
    for (int idx = 0; idx < motorCount; idx++) {
        const timerHardware_t *timHw = timOutputs->timMotors[idx];
        if (!pwmMotorConfig(timHw, idx, feature(FEATURE_PWM_OUTPUT_ENABLE))) {
            pwmInitError = PWM_INIT_ERROR_TIMER_INIT_FAILED;
            LOG_ERROR(PWM, "Timer allocation failed for motor %d", idx);
            return;
        }
    }
}

static void pwmInitServos(timMotorServoHardware_t * timOutputs)
{
    const int servoCount = getServoCount();

    if (!isMixerUsingServos()) {
        LOG_INFO(PWM, "Mixer does not use servos");
        return;
    }

    // Check if too many servos
    if (servoCount > MAX_SERVOS) {
        pwmInitError = PWM_INIT_ERROR_TOO_MANY_SERVOS;
        LOG_ERROR(PWM, "Too many servos. Mixer requested %d, max %d", servoCount, MAX_SERVOS);
        return;
    }

    // Do the pre-configuration. This should configure non-timer PWM drivers
    pwmServoPreconfigure();

    // Check if we need to init timer output for servos
    if (!servosUseHardwareTimers()) {
        // External PWM servo driver
        LOG_INFO(PWM, "Skipped timer init for servos - using external servo driver");
        return;
    }


    // If mixer requests more servos than we have timer outputs - throw an error
    uint16_t maxServos = timOutputs->maxTimServoCount;
    if (servoConfig()->servo_protocol == SERVO_TYPE_SBUS_PWM) {
        maxServos = MAX(SERVO_SBUS_MAX_SERVOS, timOutputs->maxTimServoCount);
    }

    if (servoCount > maxServos) {
        pwmInitError = PWM_INIT_ERROR_NOT_ENOUGH_SERVO_OUTPUTS;
        LOG_ERROR(PWM, "Too many servos. Mixer requested %d, timer outputs %d", servoCount, timOutputs->maxTimServoCount);
        return;
    }

    // Configure individual servo outputs
    for (int idx = 0; idx < MIN(servoCount, timOutputs->maxTimServoCount); idx++) {
        const timerHardware_t *timHw = timOutputs->timServos[idx];

        if (!pwmServoConfig(timHw, idx, servoConfig()->servoPwmRate, servoConfig()->servoCenterPulse, feature(FEATURE_PWM_OUTPUT_ENABLE))) {
            pwmInitError = PWM_INIT_ERROR_TIMER_INIT_FAILED;
            LOG_ERROR(PWM, "Timer allocation failed for servo %d", idx);
            return;
        }
    }
}


bool pwmMotorAndServoInit(void)
{
    timMotorServoHardware_t timOutputs;

    // Build temporary timer mappings for motor and servo
    pwmBuildTimerOutputList(&timOutputs, isMixerUsingServos());

    // At this point we have built tables of timers suitable for motor and servo mappings
    // Now we can actually initialize them according to motor/servo count from mixer
    pwmInitMotors(&timOutputs);
    pwmInitServos(&timOutputs);

    return (pwmInitError == PWM_INIT_ERROR_NONE);
}

#endif
#endif

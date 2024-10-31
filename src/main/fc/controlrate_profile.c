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

#include "platform.h"

#include "common/axis.h"

#include "config/config_reset.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_curves.h"
#include "fc/settings.h"

const controlRateConfig_t *currentControlRateProfile;


PG_REGISTER_ARRAY_WITH_RESET_FN(controlRateConfig_t, MAX_CONTROL_RATE_PROFILE_COUNT, controlRateProfiles, PG_CONTROL_RATE_PROFILES, 4);
#define SETTING_ROLL_RATE_DEFAULT 20
#define SETTING_PITCH_RATE_DEFAULT 20
#define SETTING_YAW_RATE_DEFAULT 20


void pgResetFn_controlRateProfiles(controlRateConfig_t *instance)
{
    for (int i = 0; i < MAX_CONTROL_RATE_PROFILE_COUNT; i++) {
        RESET_CONFIG(controlRateConfig_t, &instance[i],
            .throttle = {
                .rcMid8 = 50,
                .rcExpo8 = 0,
                .dynPID = 0,
                .pa_breakpoint = 1500,
                .fixedWingTauMs = 0
            },

            .stabilized = {
                .rcExpo8 = 70,
                .rcYawExpo8 = 20,
                .rates[FD_ROLL] = SETTING_ROLL_RATE_DEFAULT,
                .rates[FD_PITCH] = SETTING_PITCH_RATE_DEFAULT,
                .rates[FD_YAW] = SETTING_YAW_RATE_DEFAULT,
            },

            .manual = {
                .rcExpo8 = 70,
                .rcYawExpo8 = 20,
                .rates[FD_ROLL] = 100,
                .rates[FD_PITCH] = 100,
                .rates[FD_YAW] = 100
            },

            .misc = {
                .fpvCamAngleDegrees = 0
            },
        #ifdef USE_RATE_DYNAMICS
            .rateDynamics = {
                .sensitivityCenter = SETTING_RATE_DYNAMICS_CENTER_SENSITIVITY_DEFAULT,
                .sensitivityEnd = SETTING_RATE_DYNAMICS_END_SENSITIVITY_DEFAULT,
                .correctionCenter = SETTING_RATE_DYNAMICS_CENTER_CORRECTION_DEFAULT,
                .correctionEnd = SETTING_RATE_DYNAMICS_END_CORRECTION_DEFAULT,
                .weightCenter = SETTING_RATE_DYNAMICS_CENTER_WEIGHT_DEFAULT,
                .weightEnd = SETTING_RATE_DYNAMICS_END_WEIGHT_DEFAULT,
            }
        #endif
        );
    }
}

void setControlRateProfile(uint8_t profileIndex)
{
    if (profileIndex >= MAX_CONTROL_RATE_PROFILE_COUNT) {
        profileIndex = 0;
    }
    currentControlRateProfile = controlRateProfiles(profileIndex);
}

void activateControlRateConfig(void)
{
    generateThrottleCurve(currentControlRateProfile);
}

void changeControlRateProfile(uint8_t profileIndex)
{
    setControlRateProfile(profileIndex);
    activateControlRateConfig();
}

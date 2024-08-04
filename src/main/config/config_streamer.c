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

#include <string.h>
#include "platform.h"
#include "drivers/system.h"
#include "config/config_streamer.h"
#include "build/build_config.h"

#if !defined(CONFIG_IN_FLASH)
SLOW_RAM uint8_t eepromData[EEPROM_SIZE];
#endif

extern uint8_t __config_start;   // configured via linker script when building binaries.
extern uint8_t __config_end;

// Helper functions
extern void config_streamer_impl_unlock(void);
extern void config_streamer_impl_lock(void);
extern int config_streamer_impl_write_word(config_streamer_t *c, config_streamer_buffer_align_type_t *buffer);

void config_streamer_init(config_streamer_t *c)
{
    memset(c, 0, sizeof(*c));
#ifdef AURIX
    c->endinitSfty_pw = IfxScuWdt_getSafetyWatchdogPassword();
#endif
}

void config_streamer_start(config_streamer_t *c, uintptr_t base, int size)
{

    // base must start at FLASH_PAGE_SIZE boundary when using embedded flash.
    c->address = base;
    c->size = size;
#ifndef AURIX
    c->end = base + size;
    if (!c->unlocked) {
        config_streamer_impl_unlock();
        c->unlocked = true;
    }
    c->err = 0;
#else
    uint32 flash = 0;

    if (!c->unlocked)
    {
    	/* erase program flash */
    	IfxScuWdt_clearSafetyEndinit(c->endinitSfty_pw);
//    	IfxFlash_eraseSector(c->address);
    	IfxFlash_eraseMultipleSectors(c->address,4);
    	IfxScuWdt_setSafetyEndinit(c->endinitSfty_pw);

		/* wait until unbusy */
		IfxFlash_waitUnbusy(flash, IfxFlash_FlashType_D0);

        c->unlocked = true;
    }
#endif
}
#ifdef AURIX

static int write_page(config_streamer_t *c)
{
	uint32_t flash = 0;
	uint32_t offset=0;

    if (c->err != 0) {
        return c->err;
    }

    c->err = IfxFlash_enterPageMode(c->address);
    /* wait until unbusy */
    IfxFlash_waitUnbusy(flash, IfxFlash_FlashType_D0);

    for (offset=0; offset<IFXFLASH_DFLASH_PAGE_LENGTH; offset+=8)
    	IfxFlash_loadPage2X32(c->address, *((uint32_t*)(c->buffer.b + offset)), *((uint32_t*)(c->buffer.b + offset + 4)));

	/* write page */
	IfxScuWdt_clearSafetyEndinit(c->endinitSfty_pw);
	IfxFlash_writePage(c->address);
	IfxScuWdt_setSafetyEndinit(c->endinitSfty_pw);

    /* wait until unbusy */
    IfxFlash_waitUnbusy(flash, IfxFlash_FlashType_D0);

    c->address += IFXFLASH_DFLASH_PAGE_LENGTH;
    return 0;
}

int config_streamer_write(config_streamer_t *c, const uint8_t *p, uint32_t size)
{
    for (const uint8_t *pat = p; pat != (uint8_t*)p + size; pat++) {
        c->buffer.b[c->at++] = *pat;

        if (c->at == sizeof(c->buffer)) {
#ifndef AURIX
            c->err = write_word(c, c->buffer.w);
#else
            c->err = write_page(c);

#endif
            c->at = 0;
        }
    }
    return c->err;
}

int config_streamer_status(config_streamer_t *c)
{
    return c->err;
}

int config_streamer_flush(config_streamer_t *c)
{
    if (c->at != 0) {
        memset(c->buffer.b + c->at, 0, sizeof(c->buffer) - c->at);
#ifndef AURIX
        c->err = write_word(c, c->buffer.w);
#else
        c->err = write_page(c);
#endif
        c->at = 0;
    }
    return c-> err;
}

int config_streamer_finish(config_streamer_t *c)
{
    if (c->unlocked) {
#ifndef AURIX
#if defined(STM32F7)
        HAL_FLASH_Lock();
#else
        FLASH_Lock();
#endif
#endif
        c->unlocked = false;
    }
    return c->err;
}

#else
int config_streamer_write(config_streamer_t *c, const uint8_t *p, uint32_t size)
{
    if ((c->address + size) > c->end) {
        // trying to write past end of streamer
        return -1;
    }

    for (const uint8_t *pat = p; pat != (uint8_t*)p + size; pat++) {
        c->buffer.b[c->at++] = *pat;

        if (c->at == sizeof(c->buffer)) {
            c->err = config_streamer_impl_write_word(c, &c->buffer.w);
            c->at = 0;
        }
    }
    return c->err;
}

int config_streamer_status(config_streamer_t *c)
{
    return c->err;
}

int config_streamer_flush(config_streamer_t *c)
{
    if (c->at != 0) {
        if ((c->address + c->at) > c->end) {
            return -1;
        }
        memset(c->buffer.b + c->at, 0, sizeof(c->buffer) - c->at);
        c->err = config_streamer_impl_write_word(c, &c->buffer.w);
        c->at = 0;
    }
    return c-> err;
}

int config_streamer_finish(config_streamer_t *c)
{
    if (c->unlocked) {
        config_streamer_impl_lock();
        c->unlocked = false;
    }
    return c->err;
}
#endif

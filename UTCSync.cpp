/*
 * UTCSync.cpp
 *
 *  Created on: Jun 18, 2018
 *      Author: yuchen
 */


#define _UTCSYNC_
#include "task.h"

UTCSync utc_sync;

/*
 * Input 1pps tick
 * Return 1pps utc time, return 0 if something wronge
 * return 0xffffffffffffffff, if not utc time availabl.
 */
uint64_t UTCSync::get_1pps_utctime(uint64_t tick, uint32_t tick_ps)
{
    uint64_t tick_diff = tick - prev_tick_time.tick;
    uint64_t time_diff = (tick_diff + tick_ps / 2) / tick_ps * UTC_UNIT;
    uint64_t ret = (prev_tick_time.utc == 0) ? 0 : prev_tick_time.utc + time_diff;
    prev_tick_time.tick = tick;
    prev_tick_time.utc += time_diff;
    return ret;

}

void UTCSync::init()
{
    prev_tick_time.tick = 0;
    prev_tick_time.utc = 0;
}

void UTCSync::process_event()
{

}

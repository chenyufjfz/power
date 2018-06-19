/*
 * UTCSync.cpp
 *
 *  Created on: Jun 18, 2018
 *      Author: yuchen
 */


#define _UTCSYNC_
#include "task.h"

UTCSync utc_sync;


bool UTCSync::compatible(TickTime & t0, TickTime & t1, uint32_t cpu_clock)
{
    int64_t time_diff, expect_tick_diff, tick_diff;
    if (t1 > t0) {
        time_diff = t1.utc_time - t0.utc_time;
        expect_tick_diff = time_diff * cpu_clock / 1000;
        tick_diff = t1.utc_tick - t0.utc_tick;
    } else {
        time_diff = t0.utc_time - t1.utc_time;
        expect_tick_diff = time_diff * cpu_clock / 1000;
        tick_diff = t0.utc_tick - t1.utc_tick;
    }
    return (abs(expect_tick_diff - tick_diff) < MAX_CLOCK_DIFF);
}

/*
 * Input 1pps tick
 * Return 1pps utc time, return 0 if not utc time available
 */
uint64_t UTCSync::get_1pps_utctime(uint64_t tick, uint32_t cpu_clock)
{
    uint64_t tick_diff = tick - prev_tick_time.utc_tick;
    uint64_t time_diff = (tick_diff + cpu_clock / 2) / cpu_clock * 1000;
    uint64_t ret = (prev_tick_time.utc_time == 0) ? 0 : prev_tick_time.utc_time + time_diff;
    prev_tick_time.utc_tick = tick;
    prev_tick_time.utc_time += time_diff;
    return ret;

}

void UTCSync::init()
{
    prev_tick_time.utc_tick = 0;
    prev_tick_time.utc_time = 0;
}

void UTCSync::process_event()
{

}

/*
 * task.h
 *
 *  Created on: May 28, 2018
 *      Author: yuchen
 */

#ifndef TASK_H_
#define TASK_H_

#define _SIM1PPS_WITH_PWM_
#define _SIM50HZ_SIN_WITH_PWM_
#include <stdint.h>
#include <ctype.h>
#include <stdio.h>
#include "F2837xD_Cla_typedefs.h"  // F2837xD CLA Type definitions
#include "F2837xD_device.h"        // F2837xD Headerfile Include File
#include "F2837xD_GlobalPrototypes.h"
#include "F2837xD_cputimervars.h"
#include "F2837xD_Cla_defines.h"            // Macros used for CLA examples.
#include "F2837xD_EPwm_defines.h"           // Macros used for PWM examples.
#include "F2837xD_Adc_defines.h"            // Macros used for ADC examples.
#include "F2837xD_Emif_defines.h"           // Macros used for EMIF examples.
#include "F2837xD_Gpio_defines.h"           // Macros used for GPIO support code
#include "F2837xD_I2c_defines.h"            // Macros used for I2C examples.
#include "F2837xD_Ipc_defines.h"            // Macros used for IPC support code.
#include "F2837xD_Pie_defines.h"            // Macros used for PIE examples.
#include "F2837xD_Dma_defines.h"            // Macros used for DMA examples.
#include "F2837xD_SysCtrl_defines.h"        // Macros used for LPM support code
#include "F2837xD_Upp_defines.h"            // Macros used for UPP examples.
#include "F2837xD_defaultisr.h"
#ifdef __cplusplus
extern "C" {
#endif
extern void F28x_usDelay(long LoopCount);
#ifdef __cplusplus
}
#endif


#define MSG_EVENT_BIT           0
#define MSG_EVENT_MASK          (1 << MSG_EVENT_BIT)
#define ADCSYNC_EVENT_BIT       1
#define ADCSYNC_EVENT_MASK      (1 << ADCSYNC_EVENT_BIT)

#ifdef _LAUNCHXL_F28379D
#define CPU_CLOCK 200000000
#else
#define CPU_CLOCK 120000000
#endif

#define SAMPLE_RATE 10000
#define CHANNEL 8
#define COMPUTE_RATE 100
#define POWER_CYCLE 50
#define COMPUTE_CYCLE 2
#define BUF_DMA_RESERVED_COUNT 1
#define UTC_UNIT 1000

//max clock jitter is 100ppm
#define MAX_CLOCK_JITTER 100

#if SAMPLE_RATE % (POWER_CYCLE * 4) != 0
#error SAMPLE_RATE %  (POWER_CYCLE * 4) !=0
#endif
#if SAMPLE_RATE % COMPUTE_RATE !=0
#error SAMPLE_RATE % COMPUTE_RATE !=0
#endif

#define PI 3.1415926536
#define Assert(x) do {while (!(x));} while(0)
#define min(x, y) ((x) < (y) ? (x) : (y))
#define max(x, y) ((x) > (y) ? (x) : (y))

class Task {
public:
    virtual void init() = 0;
    virtual void process_event() = 0;
};

class MsgTask : public Task {
protected:
    uint16_t msg_route;
public:
    void init();
    void print_helper();
    void process_event();
};

#ifndef _MSGTASK_
extern MsgTask msg_task;
#endif

struct TickTime {
    uint64_t tick, utc;
    TickTime(uint64_t _tick, uint64_t _utc) {
        tick = _tick;
        utc = _utc;
    }
    TickTime() {}
};

#define DMASYNC_SIZE 3
#define UTCSYNC_SIZE 4

class ADCSync1PPS : public Task {
protected:
    bool keep_display, receive_1pps;
    volatile uint16_t capture_state, sync_state;
    struct DmaSync {
        int sample_period;
        uint64_t sample_idx;
        uint64_t tick;
    } dma_sync_tbl[DMASYNC_SIZE];
    TickTime utc_sync_tbl[UTCSYNC_SIZE];
    uint32_t tick_ps;
    uint64_t record_utc;
    volatile uint64_t cap1;

    void clear_utc_sync_tbl();
    void push_utc_sync_tbl(uint64_t tick, uint64_t utc);
    void clear_dma_sync_tbl();
    void push_dma_sync_tbl(int sample_period, uint64_t sample_idx, uint64_t tick);
    void update_tick_ps();
    void update_record_utc();
public:
    static bool compatible(const TickTime & t0, const TickTime & t1, uint32_t _tick_ps, uint32_t max_diff);
    uint64_t tick2sample(uint64_t tick);
    uint64_t sample2tick(uint64_t sample);
    template <class UTC_TYPE> int tick2utc(uint64_t tick, UTC_TYPE & utc);
    template <class UTC_TYPE> int utc2tick(UTC_TYPE utc, uint64_t & tick);
    template <class UTC_TYPE> int sample2utc(uint64_t sample, UTC_TYPE & utc);
    template <class UTC_TYPE> int utc2sample(UTC_TYPE utc, uint64_t & sample);

    void set_cap_state(uint16_t cap_state);
    uint64_t get_tick();
    void new_cap_evt();
    void new_cluster_ready();

    void print_helper();
    void init();
    void process_event();
    int process_cmd(char * user_cmd);
};

#ifndef _ADCSync1PPS_
extern ADCSync1PPS adc_sync_1pps;
#endif

class UTCSync : public Task {
protected:
    TickTime prev_tick_time;
public:
    uint64_t get_1pps_utctime(uint64_t tick, uint32_t tick_ps);
    void init();
    void process_event();
};

#ifndef _UTCSYNC_
extern UTCSync utc_sync;
#endif

#ifndef _MAIN_
extern volatile int16_t event_pending;
#endif

#ifdef _SIM1PPS_WITH_PWM_
class Sim1PPS {
protected:
    int enable_out;
public:
    void init();
    void print_helper();
    int process_cmd(char * user_cmd);
};

#ifndef _SIM1PPS_
extern Sim1PPS sim_1pps;
#endif
#endif

#ifdef _SIM50HZ_SIN_WITH_PWM_
class Sim50HzSin {
protected:
    int16_t amp, offset;
public:
    void init();
    void print_helper();
    int process_cmd(char * user_cmd);
};
#ifndef _Sim50HzSin_
extern Sim50HzSin sim50hz_sin;
#endif
#endif
#endif /* TASK_H_ */

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

class ADCSync1PPS : public Task {
protected:
    bool keep_display;
public:
    void init();
    uint64_t get_tick();
    void print_helper();
    void process_event();
    int process_cmd(char * user_cmd);
};

#ifndef _MAIN_
extern volatile int16_t event_pending;
#endif

#ifndef _MSGTASK_
extern MsgTask msg_task;
#endif

#ifndef _ADCSync1PPS_
extern ADCSync1PPS adc_sync_1pps;
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
    uint16_t amp, offset;
public:
    void init();
    void print_helper();
    int process_cmd(char * user_cmd);
};
#ifndef _Sim50HzSin_
extern Sim50HzSin sim50hz_sin;
#endif
#endif

#ifdef _LAUNCHXL_F28379D
#define CPU_CLOCK 200000000
#else
#define CPU_CLOCK 120000000
#endif
#endif /* TASK_H_ */

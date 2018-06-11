/*
 * ADCSync1PPS.cpp
 *
 *  Created on: Jun 6, 2018
 *      Author: yuchen
 */


#include <file.h>
#define _ADCSync1PPS_
#include "task.h"

ADCSync1PPS adc_sync_1pps;

#define USEECAP     4

#if USEECAP == 1
static volatile ECAP_REGS * ecap0 = &ECap1Regs;
static volatile ECAP_REGS * ecap1 = &ECap2Regs;
static volatile ECAP_REGS * ecap2 = &ECap3Regs;
#elif USEECAP == 4
static volatile ECAP_REGS * ecap0 = &ECap4Regs;
static volatile ECAP_REGS * ecap1 = &ECap5Regs;
static volatile ECAP_REGS * ecap2 = &ECap6Regs;
#endif

#ifdef _LAUNCHXL_F28379D
#define ECAP1_PHASE_DELAY 7
#endif

#define RESERVER_LEN 5
static uint32_t cap[RESERVER_LEN][2] = { 0 };
static uint32_t tick_h32 = 0;

static __interrupt void ecap0_isr(void)
{
    for (int i=RESERVER_LEN - 1; i>0; i--) {
        cap[i][0] = cap[i-1][0];
        cap[i][1] = cap[i-1][1];
    }
    cap[0][0] = ecap0->CAP1;
    cap[0][1] = ecap1->CAP1;

    // Clear interrupts and arm ECAP
    ecap0->ECCLR.all = 0xFFFF;           // Clear all CAP __interrupt flags
    ecap0->ECCTL2.bit.REARM = 1;
    ecap1->ECCTL2.bit.REARM = 1;

    // Acknowledge this __interrupt to receive more __interrupts from group 4
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;

    event_pending |= ADCSYNC_EVENT_MASK;
}

static __interrupt void ecap1_isr(void)
{
    tick_h32++;
    ecap1->ECCLR.all = 0xFFFF;           // Clear all CAP __interrupt flags
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}

static void ecap_init()
{
    EALLOW;
#if USEECAP == 1
    InputXbarRegs.INPUT7SELECT = 4;         // Set eCAP1 source to GPIO 4, pin 36
    InputXbarRegs.INPUT8SELECT = 4;
    PieVectTable.ECAP1_INT = &ecap0_isr;
    PieVectTable.ECAP2_INT = &ecap1_isr;
    PieCtrlRegs.PIEIER4.bit.INTx1 = 1;      //PIEIER4.1 is ECAP1
    PieCtrlRegs.PIEIER4.bit.INTx2 = 1;      //PIEIER4.2 is ECAP2
#elif USEECAP == 4
    InputXbarRegs.INPUT10SELECT = 4;         // Set eCAP4 source to GPIO 4, pin 36
    InputXbarRegs.INPUT11SELECT = 4;
    PieVectTable.ECAP4_INT = &ecap0_isr;
    PieVectTable.ECAP5_INT = &ecap1_isr;
    PieCtrlRegs.PIEIER4.bit.INTx4 = 1;      //PIEIER4.4 is ECAP4
    PieCtrlRegs.PIEIER4.bit.INTx5 = 1;      //PIEIER4.5 is ECAP2
#endif

    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_ASYNC);

    ecap0->CTRPHS = 0;               // Phase = 0;
    ecap0->ECCTL2.all = 0x2B;
    /*CONT_ONESHT:1 =1;    0 Continuous or one-shot, choose One short
      STOP_WRAP:2=1        2:1 Stop value for one-shot, Stop at event 1
      REARM:1    =1        3 One-shot re-arm
      TSCTRSTOP:1=0        4 TSCNT counter stop
      SYNCI_EN:1 =1        5 Counter sync-in select, Disable sync in
      SYNCO_SEL:2=0        7:6 Sync-out mode, let Synco output to ecap1
      SWSYNC:1   =0        8 SW forced counter sync
      CAP_APWM:1 =0        9 CAP/APWM operating mode select
      APWMPOL:1  =0        10 APWM output polarity select
      rsvd1:5;             15:11 Reserved*/

    ecap0->ECCLR.all = 0xFFFF;           // Clear all CAP __interrupt flags
    ecap0->ECEINT.all = 0x0002;          // Only enable CEvt1

    // Configure peripheral registers
    //
    ecap0->ECCTL1.all = 0x100;
    /*CAP1POL:1  = 0    0 Capture Event 1 Polarity select
      CTRRST1:1  = 0    1 Counter Reset on Capture Event 1
      CAP2POL:1  = 0    2 Capture Event 2 Polarity select
      CTRRST2:1  = 0    3 Counter Reset on Capture Event 2
      CAP3POL:1  = 0    4 Capture Event 3 Polarity select
      CTRRST3:1  = 0    5 Counter Reset on Capture Event 3
      CAP4POL:1  = 0    6 Capture Event 4 Polarity select
      CTRRST4:1  = 0    7 Counter Reset on Capture Event 4
      CAPLDEN:1  = 1    8 Enable Loading CAP1-4 regs on a Cap Event
      PRESCALE:5 = 0    13:9 Event Filter prescale select
      FREE_SOFT:2 =0    15:14 Emulation mode      */
    ecap0->ECCTL2.bit.TSCTRSTOP = 1;     // Start Counter

    ecap1->CTRPHS = ECAP1_PHASE_DELAY;          // Phase = 0;
    ecap1->ECCTL2.all = 0xEB;
    /*CONT_ONESHT:1 =1;    0 Continuous or one-shot, choose One short
      STOP_WRAP:2=1        2:1 Stop value for one-shot, Stop at event 1
      REARM:1    =1        3 One-shot re-arm
      TSCTRSTOP:1=0        4 TSCNT counter stop
      SYNCI_EN:1 =1        5 Counter sync-in select, enable sync in
      SYNCO_SEL:2=3        7:6 Sync-out mode, disable synco
      SWSYNC:1   =0        8 SW forced counter sync
      CAP_APWM:1 =0        9 CAP/APWM operating mode select
      APWMPOL:1  =0        10 APWM output polarity select
      rsvd1:5;             15:11 Reserved*/

    ecap0->ECCLR.all = 0xFFFF;           // Clear all CAP __interrupt flags
    ecap1->ECEINT.all = 0x00020;          // enable overflow bit
    ecap1->ECCTL1.all = 0x100;      // Same as ecap0

    ecap1->ECCTL2.bit.TSCTRSTOP = 1;     // Start Counter

    ecap0->ECCTL2.bit.SWSYNC = 1; //load ecap0 phase
    ecap1->ECCTL2.bit.SWSYNC = 1; //load ecap0 phase
    EDIS;

    IER |= 0x8;     //eCAPx is ACK4
}

void ADCSync1PPS::init()
{
    ecap_init();
    keep_display = false;

}

uint64_t ADCSync1PPS::get_tick()
{
    uint64_t ret = 0;
    int overflow;
    uint32_t counter;
    unsigned int old_st = __disable_interrupts();
    ret = tick_h32;
    overflow = ecap1->ECFLG.bit.CTROVF;
    counter = ecap1->TSCTR;
    if (overflow==0 && counter < 30) { //maybe overflow happen, read again
        overflow = ecap1->ECFLG.bit.CTROVF;
        counter = ecap1->TSCTR;
    }
    __restore_interrupts(old_st);
    ret += overflow;
    return (ret << 32) | counter;
}

void ADCSync1PPS::print_helper()
{
    printf("ADCSync1PPS\n\r");
    printf("h) Print this info\n\r");
    printf("t) Toggle display\n\r");
    printf("e) Exit\n\r");
}

void ADCSync1PPS::process_event()
{
    int * p_evt = (int*)&event_pending;
    __and(p_evt, ~ADCSYNC_EVENT_MASK);
    uint64_t tick = get_tick();
    printf("T=%lu:%lu, c0=%lu, c1=%lu\n\r", (uint32_t) (tick >> 32), (uint32_t) (tick & 0xffffffff), cap[0][0], cap[0][1]);
}

int ADCSync1PPS::process_cmd(char * user_cmd)
{
    switch (toupper(user_cmd[0])) {
    case 'H':
        print_helper();
        break;
    case 'T':
        keep_display = !keep_display;
        break;
    case 'E':
        return 1;
    default:
        break;

    }
    return 0;
}

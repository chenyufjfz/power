/*
 * Sim1PPS.cpp
 *
 *  Created on: May 28, 2018
 *      Author: yuchen
 */

#define _SIM1PPS_

#include "task.h"
#include <stdlib.h>
#include <utility>

#ifdef _SIM1PPS_WITH_PWM_

Sim1PPS sim_1pps;
static uint32_t f0, f_var;

// Set Period value 200000000
#define F0_DEFAULT_CYCLE CPU_CLOCK
#define F0VAR_MAX (F0_DEFAULT_CYCLE / 10)

#define USEECAP     6

#if USEECAP == 1
static volatile ECAP_REGS * ecap = &ECap1Regs;
#elif USEECAP == 2
static volatile ECAP_REGS * ecap = &ECap2Regs;
#elif USEECAP == 3
static volatile ECAP_REGS * ecap = &ECap3Regs;
#elif USEECAP == 4
static volatile ECAP_REGS * ecap = &ECap4Regs;
#elif USEECAP == 5
static volatile ECAP_REGS * ecap = &ECap5Regs;
#elif USEECAP == 6
static volatile ECAP_REGS * ecap = &ECap6Regs;
#endif

#define SIM1PPS_FREE_SOFT 0
#if SIM1PPS_FREE_SOFT > 1
#error SIM1PPS_FREE_SOFT > 1
#endif

static interrupt void ecap_1pps_isr(void)
{
    float a = rand() * 2.0 / RAND_MAX - 1; //generate rand a in [-1, 1]
    uint32_t f = f0 + f_var * a;              //f = f0 + var
    ecap->CAP1 = f - 1;
    ecap->CAP2 = f /2;                   // Set Compare value
    ecap->ECCLR.bit.CTR_PRD = 1;    //clear period interrupt flag
    ecap->ECCLR.bit.INT = 1;        //enable next ecap1 interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;       // Issue PIE ack,  eCAPx is ACK4
}

void Sim1PPS::init()
{
    EALLOW;
    ecap->ECCTL1.bit.FREE_SOFT = SIM1PPS_FREE_SOFT * 3; //free run or stop when emulation event
    ecap->ECCTL2.all = 0x2C6;     // Stop counter
    /*
    CONT_ONESHT=0       0 Continuous or one-shot
    STOP_WRAP=3         2:1 Stop value for one-shot, Wrap for continuous
    REARM=0             3 One-shot re-arm
    TSCTRSTOP=0, stop   4 TSCNT counter stop
    SYNCI_EN=0,disable  5 Counter sync-in select
    SYNCO_SEL=3, disable7:6 Sync-out mode
    SWSYNC=0            8 SW forced counter sync
    CAP_APWM=1          9 CAP/APWM operating mode select,
    APWMPOL=0           10 APWM output polarity select
    rsvd1:5;            15:11 Reserved*/
    enable_out = 0;


#if USEECAP == 1
    OutputXbarRegs.OUTPUT3MUX0TO15CFG.bit.MUX0 = 3; // Select ECAP1.OUT on Mux0, see hardware_reg table 8.3
    OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX0 = enable_out;  // Enable MUX0 for ECAP1.OUT
    PieVectTable.ECAP1_INT = ecap_1pps_isr;    // Install ecap isr
    PieCtrlRegs.PIEIER4.bit.INTx1 = 1;  //PIEIER4.1 is ECAP1
#elif USEECAP == 2
    OutputXbarRegs.OUTPUT3MUX0TO15CFG.bit.MUX2 = 3; // Select ECAP1.OUT on Mux0, see hardware_reg table 8.3
    OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX2 = enable_out;  // Enable MUX0 for ECAP1.OUT
    PieVectTable.ECAP2_INT = ecap_1pps_isr;    // Install ecap isr
    PieCtrlRegs.PIEIER4.bit.INTx2 = 1;  //PIEIER4.2 is ECAP2
#elif USEECAP == 3
    OutputXbarRegs.OUTPUT3MUX0TO15CFG.bit.MUX4 = 3; // Select ECAP1.OUT on Mux0, see hardware_reg table 8.3
    OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX4 = enable_out;  // Enable MUX0 for ECAP1.OUT
    PieVectTable.ECAP3_INT = ecap_1pps_isr;    // Install ecap isr
    PieCtrlRegs.PIEIER4.bit.INTx3 = 1;  //PIEIER4.3 is ECAP3
#elif USEECAP == 4
    OutputXbarRegs.OUTPUT3MUX0TO15CFG.bit.MUX6 = 3; // Select ECAP1.OUT on Mux0, see hardware_reg table 8.3
    OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX6 = enable_out;  // Enable MUX0 for ECAP1.OUT
    PieVectTable.ECAP4_INT = ecap_1pps_isr;    // Install ecap isr
    PieCtrlRegs.PIEIER4.bit.INTx4 = 1;  //PIEIER4.4 is ECAP3
#elif USEECAP == 5
    OutputXbarRegs.OUTPUT3MUX0TO15CFG.bit.MUX8 = 3; // Select ECAP1.OUT on Mux0, see hardware_reg table 8.3
    OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX8 = enable_out;  // Enable MUX0 for ECAP1.OUT
    PieVectTable.ECAP5_INT = ecap_1pps_isr;    // Install ecap isr
    PieCtrlRegs.PIEIER4.bit.INTx5 = 1;  //PIEIER4.5 is ECAP3
#elif USEECAP == 6
    OutputXbarRegs.OUTPUT3MUX0TO15CFG.bit.MUX10 = 3; // Select ECAP1.OUT on Mux0, see hardware_reg table 8.3
    OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX10 = enable_out;  // Enable MUX0 for ECAP1.OUT
    PieVectTable.ECAP6_INT = ecap_1pps_isr;    // Install ecap isr
    PieCtrlRegs.PIEIER4.bit.INTx6 = 1;  //PIEIER4.6 is ECAP3
#endif

    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 3;    // Select OUTPUTXBAR3 on GPIO5,pin 35

    f0 = F0_DEFAULT_CYCLE;
    f_var = 0;
    ecap->CAP1 = f0;
    ecap->CAP2 = f0 /2;                   // Set Compare value

    ecap->ECCLR.all = 0x0FF;            // Clear pending __interrupts
    ecap->ECEINT.all = 0x40;    // enable Period Equal Interrupt
    /*rsvd1:1;            0 Reserved
    CEVT1:1;            1 Capture Event 1 Interrupt Enable
    CEVT2:1;            2 Capture Event 2 Interrupt Enable
    CEVT3:1;            3 Capture Event 3 Interrupt Enable
    CEVT4:1;            4 Capture Event 4 Interrupt Enable
    CTROVF:1;           5 Counter Overflow Interrupt Enable
    CTR_EQ_PRD:1;       6 Period Equal Interrupt Enable
    CTR_EQ_CMP:1;       7 Compare Equal Interrupt Enable    */

    ecap->ECCTL2.bit.TSCTRSTOP = 1;     // Start counter
    EDIS;

    IER |= 0x8;     //eCAPx is ACK4
}

std::pair<uint16_t, uint16_t> fix_print(uint32_t a, uint32_t unit)
{
    return std::make_pair(a / unit, a % unit);
}

void Sim1PPS::print_helper()
{
    printf("Sim1PPS\n\r");
    printf("h) Print this info\n\r");
    printf("f ddd) set f to 1 + ddd/10000\n\r");
    printf("v ddd) set var to ddd/10000\n\r");
    printf("t) Toggle on/off\n\r");
    printf("e) Exit\n\r");
}

int Sim1PPS::process_cmd(char * user_cmd)
{
    int16_t a = 0;
    std::pair<uint16_t, uint16_t> b,c,d;
    switch (toupper(user_cmd[0])) {
    case 'H':
        print_helper();
        break;
    case 'F':
        if (sscanf(&user_cmd[1], " %d", &a) == 1)
            f0 = F0_DEFAULT_CYCLE +  F0_DEFAULT_CYCLE / 10000 * a;
        break;
    case 'V':
        if (sscanf(&user_cmd[1], " %d", &a) == 1)
            f_var = F0VAR_MAX / 10000 * a;
        break;
    case 'T':
        enable_out = !enable_out;
        EALLOW;
#if USEECAP == 1
        OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX0 = enable_out;
#elif USEECAP == 2
        OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX2 = enable_out;
#elif USEECAP == 3
        OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX4 = enable_out;
#elif USEECAP == 4
        OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX6 = enable_out;
#elif USEECAP == 5
        OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX8 = enable_out;
#elif USEECAP == 6
        OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX10 = enable_out;
#endif
        EDIS;
        break;
    case 'E':
        return 1;
    default:
        break;
    }
    b = fix_print(f0/1000, 1000);
    c = fix_print(f_var, 1000);
    d = fix_print(ecap->CAP1/1000, 1000);
    printf("on=%d, f0=%d.%dM, var=%d.%dK,current=%d.%dM\n\r", enable_out,
           b.first, b.second, c.first, c.second, d.first, d.second );
    return 0;
}
#endif

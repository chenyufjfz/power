/**********************************************************************
* File: Main_7.c -- Solution File for Lab 7
* Devices: TMS320F28x7x
* Author: C2000 Technical Training, Texas Instruments
**********************************************************************/

#include "task.h"

volatile int16_t event_pending = 0;

Task * all_task[] = {
                    & msg_task,
                    & adc_sync_1pps,
                    & utc_sync
};
/**********************************************************************
* Function: main()
*
* Description: Main function for C28x workshop labs
**********************************************************************/
void main(void)
{
//--- CPU Initialization
    //
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    //
    InitSysCtrl();                      // Initialize the CPU (FILE: SysCtrl.c)
    DINT;


    //
    // Step 2. Initialize GPIO:
    // This example function is found in the F2837xD_Gpio.c file and
    // illustrates how to set the GPIO to it's default state.
    InitGpio();                         // Initialize the shared GPIO pins (FILE: Gpio.c)
    //
    // Initialize PIE control registers to their default state.
    // The default state is all PIE __interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    //
    InitPieCtrl();                      // Initialize and enable the PIE (FILE: PieCtrl.c)
    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block

    EALLOW;
    CpuSysRegs.SECMSEL.bit.PF1SEL = 1;  // Ensure DMA is connected to Peripheral Frame 1 bridge which contains the DAC

    DmaRegs.DMACTRL.bit.HARDRESET = 1;  // Perform a hard reset on DMA

    DmaRegs.DEBUGCTRL.bit.FREE = 1;     // Allow DMA to run free on emulation suspend

    // Configure the prescaler to the ePWM modules.  Max ePWM input clock is 100 MHz.
    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 0;     // EPWMCLK divider from PLLSYSCLK.  0=/1, 1=/2

    // Must disable the clock to the ePWM modules if you want all ePWM modules synchronized.
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    __asm (" nop"); // one NOP required after HARDRESET
    EDIS;
    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the __interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    //
    InitPieVectTable();

    for (int i=0; i<sizeof(all_task)/sizeof(all_task[0]); i++)
        all_task[i]->init();

#ifdef _SIM50HZ_SIN_WITH_PWM_
    sim50hz_sin.init();
#endif
#ifdef _SIM1PPS_WITH_PWM_
    sim_1pps.init();
#endif

    EALLOW;                       // Enable EALLOW protected register access
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;   // TBCLK to ePWM modules enabled
    EDIS;                          // Disable EALLOW protected register access

//--- Enable global interrupts
    asm(" CLRC INTM, DBGM");            // Enable global interrupts and realtime debug

//--- Main Loop
    while(1)                            // endless loop - wait for an interrupt
    {
        while (event_pending==0);
        for (int e=0; e<16; e++) {
            if (1<<e & event_pending)
                all_task[e]->process_event();
        }
    }


} //end of main()


/*** end of file *****************************************************/

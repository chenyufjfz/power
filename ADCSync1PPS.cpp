/*
 * ADCSync1PPS.cpp
 *
 *  Created on: Jun 6, 2018
 *      Author: yuchen
 */


#include <file.h>
#define _ADCSync1PPS_
#include "task.h"
#include "math.h"

ADCSync1PPS adc_sync_1pps;

#define USEECAP     1

#if USEECAP == 1
static volatile ECAP_REGS * ecap0 = &ECap1Regs;
static volatile ECAP_REGS * ecap1 = &ECap2Regs;
static volatile ECAP_REGS * ecap2 = &ECap3Regs;
#elif USEECAP == 4
static volatile ECAP_REGS * ecap0 = &ECap4Regs;
static volatile ECAP_REGS * ecap1 = &ECap5Regs;
static volatile ECAP_REGS * ecap2 = &ECap6Regs;
#endif

#define USE_EPWM 1
#if USE_EPWM == 1
static volatile EPWM_REGS * epwm = &EPwm1Regs;
#define GPIO_EWPM 0
#elif USE_EPWM == 2
static volatile EPWM_REGS * epwm = &EPwm2Regs;
#define GPIO_EWPM 2
#elif USE_EPWM == 3
static volatile EPWM_REGS * epwm = &EPwm3Regs;
#define GPIO_EWPM 149
#elif USE_EPWM == 4
static volatile EPWM_REGS * epwm = &EPwm4Regs;
#define GPIO_EWPM 6
#elif USE_EPWM == 5
static volatile EPWM_REGS * epwm = &EPwm5Regs;
#define GPIO_EWPM 8
#endif

#define USE_ADC 2
#if USE_ADC == 1
static volatile ADC_REGS * adc = &AdcaRegs;
static volatile ADC_RESULT_REGS * adc_result = &AdcaResultRegs;
#elif USE_ADC == 2
static volatile ADC_REGS * adc = &AdcbRegs;
static volatile ADC_RESULT_REGS * adc_result = &AdcbResultRegs;
#elif USE_ADC == 3
static volatile ADC_REGS * adc = &AdccRegs;
static volatile ADC_RESULT_REGS * adc_result = &AdcdResultRegs;
#endif

#define DMA_CHANNEL  2
#if DMA_CHANNEL == 1
static volatile CH_REGS * dma_ch = &DmaRegs.CH1;
#elif DMA_CHANNEL == 2
static volatile CH_REGS * dma_ch = &DmaRegs.CH2;
#elif DMA_CHANNEL == 3
static volatile CH_REGS * dma_ch = &DmaRegs.CH3;
#endif

#define GPIO_1PPS  4

#define ADC_RESOLUTION  12

#ifdef _LAUNCHXL_F28379D
#define ECAP1_PHASE_DELAY 7
#endif

#define DEFAULT_SAMPLE_PERIOD (CPU_CLOCK / SAMPLE_RATE)
#define SIN_AMP_BIT 11
#define SIN_AMP ((1 << SIN_AMP_BIT) - 1)

#define RESERVER_LEN 5
static uint32_t tick_h32 = 0;
enum {
    CAPTURE_NONE,
    CAPTURE_1PPS,
    CAPTURE_EPWM
};

enum {
    SYNC_ENTER,
    SYNC_INIT,
    SYNC_INIT2,
    SYNC_DMA,
    SYNC_FREQ,
};
static __interrupt void ecap0_isr(void)
{
    uint16_t ecflg = ecap0->ECFLG.all;
    if (ecflg & 0x20) //overflow happen
        tick_h32++;

    if (ecflg & 0x2)  //CAP evt1
        adc_sync_1pps.new_cap_evt();

    // Clear interrupts and arm ECAP
    ecap0->ECCLR.all = ecflg;           // Clear all processing __interrupt flags

    // Acknowledge this __interrupt to receive more __interrupts from group 4
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}

static __interrupt void dma_isr(void)
{
    adc_sync_1pps.new_cluster_ready();
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7; //clear DMA interrupt
}

/*
 * ecap0 capture tick for 1pps and epwm
 * ecap1 output Ideal 50Hz signal for easy socpe observe
 */
static void ecap_init()
{
    EALLOW;
#if USEECAP == 1
    InputXbarRegs.INPUT7SELECT = GPIO_EWPM;         // Set eCAP1 source to GPIO 4, pin 36
    InputXbarRegs.INPUT8SELECT = GPIO_EWPM;         // Set eCAP2 source to GPIO 0
    InputXbarRegs.INPUT9SELECT = GPIO_1PPS;         // Set eCAP3 source to GPIO 4
    PieVectTable.ECAP1_INT = &ecap0_isr;
    PieCtrlRegs.PIEIER4.bit.INTx1 = 1;      //PIEIER4.1 is ECAP1
#elif USEECAP == 4
    InputXbarRegs.INPUT10SELECT = GPIO_EWPM;         // Set eCAP4 source to GPIO 4, pin 36
    InputXbarRegs.INPUT11SELECT = GPIO_EWPM;         // Set eCAP5 source to GPIO 0
    InputXbarRegs.INPUT12SELECT = GPIO_1PPS;
    PieVectTable.ECAP4_INT = &ecap0_isr;
    PieCtrlRegs.PIEIER4.bit.INTx4 = 1;      //PIEIER4.4 is ECAP4
#endif

    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_ASYNC);

    ecap0->CTRPHS = 0;               // Phase = 0;
    ecap0->ECCTL2.all = 0x29;
    /*CONT_ONESHT:1 =1;    0 Continuous or one-shot, choose One short
      STOP_WRAP:2=0        2:1 Stop value for one-shot, Stop at event 1
      REARM:1    =1        3 One-shot re-arm
      TSCTRSTOP:1=0        4 TSCNT counter stop
      SYNCI_EN:1 =1        5 Counter sync-in select, Disable sync in
      SYNCO_SEL:2=0        7:6 Sync-out mode, let Synco output to ecap1
      SWSYNC:1   =0        8 SW forced counter sync
      CAP_APWM:1 =0        9 CAP/APWM operating mode select
      APWMPOL:1  =0        10 APWM output polarity select
      rsvd1:5;             15:11 Reserved*/

    ecap0->ECCLR.all = 0xFFFF;           // Clear all CAP __interrupt flags
    ecap0->ECEINT.all = 0x0022;          // enable CEvt1 and overflow

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
      FREE_SOFT:2 =0    15:14 Emulation mode, stop      */
    ecap0->ECCTL2.bit.TSCTRSTOP = 1;     // Start Counter

    ecap1->CTRPHS = ECAP1_PHASE_DELAY;          // Phase = 0;
    ecap1->ECCTL2.all = 0xE6;
    /*CONT_ONESHT:1 =0;    0 Continuous or one-shot, choose Continuous
      STOP_WRAP:2=3        2:1 Stop value for one-shot
      REARM:1    =0        3 One-shot re-arm,disabled
      TSCTRSTOP:1=0        4 TSCNT counter stop
      SYNCI_EN:1 =1        5 Counter sync-in select, enable sync in
      SYNCO_SEL:2=3        7:6 Sync-out mode, disable synco
      SWSYNC:1   =0        8 SW forced counter sync
      CAP_APWM:1 =0        9 CAP/APWM operating mode select
      APWMPOL:1  =0        10 APWM output polarity select
      rsvd1:5;             15:11 Reserved*/

    ecap1->ECCLR.all = 0xFFFF;      // Clear all CAP __interrupt flags
    ecap1->ECEINT.all = 0x0000;    // enable overflow bit
    ecap1->ECCTL1.all = 0x100;      // Same as ecap0
    ecap1->ECCTL2.bit.TSCTRSTOP = 1;     // Start Counter

    ecap2->CTRPHS = ECAP1_PHASE_DELAY * 2;
    ecap2->ECCTL2.all = 0xE6;
    /*CONT_ONESHT:1 =0;    0 Continuous or one-shot, choose Continuous
      STOP_WRAP:2=3        2:1 Stop value for one-shot, Stop at event 1
      REARM:1    =0        3 One-shot re-arm,disabled
      TSCTRSTOP:1=0        4 TSCNT counter stop
      SYNCI_EN:1 =1        5 Counter sync-in select, enable sync in
      SYNCO_SEL:2=3        7:6 Sync-out mode, disable synco
      SWSYNC:1   =0        8 SW forced counter sync
      CAP_APWM:1 =0        9 CAP/APWM operating mode select
      APWMPOL:1  =0        10 APWM output polarity select
      rsvd1:5;             15:11 Reserved*/
    ecap2->ECCLR.all = 0xFFFF;      // Clear all CAP __interrupt flags
    ecap2->ECEINT.all = 0x00000;    // disable overflow bit
    ecap2->ECCTL1.all = 0x100;      // Same as ecap2
    ecap2->ECCTL2.bit.TSCTRSTOP = 1;     // Start Counter

    ecap0->ECCTL2.bit.SWSYNC = 1; //load ecap0 phase
    ecap1->ECCTL2.bit.SWSYNC = 1; //load ecap1 phase
    ecap2->ECCTL2.bit.SWSYNC = 1; //load ecap2 phase
    EDIS;

    IER |= 0x8;     //eCAPx is ACK4
}

static void epwm_init()
{
    EALLOW;

#if USE_EPWM == 1
    DevCfgRegs.SOFTPRES2.bit.EPWM1 = 1;     // ePWM1 is reset
    DevCfgRegs.SOFTPRES2.bit.EPWM1 = 0;     // ePWM1 is released from reset
#elif USE_EPWM == 2
    DevCfgRegs.SOFTPRES2.bit.EPWM2 = 1;     // ePWM2 is reset
    DevCfgRegs.SOFTPRES2.bit.EPWM2 = 0;     // ePWM2 is released from reset
#elif USE_EPWM == 3
    DevCfgRegs.SOFTPRES2.bit.EPWM3 = 1;     // ePWM3 is reset
    DevCfgRegs.SOFTPRES2.bit.EPWM3 = 0;     // ePWM3 is released from reset
#elif USE_EPWM == 4
    DevCfgRegs.SOFTPRES2.bit.EPWM4 = 1;     // ePWM1 is reset
    DevCfgRegs.SOFTPRES2.bit.EPWM4 = 0;     // ePWM1 is released from reset
#elif USE_EPWM == 5
    DevCfgRegs.SOFTPRES2.bit.EPWM5 = 1;     // ePWM1 is reset
    DevCfgRegs.SOFTPRES2.bit.EPWM5 = 0;     // ePWM1 is released from reset
#endif
    EDIS;

    epwm->TBCTL.all = 0x0033;           // Configure timer control register
    // bit 15-14     00:     FREE/SOFT, 00 = stop when emulation suspend
    // bit 13        0:      PHSDIR, 0 = count down after sync event
    // bit 12-10     000:    CLKDIV, 000 => TBCLK = HSPCLK/1, 111 => 8
    // bit 9-7       000:    HSPCLKDIV, 000 => HSPCLK = EPWMCLK/1, 111 => 8
    // bit 6         0:      SWFSYNC, 0 = no software sync produced
    // bit 5-4       11:     SYNCOSEL, 11 = sync-out disabled
    // bit 3         0:      PRDLD, 0 = reload PRD on counter=0
    // bit 2         0:      PHSEN, 0 = phase control disabled
    // bit 1-0       11:     CTRMODE, 11 = timer stopped (disabled)

    epwm->TBCTR = 0x0000;               // Clear timer counter
    epwm->TBPRD = DEFAULT_SAMPLE_PERIOD - 1;      // Set timer period
    epwm->TBPHS.bit.TBPHS = 0x0000;     // Set timer phase

    epwm->ETPS.all = 0x0100; // Configure SOCA
    // bit 15-14     00:     EPWMxSOCB, read-only
    // bit 13-12     00:     SOCBPRD, don't care
    // bit 11-10     00:     EPWMxSOCA, read-only
    // bit 9-8       01:     SOCAPRD, 01 = generate SOCA on first event
    // bit 7-4       0000:   reserved
    // bit 3-2       00:     INTCNT, don't care
    // bit 1-0       00:     INTPRD, don't care

    epwm->ETSEL.all = 0x0A00;           // Enable SOCA to DMA
    // bit 15        0:      SOCBEN, 0 = disable SOCB
    // bit 14-12     000:    SOCBSEL, don't care
    // bit 11        1:      SOCAEN, 1 = enable SOCA
    // bit 10-8      010:    SOCASEL, 010 = SOCA on PRD event
    // bit 7-4       0000:   reserved
    // bit 3         0:      INTEN, 0 = disable interrupt
    // bit 2-0       000:    INTSEL, don't care
    epwm->AQCTLA.bit.PRD = 2;         // Set on PRD
    epwm->AQCTLA.bit.CAU = 1;       // Clear on CMPA
    epwm->CMPA.bit.CMPA = DEFAULT_SAMPLE_PERIOD / 4-1;      //set to 0 when equal to CMPA
}

/*
 * route epwm signal to gpio output for ecap and easy scope observe
 */
static void init_xbar_gpio()
{
    EALLOW;
#if USE_EPWM == 1
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up on GPIO0 (EPWM1A), see pinmux table 4.4
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
#elif USE_EPWM == 2
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;    // Disable pull-up on GPIO2 (EPWM2A)
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A
#elif USE_EPWM == 3
    GpioCtrlRegs.GPEPUD.bit.GPIO149 = 1;    // Disable pull-up on GPIO149 (EPWM3A)
    GpioCtrlRegs.GPEMUX2.bit.GPIO149 = 1;   // Configure GPIO149 as EPWM3A
#elif USE_EPWM == 4
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;    // Disable pull-up on GPIO6 (EPWM4A)
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // Configure GPIO6 as EPWM4A
#elif USE_EPWM == 5
    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 1;    // Disable pull-up on GPIO8 (EPWM5A)
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;   // Configure GPIO8 as EPWM5A
#endif
    EDIS;
}

/*
 * adc capture power line signal
 */
static void adc_init()
{
    uint16_t which_adc, resolution, signalmode, acqps;
    EALLOW;
    adc->ADCCTL2.bit.PRESCALE = 0;
#if ADC_RESOLUTION == 12
    resolution = ADC_RESOLUTION_12BIT;
    signalmode = ADC_SIGNALMODE_SINGLE;
    acqps = 14; // 75ns
#else
    resolution = ADC_RESOLUTION_16BIT;
    signalmode = ADC_SIGNALMODE_DIFFERENTIAL;
    acqps = 63; // 320ns
#endif

#if USE_ADC ==1
    which_adc = ADC_ADCA;
#elif USE_ADC ==2
    which_adc = ADC_ADCB;
#elif USE_ADC ==3
    which_adc = ADC_ADCC;
#endif
    AdcSetMode(which_adc, resolution, signalmode);

    // Set pulse positions to late
    //
    adc->ADCCTL1.bit.INTPULSEPOS = 1;

    // Power up the ADC
    //
    adc->ADCCTL1.bit.ADCPWDNZ = 1;
    F28x_usDelay(20000);

    adc->ADCSOC0CTL.bit.CHSEL  = 2; //Choose ADCIN2, for ADCINA2, it is pin 29
    adc->ADCSOC1CTL.bit.CHSEL  = 2;
    adc->ADCSOC2CTL.bit.CHSEL  = 2;
    adc->ADCSOC3CTL.bit.CHSEL  = 2;
    adc->ADCSOC4CTL.bit.CHSEL  = 2;
    adc->ADCSOC5CTL.bit.CHSEL  = 2;
    adc->ADCSOC6CTL.bit.CHSEL  = 2;
    adc->ADCSOC7CTL.bit.CHSEL  = 2;

    adc->ADCSOC0CTL.bit.ACQPS  = acqps;
    adc->ADCSOC1CTL.bit.ACQPS  = acqps;
    adc->ADCSOC2CTL.bit.ACQPS  = acqps;
    adc->ADCSOC3CTL.bit.ACQPS  = acqps;
    adc->ADCSOC4CTL.bit.ACQPS  = acqps;
    adc->ADCSOC5CTL.bit.ACQPS  = acqps;
    adc->ADCSOC6CTL.bit.ACQPS  = acqps;
    adc->ADCSOC7CTL.bit.ACQPS  = acqps;

    // Trigger SOC0 from EPWM
    //
#if USE_EPWM == 1
    adc->ADCSOC0CTL.bit.TRIGSEL = 5; //EPWM 1
#elif USE_EPWM == 2
    adc->ADCSOC0CTL.bit.TRIGSEL = 7; //EPWM 2
#elif USE_EPWM == 3
    adc->ADCSOC0CTL.bit.TRIGSEL = 9; //EPWM 3
#elif USE_EPWM == 4
    adc->ADCSOC0CTL.bit.TRIGSEL = 11; //EPWM 4
#elif USE_EPWM == 5
    adc->ADCSOC0CTL.bit.TRIGSEL = 13; //EPWM 5
#endif

    // Trigger all other SOCs from INT1 (EOC on SOC0)
    //
    adc->ADCINTSOCSEL1.bit.SOC1 = 1;
    adc->ADCINTSOCSEL1.bit.SOC2 = 1;
    adc->ADCINTSOCSEL1.bit.SOC3 = 1;
    adc->ADCINTSOCSEL1.bit.SOC4 = 1;
    adc->ADCINTSOCSEL1.bit.SOC5 = 1;
    adc->ADCINTSOCSEL1.bit.SOC6 = 1;
    adc->ADCINTSOCSEL1.bit.SOC7 = 1;

    adc->ADCINTSEL1N2.bit.INT1E = 1;    // Enable INT1 flag
    adc->ADCINTSEL1N2.bit.INT2E = 1;    // Enable INT2 flag
    adc->ADCINTSEL3N4.bit.INT3E = 0;    // Disable INT3 flag
    adc->ADCINTSEL3N4.bit.INT4E = 0;    // Disable INT4 flag

    adc->ADCINTSEL1N2.bit.INT1CONT = 1;
    adc->ADCINTSEL1N2.bit.INT2CONT = 1;
    adc->ADCINTSEL1N2.bit.INT1SEL = 0;  // End of SOC0
    adc->ADCINTSEL1N2.bit.INT2SEL = 7; // End of SOC7

    EDIS;
}

/*
 * dma fetch ADC result to memory
 */
static void dma_init(uint16_t * buf)
{
    EALLOW;

    // Set up SOURCE address:
    //
    dma_ch->SRC_BEG_ADDR_SHADOW = (Uint32) &(adc_result->ADCRESULT0);
    dma_ch->SRC_ADDR_SHADOW =     (Uint32) &(adc_result->ADCRESULT0);

    // Set up DESTINATION address:
    //
    dma_ch->DST_BEG_ADDR_SHADOW = (Uint32) buf;  // Point to beginning of destination buffer
    dma_ch->DST_ADDR_SHADOW =     (Uint32) buf;

    // Set up BURST registers:
    //
    dma_ch->BURST_SIZE.all = CHANNEL - 1;      // Number of words(X-1) x-ferred in a burst.
    dma_ch->SRC_BURST_STEP = 1;   // Increment source addr between each word x-ferred.
    dma_ch->DST_BURST_STEP = 1;   // Increment dest addr between each word x-ferred.

    // Set up TRANSFER registers:
    //
    dma_ch->TRANSFER_SIZE = ADC_DMA_TRANSFER_SIZE - 1;        // Number of bursts per transfer, DMA interrupt will occur after completed transfer.
    dma_ch->SRC_TRANSFER_STEP = 1 - CHANNEL; // TRANSFER_STEP is ignored when WRAP occurs.
    dma_ch->DST_TRANSFER_STEP = 1; // TRANSFER_STEP is ignored when WRAP occurs.

    // Set up WRAP registers:
    //
    dma_ch->SRC_WRAP_SIZE = ADC_DMA_TRANSFER_SIZE - 1; // Wrap source address after N bursts, equal to TRANSFER_SIZE disable WARP
    dma_ch->SRC_WRAP_STEP = 0; // Step for source wrap
    dma_ch->DST_WRAP_SIZE = 0; // Wrap destination address after  N bursts.
    dma_ch->DST_WRAP_STEP = 0; // Step for destination wrap


#if USE_ADC == 1
    int trig_source = DMA_ADCAINT2;    // persel - Source select
#elif USE_ADC == 2
    int trig_source = DMA_ADCBINT2;    // persel - Source select
#elif USE_ADC == 3
    int trig_source = DMA_ADCCINT2;    // persel - Source select
#endif

#if DMA_CHANNEL == 1
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH1 = trig_source; // persel - Source select
    PieVectTable.DMA_CH1_INT = &dma_isr;
    PieCtrlRegs.PIEIER7.bit.INTx1 = 1;      //PIEIER7.1 is DMA CH1
#elif DMA_CHANNEL == 2
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH2 = trig_source; // persel - Source select
    PieVectTable.DMA_CH2_INT = &dma_isr;
    PieCtrlRegs.PIEIER7.bit.INTx2 = 1;      //PIEIER7.2 is DMA CH2
#elif DMA_CHANNEL == 3
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH3 = trig_source; // persel - Source select
    PieVectTable.DMA_CH3_INT = &dma_isr;
    PieCtrlRegs.PIEIER7.bit.INTx3 = 1;      //PIEIER7.3 is DMA CH3
#endif

    dma_ch->MODE.all = 0x8900;
    dma_ch->MODE.bit.PERINTSEL = DMA_CHANNEL;
    /*PERINTSEL:5 =DMA     4:0  Peripheral Interrupt and Sync Select
      rsvd1:2              6:5  Reserved
      OVRINTE:1   =0       7    Overflow Interrupt Enable
      PERINTE:1   =1       8    Peripheral Interrupt Enable
      CHINTMODE:1 =0       9    Channel Interrupt Mode, 0 means at the begin of transfer
      ONESHOT:1   =0       10   One Shot Mode Bit
      CONTINUOUS:1=1       11   Continuous Mode Bit
      rsvd2:2     =0       13:12    Reserved
      DATASIZE:1  =0       14   Data Size Mode Bit
      CHINTE:1    =1       15   Channel Interrupt Enable Bit
      */

    //Clear any spurious flags: interrupt and sync error flags
    dma_ch->CONTROL.bit.PERINTCLR = 1;
    dma_ch->CONTROL.bit.ERRCLR = 1;

    EDIS;
    IER |= 0x40;     //DMAx is ACK7
}

/*
 * Inout sin_tbl, store sin value in sin_tbl
 */
static void sin_table_init(int16_t * sin_tbl)
{
    for (int i=0; i<SIN_TABLE_SIZE-1; i++) {
        sin_tbl[i] = sin(i * PI/2/(SIN_TABLE_SIZE - 1)) * SIN_AMP;
    }
    sin_tbl[SIN_TABLE_SIZE - 1] = SIN_AMP;
}

//Input cap_state
void ADCSync1PPS::set_cap_state(uint16_t cap_state)
{
    capture_state = cap_state;
    switch (cap_state) {
    case CAPTURE_NONE:
        ecap0->ECEINT.bit.CEVT1 = 0;
        break;
    case CAPTURE_EPWM:
#if USEECAP == 1
        InputXbarRegs.INPUT7SELECT = GPIO_EWPM;
#elif USEECAP == 4
        InputXbarRegs.INPUT10SELECT = GPIO_EWPM;
#endif
        ecap0->ECEINT.bit.CEVT1 = 1;
        break;
    case CAPTURE_1PPS:
#if USEECAP == 1
        InputXbarRegs.INPUT7SELECT = GPIO_1PPS;
#elif USEECAP == 4
        InputXbarRegs.INPUT10SELECT = GPIO_1PPS;
#endif
        ecap0->ECEINT.bit.CEVT1 = 1;
        break;
    }
}

//it runs at interrupt context
void ADCSync1PPS::new_cap_evt()
{
    EALLOW;
    uint16_t ecflg = ecap0->ECFLG.all;
    uint32_t ecap1 = ecap0->CAP1;
    cap1 = tick_h32;
    if (ecflg & 0x20) { //overflow happen
        if (ecap1 > 0x70000000)
            cap1--;
    }
    cap1 = (cap1 << 32) + ecap1;
    switch (sync_state) {
    case SYNC_ENTER:
        break;
    case SYNC_INIT:
        sync_state = SYNC_INIT2;
        ecap0->ECEINT.all = 0x20; //only enable overflow disable CAP1
        //set ecap0 to continuous mode, still monitor epwm signal until next DMA interrupt happen
        ecap0->ECCTL2.bit.CONT_ONESHT = 0;
        break;
    case SYNC_FREQ:
        break;
    case SYNC_INIT2:
    case SYNC_DMA:
    default:
        Assert(0);
    }

    ecap0->ECCTL2.bit.REARM = 1;

    EDIS;

}

//it runs at interrupt context
void ADCSync1PPS::new_cluster_ready()
{
    switch (sync_state) {
    case SYNC_INIT:
        Assert(0);
    case SYNC_INIT2:
        sync_state = SYNC_DMA;
        return;
    case SYNC_DMA:
    {
        cap1 = ecap0->CAP1; //assume tick_h32 is 0
        uint32_t diff = cap1 - dma_sync[0].tick;
        Assert(diff == ADC_DMA_TRANSFER_SIZE * (uint32_t) dma_sync[0].sample_period);
        sync_state = SYNC_FREQ;
        return;
    }
    case SYNC_FREQ:
        return;
    default:
        return;
    }

    adc_buf_tail++;
    dma_ch->DST_ADDR_SHADOW = (Uint32) &(adc_buf[adc_buf_tail % ADC_DMA_CLUSTER_COUNT]);
    event_pending |= ADCSYNC_EVENT_MASK;
}

uint64_t ADCSync1PPS::get_tick()
{
    uint64_t ret = 0;
    int overflow;
    uint32_t counter;
    unsigned int old_st = __disable_interrupts();
    ret = tick_h32;
    overflow = ecap0->ECFLG.bit.CTROVF;
    counter = ecap0->TSCTR;
    if (overflow==0 && counter < 30) { //maybe overflow happen, read again
        overflow = ecap0->ECFLG.bit.CTROVF;
        counter = ecap0->TSCTR;
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

void ADCSync1PPS::init()
{
    adc_buf_head = 0;
    adc_buf_tail = 0;
    keep_display = false;
    capture_state = CAPTURE_EPWM;
    sync_state = SYNC_ENTER;
    adc_init();
    sin_table_init(sin_tbl);
    init_xbar_gpio();
    dma_init(adc_buf);
    ecap_init();
    epwm_init();
    event_pending |= ADCSYNC_EVENT_MASK;
}

void ADCSync1PPS::process_event()
{
    int * p_evt = (int*)&event_pending;
    __and(p_evt, ~ADCSYNC_EVENT_MASK);

    if (sync_state == SYNC_ENTER) {
        sync_state = SYNC_INIT;
        uint16_t old_ier = IER;
        IER = 0x48; //Enable ecap and DMA IER, Now ecap is in epwm capture state
        EALLOW;
        // Start DMA channel
        //
        dma_ch->CONTROL.bit.PERINTCLR = 1;
        dma_ch->CONTROL.bit.RUN = 1;

        // Enable the timer for epwm in count up mode,
        // Ecap and dma interrupt will happen and change sync_state
        epwm->TBCTL.bit.CTRMODE = 0x0;
        EDIS;
        while (sync_state == SYNC_INIT || sync_state == SYNC_INIT2);    // hold CPU until ecap and dma interrupt finish

        //recording 1st sample time base
        dma_sync[0].sample_idx = 0;
        dma_sync[0].tick = cap1;
        dma_sync[0].sample_period = DEFAULT_SAMPLE_PERIOD;

        IER = 0x40;
        //now sync== SYNC_DMA, and ecap cap1 interrupt is disabled
        while (sync_state == SYNC_DMA); //  // hold CPU until dma interrupt finish

        //now sync==SYNC_FREQ
        set_cap_state(CAPTURE_1PPS);

        IER = old_ier;

        printf("1st dma sample time=%lu:%lu\n\r", (uint32_t) (dma_sync[0].tick >> 32), (uint32_t) (dma_sync[0].tick & 0xffffffff));
    }

    uint64_t a = 1;
    uint64_t b = 3;
    uint64_t c = a;
    uint64_t tb, te;
    uint32_t td;
    tb = get_tick();
    for (int i=0; i<100; i++)
        c = c + b;
    te = get_tick();
    td = te - tb;
    uint64_t d = c;
    printf("%lu, c=%lx:%lx\n\r", td, (uint32_t) (d >> 32), (uint32_t) (d & 0xffffffff));
    tb = get_tick();
    te = get_tick();
    td = te - tb;
    d = c;
    printf("%lu, c=%lx:%lx\n\r", td, (uint32_t) (d >> 32), (uint32_t) (d & 0xffffffff));
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

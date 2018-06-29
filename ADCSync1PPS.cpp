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

#define USE_ADC 1
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
#define MAX_CLOCK_DIFF (CPU_CLOCK / 1000000 * MAX_CLOCK_JITTER)


#define ADC_DMA_TRANSFER_SIZE (SAMPLE_RATE / COMPUTE_RATE)
#define ADC_DMA_CLUSTER_SIZE (ADC_DMA_TRANSFER_SIZE * CHANNEL)
#define SIN_TABLE_SIZE (SAMPLE_RATE / (POWER_CYCLE * 4) + 1)
#define COMPUTE_SAMPLE (SAMPLE_RATE / POWER_CYCLE * COMPUTE_CYCLE)
#define COMPUTE_BUF_SIZE (COMPUTE_SAMPLE * CHANNEL)
#define ADC_DMA_CLUSTER_COUNT ( (COMPUTE_BUF_SIZE - 1) / ADC_DMA_CLUSTER_SIZE + 3 + BUF_DMA_RESERVED_COUNT)
#define ADC_BUF_SIZE (ADC_DMA_CLUSTER_SIZE * ADC_DMA_CLUSTER_COUNT)

#define VI_SHIFT_BIT2 (ADC_RESOLUTION + SIN_AMP_BIT - 1 - 16)

#if ADC_RESOLUTION == 12
// 100ns
#define ACQPS_WINDOW 20
#elif ADC_RESOLUTION == 16
// 400ns
#define ACQPS_WINDOW 80
#endif

#if (SIN_TABLE_SIZE - 1) > (1 << (32 - ADC_RESOLUTION - SIN_AMP_BIT - 1))
#error (SIN_TABLE_SIZE - 1) > (1 << (32 - ADC_RESOLUTION - SIN_AMP_BIT - 1))
#endif
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
    SYNC_UTC0,
    SYNC_UTC1,
    SYNC_UTC2,
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
    //--- Reset the ADC.  This is good programming practice.
    DevCfgRegs.SOFTPRES13.bit.ADC_A = 1;    // ADC is reset
    DevCfgRegs.SOFTPRES13.bit.ADC_A = 0;    // ADC is released from reset

    //--- Configure the ADC base registers
    adc->ADCCTL1.all = 0x0004;      // Main ADC configuration
    // bit 15-14     00:     reserved
    // bit 13        0:      ADCBSY, ADC busy, read-only
    // bit 12        0:      reserved
    // bit 11-8      0's:    ADCBSYCHN, ADC busy channel, read-only
    // bit 7         0:      ADCPWDNZ, ADC power down, 0=powered down, 1=powered up
    // bit 6-3       0000:   reserved
    // bit 2         1:      INTPULSEPOS, INT pulse generation, 0=start of conversion, 1=end of conversion
    // bit 1-0       00:     reserved

    adc->ADCCTL2.all = 0x0006;      // ADC clock configuration
    // bit 15-8      0's:    reserved
    // bit 7         0:      SIGNALMODE, configured by AdcSetMode() below to get calibration correct
    // bit 6         0:      RESOLUTION, configured by AdcSetMode() below to get calibration correct
    // bit 5-4       00:     reserved
    // bit 3-0       0110:   PRESCALE, ADC clock prescaler.  0110=CPUCLK/4

    AdcaRegs.ADCBURSTCTL.all = 0x0000;
// bit 15        0:      BURSTEN, 0=burst mode disabled, 1=burst mode enabled
// bit 14-12     000:    reserved
// bit 11-8      0000:   BURSTSIZE, 0=1 SOC converted (don't care)
// bit 7-6       00:     reserved
// bit 5-0       000000: BURSTTRIGSEL, 00=software only (don't care)
#if ADC_RESOLUTION == 12
    resolution = ADC_RESOLUTION_12BIT;
    signalmode = ADC_SIGNALMODE_SINGLE;
#else
    resolution = ADC_RESOLUTION_16BIT;
    signalmode = ADC_SIGNALMODE_DIFFERENTIAL;
#endif

    acqps = ACQPS_WINDOW - 1;

#if USE_ADC ==1
    which_adc = ADC_ADCA;
#elif USE_ADC ==2
    which_adc = ADC_ADCB;
#elif USE_ADC ==3
    which_adc = ADC_ADCC;
#endif
    AdcSetMode(which_adc, resolution, signalmode);


    adc->ADCSOC0CTL.bit.CHSEL  = 2; //Choose ADCIN2, for ADCINA2, it is pin 29, for ADCINB2 it is pin 28
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


    // Power up the ADC
    //
    adc->ADCCTL1.bit.ADCPWDNZ = 1;
    F28x_usDelay(100000);

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
    dma_ch->DST_WRAP_SIZE = ADC_DMA_TRANSFER_SIZE - 1; // Wrap destination address after  N bursts.
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
static void sin_table_init(int16_t * sin_tbl, int sin_tbl_size)
{
    for (int i=0; i<sin_tbl_size-1; i++) {
        sin_tbl[i] = sin(i * PI/2/(sin_tbl_size - 1)) * SIN_AMP;
    }
    sin_tbl[sin_tbl_size - 1] = SIN_AMP;
}

struct VIBuf {
    union {
        uint64_t tick;
        uint64_t sample;
        uint64_t utc;
    } t;
    int16_t vi[CHANNEL][2];
};

class ComputeVI {
protected:
    int16_t sin_tbl[SIN_TABLE_SIZE];
    uint16_t adc_buf[ADC_BUF_SIZE];
    volatile uint64_t adc_buf_tail;
public:
    void init();
    void new_cluster_ready();
    uint64_t get_latest_sample() const {
        return adc_buf_tail / CHANNEL;
    }
    uint64_t get_earliest_sample() const {
        return (adc_buf_tail - ADC_BUF_SIZE) / CHANNEL + 2;
    }
    int operator () (VIBuf & vibuf);
} compute_vi;

void ComputeVI::init()
{
    adc_buf_tail = 0;
    sin_table_init(sin_tbl, SIN_TABLE_SIZE);
    dma_init(adc_buf);
}

//it run at interrupt context
void ComputeVI::new_cluster_ready()
{
    adc_buf_tail +=ADC_DMA_CLUSTER_SIZE;

    uint32_t addr = (uint32_t) &adc_buf[adc_buf_tail % ADC_BUF_SIZE];
    EALLOW;
    dma_ch->DST_ADDR_SHADOW = addr;
    dma_ch->DST_BEG_ADDR_SHADOW = addr;
    EDIS;
}

enum {
    COMPUTEVI_INTIME,
    COMPUTEVI_LATE,
    COMPUTEVI_EARLY,
};
/* Main compute function, it doesn't run at interrupt context
 * Inout: vibuf
 *        vibuf.sample, In: the expected sample wanted; Out: the actual sample
 * Return 0, in valid time
 *        1, too late
 *        2, too early
 */
int ComputeVI::operator() (VIBuf & vibuf)
{
    uint64_t valid_tail = (adc_buf_tail - ADC_DMA_CLUSTER_SIZE) / CHANNEL;
    uint64_t valid_head = (adc_buf_tail - ADC_BUF_SIZE) / CHANNEL + 2;

    if (vibuf.t.sample < valid_head) {
        vibuf.t.sample = valid_head;
        return COMPUTEVI_LATE;
    }
    if (vibuf.t.sample + COMPUTE_SAMPLE - 1 > valid_tail) {
        vibuf.t.sample = valid_head;
        return COMPUTEVI_EARLY;
    }

    uint16_t * head_sample = &adc_buf[(vibuf.t.sample * CHANNEL) % ADC_BUF_SIZE];
    int32_t vi[CHANNEL][2] = {0};
    for (int phase=0; phase<=3; phase++) {
        int16_t * psin;
        int16_t * pcos;
        int ds;
        switch (phase) {
        case 0:
            psin = &sin_tbl[0];
            pcos = &sin_tbl[SIN_TABLE_SIZE - 1];
            ds = 1;
            break;
        case 1:
            psin = &sin_tbl[SIN_TABLE_SIZE - 1];
            pcos = &sin_tbl[0];
            ds = -1;
            break;
        case 2:
            psin = &sin_tbl[0];
            pcos = &sin_tbl[SIN_TABLE_SIZE - 1];
            ds = 1;
            break;
        case 3:
            psin = &sin_tbl[SIN_TABLE_SIZE - 1];
            pcos = &sin_tbl[0];
            ds = -1;
            break;
        }
        int loop_len = min(SIN_TABLE_SIZE - 1, (&adc_buf[ADC_BUF_SIZE] - head_sample) / CHANNEL);
        int32_t vi1[CHANNEL][2] = {0};
        for (int i=0; i<loop_len; i++, psin += ds, pcos -= ds) {
            for (int j=0; j<CHANNEL; j++) {
                vi1[j][0] += __mpyxu(*psin, *head_sample);
                vi1[j][1] += __mpyxu(*pcos, *head_sample++);
            }
        }
        if (head_sample == &adc_buf[ADC_BUF_SIZE]) {
            head_sample = adc_buf; //turn round
            loop_len = SIN_TABLE_SIZE - 1 -loop_len;
            for (int i=0; i<loop_len; i++, psin += ds, pcos -= ds) {
                for (int j=0; j<CHANNEL; j++) {
                    vi1[j][0] += __mpyxu(*psin, *head_sample);
                    vi1[j][1] += __mpyxu(*pcos, *head_sample++);
                }
            }
        }
        for (int j=0; j<CHANNEL; j++) {
            switch (phase) {
            case 0:
                vi[j][0] +=vi1[j][0] / (SIN_TABLE_SIZE - 1);
                vi[j][1] +=vi1[j][1] / (SIN_TABLE_SIZE - 1);
                break;
            case 1:
                vi[j][0] +=vi1[j][0] / (SIN_TABLE_SIZE - 1);
                vi[j][1] -=vi1[j][1] / (SIN_TABLE_SIZE - 1);
                break;
            case 2:
                vi[j][0] -=vi1[j][0] / (SIN_TABLE_SIZE - 1);
                vi[j][1] -=vi1[j][1] / (SIN_TABLE_SIZE - 1);
                break;
            case 3:
                vi[j][0] -=vi1[j][0] / (SIN_TABLE_SIZE - 1);
                vi[j][1] +=vi1[j][1] / (SIN_TABLE_SIZE - 1);
                break;
            }
        }
    }

    for (int j=0; j<CHANNEL; j++) { //A * sin(t+phase)
        int32_t x = vi[j][0] / (4 * COMPUTE_CYCLE) >> VI_SHIFT_BIT2; //sin
        int32_t y = vi[j][1] / (4 * COMPUTE_CYCLE) >> VI_SHIFT_BIT2; //cos
        vibuf.vi[j][0] = sqrt(y * y + x * x); //it is A
        vibuf.vi[j][1] = atan2(vi[j][1], vi[j][0]) / PI * 18000; //it is phase
    }
    return COMPUTEVI_INTIME;
}

/*
 * Input t0,
 * Input t1,
 * Input _tick_ps
 * Input max_diff
 * It t0 and t1 is compatible, return true else false
 */
bool ADCSync1PPS::compatible(const TickTime & t0, const TickTime & t1, uint32_t _tick_ps, uint32_t max_diff)
{
    int64_t time_diff, expect_tick_diff, tick_diff;
    if (t1.utc > t0.utc) {
        time_diff = t1.utc - t0.utc;
        expect_tick_diff = time_diff * _tick_ps / UTC_UNIT;
        tick_diff = t1.tick - t0.tick;
    } else {
        time_diff = t0.utc - t1.utc;
        expect_tick_diff = time_diff * _tick_ps / UTC_UNIT;
        tick_diff = t0.tick - t1.tick;
    }
    return (abs(expect_tick_diff - tick_diff) < max_diff);
}

void ADCSync1PPS::clear_utc_sync_tbl()
{
    for (int i=0; i<UTCSYNC_SIZE; i++) {
        utc_sync_tbl[i].tick = 0;
        utc_sync_tbl[i].utc = 0;
    }
    printf("clear utc sync table\n\r");
}

void ADCSync1PPS::push_utc_sync_tbl(uint64_t tick, uint64_t utc)
{
    for (int i=0; i<UTCSYNC_SIZE-1; i++)
        utc_sync_tbl[i] = utc_sync_tbl[i+1];

    utc_sync_tbl[UTCSYNC_SIZE-1].tick = tick;
    utc_sync_tbl[UTCSYNC_SIZE-1].utc = utc;
}

void ADCSync1PPS::clear_dma_sync_tbl()
{
    for (int i=0; i<DMASYNC_SIZE; i++) {
        dma_sync_tbl[i].sample_idx = 0;
        dma_sync_tbl[i].tick = 0;
        dma_sync_tbl[i].sample_period = DEFAULT_SAMPLE_PERIOD;
    }
}

void ADCSync1PPS::push_dma_sync_tbl(int sample_period, uint64_t sample_idx, uint64_t tick)
{
    for (int i=0; i<DMASYNC_SIZE-1; i++)
        dma_sync_tbl[i] = dma_sync_tbl[i+1];

    dma_sync_tbl[DMASYNC_SIZE-1].tick = tick;
    dma_sync_tbl[DMASYNC_SIZE-1].sample_period = sample_period;
    dma_sync_tbl[DMASYNC_SIZE-1].sample_idx = sample_idx;
}

void ADCSync1PPS::update_tick_ps()
{
    uint64_t time_diff, tick_diff;
    uint32_t new_tick_ps;

    time_diff = utc_sync_tbl[UTCSYNC_SIZE-1].utc - utc_sync_tbl[UTCSYNC_SIZE-2].utc;
    tick_diff = utc_sync_tbl[UTCSYNC_SIZE-1].tick - utc_sync_tbl[UTCSYNC_SIZE-2].tick;
    new_tick_ps = tick_diff * 1000 / time_diff;
    tick_ps = (tick_ps * 3 + new_tick_ps) / 4;
}

#define COMPUTE_PERIOD (UTC_UNIT / COMPUTE_RATE)
#if UTC_UNIT % COMPUTE_RATE != 0
#error UTC_UNIT % COMPUTE_RATE != 0
#endif

void ADCSync1PPS::update_record_utc()
{
    uint64_t sample = compute_vi.get_earliest_sample();
    sample2utc(sample, record_utc);
    record_utc = record_utc / COMPUTE_PERIOD;  //here record_utc is compute cycles
    record_utc = (record_utc + 1) * COMPUTE_PERIOD; //conver record_utc to time
    printf("update_record_utc %lld\n\r", record_utc);
}
/*
 * In tick
 * Return sample
 */
uint64_t ADCSync1PPS::tick2sample(uint64_t tick)
{
    for (int i=DMASYNC_SIZE-1; i>=0; i--)
        if (tick >= dma_sync_tbl[i].tick && dma_sync_tbl[i].tick!=0) {
            uint64_t ret = (tick - dma_sync_tbl[i].tick + dma_sync_tbl[i].sample_period / 2) / dma_sync_tbl[i].sample_period;
            return ret + dma_sync_tbl[i].sample_idx;
        }
    return 0;
}

/*
 * In sample
 * Return tick
 */
uint64_t ADCSync1PPS::sample2tick(uint64_t sample)
{
    for (int i=DMASYNC_SIZE-1; i>=0; i--)
        if (sample >= dma_sync_tbl[i].sample_idx && dma_sync_tbl[i].tick!=0) {
            uint64_t ret = (sample - dma_sync_tbl[i].sample_idx) * dma_sync_tbl[i].sample_period;
            return ret + dma_sync_tbl[i].tick + ACQPS_WINDOW;
        }
    return 0;
}

enum {
    PERFECT_ESTIMATE,
    FUTURE_ESTIMATE,
    PAST_ESTIMATE,
    WRONG_ESTIMATE
};
/*
 * In tick
 * Out utc
 * Return 0, perfect estimate
 *        1, future estimate
 *        2, past estimate
 *        3, wrong estimate
 */
#define LIMIT_TICK_TH 0x80000000000
#define LIMIT_UTC_TH (LIMIT_TICK_TH / CPU_CLOCK * UTC_UNIT)
template <class UTC_TYPE> int ADCSync1PPS::tick2utc(uint64_t tick, UTC_TYPE & utc)
{
    int ret, i;
    for (i=UTCSYNC_SIZE-1; i>=0; i--) {
        if (utc_sync_tbl[i].tick==0) {
            i=i+1;
            break;
        }
        if (tick >= utc_sync_tbl[i].tick) {
            if (i==UTCSYNC_SIZE-1) {
                if (tick - utc_sync_tbl[i].tick > LIMIT_TICK_TH) {
                    utc = 0;
                    return WRONG_ESTIMATE;
                }
                utc = (UTC_TYPE) (tick - utc_sync_tbl[i].tick) * UTC_UNIT / tick_ps;
                ret = FUTURE_ESTIMATE;
            } else {
                utc = (UTC_TYPE) (tick - utc_sync_tbl[i].tick) * (utc_sync_tbl[i+1].utc - utc_sync_tbl[i].utc) / (utc_sync_tbl[i+1].tick - utc_sync_tbl[i].tick);
                ret = PERFECT_ESTIMATE;
            }
            utc += utc_sync_tbl[i].utc;
            return ret;
        }
    }
    if (i<0)
        i=0;
    if (i == UTCSYNC_SIZE || utc_sync_tbl[i].tick == 0 || utc_sync_tbl[i].tick - tick > LIMIT_TICK_TH) {
        utc = 0;
        return WRONG_ESTIMATE;
    }

    utc = (UTC_TYPE) (utc_sync_tbl[i].tick - tick) * UTC_UNIT / tick_ps;
    Assert(utc < utc_sync_tbl[i].utc);
    utc = utc_sync_tbl[i].utc - utc;
    return PAST_ESTIMATE;
}

/*
 * In utc
 * Out tick
 * Return 0, perfect estimate
 *        1, future estimate
 *        2, past estimate
 *        3, wrong estimate
 */
template <class UTC_TYPE> int ADCSync1PPS::utc2tick(UTC_TYPE utc, uint64_t & tick)
{
    int ret, i;
    for (i=UTCSYNC_SIZE-1; i>=0; i--) {
        if (utc_sync_tbl[i].tick==0) {
            i=i+1;
            break;
        }
        if (utc >= utc_sync_tbl[i].utc) {
            if (i==UTCSYNC_SIZE-1) {
                if (utc - utc_sync_tbl[i].utc > LIMIT_UTC_TH) {
                    tick = 0;
                    return WRONG_ESTIMATE;
                }
                tick = (utc - utc_sync_tbl[i].utc) * tick_ps / UTC_UNIT;
                ret = FUTURE_ESTIMATE;
            } else {
                tick = (utc - utc_sync_tbl[i].utc) * (utc_sync_tbl[i+1].tick - utc_sync_tbl[i].tick) / (utc_sync_tbl[i+1].utc - utc_sync_tbl[i].utc);
                ret = PERFECT_ESTIMATE;
            }
            tick += utc_sync_tbl[i].tick;
            return ret;
        }
    }
    if (i<0)
        i=0;
    if (i == UTCSYNC_SIZE ||utc_sync_tbl[i].tick == 0 || utc_sync_tbl[i].utc - utc > LIMIT_UTC_TH) {
        tick = 0;
        return WRONG_ESTIMATE;
    }
    tick = (utc_sync_tbl[i].utc - utc) * tick_ps / UTC_UNIT;
    Assert(tick < utc_sync_tbl[i].tick);
    tick = utc_sync_tbl[i].tick - tick;
    return PAST_ESTIMATE;
}

/*
 * In sample
 * Out utc
 * Return 0, perfect estimate
 *        1, future estimate
 *        2, past estimate
 *        3, wrong estimate
 */
template <class UTC_TYPE> int ADCSync1PPS::sample2utc(uint64_t sample, UTC_TYPE & utc)
{
    uint64_t tick = sample2tick(sample);
    if (tick==0)
        return WRONG_ESTIMATE;
    return tick2utc(tick, utc);
}

/*
 * In utc
 * Out sample
 * Return 0, perfect estimate
 *        1, future estimate
 *        2, past estimate
 *        3, wrong estimate
 */
template <class UTC_TYPE> int ADCSync1PPS::utc2sample(UTC_TYPE utc, uint64_t & sample)
{
    uint64_t tick;
    int ret = utc2tick(utc, tick);
    if (ret != WRONG_ESTIMATE) {
        sample = tick2sample(tick);
        if (sample == 0)
            ret = WRONG_ESTIMATE;
    }
    return ret;
}

//Input cap_state
void ADCSync1PPS::set_cap_state(uint16_t cap_state)
{
    EALLOW;
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
    EDIS;
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
    case SYNC_INIT: //receive EPWM interrupt
        sync_state = SYNC_INIT2;
        ecap0->ECEINT.all = 0x20; //only enable overflow disable CAP1
        //set ecap0 to continuous mode, still monitor epwm signal until next DMA interrupt happen
        ecap0->ECCTL2.bit.CONT_ONESHT = 0;
        break;
    case SYNC_UTC0:
    case SYNC_UTC1:
    case SYNC_UTC2:
        if (capture_state == CAPTURE_1PPS) {
            receive_1pps = true;
            event_pending |= ADCSYNC_EVENT_MASK;
        }
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
    case SYNC_INIT2:
        sync_state = SYNC_DMA;
        compute_vi.new_cluster_ready();
        return;
    case SYNC_DMA:
    {
        cap1 = ecap0->CAP1; //assume tick_h32 is 0
        uint32_t diff = cap1 - dma_sync_tbl[DMASYNC_SIZE - 1].tick;
        Assert(diff % (ADC_DMA_TRANSFER_SIZE * DEFAULT_SAMPLE_PERIOD) == 0);
        compute_vi.new_cluster_ready();
        if (compute_vi.get_latest_sample() >= ADC_BUF_SIZE / CHANNEL)
            sync_state = SYNC_UTC0;
        return;
    }
    case SYNC_UTC0:
    case SYNC_UTC1:
    case SYNC_UTC2:
        compute_vi.new_cluster_ready();
        event_pending |= ADCSYNC_EVENT_MASK;
        return;
    case SYNC_INIT:
    default:
            Assert(0);
    }

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
    keep_display = false;
    capture_state = CAPTURE_EPWM;
    sync_state = SYNC_ENTER;
    clear_utc_sync_tbl();
    clear_dma_sync_tbl();
    record_utc = 0;
    tick_ps = CPU_CLOCK;
    receive_1pps = false;
    adc_init();
    compute_vi.init();
    init_xbar_gpio();
    ecap_init();
    epwm_init();
    event_pending |= ADCSYNC_EVENT_MASK;
}

void ADCSync1PPS::process_event()
{
    int16_t * p_evt = (int16_t*)&event_pending;
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
        push_dma_sync_tbl(DEFAULT_SAMPLE_PERIOD, 0, cap1);

        IER = 0x40;
        //now sync== SYNC_DMA, and ecap cap1 interrupt is disabled
        while (sync_state == SYNC_DMA); // hold CPU until dma interrupt finish

        Assert(sync_state == SYNC_UTC0); //now sync==SYNC_UTC0
        set_cap_state(CAPTURE_1PPS);

        IER = old_ier;

        printf("1st dma sample time=%lu:%lu\n\r", (uint32_t) (dma_sync_tbl[DMASYNC_SIZE -1].tick >> 32),
               (uint32_t) (dma_sync_tbl[DMASYNC_SIZE - 1].tick & 0xffffffff));
    }
    bool finish=false;

    while (!finish) {
        finish = true;
        if (receive_1pps) {
            receive_1pps = false;
            uint64_t utc = utc_sync.get_1pps_utctime(cap1, tick_ps); //get utc time for new arriving 1pps
            /*
             * SYNC_UTC0 means utc_sync_tbl has no valid item
             * SYNC_UTC1 means utc_sync_tbl has 1 valid item
             * SYNC_UTC2 means utc_sync_tbl has >= 2 valid item
             */
            switch (sync_state) {
            case SYNC_UTC0:
                if (utc!=0 && utc!=0xffffffffffffffff) { //got valid utc, change to SYNC_UTC1
                    push_utc_sync_tbl(cap1, utc);
                    sync_state = SYNC_UTC1;
                }
                break;

            case SYNC_UTC1:
            case SYNC_UTC2:
                if (utc==0) { //utc=0 means lost utc time
                    clear_utc_sync_tbl();
                    sync_state = SYNC_UTC0;
                }
                else
                if (utc!=0xffffffffffffffff) { //utc=0xffffffffffffffff means no utc available
                    if (!compatible(utc_sync_tbl[UTCSYNC_SIZE - 1], TickTime(cap1, utc), tick_ps, MAX_CLOCK_DIFF)) {
                        printf("tick diff=%lld, utc diff=%lld, tick_ps=%ld\n\r", cap1 - utc_sync_tbl[UTCSYNC_SIZE - 1].tick,
                               utc - utc_sync_tbl[UTCSYNC_SIZE - 1].utc, tick_ps); //not compatible with previous one, maybe 1pps signal or crystal error
                        clear_utc_sync_tbl();
                        sync_state = SYNC_UTC1;
                        push_utc_sync_tbl(cap1, utc);
                    }
                    else { //has at least 2 valid item, update tick_ps
                        sync_state = SYNC_UTC2;
                        push_utc_sync_tbl(cap1, utc);
                        update_tick_ps();
                    }
                }
                break;
            }
        }

        if (sync_state == SYNC_UTC1 || sync_state == SYNC_UTC2) {
            uint64_t sample;
            int ret = utc2sample(record_utc, sample);
            if (ret != WRONG_ESTIMATE) {
                VIBuf vibuf;
                vibuf.t.sample = sample;
                ret = compute_vi(vibuf);
                if (ret == COMPUTEVI_LATE) {
                    update_record_utc();
                    finish = false;
                }
                if (ret == COMPUTEVI_EARLY) {
                    if (sample > compute_vi.get_latest_sample()) {
                        update_record_utc();
                        finish = false;
                    }
                }
                if (ret == COMPUTEVI_INTIME) {
                    long double actual_utc;
                    sample2utc(vibuf.t.sample, actual_utc);
                    actual_utc -= utc_sync_tbl[UTCSYNC_SIZE-1].utc;
                    actual_utc = actual_utc / UTC_UNIT * POWER_CYCLE;
                    actual_utc -= (int64_t) actual_utc;
                    int32_t phase_adjust = actual_utc * 36000;
                    for (int j=0; j<CHANNEL; j++) {
                        int32_t phase = (int) vibuf.vi[j][1] - phase_adjust;
                        while (phase < -18000)
                            phase += 36000;
                        while (phase >= 18000)
                            phase -= 36000;
                        vibuf.vi[j][1] = phase;
                    }
                    if (record_utc % 1000 == 0 || record_utc % 1000 == 10) {
                        for (int j=1; j<3; j++)
                            printf("(%d,%d.%2d),", vibuf.vi[j][0], vibuf.vi[j][1] / 100, abs(vibuf.vi[j][1] % 100));
                        printf("pa=%d, tick_ps=%ld\n\r", phase_adjust, tick_ps);
                    }
                    record_utc = record_utc + COMPUTE_PERIOD;
                    finish = false;
                }
            }
            else { //WRONG estimate
                update_record_utc();
                finish = false;

            }
        }
    }
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

/*
 * Sim50HzSin.cpp
 *
 *  Created on: Jun 3, 2018
 *      Author: yuchen
 */

#define _Sim50HzSin_
#include "task.h"

#ifdef _SIM50HZ_SIN_WITH_PWM_
#include "math.h"

#undef SIN_TABLE_SIZE
#define SIN_TABLE_SIZE 200

#define TIMER_50HZ (CPU_CLOCK / 50 / SIN_TABLE_SIZE)

#define DMA_CHANNEL  3
#if DMA_CHANNEL == 1
static volatile CH_REGS * dma_ch = &DmaRegs.CH1;
#elif DMA_CHANNEL == 2
static volatile CH_REGS * dma_ch = &DmaRegs.CH2;
#elif DMA_CHANNEL == 3
static volatile CH_REGS * dma_ch = &DmaRegs.CH3;
#endif

#define USE_DAC  1
#if USE_DAC == 1
static volatile DAC_REGS * dac = &DacaRegs;
#elif USE_DAC == 2
static volatile DAC_REGS * dac = &DacbRegs;
#endif

#define USE_EPWM 3
#if USE_EPWM == 1
static volatile EPWM_REGS * epwm = &EPwm1Regs;
#elif USE_EPWM == 2
static volatile EPWM_REGS * epwm = &EPwm2Regs;
#elif USE_EPWM == 3
static volatile EPWM_REGS * epwm = &EPwm3Regs;
#endif
Sim50HzSin sim50hz_sin;
static uint16_t sine_tbl[SIN_TABLE_SIZE];

#define SIM50HZ_FREE_SOFT 0
#if SIM50HZ_FREE_SOFT > 1
#error SIM50HZ_FREE_SOFT > 1
#endif

static void dac_init()
{
    EALLOW;

    dac->DACCTL.bit.DACREFSEL = 1;      //use AD reference
    dac->DACOUTEN.bit.DACOUTEN = 1;
    dac->DACVALS.all = 0;

    F28x_usDelay(200); // Delay for buffered DAC to power up, pin 30

    EDIS;
}

static void dma_init()
{
    EALLOW;

    // Set up SOURCE address:
    //
    dma_ch->SRC_BEG_ADDR_SHADOW = (Uint32)sine_tbl;   // Point to beginning of source buffer
    dma_ch->SRC_ADDR_SHADOW =     (Uint32)sine_tbl;

    // Set up DESTINATION address:
    //
    dma_ch->DST_BEG_ADDR_SHADOW = (Uint32) &(dac->DACVALS);  // Point to beginning of destination buffer
    dma_ch->DST_ADDR_SHADOW =     (Uint32) &(dac->DACVALS);

    // Set up BURST registers:
    //
    dma_ch->BURST_SIZE.all = 0;      // Number of words(X-1) x-ferred in a burst.
    dma_ch->SRC_BURST_STEP = 0;   // Increment source addr between each word x-ferred.
    dma_ch->DST_BURST_STEP = 0;   // Increment dest addr between each word x-ferred.

    // Set up TRANSFER registers:
    //
    dma_ch->TRANSFER_SIZE = SIN_TABLE_SIZE - 1;        // Number of bursts per transfer, DMA interrupt will occur after completed transfer.
    dma_ch->SRC_TRANSFER_STEP = 1; // TRANSFER_STEP is ignored when WRAP occurs.
    dma_ch->DST_TRANSFER_STEP = 0; // TRANSFER_STEP is ignored when WRAP occurs.

    // Set up WRAP registers:
    //
    dma_ch->SRC_WRAP_SIZE = SIN_TABLE_SIZE - 1; // Wrap source address after N bursts, equal to TRANSFER_SIZE disable WARP
    dma_ch->SRC_WRAP_STEP = 0; // Step for source wrap
    dma_ch->DST_WRAP_SIZE = 0; // Wrap destination address after  N bursts.
    dma_ch->DST_WRAP_STEP = 0; // Step for destination wrap

#if USE_EPWM == 1
    int trig_source = DMA_EPWM1A;    // persel - Source select
#elif USE_EPWM == 2
    int trig_source = DMA_EPWM2A;    // persel - Source select
#elif USE_EPWM == 3
    int trig_source = DMA_EPWM3A;    // persel - Source select
#endif

#if DMA_CHANNEL == 1
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH1 = trig_source; // persel - Source select
#elif DMA_CHANNEL == 2
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH2 = trig_source; // persel - Source select
#elif DMA_CHANNEL == 3
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH3 = trig_source; // persel - Source select
#endif
    // Set up MODE Register:
    //
    dma_ch->MODE.all = 0xb00;
    dma_ch->MODE.bit.PERINTSEL = DMA_CHANNEL;
  /*PERINTSEL:5 =DMA     4:0  Peripheral Interrupt and Sync Select
    rsvd1:2              6:5  Reserved
    OVRINTE:1   =0       7    Overflow Interrupt Enable
    PERINTE:1   =1       8    Peripheral Interrupt Enable
    CHINTMODE:1 =1       9    Channel Interrupt Mode
    ONESHOT:1   =0       10   One Shot Mode Bit
    CONTINUOUS:1=1       11   Continuous Mode Bit
    rsvd2:2     =0       13:12    Reserved
    DATASIZE:1  =0       14   Data Size Mode Bit
    CHINTE:1    =0       15   Channel Interrupt Enable Bit
    */


    // Clear any spurious flags: interrupt and sync error flags
    //
    dma_ch->CONTROL.bit.PERINTCLR = 1;
    dma_ch->CONTROL.bit.ERRCLR = 1;

    // Start DMA channel
    //
    dma_ch->CONTROL.bit.RUN = 1;
    EDIS;
}

void timer_init()
{
    // Counter decrements PRD+1 times each period
    //
    CpuTimer0Regs.PRD.all = TIMER_50HZ - 1;

    // Set pre-scale counter to divide by 1 (SYSCLKOUT):
    //
    CpuTimer0Regs.TPR.all  = 0;
    CpuTimer0Regs.TPRH.all  = 0;

    //
    // Initialize timer control register:
    //
    CpuTimer0Regs.TCR.bit.TRB = 1;
    CpuTimer0Regs.TCR.all = 0x4000;
    // bits description
    // 3:0 Reserved
    // 4 CPU-Timer stop status bit.
    // 5 Timer reload
    // 9:6 Reserved
    // 10 Emulation modes
    // 11 Emulation modes
    // 13:12 Reserved
    // 14 CPU-Timer Interrupt Enable.
    // 15 CPU-Timer Interrupt Flag.

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
#endif
    EDIS;

    epwm->TBCTL.all = 0x0033;           // Configure timer control register
    epwm->TBCTL.bit.FREE_SOFT = SIM50HZ_FREE_SOFT * 3;
    // bit 15-14     11:     FREE/SOFT, 11 = ignore emulation suspend
    // bit 13        0:      PHSDIR, 0 = count down after sync event
    // bit 12-10     000:    CLKDIV, 000 => TBCLK = HSPCLK/1, 111 => 8
    // bit 9-7       000:    HSPCLKDIV, 000 => HSPCLK = EPWMCLK/1, 111 => 8
    // bit 6         0:      SWFSYNC, 0 = no software sync produced
    // bit 5-4       11:     SYNCOSEL, 11 = sync-out disabled
    // bit 3         0:      PRDLD, 0 = reload PRD on counter=0
    // bit 2         0:      PHSEN, 0 = phase control disabled
    // bit 1-0       11:     CTRMODE, 11 = timer stopped (disabled)

    epwm->TBCTR = 0x0000;               // Clear timer counter
    epwm->TBPRD = TIMER_50HZ - 1;      // Set timer period
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

    epwm->TBCTL.bit.CTRMODE = 0x0;      // Enable the timer in count up mode
}

static void set_waveform(uint16_t amp, uint16_t offset)
{
    for (uint16_t i=0; i < SIN_TABLE_SIZE; i++) {
        sine_tbl[i] = (sin(i *2*PI/SIN_TABLE_SIZE + offset * PI / 180)+1.02) * amp;
    }
}

void Sim50HzSin::init()
{
    dac_init();
    // Make sure timer is stopped:
    //
    amp = 1000;
    offset = 0;
    //stop timer
    CpuTimer0Regs.TCR.bit.TSS = 1;
    set_waveform(amp, offset);
    dma_init();
    epwm_init();
}

void Sim50HzSin::print_helper()
{
    printf("Sim50HzSin %dsin(t+%d);\n\r", amp, offset);
    printf("h) Print this info\n\r");
    printf("a ddd) Set amp to ddd\n\r");
    printf("o ddd) Set offset to ddd\n\r");
    printf("e) Exit\n\r");
}


int Sim50HzSin::process_cmd(char * user_cmd)
{
    int16_t a = 0;
    switch (toupper(user_cmd[0])) {
    case 'H':
        print_helper();
        break;
    case 'A':
        if (sscanf(&user_cmd[1], " %d", &a) == 1) {
            amp = a;
            set_waveform(amp, offset);
        }
        break;
    case 'O':
        if (sscanf(&user_cmd[1], " %d", &a) == 1) {
            offset = a % 360;
            set_waveform(amp, offset);
        }
        break;
    case 'E':
        return 1;
    default:
        break;
    }
    printf("amp=%d, offset=%d\n\r", amp, offset);
    return 0;
}
#endif

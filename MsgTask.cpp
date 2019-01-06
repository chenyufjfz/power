/*
 * MsgTask.cpp
 *
 *  Created on: May 28, 2018
 *      Author: yuchen
 */

#include <file.h>
#define _MSGTASK_
#include "task.h"
MsgTask msg_task;

#define SCI_TX_BUF_SHIFT 5
#define SCI_TX_BUF_SIZE (1 << SCI_TX_BUF_SHIFT)
#define SCI_TX_BUF_MASK (SCI_TX_BUF_SIZE - 1)

#define SCI_RX_BUF_SHIFT 5
#define SCI_RX_BUF_SIZE (1 << SCI_RX_BUF_SHIFT)
#define SCI_RX_BUF_MASK (SCI_RX_BUF_SIZE - 1)

enum {
    ROUTE_TO_MST_TASK = 0,
    ROUTE_TO_ADCSYNC,
    ROUTE_TO_SIM1PPS,
    ROUTE_TO_SIM50HZSIN
};

static struct SciBuffer {
    uint16_t device_open;
    uint16_t tx_buf[SCI_TX_BUF_SIZE];
    uint16_t rx_buf[SCI_RX_BUF_SIZE];
    volatile uint16_t tx_head, tx_tail, rx_read, rx_len;
} sci_buf;



// **************************************************************************
// the SCI functions

int sci_open(const char * path, unsigned flags, int llv_fd)
{
    if(sci_buf.device_open){
        return (-1);
    }else{
        sci_buf.tx_head = 0;
        sci_buf.tx_tail = 0;
        sci_buf.rx_read =0;
        sci_buf.rx_len =0;
        sci_buf.device_open = 1;
        return (1);
    }

}

int sci_close(int dev_fd)
{
    if((dev_fd != 1) || (!sci_buf.device_open)){
        return (-1);
    }else{
        sci_buf.device_open = 0;
        return (0);
    }

}

int scia_write_block(int dev_fd, const char * buf, unsigned count)
{
    uint16_t writeCount = 0;
    uint16_t * bufPtr = (uint16_t *) buf;

    while(writeCount < count) {
        while(!SciaRegs.SCICTL2.bit.TXRDY);
        SciaRegs.SCITXBUF.all = *bufPtr;
        writeCount++;
        bufPtr++;
    }

    return (writeCount);

}

void scia_buf2txfifo()
{
    unsigned int old_st = __disable_interrupts();
    //copy buf from memory to fifo
    while (sci_buf.tx_head != sci_buf.tx_tail && SciaRegs.SCIFFTX.bit.TXFFST < 16) {
        SciaRegs.SCITXBUF.all = sci_buf.tx_buf[sci_buf.tx_head & SCI_TX_BUF_MASK];
        sci_buf.tx_head++;
    }
    if (sci_buf.tx_head != sci_buf.tx_tail)
        SciaRegs.SCIFFTX.bit.TXFFIENA = 1; //still left buf in memory, enable interrupt
    else
        SciaRegs.SCIFFTX.bit.TXFFIENA = 0; //all copy to fifo, disable interrupt
    SciaRegs.SCIFFTX.bit.TXFFINTCLR=1; //clear tx_finish interrupt
    __restore_interrupts(old_st);
}

static interrupt void sciatxfifo_isr(void)
{
    scia_buf2txfifo();
    PieCtrlRegs.PIEACK.all |= 0x100;       // Issue PIE ACK
}

int scia_write_noblock(int dev_fd, const char * buf, unsigned count)
{
    if (count==0)
        return count;
    if (sci_buf.tx_head != sci_buf.tx_tail)
        scia_buf2txfifo();

    uint16_t * buf_ptr = (uint16_t *) buf;

    for (uint16_t wc = 0; wc < count; wc++) {
        while(sci_buf.tx_tail - sci_buf.tx_head >= SCI_TX_BUF_SIZE); //block to wait sci ready
        sci_buf.tx_buf[sci_buf.tx_tail & SCI_TX_BUF_MASK] = *buf_ptr++;
        sci_buf.tx_tail++;
    }
    scia_buf2txfifo();
    return count;
}

int scia_read(int dev_fd, char * buf, unsigned count)
{
    uint16_t readCount = 0;
    uint16_t * bufPtr = (uint16_t *) buf;

    unsigned int old_st = __disable_interrupts();

    while((readCount < count) && SciaRegs.SCIFFRX.bit.RXFFST != 0) {
        *bufPtr = SciaRegs.SCIRXBUF.all;
        readCount++;
        bufPtr++;
    }
    __restore_interrupts(old_st);
    return (readCount);

}


static interrupt void sciarxfifo_isr(void)
{
    while (SciaRegs.SCIFFRX.bit.RXFFST != 0) { //loop until Rx fifo empty or get one complete message
        uint16_t a = SciaRegs.SCIRXBUF.all;
        if (a!=8 && a!=127) //8 is backspace, 127 is del
            sci_buf.rx_buf[sci_buf.rx_len++] = a; //copy from fifo to buf
        else
            if (sci_buf.rx_len>0)   //backspace, delete
                sci_buf.rx_len--;
        if (sci_buf.tx_tail - sci_buf.tx_head < SCI_TX_BUF_SIZE) {
            scia_write_noblock(0, (char *) &a, 1);
            if (a==13) {    //13 is \n, append \r(10)
                uint16_t b = 10;
                if (sci_buf.tx_tail - sci_buf.tx_head < SCI_TX_BUF_SIZE)
                scia_write_noblock(0, (char *) &b, 1);
            }
        }
        if (sci_buf.rx_len == SCI_RX_BUF_SIZE || a==13) { //get one complete message
            event_pending |= MSG_EVENT_MASK;
            SciaRegs.SCIFFRX.bit.RXFFIENA = 0;     //disable interrupt
        }
    }

    SciaRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
    SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ack
}

int scia_read_buf(int dev_fd, const char * buf, unsigned count)
{
   uint16_t * bufptr = (uint16_t *) buf;
   uint16_t read_cnt = 0;

   while(sci_buf.rx_read < sci_buf.rx_len && read_cnt < count) {
       *bufptr = sci_buf.rx_buf[sci_buf.rx_read]; //copy from rx_buf to caller
       sci_buf.rx_read++;
       read_cnt++;
       bufptr++;
   }
   if (sci_buf.rx_read == sci_buf.rx_len) { //read complete mssage
       sci_buf.rx_len = 0;
       sci_buf.rx_read = 0;
       SciaRegs.SCIFFRX.bit.RXFFIENA = 1; //enable interrupt
   }
   return read_cnt;
}

off_t sci_lseek(int dev_fd, off_t offset, int origin)
{
    return (0);
}

int sci_unlink(const char * path)
{
    return (0);
}

int sci_rename(const char * old_name, const char * new_name)
{
    return (0);
}

// SCIA  8-bit word, baud rate 0x000F, default, 1 STOP bit, no parity
void scia_init()
{

    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function

    SciaRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
    SciaRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE

    SciaRegs.SCICTL2.all = 0x0003; // Tx interrupt and Rx interrupt

    //
    // SCIA at 115200 baud
    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x00 and LBAUD = 0x36.
    //
    SciaRegs.SCIHBAUD.all = 0x0000;
    SciaRegs.SCILBAUD.all = 0x0036;

    SciaRegs.SCIFFTX.all = 0xC000; //disable Tx interrupt, set fifo interrupt level to 0
    SciaRegs.SCIFFRX.all = 0x0021; //enable Rx interrupt, set fifo interrupt level to 1
    SciaRegs.SCIFFCT.all = 0x00;   //set tx delay to 0

    SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
    SciaRegs.SCIFFTX.bit.TXFIFORESET = 1;
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;

    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.SCIA_RX_INT = &sciarxfifo_isr;
    PieVectTable.SCIA_TX_INT = &sciatxfifo_isr;

    GPIO_SetupPinMux(43, GPIO_MUX_CPU1, 15);  //route SCIA Rx to GPIO43
    GPIO_SetupPinOptions(43, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(42, GPIO_MUX_CPU1, 15);
    GPIO_SetupPinOptions(42, GPIO_OUTPUT, GPIO_ASYNC);
    EDIS; // This is needed to disable write to EALLOW protected registers

    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;   // PIE Group 9, INT1
    PieCtrlRegs.PIEIER9.bit.INTx2 = 1;   // PIE Group 9, INT2
    IER |= 0x100;                         // Enable CPU INT
    return;
}

void get_sci_input(char * msg, int msg_len)
{
    int len = scia_read_buf(1, msg, msg_len - 1);
    if (msg[len-1] == 13) {
        msg[len-1] = 0;
    } else
        msg[len] = 0;
}

void MsgTask::init()
{
    sci_buf.device_open = 0;
    msg_route = ROUTE_TO_MST_TASK;
    scia_init();

    //Redirect STDOUT to SCI
    add_device("scia", _SSA, sci_open, sci_close, scia_read, scia_write_noblock, sci_lseek, sci_unlink, sci_rename);
    fopen("scia","rw");
    freopen("scia:stdout", "w", stdout);
    setvbuf(stdout, NULL, _IONBF, 0);
}

void MsgTask::print_helper()
{
    printf("MsgTask wait command\n\r");
    printf("h) print this helper info\n\r");
    printf("s) simulate 1pps\n\r");
    printf("g) generate 50Hz wav\n\r");
    printf("a) adc sync 1pps\n\r");
}

void MsgTask::process_event()
{
    int * p_evt = (int*)&event_pending;
    __and(p_evt, ~MSG_EVENT_MASK);

    char user_cmd[SCI_RX_BUF_SIZE + 1];
    get_sci_input(user_cmd, sizeof(user_cmd));

    if (msg_route == ROUTE_TO_MST_TASK)
    switch (toupper(user_cmd[0])) {
    case 'H':
        print_helper();
        return;
    case 'S':
#ifdef _SIM1PPS_WITH_PWM_
        sim_1pps.process_cmd("h");
        msg_route = ROUTE_TO_SIM1PPS;
#else
        printf("sim1pps not support in this version");
#endif
        return;
    case 'G':
#ifdef _SIM50HZ_SIN_WITH_PWM_
        sim50hz_sin.process_cmd("h");
        msg_route = ROUTE_TO_SIM50HZSIN;
#else
        printf("sim50Hzsin not support in this version");
#endif
        return;
    case 'A':
        adc_sync_1pps.process_cmd("h");
        msg_route = ROUTE_TO_ADCSYNC;
    default:
        return;
    }

    int ret=0;
    switch (msg_route) {
    case ROUTE_TO_SIM1PPS:
        ret = sim_1pps.process_cmd(user_cmd);
        break;
    case ROUTE_TO_SIM50HZSIN:
        ret = sim50hz_sin.process_cmd(user_cmd);
        break;
    case ROUTE_TO_ADCSYNC:
        ret = adc_sync_1pps.process_cmd(user_cmd);
        break;
    }
    if (ret!=0) {
        print_helper();
        msg_route = ROUTE_TO_MST_TASK;
    }
}

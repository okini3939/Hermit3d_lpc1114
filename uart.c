#include "LPC11xx.h"
#include "uart.h"
#include <string.h>
#include <math.h>

static char ser_buf[SER_BUF_SIZE];
static int ser_count = 0;


void output_func (unsigned char c) {
	while (! (LPC_UART->LSR & (1<<5)));
	LPC_UART->THR = c;
};

unsigned char input_func (void) {
	return 0;
}


void send_uart (const char *buf, int len) {
	int i;

	for (i = 0; i < len; i ++) {
		while (! (LPC_UART->LSR & (1<<5)));
		LPC_UART->THR = buf[i];
	}
}

int recv_uart (char *buf, int len) {
	int i;

	for (i = 0; i < len; i ++) {
		if (i >= ser_count) {
			ser_count = 0;
			break;
		}
		buf[i] = ser_buf[i];
	}
	return i;
}

void UART_IRQHandler () {
	uint8_t iir, lsr;
	char d;

	iir = (LPC_UART->IIR >> 1) & 0x07;
	switch (iir) {
	case 3: // rls
		lsr = LPC_UART->LSR;
		if (lsr & ((1<<7)|(1<<4)|(1<<3)|(1<<2)|(1<<1))) {
			// error
			d = LPC_UART->RBR; // dummy
		} else
		if (lsr & (1<<0)) {
			// recv
			d = LPC_UART->RBR;
//	    LPC_UART->THR = d;
			if (ser_count < SER_BUF_SIZE) {
				ser_buf[ser_count] = d;
				ser_count ++;
			}
		}
		break;
	case 2: // rda
		// recv
		d = LPC_UART->RBR;
//	  LPC_UART->THR = d;
		if (ser_count < SER_BUF_SIZE) {
			ser_buf[ser_count] = d;
			ser_count ++;
		}
		break;
	case 6: // cti
		// cti error
		break;
	case 1: // thre
		lsr = LPC_UART->LSR;
		if (lsr & (1<<5)) {
			// tx empty
		}
		break;
	}
}

void init_uart (int baud) {
	uint32_t fdiv, reg;

	LPC_IOCON->PIO1_6 &= ~0x07;
	LPC_IOCON->PIO1_6 |= 0x01; // UART RXD
	LPC_IOCON->PIO1_7 &= ~0x07;
	LPC_IOCON->PIO1_7 |= 0x01; // UART TXD

	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<12); // Enable AHB clock to the UART
	LPC_SYSCON->UARTCLKDIV = 1; // PCLK=12MHz
/*
	LPC_UART->LCR = (1<<7); // Enable access to Divisor Latches (DLAB=1)
	reg = LPC_SYSCON->UARTCLKDIV;
	fdiv = (((SystemCoreClock * LPC_SYSCON->SYSAHBCLKDIV) / reg) / 16) / baud;
	LPC_UART->DLM = fdiv / 256; // Load divisor MSB
	LPC_UART->DLL = fdiv % 256; // Load divisor LSB
	LPC_UART->LCR = (3<<0); // 8 bits, no Parity, 1 Stop bit
//	LPC_UART->FDR = (8<<4)|(5<<0); // MULVAL, DIVADDVAL (FR=1.625)
//	LPC_UART->FDR = (15<<4)|(8<<0); // MULVAL, DIVADDVAL (FR=1.533)
*/
    uint32_t PCLK = SystemCoreClock;
    uint16_t DL = PCLK / (16 * baud);
    uint8_t DivAddVal = 0;
    uint8_t MulVal = 1;
    int hit = 0;
    uint16_t dlv;
    uint8_t mv, dav;
    if ((PCLK % (16 * baud)) != 0) {     // Checking for zero remainder
        float err_best = (float) baud;
        uint16_t dlmax = DL;
        for ( dlv = (dlmax/2); (dlv <= dlmax) && !hit; dlv++) {
            for ( mv = 1; mv <= 15; mv++) {
                for ( dav = 1; dav < mv; dav++) {
                    float ratio = 1.0f + ((float) dav / (float) mv);
                    float calcbaud = (float)PCLK / (16.0f * (float) dlv * ratio);
                    float err = fabs(((float) baud - calcbaud) / (float) baud);
                    if (err < err_best) {
                        DL = dlv;
                        DivAddVal = dav;
                        MulVal = mv;
                        err_best = err;
                        if (err < 0.001f) {
                            hit = 1;
                        }
                    }
                }
            }
        }
    }
    LPC_UART->LCR |= (1<<7); // enable writing to divider registers
    LPC_UART->DLM = (DL >> 8) & 0xFF;
    LPC_UART->DLL = (DL >> 0) & 0xFF;
    LPC_UART->FDR = ((uint32_t)DivAddVal << 0) | ((uint32_t)MulVal << 4);
    LPC_UART->LCR &= ~(1 << 7);

	LPC_UART->LCR = (3<<0); // 8 bits, no Parity, 1 Stop bit
	LPC_UART->FCR = (1<<2)|(1<<1)|(1<<0); // Enable and reset TX and RX FIFO, trigger level 0
	LPC_UART->MCR = 0;
	LPC_UART->IER = (1<<2)|(1<<0); // interrupt RX

	xfunc_out = &output_func;
	xfunc_in = &input_func;

	NVIC_SetPriority(UART_IRQn, 6);
	NVIC_EnableIRQ(UART_IRQn);
}

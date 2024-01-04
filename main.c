#include <LPC11xx.h>
#include <string.h>
#include <stdlib.h>
#include "l6470.h"
#include "hermit.h"
#include "adc.h"

int my_dmx_address;
volatile unsigned int systime = 0, w = 0;

extern void disk_timerproc (void);
extern StepperMotor sm;
extern XYZ target, sm_target;

//void _ttywrch(int ch) {};
//void _sys_exit(int return_code) {};

void SysTick_Handler (void) {
	systime ++;
	if (w) w --;
//	disk_timerproc();
	if (!sm.work || systime & 16) disk_timerproc();
}

void wait_ms (int msec) {
	w = msec;
	while (w);
}

extern XYZ target;
extern StepperMotor sm;

int command (char *buf) {
    int i, r = -1;
    char *tmp;

    switch (buf[0]) {
    case 'L':
        listfile();
        r = 0;
        break;
    case 'P':
        tmp = strstr(&buf[1], ",");
        if (tmp) {
            *tmp = 0;
            tmp ++;
            i = atoi(tmp);
        } else {
            i = 0;
        }
        r = gcode(&buf[1]);
        break;
/*
    case 'W':
        tmp = strstr(&buf[1], ",");
        if (tmp) {
            *tmp = 0;
            tmp ++;
            i = atoi(tmp);
        } else {
            i = 0;
        }
        r = recvfile(&buf[1], i);
        break;

    case 'S':
        stop_flg = 1;
        r = 0;
        break;
    case 'H':
        r = findhome3d();
        break;
*/
    case 'V':
        xprintf("@%d,%d\r\n", sm.work, sm.gcode_line);
        r = 0;
        break;
    case 'G':
    case 'M':
        parseGcode(buf, &target);
        calc_end();
        r = 0;
        break;
    case 'h':
        parseGcode("G00X0Y0F1000", &target);
        parseGcode("G00X0Y0Z0F1000", &target);
        calc_end();
        r = 0;
        break;
		case 'T':
				xprintf("ext %d / bed %d\r\n", sm.temp_ext, sm.temp_bed);
				break;
    }
    return r;
}


int poll (void) {
	if (BUTTON_INPUT) {
		return -1;
	}

	sm.temp_ext = get_temp(ADC_TEMP_EXT);
	if (sm_target.temp_ext && sm.temp_ext > 0) {
		if (sm.temp_ext < sm_target.temp_ext) {
			HEATER1(1);
			if (sm.temp_ext_flg > sm_target.temp_ext) sm.temp_ext_flg = 0;
		} else
		if (sm.temp_ext > sm_target.temp_ext) {
			HEATER1(0);
			if (sm.temp_ext_flg < sm_target.temp_ext) sm.temp_ext_flg = 0;
		}
	} else {
		HEATER1(0);
		sm.temp_ext_flg = 0;
	}

	sm.temp_bed = get_temp(ADC_TEMP_BEAD1);
	if (sm_target.temp_bed && sm.temp_bed > 0) {
		if (sm.temp_bed < sm_target.temp_bed) {
			HEATER2(1);
			if (sm.temp_bed_flg > sm_target.temp_bed) sm.temp_bed_flg = 0;
		} else
		if (sm.temp_bed > sm_target.temp_bed) {
			HEATER2(0);
			if (sm.temp_bed_flg < sm_target.temp_bed) sm.temp_bed_flg = 0;
		}
	} else {
		HEATER2(0);
		sm.temp_bed_flg = 0;
	}
/*
								if (pc.readable()) {
                    pc.getc();
                    DBG("*** break\r\n");
                    goto exit;
                }
                if (uart.readable()) {
                    char c;
                    c = uart.getc();
                    if (c == '!') {
                        DBG("*** break\r\n");
                        goto exit;
                    }
                    if (c == 'V') {
                        uart.printf("@%d,%d\r\n", sm.work, sm.gcode_line);
                    }
                }
*/
	return 0;
}

int main (void) {
	int i;
  char buf[50];
	char c;
	int count = 0;

	SCB->SHCSR |= (1<<16); // fault handler enable
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<16)|(1<<6); // Enable AHB clock to the IOCON and GPIO domain
	LPC_IOCON->R_PIO1_2 = 0xd1;
	LPC_GPIO1->DIR |= (1<<2); // led
	LED_OFF;

	LPC_GPIO1->DIR &= ~(1<<10); // button

	LPC_GPIO1->DIR |= (1<<11);  // heater2
	LPC_GPIO3->DIR |= (1<<5)|(1<<3)|(1<<2);   // mreset, fan, heater1
	MRESET(1);
  HEATER1(0);
  HEATER2(0);
	COOLANT(0);

 	NVIC_SetPriority(SysTick_IRQn, 7);
	SysTick_Config(SystemCoreClock / 1000);
	init_uart(115200 * 4);
	init_adc(ADC_CLK);
	xprintf("*** Hermit\r\n");
  if (init_filesystem()) error(3);
  init_stepper();

	LED_ON;
	for (;;) {
		poll();

		i = recv_uart(&buf[count], 79 - count);
		if (i <= 0) continue;
		count += i;
		c = buf[count - 1];

		if (c == 0x1b) {
			count = 0;
		} else
    if (buf[0] == 0x1b && buf[1] == '[' && count == 3) {
        if (c == 'A') {
            parseGcode("G01Z1F240", &target);
            calc_end();
        } else
        if (c == 'B') {
            parseGcode("G01Z-1F240", &target);
            calc_end();
        } else
        if (c == 'C') {
            parseGcode("G01E1F300", &target);
            calc_end();
        } else
        if (c == 'D') {
            parseGcode("G01E-10F300", &target);
            calc_end();
        }
        count = 0;
    } else
    if (c == '\r' || c == '\n') {
        if (count) {
            buf[count] = 0;
            if (command(buf)) {
                xprintf("error\r\n");
            } else {
                xprintf("ok\r\n");
            }
            count = 0;
        }
    }

	}
}

void error (int n) {
	sm.dir[0] = 0; sm.dir[1] = 0; sm.dir[2] = 0; sm.dir[3] = 0;
  HEATER1(0);
  HEATER2(0);
	COOLANT(0);
  LPC_IOCON->R_PIO1_1 = 0xd1;
	LPC_GPIO1->DIR |= (1<<1);
	for (;;) {
		LED_ON;
		wait_ms(100 * n);
		LED_OFF;
		wait_ms(100 * n);
	}
}

void HardFault_Handler (void) {
	register unsigned int _msp __asm("msp");

	sm.dir[0] = 0; sm.dir[1] = 0; sm.dir[2] = 0; sm.dir[3] = 0;
  HEATER1(0);
  HEATER2(0);
	COOLANT(0);
	xprintf("HardFault %08x\r\n", _msp);
	for (int i = 0; i < 10; i ++) {
		xprintf(" %08x", *(volatile uint32_t*)_msp);
	}
  LPC_IOCON->R_PIO1_1 = 0xd1;
	LPC_GPIO1->DIR |= (1<<2);
  for (;;) {
    LED_ON;
    for (volatile int w = 0; w < 100000; w ++);
    LED_OFF;
    for (volatile int w = 0; w < 1000000; w ++);
  }
}

void MemManage_Handler() {
	sm.dir[0] = 0; sm.dir[1] = 0; sm.dir[2] = 0; sm.dir[3] = 0;
  HEATER1(0);
  HEATER2(0);
	COOLANT(0);
  LPC_IOCON->R_PIO1_1 = 0xd1;
	LPC_GPIO1->DIR |= (1<<2);
  for (;;) {
    LED_ON;
    for (volatile int w = 0; w < 1000000; w ++);
    LED_OFF;
    for (volatile int w = 0; w < 100000; w ++);
  }
}

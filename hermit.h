#ifndef _HERMIT_H_
#define _HERMIT_H_

#include "stepper.h"
#include "uart.h"
#include <stdio.h>

#define DBG(...)
//#define DBG(...) xprintf("" __VA_ARGS__)
#define ERROR(...) xprintf("" __VA_ARGS__)

#define NUM_MODE 1 // 0:normal 1:buggy

#define CBUFFER_NUM 5

#define ERROR_GCODE_UNKNOWN     0x0001
#define ERROR_OVER_X            0x0002
#define ERROR_OVER_Y            0x0004
#define ERROR_OVER_Z            0x0008
#define ERROR_OVER_M_PER_SEC    0x0010
#define ERROR_OVER_M_PER_SEC_X  0x0020
#define ERROR_OVER_M_PER_SEC_Y  0x0040
#define ERROR_OVER_M_PER_SEC_Z  0x0080
#define ERROR_STEP_LOSS_B_X     0x0400
#define ERROR_STEP_LOSS_A_X     0x0200
#define ERROR_OCD_X             0x0100
#define ERROR_STEP_LOSS_B_Y     0x4000
#define ERROR_STEP_LOSS_A_Y     0x2000
#define ERROR_OCD_Y             0x1000
#define ERROR_OVER_E            0x10000
#define ERROR_OVER_M_PER_SEC_E  0x20000

void wait_ms (int msec);
void error (int n);
int poll (void);

int gcode (char *filename);
//int callback_gcode (void);

void parseGcode (char *buf, XYZ *target);
void listfile (void);

void init_timer (void);
int init_filesystem (void);


#define GPIO(gpio, pin, val) gpio->MASKED_ACCESS[(1<<pin)] = (val<<pin)

#define LED_ON	GPIO(LPC_GPIO1, 2, 1)
#define LED_OFF	GPIO(LPC_GPIO1, 2, 0)

#define M1_CS(n) GPIO(LPC_GPIO2, 0, n)
#define M2_CS(n) GPIO(LPC_GPIO2, 8, n)
#define M3_CS(n) GPIO(LPC_GPIO2, 4, n)
#define M4_CS(n) GPIO(LPC_GPIO2, 9, n)

#define M1_PULSE(n) GPIO(LPC_GPIO2, 6, n)
#define M2_PULSE(n) GPIO(LPC_GPIO2, 7, n)
#define M3_PULSE(n) GPIO(LPC_GPIO3, 4, n)
#define M4_PULSE(n) GPIO(LPC_GPIO2, 5, n)

#define M1_DIR(n) L6470_StepClock(0, n)
#define M2_DIR(n) L6470_StepClock(1, n)
#define M3_DIR(n) L6470_StepClock(2, n)
#define M4_DIR(n) L6470_StepClock(3, n)

#define M1_STBY() L6470_SoftHiZ(0)
#define M2_STBY() L6470_SoftHiZ(1)
#define M3_STBY() L6470_SoftHiZ(2)
#define M4_STBY() L6470_SoftHiZ(3)

#define M1_LIMIT(n) (L6470_GetParam(0, STATUS) & (1<<2) ? 0 : 1);
#define M2_LIMIT(n) (L6470_GetParam(1, STATUS) & (1<<2) ? 0 : 1);
#define M3_LIMIT(n) (L6470_GetParam(2, STATUS) & (1<<2) ? 0 : 1);
#define M4_LIMIT(n) (L6470_GetParam(3, STATUS) & (1<<2) ? 0 : 1);

#define MRESET(n) GPIO(LPC_GPIO3, 5, n)
#define HEATER1(n) GPIO(LPC_GPIO3, 2, n)
#define HEATER2(n) GPIO(LPC_GPIO1, 11, n)
#define COOLANT(n) GPIO(LPC_GPIO3, 3, n)

#define BUTTON_INPUT (!LPC_GPIO1->MASKED_ACCESS[(1<<10)])

#endif

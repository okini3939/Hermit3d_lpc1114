#include <LPC11xx.h>
#include "stepper.h"

#define TIMER_REF 1000000 // Hz

extern Timeout t[];
extern StepperMotor sm;

void move (int n);

void attach_timer0 (timer_func func, float t) {
	t = t * TIMER_REF;
	if (t < 1) t = 1;
	LPC_TMR16B0->MR0 = (int)t;
	LPC_TMR16B0->TCR = (1<<0);
}
void attach_timer1 (timer_func func, float t) {
	t = t * TIMER_REF;
	if (t < 1) t = 1;
	LPC_TMR16B1->MR0 = (int)t;
	LPC_TMR16B1->TCR = (1<<0);
}
void attach_timer2 (timer_func func, float t) {
	t = t * TIMER_REF;
	if (t < 1) t = 1;
	LPC_TMR32B0->MR0 = (int)t;
	LPC_TMR32B0->TCR = (1<<0);
}
void attach_timer3 (timer_func func, float t) {
	t = t * TIMER_REF;
	if (t < 1) t = 1;
	LPC_TMR32B1->MR0 = (int)t;
	LPC_TMR32B1->TCR = (1<<0);
}

void detach_timer0 (void) {
	LPC_TMR16B0->TCR = (1<<1);
}
void detach_timer1 (void) {
	LPC_TMR16B1->TCR = (1<<1);
}
void detach_timer2 (void) {
	LPC_TMR32B0->TCR = (1<<1);
}
void detach_timer3 (void) {
	LPC_TMR32B1->TCR = (1<<1);
}

void init_timer (void) {
	t[0].attach = &attach_timer0;
	t[1].attach = &attach_timer1;
	t[2].attach = &attach_timer2;
	t[3].attach = &attach_timer3;
	t[0].detach = &detach_timer0;
	t[1].detach = &detach_timer1;
	t[2].detach = &detach_timer2;
	t[3].detach = &detach_timer3;

	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<7); // CT16B0
	LPC_TMR16B0->TCR = (1<<1);
	LPC_TMR16B0->CTCR = 0;
	LPC_TMR16B0->PWMC = 0;
	LPC_TMR16B0->IR = 0x1f;
	LPC_TMR16B0->PR = SystemCoreClock / TIMER_REF - 1;
	LPC_TMR16B0->MR0 = 10; // 1us
	LPC_TMR16B0->MCR = (1<<1)|(1<<0); // Interrupt and Reset on MR0
	NVIC_SetPriority(TIMER_16_0_IRQn, 2);
	NVIC_EnableIRQ(TIMER_16_0_IRQn);

	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<8); // CT16B1
	LPC_TMR16B1->TCR = (1<<1);
	LPC_TMR16B1->CTCR = 0;
	LPC_TMR16B1->PWMC = 0;
	LPC_TMR16B1->IR = 0x1f;
	LPC_TMR16B1->PR = SystemCoreClock / TIMER_REF - 1;
	LPC_TMR16B1->MR0 = 10; // 1us
	LPC_TMR16B1->MCR = (1<<1)|(1<<0); // Interrupt and Reset on MR0
	NVIC_SetPriority(TIMER_16_1_IRQn, 3);
	NVIC_EnableIRQ(TIMER_16_1_IRQn);

	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<9); // CT32B0
	LPC_TMR32B0->TCR = (1<<1);
	LPC_TMR32B0->CTCR = 0;
	LPC_TMR32B0->PWMC = 0;
	LPC_TMR32B0->IR = 0x1f;
	LPC_TMR32B0->PR = SystemCoreClock / TIMER_REF - 1;
	LPC_TMR32B0->MR0 = 10; // 1us
	LPC_TMR32B0->MCR = (1<<1)|(1<<0); // Interrupt and Reset on MR0
	NVIC_SetPriority(TIMER_32_0_IRQn, 4);
	NVIC_EnableIRQ(TIMER_32_0_IRQn);

	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<10); // CT32B1
	LPC_TMR32B1->TCR = (1<<1);
	LPC_TMR32B1->CTCR = 0;
	LPC_TMR32B1->PWMC = 0;
	LPC_TMR32B1->IR = 0x1f;
	LPC_TMR32B1->PR = SystemCoreClock / TIMER_REF - 1;
	LPC_TMR32B1->MR0 = 10; // 1us
	LPC_TMR32B1->MCR = (1<<1)|(1<<0); // Interrupt and Reset on MR0
	NVIC_SetPriority(TIMER_32_1_IRQn, 5);
	NVIC_EnableIRQ(TIMER_32_1_IRQn);
}

void TIMER16_0_IRQHandler (void) {
	LPC_TMR16B0->IR = 1;
	LPC_TMR16B0->TCR = (1<<1);
	move(0);
}
void TIMER16_1_IRQHandler (void) {
	LPC_TMR16B1->IR = 1;
	LPC_TMR16B1->TCR = (1<<1);
	move(1);
}
void TIMER32_0_IRQHandler (void) {
	LPC_TMR32B0->IR = 1;
	LPC_TMR32B0->TCR = (1<<1);
	move(2);
}
void TIMER32_1_IRQHandler (void) {
	LPC_TMR32B1->IR = 1;
	LPC_TMR32B1->TCR = (1<<1);
	move(3);
}

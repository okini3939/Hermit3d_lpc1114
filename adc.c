#include <LPC11xx.h>
#include <math.h>
#include "adc.h"


int get_temp (int ch) {
	float ad, f;

	ad = get_adc(ch);
	ad = ad / (float)0x3ff * 3.3;

	switch (ch) {
  case ADC_TEMP_BEAD1:
  case ADC_TEMP_BEAD2:
    f = ad;
//    f = THERMISTOR_R0 * f / (1.0 - f);
    f = f / ((3.3 - f) / THERMISTOR_R0);
    f = (1.0 / ((1.0 / (THERMISTOR_T0 + 273.15)) + ((float)log(f / THERMISTOR_R0) / THERMISTOR_B))) - 273.15;
    break;
  case ADC_TEMP_EXT:
    f = ad * 0.97;
//    f = THERMISTOR_ER0 * f / (1.0 - f);
    f = f / ((3.3 - f) / THERMISTOR_ER0);
    f = (1.0 / ((1.0 / (THERMISTOR_ET0 + 273.15)) + ((float)log(f / THERMISTOR_ER0) / THERMISTOR_EB))) - 273.15;
    break;
  }
	return (int)f;
}


uint32_t get_adc (int ch) {
	uint32_t ad = 0, stat;

	LPC_ADC->CR &= ~0xff;
	LPC_ADC->CR |= (1<<24)|(1<<ch); // start adc
	for (;;) {
		stat = LPC_ADC->STAT;
		if (stat & (1<<ch)) {
			ad = (LPC_ADC->DR[ch] >> 6) & 0x3ff;
			break;
		}
		if (stat & (1<<(ch+8))) { // overrun
			stat = LPC_ADC->DR[ch];
			break;
		}
	}
	LPC_ADC->CR &= ~(1<<24);	// stop
	return ad;
}

void init_adc (uint32_t ADC_Clk) {

	LPC_SYSCON->PDRUNCFG &= ~(1<<4); // power up
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<13); // AHB clock
	LPC_ADC->CR = ((SystemCoreClock / LPC_SYSCON->SYSAHBCLKDIV) / ADC_Clk - 1) << 8;

	// ADC I/O config
	LPC_IOCON->R_PIO0_11 &= ~0x8F; // ADC0
	LPC_IOCON->R_PIO0_11 |= 0x02;
	LPC_IOCON->R_PIO1_0  &= ~0x8F; // ADC1
	LPC_IOCON->R_PIO1_0  |= 0x02;
	LPC_IOCON->R_PIO1_1  &= ~0x8F; // ADC2
	LPC_IOCON->R_PIO1_1  |= 0x02;
	LPC_IOCON->R_PIO0_11 = 0x02; // Select AD0 pin function
	LPC_IOCON->R_PIO1_0 = 0x02;	// Select AD1 pin function
	LPC_IOCON->R_PIO1_1 = 0x02;	// Select AD2 pin function
}


#define ADC_TEMP_BEAD1 0
#define ADC_TEMP_BEAD2 1
#define ADC_TEMP_EXT 2

#define ADC_CLK			2400000		/* set to 2.4Mhz */

#define THERMISTOR_B   3435
#define THERMISTOR_R0  10.0 // k ohm
#define THERMISTOR_T0  25.0
#define THERMISTOR_EB  4092
#define THERMISTOR_ER0 47.0 // k ohm
#define THERMISTOR_ET0 25.0

int get_temp (int ch);
uint32_t get_adc (int ch);
void init_adc (uint32_t ADC_Clk);

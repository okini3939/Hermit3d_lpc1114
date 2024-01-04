#include <LPC11xx.h>
#include "l6470.h"
#include "hermit.h"

static volatile int busy_spi = 0;

static uint8_t send (int dev, uint8_t dat) {
  volatile int w;
  uint8_t ret;

  while (busy_spi);
  busy_spi = 1;
	switch (dev) {
		case 0: M1_CS(0); break;
		case 1: M2_CS(0); break;
		case 2: M3_CS(0); break;
		case 3: M4_CS(0); break;
	}
	__NOP();
  LPC_SSP1->DR = dat;
  while (!(LPC_SSP1->SR & (1<<2)));
  ret = LPC_SSP1->DR;
	switch (dev) {
		case 0: M1_CS(1); break;
		case 1: M2_CS(1); break;
		case 2: M3_CS(1); break;
		case 3: M4_CS(1); break;
	}
  busy_spi = 0;
  for (w = 0; w < 24; w ++) __NOP();
  return ret;
}

void L6470_init2 (void) {
	LPC_GPIO2->DIR |= (1<<8)|(1<<0);
}

void L6470_init (void) {
  int i;

	/* Initialize SPI1 module and attach it to the I/O pad */
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<18);
	LPC_SYSCON->PRESETCTRL &= ~(1<<2);	/* Set SSP1 reset */
	LPC_SYSCON->PRESETCTRL |= (1<<2);	/* Release SSP1 reset */
	LPC_SYSCON->SSP1CLKDIV = 1;			/* PCLK = sysclk */
//	LPC_SSP1->CPSR = 0x02;		/* fc=PCLK/2 */
  LPC_SSP1->CPSR = 12;		/* fc=PCLK/12 */
	LPC_SSP1->CR0 = 0x0007;		/* Mode-0, 8-bit */
	LPC_SSP1->CR1 = 0x02;			/* Enable SPI */
	LPC_IOCON->PIO2_1 = 0x02;	/* SCK0 */
	LPC_IOCON->PIO2_3 = 0x02;	/* MOSI0 */
	LPC_IOCON->PIO2_2 = 0x12;	/* MISO0/pull-up */

  for (i = 0; i < 4; i ++) {
    L6470_Resets(i);
    L6470_SetParam(i, KVAL_HOLD, 0xa0);
    L6470_SetParam(i, KVAL_RUN, 0xc0);
    L6470_SetParam(i, MAX_SPEED, 26);
    L6470_SetParam(i, ACC, 0x80);
    L6470_SetParam(i, DEC, 0x80);
    L6470_SetParam(i, OCR_TH, 0x0f); // 375*(n+1) mA
    L6470_SetParam(i, STALL_TH, 0x20); // 31.25*(n+1) mA
    L6470_SetParam(i, STEP_MODE, 4); // 1/16
    L6470_ResetPos(i);
  }
}


static void send_bytes (int dev, unsigned char temp[], int i){
    while(0 < i--){
        temp[i] = send(dev, temp[i]);
    }
}

static void send_nop(int dev){
    send(dev, 0x00);
}

void L6470_SetParam(int dev, int param,int value){
    int n = (param>>8)/8;
    int m = (param>>8)%8;
    unsigned char temp[32] = {0};
//        pc.printf("palam = %x\n",param);
//        pc.printf("n = %x\n",n);
//        pc.printf("m = %x\n",m);
    if(m==0){
        temp[n] = 0x00|(unsigned char)(param&0xFF);
        while(0 < n--){
            temp[n]=(unsigned char) (value >> 8*n)&0xFF;
        }
        send_bytes(dev, temp,sizeof temp/sizeof temp[0]);
    }else{
        temp[n+1] = 0x00|(unsigned char)(param&0xFF);
        temp[n] =(unsigned char) (value >> 8*n)&~(0xff<<m);
        while(0 < n--){
            temp[n]=(unsigned char) (value >> 8*n)&0xFF;
        }
        send_bytes(dev, temp,sizeof temp/sizeof temp[0]);
    }
}


int L6470_GetParam(int dev, int param){
    int i, value = 0;
    int n = (param>>8)/8;
    int m = (param>>8)%8;
    unsigned char temp[32] = {0};
//        pc.printf("palam = %x\n",param);
//        pc.printf("n = %x\n",n);
//        pc.printf("m = %x\n",m);
    if(m==0){
        for(i = 0; i < n+1; i++){
            temp[i]=0;
        }
        temp[n] = 0x20|(unsigned char)(param&0xFF);
        send_bytes(dev, temp,sizeof temp/sizeof temp[0]);
        while(0 < n--){
            value |= (int)temp[n] << 8*n;
        }
    }else{
        n++;
        for(i = 0; i < n+2; i++){
            temp[i]=0;
        }
        temp[n] = 0x20|(unsigned char)(param&0xFF);
        send_bytes(dev, temp,sizeof temp/sizeof temp[0]);
        while(0 < n--){
            value |= (int)temp[n] << 8*n;
        }
    }
    return(value);    
}

void L6470_Run(int dev, unsigned char dir,int spd){
    unsigned char temp[4];        
    temp[3] = 0x50|dir;
    temp[2] = (unsigned char) (spd >> 16)&0x0F;
    temp[1] = (unsigned char) (spd >>  8)&0xFF;
    temp[0] = (unsigned char) (spd >>  0)&0xFF;
    send_bytes(dev, temp,sizeof temp/sizeof temp[0]);
}

void L6470_StepClock(int dev, unsigned char dir){
    send(dev, 0x58|dir);
}


void L6470_Move(int dev, unsigned char dir,int n_step){
    unsigned char temp[4];        
    temp[3] = 0x40|dir;
    temp[2] = (unsigned char) (n_step >> 16)&0x3F;
    temp[1] = (unsigned char) (n_step >>  8)&0xFF;
    temp[0] = (unsigned char) (n_step >>  0)&0xFF;
    send_bytes(dev, temp,sizeof temp/sizeof temp[0]);
}
 
void L6470_GoTo(int dev, int abs_pos){
    unsigned char temp[4];        
    temp[3] = 0x60;
    temp[2] = (unsigned char) (abs_pos >> 16)&0x3F;
    temp[1] = (unsigned char) (abs_pos >>  8)&0xFF;
    temp[0] = (unsigned char) (abs_pos >>  0)&0xFF;
    send_bytes(dev, temp,sizeof temp/sizeof temp[0]);
}


void L6470_GoTo_DIR(int dev, unsigned char dir,int abs_pos){
    unsigned char temp[4];     
    temp[3] = 0x68|dir;
    temp[2] = (unsigned char) (abs_pos >> 16)&0x3F;
    temp[1] = (unsigned char) (abs_pos >>  8)&0xFF;
    temp[0] = (unsigned char) (abs_pos >>  0)&0xFF;
    send_bytes(dev, temp,sizeof temp/sizeof temp[0]);
}


void L6470_GoUntil(int dev, unsigned char act,unsigned char dir,int spd){
    unsigned char temp[4];     
    temp[3] = 0x82|(act << 3)|dir;
    temp[2] = (unsigned char) (spd >> 16)&0x0F;
    temp[1] = (unsigned char) (spd >>  8)&0xFF;
    temp[0] = (unsigned char) (spd >>  0)&0xFF;
    send_bytes(dev, temp,sizeof temp/sizeof temp[0]);
}

void L6470_ReleaseSW(int dev, unsigned char act,unsigned char dir){
    send(dev, 0x92|(act << 3)|dir);
}

void L6470_GoHome(int dev){
    send(dev, 0x70);
}

void L6470_GoMark(int dev){
    send(dev, 0x78);
}

void L6470_ResetPos(int dev){
    send(dev, 0xD8);
}

void L6470_ResetDevice(int dev){
    send(dev, 0xC0);
}

void L6470_SoftStop(int dev){
    send(dev, 0xB0);
}

void L6470_HardStop(int dev){
    send(dev, 0xB8);
}


void L6470_SoftHiZ(int dev){
    send(dev, 0xA0);
}

void L6470_HardHiZ(int dev){
    send(dev, 0xA8);
}

void L6470_Resets(int dev){
    L6470_ResetDevice(dev);
    L6470_SetParam(dev, ABS_POS,RH_ABS_POS);
    L6470_SetParam(dev, EL_POS,RH_EL_POS);
    L6470_SetParam(dev, MARK,RH_MARK);
    L6470_SetParam(dev, SPEED,RH_SPEED);
    L6470_SetParam(dev, ACC,RH_ACC);
    L6470_SetParam(dev, DEC,RH_DEC);
    L6470_SetParam(dev, MAX_SPEED,RH_MAX_SPEED);
    L6470_SetParam(dev, MIN_SPEED,RH_MIN_SPEED);
    L6470_SetParam(dev, KVAL_HOLD,RH_KVAL_HOLD);
    L6470_SetParam(dev, KVAL_RUN,RH_KVAL_RUN);
    L6470_SetParam(dev, KVAL_ACC,RH_KVAL_ACC);
    L6470_SetParam(dev, KVAL_DEC,RH_KVAL_DEC);
    L6470_SetParam(dev, INT_SPD,RH_INT_SPD);
    L6470_SetParam(dev, ST_SLP,RH_ST_SLP);
    L6470_SetParam(dev, FN_SLP_ACC,RH_FN_SLP_ACC);
    L6470_SetParam(dev, FN_SLP_DEC,RH_FN_SLP_DEC);
    L6470_SetParam(dev, K_THERA,RH_K_THERA);
    L6470_SetParam(dev, OCR_TH,RH_OCR_TH);      
    L6470_SetParam(dev, STALL_TH,RH_STALL_TH);
    L6470_SetParam(dev, FS_SPD,RH_FS_SPD);
    L6470_SetParam(dev, STEP_MODE,RH_STEP_MODE);
    L6470_SetParam(dev, ARARM_FN,RH_ARARM_FN);
    L6470_SetParam(dev, CONFIG,RH_CONFIG);            
}

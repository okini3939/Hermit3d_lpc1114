#ifndef _L6470_H_
#define _L6470_H_

#define FORWARD     1
#define REVERSE     0

// name         (length<<8)+address
#define ABS_POS      ((22<<8)+0x01)    //Current position         
#define EL_POS        ((9<<8)+0x02)    //Electrical position
#define MARK         ((22<<8)+0x03)    //Mark position
#define SPEED        ((20<<8)+0x04)    //Current speed
#define ACC          ((12<<8)+0x05)    //Acceleration
#define DEC          ((12<<8)+0x06)    //Deceleration
#define MAX_SPEED    ((10<<8)+0x07)    //Maximum speed
#define MIN_SPEED    ((13<<8)+0x08)    //Minimum speed
#define KVAL_HOLD     ((8<<8)+0x09)    //Holding KVAL
#define KVAL_RUN      ((8<<8)+0x0A)    //Constant speed KVAL
#define KVAL_ACC      ((8<<8)+0x0B)    //Acceleration starting KVAL
#define KVAL_DEC      ((8<<8)+0x0C)    //Deceleration starting KVAL
#define INT_SPD      ((14<<8)+0x0D)    //Intersect speed
#define ST_SLP        ((8<<8)+0x0E)    //Start slope
#define FN_SLP_ACC    ((8<<8)+0x0F)    //Acceleration final slope
#define FN_SLP_DEC    ((8<<8)+0x10)    //Deceleration final slope
#define K_THERA       ((4<<8)+0x11)    //Thermal compensation factor
#define ADC_OUT       ((5<<8)+0x12)    //ADC output 
#define OCR_TH        ((4<<8)+0x13)    //OCD threshold
#define STALL_TH      ((7<<8)+0x14)    //STALL threshold
#define FS_SPD       ((10<<8)+0x15)    //Full step speed
#define STEP_MODE     ((8<<8)+0x16)    //Step mode
#define ARARM_FN      ((8<<8)+0x17)    //Alarms enables
#define CONFIG       ((16<<8)+0x18)    //IC configuration
#define STATUS       ((16<<8)+0x19)    //Status



//RH = ResetHex
#define RH_ABS_POS     0       //Current position         
#define RH_EL_POS      0       //Electrical position
#define RH_MARK        0       //Mark position
#define RH_SPEED       0       //Current speed
#define RH_ACC         0x8A    //Acceleration
#define RH_DEC         0x8A    //Deceleration
#define RH_MAX_SPEED   0x20    //Maximum speed
#define RH_MIN_SPEED   0       //Minimum speed
#define RH_KVAL_HOLD   0xFF    //Holding KVAL
#define RH_KVAL_RUN    0xFF    //Constant speed KVAL
#define RH_KVAL_ACC    0xFF    //Acceleration starting KVAL
#define RH_KVAL_DEC    0xFF    //Deceleration starting KVAL
#define RH_INT_SPD     0x408   //Intersect speed
#define RH_ST_SLP      0x19    //Start slope
#define RH_FN_SLP_ACC  0x29    //Acceleration final slope
#define RH_FN_SLP_DEC  0x29    //Deceleration final slope
#define RH_K_THERA     0x0     //Thermal compensation factor
//         ADC_OUT     ReadOnly //ADC output 
#define RH_OCR_TH      0xF     //OCD threshold
#define RH_STALL_TH    0x7F    //STALL threshold
#define RH_FS_SPD      0x27    //Full step speed
#define RH_STEP_MODE   0x7     //Step mode
#define RH_ARARM_FN    0xFF    //Alarms enables
#define RH_CONFIG      0x2E88  //IC configuration
//      RH_STATUS      ReadOnly //Status


void L6470_init (void);

void L6470_SetParam(int dev, int param,int value);
int L6470_GetParam(int dev, int param);
void L6470_Run(int dev, unsigned char dir,int spd);
void L6470_StepClock(int dev, unsigned char dir);
void L6470_Move(int dev, unsigned char dir,int n_step);
void L6470_GoTo(int dev, int abs_pos);
void L6470_GoTo_DIR(int dev, unsigned char dir,int abs_pos);
void L6470_GoUntil(int dev, unsigned char act,unsigned char dir,int spd);
void L6470_ReleaseSW(int dev, unsigned char act,unsigned char dir);
void L6470_GoHome(int dev);
void L6470_GoMark(int dev);
void L6470_ResetPos(int dev);
void L6470_ResetDevice(int dev);
void L6470_SoftStop(int dev);
void L6470_HardStop(int dev);
void L6470_SoftHiZ(int dev);
void L6470_HardHiZ(int dev);
void L6470_Resets(int dev);

#endif

#ifndef _STEPPER_H_
#define _STEPPER_H_

#define ENABLE_ACC
#define ACC_DEG     0.7
#define ACC_F       8 // mm/sec
#define ACC_STEP    0.02
#define DEC_F       8 // mm/sec
#define DEC_STEP    0.08
//#define BETWEEN     0.1

#define LIMIT_F      mm_per_cycle_x // mm/sec
#define LIMIT_F_Z    mm_per_cycle_z // mm/sec
#define LIMIT_F_E    mm_per_cycle_e // mm/sec

#define mm_per_cycle_x     60
#define mm_per_cycle_y     60
#define mm_per_cycle_z     4
#define mm_per_cycle_e     5
#define step_per_cycle_x   (200 * 16)
#define step_per_cycle_y   (200 * 16)
#define step_per_cycle_z   (200 * 16)
#define step_per_cycle_e   (200 * 16)
#define min_pps_x          (100 * 16)
#define max_mps_x          (500 * 16)
#define dir_rev_x          0
#define dir_rev_y          0
#define dir_rev_z          0
#define dir_rev_e          0

#define V_STEP 5

typedef struct {
    float x, y, z, e, f;
    float xyz[4];
    float v, d, s, ff;
    float diff[4], spp[4];
    int step[4], diff_step[4], dir[4];
    int acc, acc_xyz, dec, dec_xyz, dec_start;
    float acc_percent, dec_percent;
    int gcode_line;
    float offset[4];
    int incremental;
	  int temp_bed, temp_ext;
} XYZ;

typedef struct {
    float msec;
		int pulse;
    int current[4], dir[4];
    volatile int work;
    int gcode_line;
    float acc_percent, dec_percent;
    int trigger;
	  int temp_bed, temp_ext;
	  int temp_bed_flg, temp_ext_flg;
//    XYZ target;
} StepperMotor;

typedef void(*timer_func)(void);

typedef struct {
    void(*attach)(timer_func func, float t);
	  void(*detach)(void);
} Timeout;

void init_stepper (void);
void calc_xyz (XYZ *target);
void calc_reset (void);
void calc_end (void);
void copy_xyz (XYZ *dst, XYZ *src);

#endif

#include <LPC11xx.h>
#include <math.h>
#include "l6470.h"
#include "hermit.h"
#include "stepper.h"
#include "cbuffer.h"
#include <string.h>
#include <stdlib.h>

Timeout t[4];
static XYZ cbuf[CBUFFER_NUM + 1];
StepperMotor sm;
XYZ sm_target;
XYZ target;
static XYZ late;
static float acc_percent = 0;

static const float mm_per_step[4] = {(float)mm_per_cycle_x / step_per_cycle_x,
                         (float)mm_per_cycle_y / step_per_cycle_y,
                         (float)mm_per_cycle_z / step_per_cycle_z,
                         (float)mm_per_cycle_e / step_per_cycle_e};
static const XYZ target_zero = {0};
static const StepperMotor sm_zero = {0};

void init_stepper (void) {
	MRESET(0);
	LPC_GPIO2->DIR |= (1<<9)|(1<<8)|(1<<7)|(1<<6)|(1<<5)|(1<<4)|(1<<0);
	LPC_GPIO3->DIR |= (1<<4);
	M1_CS(1);
	M2_CS(1);
	M3_CS(1);
	M4_CS(1);
  M1_PULSE(0);
  M2_PULSE(0);
  M3_PULSE(0);
  M4_PULSE(0);
  wait_ms(1);
	MRESET(1);
  wait_ms(10);

  L6470_init();

  init_cbuffer(CBUFFER_NUM, cbuf);
	init_timer();
}

void move (int n);
void isr_move_x () {
    move(0);
}
void isr_move_y () {
    move(1);
}
void isr_move_z () {
    move(2);
}
void isr_move_e () {
    move(3);
}

// set next interrupt
void move_config (int n) {
  float tt = 0;

  t[n].detach();
  if (sm_target.acc && sm.acc_percent > 1.0) {
//  if (sm_target.acc && sm.acc_percent > 1.0 && n != 3) {
    // acc
    tt = sm_target.spp[n] * sm.acc_percent;
    if (sm_target.acc_xyz == n) {
      sm.acc_percent -= ACC_STEP;
    }
  }
  if (sm.trigger && sm_target.dec) {
    // dec
    if (!tt || sm.dec_percent > sm.acc_percent) {
      tt = sm_target.spp[n] * sm.dec_percent;
    }
//    if (sm_target.dec_xyz == n && sm.dec_percent < DEC_PERCENT) {
    if (sm_target.dec_xyz == n && sm.dec_percent < (sm_target.ff / DEC_F)) {
      sm.dec_percent += DEC_STEP;
    }
  }
  if (!tt) {
    // top speed
    tt = sm_target.spp[n];
  }
  if (tt) {
    if (tt < 0.000002) tt = 0.000002;
    switch (n) {
      case 0: t[n].attach(&isr_move_x, tt); break;
      case 1: t[n].attach(&isr_move_y, tt); break;
      case 2: t[n].attach(&isr_move_z, tt); break;
      case 3: t[n].attach(&isr_move_e, tt); break;
    }
  }
}

// get next data
int move_next () {
  int i, j;

  for (i = 0; i < 4; i ++) {
    t[i].detach();
  }

    // end of 1 move
#ifdef BETWEEN
    if (sm_target.dec == 2) {
      // between wait
      sm_target.dec = 0;
//      t[0].attach(&isr_move_x, BETWEEN);
//      t[1].attach(&isr_move_y, BETWEEN);
//      t[2].attach(&isr_move_z, BETWEEN);
      t[3].attach(&isr_move_e, BETWEEN);
      return 1;
    } else
#endif
    if (dequeue(&sm_target)) {
      // next move
      sm.gcode_line = sm_target.gcode_line;
						if (sm.gcode_line & 1) {
							LED_ON;
						} else {
							LED_OFF;
						}
/*
DBG("<%d>", sm.gcode_line);
DBG(" sec=%0.3f vect=%0.3f deg=%0.3f f=%0.3f\r\n", sm_target.gcode_line, sm_target.s, sm_target.v, sm_target.d, sm_target.f);
DBG(" step: %4d %4d %4d %4d\r\n", sm_target.step[0], sm_target.step[1], sm_target.step[2], sm_target.step[3]);
DBG(" spp : %0.6f %0.6f %0.6f %0.6f\r\n", sm_target.spp[0], sm_target.spp[1], sm_target.spp[2], sm_target.spp[3]);
DBG(" acc: %d [%d] %0.3f /", sm_target.acc, sm_target.acc_xyz, sm_target.acc_percent);
DBG(" dec: %d [%d] %d %0.3f\r\n", sm_target.dec, sm_target.dec_xyz, sm_target.dec_start, sm_target.dec_percent);
*/
      sm.work = 1;
      for (i = 0; i < 4; i ++) {
        j = sm_target.dir[i];
        if (sm.dir[i] != j) {
          sm.dir[i] = j;
          switch (i) {
            case 0:
#if dir_rev_x
                j = -j;
#endif 
                M1_DIR(j >= 0 ? 0 : 1);
                break;
            case 1: 
#if dir_rev_y
                j = -j;
#endif 
                M2_DIR(j >= 0 ? 0 : 1);
                break;
            case 2: 
#if dir_rev_z
                j = -j;
#endif 
                M3_DIR(j >= 0 ? 0 : 1); 
                break;
            case 3:
#if dir_rev_e
                j = -j;
#endif 
                M4_DIR(j >= 0 ? 0 : 1); 
                break;
          }
        }
      }

      sm.trigger = 0;
      if (sm_target.acc) {
        sm.acc_percent = sm_target.acc_percent;
      }
      if (sm_target.dec && sm.current[sm_target.dec_xyz] == sm_target.dec_start) {
        sm.dec_percent = sm_target.dec_percent;
        sm.trigger = 1;
      }

			sm.pulse = 0;
      for (i = 0; i < 4; i ++) {
        move_config(i);
      }
    } else {
      // all done
      for (i = 0; i < 4; i ++) {
        sm.dir[i] = 0;
      }
      M1_STBY(); M2_STBY(); M3_STBY(); M4_STBY();
      sm.work = 0;
      DBG("*** done\r\n");
      return 1;
    }
    return 0;
}

// call from interrupt
void move (int n) {
  if (!sm.pulse &&
    !sm.dir[0] && !sm.dir[1] && !sm.dir[2] && !sm.dir[3]) {
      // 1 line done, next line
      move_next();
			return;
  }

  // create pulse
  if (sm.pulse & (1<<n)) {
    // low
    switch (n) {
      case 0: M1_PULSE(0); break;
      case 1: M2_PULSE(0); break;
      case 2: M3_PULSE(0); break;
      case 3: M4_PULSE(0); break;
    }
    sm.pulse &= ~(1<<n);
  } else
  if (sm.dir[n]) {
    // high
    switch (n) {
      case 0: M1_PULSE(1); break;
      case 1: M2_PULSE(1); break;
      case 2: M3_PULSE(1); break;
      case 3: M4_PULSE(1); break;
    }
    sm.current[n] += sm.dir[n];
    if ((sm.dir[n] > 0 && sm.current[n] >= sm_target.step[n]) ||
      (sm.dir[n] < 0 && sm.current[n] <= sm_target.step[n])) {
        // end of move
        sm.dir[n] = 0;
    }
    sm.pulse |= (1<<n);

    if (sm_target.dec && sm_target.dec_xyz == n) {
      if (sm.current[n] == sm_target.dec_start) {
        sm.dec_percent = sm_target.dec_percent;
        sm.trigger = 1;
      }
    }
  }

  move_config(n);
}

int max_xyz (float x, float y, float z, float e) {
    x = (float)fabs(x);
    y = (float)fabs(y);
    z = (float)fabs(z);
    e = (float)fabs(e);
    if (x >= y && x >= z && x >= e) {
        return 0;
    } else
    if (y > x && y > z && y > e) {
        return 1;
    } else
    if (z > x && z > y && z > e) {
        return 2;
    } else
    if (e > x && e > y && e > z) {
        return 3;
    }
    return 4;
}

void calc_dec (float f) {
  int i, n;
  XYZ *tmp;
//  float dec_percent = DEC_PERCENT;
  float dec_percent = f / DEC_F;

  for (i = 0; i < available(); i ++) {
    if (poke(&tmp, - i - 1) == 0) break;
		if (tmp->dec) break;

    tmp->dec = i == 0 ? 2 : 1;
    tmp->dec_xyz = max_xyz(tmp->diff[0], tmp->diff[1], tmp->diff[2], tmp->diff[3]);
    n = abs(tmp->diff_step[tmp->dec_xyz]);
    tmp->dec_percent = dec_percent - (n * DEC_STEP);
    if (tmp->dec_percent < 1.0) {
        tmp->dec_percent = 1.0;
        n = (dec_percent - tmp->dec_percent) / DEC_STEP;
    }
    dec_percent = tmp->dec_percent;
    if (tmp->dir[tmp->dec_xyz] == 1) {
        tmp->dec_start = tmp->step[tmp->dec_xyz] - n;
    } else {
        tmp->dec_start = tmp->step[tmp->dec_xyz] + n;
    }

    if (dec_percent <= 1.0) break;
  }
}

float calc_acc (XYZ *target, float acc_percent) {
  int n;

  target->acc = 1;
  target->acc_xyz = max_xyz(target->diff[0], target->diff[1], target->diff[2], target->diff[3]);
  n = abs(target->diff_step[target->acc_xyz]);
  target->acc_percent = acc_percent;
  acc_percent = acc_percent - (n * ACC_STEP);
  if (acc_percent <= 1.0) {
    target->acc = 2;
    return 0;
  }
  return acc_percent;
}

// calculate g-param to steps
void calc_xyz (XYZ *target) {
  int i;
	float f;

	if (!target) {
		late = target_zero;
		acc_percent = 0;
		return;
	}

  target->xyz[0] = target->x;
  target->xyz[1] = target->y;
  target->xyz[2] = target->z;
  target->xyz[3] = target->e;
//  target->xyz[3] = target->e * (1.0 + ((volume - 0.5) / 2.0));

  // limit
  f = target->f / 60; // mm/min -> mm/sec
  if (f > LIMIT_F) f = LIMIT_F;
  if (f == 0) f = ACC_F;
/*
  // get late
  if (poke(&tmp, -1) == 0) {
    tmp = &dummy;
  }
*/
  for (i = 0; i < 4; i ++) {
    // add offset
    target->xyz[i] = target->xyz[i] + target->offset[i];
    if (target->incremental) {
      target->offset[i] = target->xyz[i];
    }
    // mm -> step
    target->step[i] = (int)floor(target->xyz[i] / mm_per_step[i] + 0.5);
    // diff (mm , step)
    target->diff[i] = target->xyz[i] - late.xyz[i];
    target->diff_step[i] = target->step[i] - late.step[i];
  }

  // moving distance
  if (target->diff[0] || target->diff[1] || target->diff[2]) {
    target->v = (float)sqrt(target->diff[0] * target->diff[0] + target->diff[1] * target->diff[1] + target->diff[2] * target->diff[2]); // mm
  } else
  if (target->diff[3]) {
    target->v = (float)fabs(target->diff[3]);
  } else {
    return;
  }
  if (late.v && target->v) {
    // 1(0deg), 0.7(45deg), 0(90deg), -0.7(-135deg), -1(180deg)
    target->d = (late.diff[0] * target->diff[0] + late.diff[1] * target->diff[1] + late.diff[2] * target->diff[2]) / (late.v * target->v); // rad
//    target->d = acos(target->d);
  } else {
    target->d = -99;
  }
recalc:
  // moving time
  target->s = target->v / f; // sec

  // limit z (only z)
  if (target->diff[0] == 0 && target->diff[1] == 0 && target->diff[2] != 0 && 
    fabs(target->v / target->s) > LIMIT_F_Z) {
DBG("Z");
      f = LIMIT_F_Z;
      goto recalc;
  }
  // limit e (only e)
  if (target->diff[0] == 0 && target->diff[1] == 0 && target->diff[2] == 0 && target->diff[3] != 0 &&
    fabs(target->v / target->s) > LIMIT_F_E) {
DBG("E");
      f = LIMIT_F_E;
      goto recalc;
  }
  target->ff = f;
//  if(!isfinite(target->v) || !isfinite(target->s)) error(5);

  // spp and dir
  for (i = 0; i < 4; i ++) {
    if (target->diff_step[i]) {
			if (target->diff_step[i] < -1000000 || target->diff_step[i] > 1000000) {
				error(5);
			}
      target->spp[i] = (float)fabs(target->s / (float)target->diff_step[i]) / 2.0; // sec/step(pulse)
      target->dir[i] = target->diff_step[i] > 0 ? 1 : -1;
			if (target->spp[i] < 0.000001) {
ERROR("S [%d] %d/%d ", i, (int)(target->s * 1000), target->diff_step[i]);
				f = f - 1;
				if (f < 1) return;
				goto recalc;
			}
    } else {
			target->spp[i] = 0;
      target->dir[i] = 0; // no step
    }
  }
	if (!target->dir[0] && !target->dir[1] && !target->dir[2] && !target->dir[3]) return;

  // insert wait
  target->acc = 0;
  target->dec = 0;
#ifdef ENABLE_ACC
  if (target->d <= ACC_DEG ||
    (late.diff[3] != 0 && target->diff[3] == 0) || (late.diff[3] == 0 && target->diff[3] != 0)) {
    // < 90degrees or stop E
    if (late.ff > DEC_F) {
      calc_dec(late.ff); // begin dec and calc dec
        DBG("-");
    }
//    acc_percent = ACC_PERCENT;
    if (f > ACC_F) {
      acc_percent = f / ACC_F; // begin acc
        DBG("+");
    } else {
      acc_percent = 0;
    }
  }
  if (acc_percent) {
    acc_percent = calc_acc(target, acc_percent); // calc acc
  }
#endif
/*
DBG("\r\n(%d) sec=%0.3f vect=%0.3f deg=%0.3f f=%d\r\n", target->gcode_line, target->s, target->v, target->d, f);
DBG("step: %4d %4d %4d %4d\r\n", target->step[0], target->step[1], target->step[2], target->step[3]);
//DBG("dir : %4d %4d %4d %4d\r\n", target->dir[0], target->dir[1], target->dir[2], target->dir[3]);
//DBG("mm/s: %0.3f %0.3f %0.3f %0.3f\r\n", mm_per_step_x, mm_per_step_y, mm_per_step_z, mm_per_step_e);
//DBG("spp : %0.6f %0.6f %0.6f %0.6f\r\n", target->spp[0], target->spp[1], target->spp[2], target->spp[3]);
DBG("dec : %d [%d] %d %0.3f / acc : %d [%d] %0.3f\r\n", tmp->dec, tmp->dec_xyz, tmp->dec_start, tmp->dec_percent, target->acc, target->acc_xyz, target->acc_percent);
*/
	__disable_irq();
	late = *target;
  queue(target);
	__enable_irq();

  if (sm.work == 0 && isFull()) {
    DBG("*** begin\r\n");
    move_next();
  }
}

void calc_reset () {
    int i;

    flush();
		calc_xyz(NULL);
		sm = sm_zero;
		sm_target = target_zero;
		target = target_zero;
}

void calc_end () {
  XYZ *tmp;
#ifdef ENABLE_ACC
  if (poke(&tmp, -1)) {
    if (tmp->ff > DEC_F) {
      calc_dec(tmp->ff);
    }
  }
#endif
  if (sm.work == 0 && available() > 0) {
    DBG("*** begin\r\n");
    move_next();
  }
  while (sm.work) {
		poll();
	}

  calc_reset();
  flush();
}
/*
void copy_xyz (XYZ *dst, XYZ *src) {
	int i;
	for (i = 0; i < 4; i ++) {
		dst->xyz[i] = dst->xyz[i];
		dst->diff[i] = dst->diff[i];
		dst->spp[i] = dst->spp[i];
		dst->step[i] = dst->step[i];
		dst->diff_step[i] = dst->diff_step[i];
		dst->dir[i] = dst->dir[i];
		dst->offset[i] = dst->offset[i];
	}
	dst->x = dst->x;
	dst->y = dst->y;
	dst->z = dst->z;
	dst->e = dst->e;
	dst->f = dst->f;
	dst->v = dst->v;
	dst->d = dst->d;
	dst->s = dst->s;
	dst->ff = dst->ff;
	dst->acc = dst->acc;
	dst->acc_xyz = dst->acc_xyz;
	dst->dec = dst->dec;
	dst->dec_xyz = dst->dec_xyz;
	dst->dec_start = dst->dec_start;
	dst->acc_percent = dst->acc_percent;
	dst->dec_percent = dst->dec_percent;
	dst->gcode_line = dst->gcode_line;
	dst->incremental = dst->incremental;
	dst->temp_bed = dst->temp_bed;
	dst->temp_ext = dst->temp_ext;
}
*/

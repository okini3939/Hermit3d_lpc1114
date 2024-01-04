#include <LPC11xx.h>
#include <math.h>
#include "hermit.h"
#include "stepper.h"
#include "cbuffer.h"
#include "FatFs/ff.h"
#include <string.h>
#include <stdlib.h>

volatile int gcode_line, gcode_count, gcode_error;

extern StepperMotor sm;
extern XYZ target, sm_target;

int zatoi (char *buf) {
	int n = 0;
	char c;
	for (;;) {
		c = *buf;
		buf ++;
		if (c >= '0' && c <= '9') {
			n = (n * 10) + (c - '0');
    } else {
			break;
		}
	}
	return n;
}

char *parseNum (char *buf, int *num, int flg) {
    int n = 0, m = -1, s = 0;
    char c;

    for (;;) {
        c = *buf;
        if (c >= '0' && c <= '9') {
            if (m < 0) {
                n = (n * 10) + (c - '0');
            } else
            if (m < 3) {
                n = n + ((c - '0') * (int)pow(10.0, 2.0 - m));
                m ++;
            }
            buf ++;
        } else
        if (c == '.') {
            if (m >= 0) break;
            n = n * 1000;
            m = 0;
            buf ++;
        } else
        if (c == '-') {
            s = -1;
            buf ++;
        } else {
            if (flg && m < 0) {
                n = n * 1000;
            }
            if (s) n = - n;
            *num = n;
            return buf;
        }
    }
    return buf;
}

char *parseXYZ (char *buf, XYZ *target) {
    int n;
    char c;

    for (;;) {
        c = *buf;
        switch (c) {
        case 'X':
            buf = parseNum(buf + 1, &n, NUM_MODE);
            if (buf == NULL) break;
            target->x = (float)n / 1000.0;
            break;
        case 'Y':
            buf = parseNum(buf + 1, &n, NUM_MODE);
            if (buf == NULL) break;
            target->y = (float)n / 1000.0;
            break;
        case 'Z':
            buf = parseNum(buf + 1, &n, NUM_MODE);
            if (buf == NULL) break;
            target->z = (float)n / 1000.0;
            break;
        case 'E':
            buf = parseNum(buf + 1, &n, 1);
            if (buf == NULL) break;
            target->e = (float)n / 1000.0;
            break;
        case 'F':
            buf = parseNum(buf + 1, &n, 1);
            if (buf == NULL) break;
            target->f = n / 1000;
            break;
        case '\n':
            buf ++;
						/* FALLTHROUGH */
        case 0:
            return buf;
        default:
            buf ++;
            break;
        }
        if (buf == NULL) break;
    }
    return buf;
}

char *parseParam (char *buf, char key, int *val) {
    int n;
    char c;

    for (;;) {
        c = *buf;
        if (c == key) {
            buf = parseNum(buf + 1, &n, NUM_MODE);
            if (buf == NULL) break;
            *val = n / 1000;
				} else
				if (c == '\n') {
						buf ++;
            return buf;
				} else
				if (c == 0) {
            return buf;
				} else {
						buf ++;
        }
				if (buf == NULL) break;
		}
    return buf;
}

/**
 * @param buf G-code line
 * @param target target pos result
 * @param sm current pos
 * @param resume line number
 */
void parseGcode (char *buf, XYZ *target) {
    int i, n;

    gcode_line ++;
    target->gcode_line = gcode_line;
    switch (buf[0]) {
    case 'G':
        n = zatoi(&buf[1]);
        for (i = 1; i < 10; i ++) {
            if (buf[i] < '0' || buf[i] > '9') break;
        }
        if (i == 1 || i >= 10) break;
        gcode_count ++;
        DBG("[%d:G%02d] ", gcode_line, n);

        switch (n) {
        case 0: // Rapid positioning
        case 1: // Linear interpolation
            if (parseXYZ(&buf[i], target) == NULL) {
                DBG("error: param\r\n");
                break;
            }
            calc_xyz(target);
DBG("  x%d y%d z%d e%d f%d / v%d s%d d%d f%d t%d\r\n", (int)target->x, (int)target->y, (int)target->z, (int)(target->e * 1000), (int)target->f, (int)(target->v * 1000), (int)(target->s * 1000), (int)(target->d * 10), (int)target->ff, target->temp_ext);
if (target->v > 100 || target->s > 10 || target->temp_ext < 0 || target->temp_ext > 1000) {
	flush();
ERROR("[%d:G%02d] ", gcode_line, n);
ERROR("  x%d y%d z%d e%d f%d / v%d s%d d%d f%d t%d\r\n", (int)target->x, (int)target->y, (int)target->z, (int)(target->e * 1000), (int)target->f, (int)(target->v * 1000), (int)(target->s * 1000), (int)(target->d * 10), (int)target->ff, target->temp_ext);
  ERROR("  buf %d\r\n", available());
	ERROR("\a  step %d %d %d %d / diff %d %d %d %d\r\n", target->step[0], target->step[1], target->step[2], target->step[3], (int)(target->diff[0] * 1000), (int)(target->diff[1] * 1000), (int)(target->diff[2] * 1000), (int)(target->diff[3] * 1000));
	error(2);
	/* NOTREACHED */
}
            break;
        case 28: // Return to home position
            DBG("Return to home position\r\n");
            break;
        case 90: // Absolute
            target->incremental = 0;
            DBG("Absolute");
            break;
        case 91: // Incremental
            target->incremental = 1;
            DBG("Incremental");
            break;
        case 92: // Position register
            if (strchr(&buf[i], 'X')) { target->offset[0] = target->xyz[0]; target->x = 0; }
            if (strchr(&buf[i], 'Y')) { target->offset[1] = target->xyz[1]; target->y = 0; }
            if (strchr(&buf[i], 'Z')) { target->offset[2] = target->xyz[2]; target->z = 0; }
            if (strchr(&buf[i], 'E')) { target->offset[3] = target->xyz[3]; target->e = 0; }
            DBG("Position register\r\n");
            break;
        default:
            gcode_error |= ERROR_GCODE_UNKNOWN;
            DBG("%s\r\n", buf);
            break;
        }
        break;
    case 'M':
        n = zatoi(&buf[1]);
        for (i = 1; i < 10; i ++) {
            if (buf[i] < '0' || buf[i] > '9') break;
        }
        if (i == 1 || i >= 10) break;
        DBG("[%d:M%02d] ", gcode_line, n);

        switch (n) {
        case 106: // Fan on
					COOLANT(1);
					DBG("Fan on\r\n");
					break;
        case 107: // Fan off
					COOLANT(0);
					DBG("Fan off\r\n");
					break;
        case 104: // Set extruder target temp
					sm.temp_ext_flg = target->temp_ext;
					parseParam(&buf[i], 'S', (int*)&target->temp_ext);
				  sm_target.temp_ext = target->temp_ext;
					DBG("Extruder %d C\r\n", target->temp_ext);
					break;
        case 109: // Wait for extruder current temp, s: Waits only when heating, r: Waits when heating and cooling
					sm.temp_ext_flg = sm.temp_ext;
					parseParam(&buf[i], 'S', (int*)&target->temp_ext);
				  sm_target.temp_ext = target->temp_ext;
					DBG("Extruder %d C wait\r\n", target->temp_ext);
					while (sm.temp_ext_flg) {
						if (poll()) break;
						DBG(" ext %d -> %d\r\n", sm.temp_ext, target->temp_ext);
						LED_ON; wait_ms(500); LED_OFF; wait_ms(500);
					}
					break;
        case 140: // Set bed target temp
					sm.temp_bed_flg = target->temp_bed;
					parseParam(&buf[i], 'S', (int*)&target->temp_bed);
				  sm_target.temp_bed = target->temp_bed;
					DBG("Bed %d C\r\n", target->temp_bed);
					break;
        case 190: // Wait for bed temperature to reach target temp
					sm.temp_bed_flg = sm.temp_bed;
					parseParam(&buf[i], 'S', (int*)&target->temp_bed);
				  sm_target.temp_bed = target->temp_bed;
					DBG("Bed %d C wait\r\n", target->temp_bed);
					while (sm.temp_bed_flg) {
						if (poll()) break;
						DBG(" bed %d -> %d\r\n", sm.temp_bed, target->temp_bed);
						LED_ON; wait_ms(500); LED_OFF; wait_ms(500);
					}
          break;
        default:
            gcode_error |= ERROR_GCODE_UNKNOWN;
            DBG("%s\r\n", buf);
            break;
        }
        break;
    case 0:
    case ';':
        break;
    default:
        gcode_error |= ERROR_GCODE_UNKNOWN;
        break;
    }
}

/**
 * @param filename G-code filename
 * @param resume line number
 */
int gcode (char *filename) {
		int i;
    FIL fp;
    char buf[80];

		calc_reset();
    gcode_line = 0;
    gcode_count = 0;
    gcode_error = 0;

    strcpy(buf, "/gcode/");
    strcat(buf, filename);
//  	if (f_chdir("gcode") != FR_OK) return -1;
    if (f_open(&fp, buf, FA_READ) == FR_OK) {
        for (;;) {
//            if (f_gets(buf, sizeof(buf), &fp) != FR_OK) break;
//            if (zgets(buf, 80, &fp) == NULL) break;
            if (f_gets(buf, 80, &fp) == NULL) break;
            if (f_eof(&fp)) break;
            while (isFull()) {
							if (poll()) {
                DBG("*** break\r\n");
								flush();
								goto exit;
							}

							i = recv_uart(buf, 79);
							if (i > 0) {
								if (buf[0] == '!') {
									DBG("*** break\r\n");
									flush();
									goto exit;
								} else
								if (buf[0] == 'V') {
                  xprintf("@%d,%d\r\n", sm.work, sm.gcode_line);
								}
							}
            }
            parseGcode(buf, &target);
						poll();

            if (gcode_error) {
                DBG("<warning %04x>\r\n", gcode_error);
                gcode_error = 0;
            }
        }
exit:
        f_close(&fp);

				HEATER1(0);
				HEATER2(0);
				COOLANT(0);
        while (isFull());
//        parseGcode("G00X0Y0F1000", &target);
        parseGcode("G92Z0E0", &target);
        while (isFull());
        parseGcode("G01Z5.E-1.F300", &target);
        while (isFull());
        parseGcode("G01X0Y0F3000", &target);
        while (isFull());
        parseGcode("G01Z0E0F300", &target);
        calc_end();
/*
        while (isFull());
        wait(3);
        NVIC_SystemReset();
*/
    } else {
			return -1;
		}
    return 0;
}

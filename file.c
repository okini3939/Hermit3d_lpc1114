#include <LPC11xx.h>
#include "FatFs/ff.h"
#include "hermit.h"

static FATFS Fatfs;

int init_filesystem (void) {
	FRESULT r = f_mount(&Fatfs, "0:", 1);
  if (r == FR_OK) return 0;
	return -1;
}

void listfile (void) {
	FRESULT res;
	DIR dir;
  FILINFO fno;
	char *fn;
  static char lfn[50 + 1];
  fno.lfname = lfn;
  fno.lfsize = 50 + 1;

  if (f_opendir(&dir, "/gcode") != FR_OK) return;
  for (;;) {
		res = f_readdir(&dir, &fno);
    if (res != FR_OK || fno.fname[0] == 0) break;
		fn = *fno.lfname ? fno.lfname : fno.fname;
    xprintf("%s\r\n", fn);
  }
  f_closedir(&dir);
}

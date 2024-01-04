#include "cbuffer.h"

static volatile int write;
static volatile int read;
static int size;
static XYZ * buf;

    void init_cbuffer (int length, XYZ *addr) {
        write = 0;
        read = 0;
        size = length + 1;
//        buf = (XYZ *)malloc(size * sizeof(XYZ));
        buf = addr;
    };

    int isFull(void) {
        return (((write + 1) % size) == read);
    };

    int isEmpty(void) {
        return (read == write);
    };

    int queue(XYZ *k) {
        if (isFull()) {
            return 0;
        }
				__disable_irq();
        buf[write++] = *k;
        write %= size;
				__enable_irq();
        return 1;
    }
    
    void flush(void) {
        read = 0;
        write = 0;
    }
    
    int available(void) {
        return (write >= read) ? write - read : size - read + write;
    };

    int dequeue(XYZ * c) {
        int empty = isEmpty();
        if (!empty) {
						__disable_irq();
            *c = buf[read++];
						read %= size;
						__enable_irq();
        }
        return(!empty);
    };

    int poke(XYZ ** c, int n) {
//        if (n < 0 && -n >= available()) return 0;
        int empty = isEmpty();
				__disable_irq();
        if (n >= 0 || n < -available()) empty = 1;
        if (!empty) {
						int i = (write + size + n) % size;
            *c = &buf[i];
        }
				__enable_irq();
        return(!empty);
    };

#ifndef CIRCBUFFER_H_
#define CIRCBUFFER_H_

#include "stepper.h"

void init_cbuffer (int length, XYZ *addr);
int isFull(void);
int isEmpty(void);
int queue(XYZ *k);
void flush(void);
int available(void);
int dequeue(XYZ * c);
int poke(XYZ ** c, int n);

#endif

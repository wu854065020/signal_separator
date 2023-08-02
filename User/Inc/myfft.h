#ifndef MYFFT_H
#define MYFFT_H

#include <stdint.h>

void MyFFTInit(void);
void MyFFT(float *signvolt);
void GetFFTMag(float *input, float *output);

#endif

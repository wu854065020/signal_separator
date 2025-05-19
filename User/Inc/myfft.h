/*
 * @Author: wzccccccc
 * @Date: 2023-08-02 09:15:44
 * @LastEditors: wzccccccc wu854065020@gmail.com
 * @LastEditTime: 2024-04-01 14:40:14
 * @FilePath: \signal_separator\User\Inc\myfft.h
 */
#ifndef MYFFT_H
#define MYFFT_H

#include <stdint.h>

void MyFFTInit(void);
void MyFFT(float *signvolt);
void ALterFFT(float *signvolt);
void GetFFTMag(float *input, float *output);

#endif

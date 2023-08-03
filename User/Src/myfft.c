#include "myfft.h"
#include "arm_math.h"
#include "sample_config.h"


// arm_cfft_radix2_instance_f32 g_1024fft_h;
arm_cfft_radix2_instance_f32 g_myfft_h;
arm_cfft_radix2_instance_f32 g_alterfft_h;
// extern const arm_cfft_instance_f32 arm_cfft_sR_f32_len1024;

void MyFFTInit(void)
{
  // arm_cfft_radix2_init_f32(&g_1024fft_h, FFT_NUM, 0, 1);
  arm_cfft_radix2_init_f32(&g_myfft_h, FFT_NUM, 0, 1);
  arm_cfft_radix2_init_f32(&g_alterfft_h, PHASE_LOCKED_FFT_NUM, 0, 1);
}

void MyFFT(float *signvolt)
{
  // arm_cfft_radix2_f32(&g_1024fft_h, signvolt);
  arm_cfft_radix2_f32(&g_myfft_h, signvolt);
}

void ALterFFT(float *signvolt)
{
  arm_cfft_radix2_f32(&g_alterfft_h, signvolt);
}

void GetFFTMag(float *input, float *output)
{
  arm_cmplx_mag_f32(input, output, FFT_NUM);
  output[0] /= FFT_NUM;
  for (uint16_t i=1;i<FFT_NUM;i++)
  {
    output[i] /= (FFT_NUM/2);
  }
}

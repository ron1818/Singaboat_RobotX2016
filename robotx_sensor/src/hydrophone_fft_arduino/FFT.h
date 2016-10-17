#ifndef __FFT_H__  
#define __FFT_H__  
  
typedef struct complex //复数
{  
  float real;       //实数
  float imag;       //虚数
}complex;  
  
//#define PI 3.1415926535897932384626433832795028841971  
  
  
///////////////////////////////////////////  
void conjugate_complex(int n,complex in[],complex out[]);  
void c_plus(complex a,complex b,complex *c);
void c_mul(complex a,complex b,complex *c);
void c_sub(complex a,complex b,complex *c);
void c_div(complex a,complex b,complex *c);
void fft(int N,complex f[]);//傅里叶变换 输出结果就在f[]里面  
void ifft(int N,complex f[]); //傅里叶逆变换
void c_abs(complex f[],float out[],int n);//复数数组取模
////////////////////////////////////////////  
#endif  

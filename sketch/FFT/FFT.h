#ifndef __FFT_H__  
#define __FFT_H__  
  
typedef struct complex //澶嶆暟
{  
  float real;       //瀹炴暟
  float imag;       //铏氭暟
}complex;  
  
//#define PI 3.1415926535897932384626433832795028841971  
  
  
///////////////////////////////////////////  
void conjugate_complex(int n,complex in[],complex out[]);  
void c_plus(complex a,complex b,complex *c);
void c_mul(complex a,complex b,complex *c);
void c_sub(complex a,complex b,complex *c);
void c_div(complex a,complex b,complex *c);
void fft(int N,complex f[]);//鍌呴噷鍙跺彉鎹� 杈撳嚭缁撴灉灏卞湪f[]閲岄潰  
void ifft(int N,complex f[]); //鍌呴噷鍙堕�嗗彉鎹�
void c_abs(complex f[],float out[],int n);//澶嶆暟鏁扮粍鍙栨ā
////////////////////////////////////////////  
#endif  


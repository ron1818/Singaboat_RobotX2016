#include "math.h"  
#include "FFT.h"
#include "ros.h"  
#include <std_msgs/Float64.h>

//精度0.0001弧度  

ros::NodeHandle nh;
std_msgs::Float64 info; //for hydrophone
ros::Publisher hydrophone("hydrophone/raw", &info);
  
void conjugate_complex(int n,complex in[],complex out[])  
{  
  int i = 0;  
  for(i=0;i<n;i++)  
  {  
    out[i].imag = -in[i].imag;  
    out[i].real = in[i].real;  
  }   
}  
  
void c_abs(complex f[],float out[],int n)  
{  
  int i = 0;  
  float t;  
  for(i=0;i<n;i++)  
  {  
    t = f[i].real * f[i].real + f[i].imag * f[i].imag;  
    out[i] = sqrt(t);  
  }   
}  
  
  
void c_plus(complex a,complex b,complex *c)  
{  
  c->real = a.real + b.real;  
  c->imag = a.imag + b.imag;  
}  
  
void c_sub(complex a,complex b,complex *c)  
{  
  c->real = a.real - b.real;  
  c->imag = a.imag - b.imag;   
}  
  
void c_mul(complex a,complex b,complex *c)  
{  
  c->real = a.real * b.real - a.imag * b.imag;  
  c->imag = a.real * b.imag + a.imag * b.real;     
}  
  
void c_div(complex a,complex b,complex *c)  
{  
  c->real = (a.real * b.real + a.imag * b.imag)/(b.real * b.real +b.imag * b.imag);  
  c->imag = (a.imag * b.real - a.real * b.imag)/(b.real * b.real +b.imag * b.imag);  
}  
  
#define SWAP(a,b)  tempr=(a);(a)=(b);(b)=tempr  
  
void Wn_i(int n,int i,complex *Wn,char flag)  
{  
  Wn->real = cos(2*PI*i/n);  
  if(flag == 1)  
  Wn->imag = -sin(2*PI*i/n);  
  else if(flag == 0)  
  Wn->imag = -sin(2*PI*i/n);  
}  
  
//傅里叶变化  
void fft(int N,complex f[])  
{  
  complex t,wn;//中间变量  
  int i,j,k,m,n,l,r,M;  
  int la,lb,lc;  
  /*----计算分解的级数M=log2(N)----*/  
  for(i=N,M=1;(i=i/2)!=1;M++);   
  /*----按照倒位序重新排列原信号----*/  
  for(i=1,j=N/2;i<=N-2;i++)  
  {  
    if(i<j)  
    {  
      t=f[j];  
      f[j]=f[i];  
      f[i]=t;  
    }  
    k=N/2;  
    while(k<=j)  
    {  
      j=j-k;  
      k=k/2;  
    }  
    j=j+k;  
  }  
  
  /*----FFT算法----*/  
  for(m=1;m<=M;m++)  
  {  
    la=pow(2,m); //la=2^m代表第m级每个分组所含节点数       
    lb=la/2;    //lb代表第m级每个分组所含碟形单元数  
                 //同时它也表示每个碟形单元上下节点之间的距离  
    /*----碟形运算----*/  
    for(l=1;l<=lb;l++)  
    {  
      r=(l-1)*pow(2,M-m);     
      for(n=l-1;n<N-1;n=n+la) //遍历每个分组，分组总数为N/la  
      {  
        lc=n+lb;  //n,lc分别代表一个碟形单元的上、下节点编号       
        Wn_i(N,r,&wn,1);//wn=Wnr  
        c_mul(f[lc],wn,&t);//t = f[lc] * wn复数运算  
        c_sub(f[n],t,&(f[lc]));//f[lc] = f[n] - f[lc] * Wnr  
        c_plus(f[n],t,&(f[n]));//f[n] = f[n] + f[lc] * Wnr  
      }  
    }  
  }  
}  
  
//傅里叶逆变换  
void ifft(int N,complex f[])  
{  
  int i=0;  
  conjugate_complex(N,f,f);  
  fft(N,f);  
  conjugate_complex(N,f,f);  
  for(i=0;i<N;i++)  
  {  
    f[i].imag = (f[i].imag)/N;  
    f[i].real = (f[i].real)/N;  
  }  
}  

complex b[1024];
float out[1024];
float fft_max = 0;
float frequency, sample_freq;
float receiving = 0;
int fft_max_point, gate, channel;
int pinger = 0;
long AA,BB,timediff, amplitude;
unsigned long begt, runt;



void setup() {
  nh.initNode();
  nh.advertise(hydrophone);
  
  Serial.begin(115200);
  analogReadResolution(12);
  //Serial.println(String("Program started!"));
  Serial.flush();
  delay(100);
}
void loop( ) {
  AA = analogRead(A0); //channel A0 println 54
  BB = analogRead(A1); //channel A1 println 55

    //left hydrophone got signal first
    if(AA <2170)                           // voltage smaller than ~1.75 V consider signal detected
    {
       receiving = 1;                      // if left hydrophone got signal already, stop right hydrophone alogrithm
       begt = micros();
       for(int i=1; i< 512; i++)           // wait right hydrophone for signal and calculate angle
       {
          BB = analogRead(A1);
          if(BB <2170)
          {
            timediff = micros() - begt;
            //Serial.println(String("voltage = ") + BB);
            pinger = 1;                    // flag for start of fft analysis
            channel = A1;                  // use right hydrophone data for ADC
            i=512;                         // quit the loop
          }
        }
     }

     //right hydrophone got signal first
    if((BB <2170) && (receiving == 0))   //voltage greater than ~2.4V consider signal on
    {
       receiving = 1;
       begt = micros();
       for(int i=1; i< 512; i++)          // wait left hydrophone for signal and calculate angle
       {
          AA = analogRead(A0);
          if(AA < 2170)
          {
            timediff = micros() - begt;
           // Serial.println(String("voltage = ") + BB);
            pinger = 1;
            channel = A0;
            i=512;
          }
        }
     }

  // time difference for signal to arrive at two hydrophone to indicate direction
  if(timediff < 500)                          // if hydrophone apart by 2m, the timediff threshhold should be around 0.5ms(largest distance difference may be 1.5m which is around 1ms)
  {
      gate = 2;  //mid
  }
  else
  {
    if(channel == A0)
    {
      gate = 3;  //right
    }
    else
    {
      gate = 1;  //left
    }
  }
  
  if(pinger == 1)
  {
        begt = micros();
        for(int i=0; i< 1024; i++)
        {
          b[i].real = analogRead(channel);
          b[i].imag = 0.0;
        }
        runt = micros() - begt;
        sample_freq = 1000000.0/runt*1024;
        fft(1024,b);
        c_abs(b,out,1024);
        for(int i=125; i<225; i++)         //Search frequence range f1(<30k) to f2(>40k)
        {
          if(out[i]>fft_max)
          {
            fft_max = out[i];
            fft_max_point = i;
          }
        }
        frequency = sample_freq/1024*fft_max_point;

        //for result, fft frequency & pinger direction
        //Serial.println(String("Sample Frequency = ") + sample_freq);
        //Serial.println(String("Frequency = ") + frequency);
        //Serial.println(String("Gate = ") + gate);                // 1, left; 2, mid; 3, right
        //Serial.println(String("Time difference = ") + timediff);
        //Serial.println(String("FFT_MAX = ") + fft_max);
        AA = analogRead(A0); //channel A0 println 54
        BB = analogRead(A1); //channel A1 println 55

        if (AA>BB){
          amplitude=BB;
        }
        else{
          amplitude=AA;
        }
        
        Serial.println( amplitude+ String(",") + frequency);
        


        info.data=fft_max;
        hydrophone.publish(&info);
  } 

    nh.spinOnce();
    delay(100);
    
    // reset parameter for next loop
    pinger = 0;
    receiving = 0;
    fft_max = 0;
    fft_max_point = 0;
}

  /* for test of sampling ability
  long begt, runt, total;
  total = 0;  // clear before sampling
  begt = micros();
  for(int i=0; i< n; i++) {
     total += analogRead(pin);
  }
  runt = micros() - begt;  // elapsed time
  Serial.println(String("Average=") + total/n);
  Serial.print(String("Time per sample: ")+runt/1.0/n +"us");
  Serial.println(String(", Frequency: ")+1000000.0/runt*n +" Hz");
  Serial.println(String("***********************"));
  
  delay(5566);
  */

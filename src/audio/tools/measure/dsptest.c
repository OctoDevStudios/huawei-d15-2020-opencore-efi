#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "../../plugin/MateBookDSP.h"
static MBDSP d;
int main(void){
  enum{N=48000};
  static float in[2*N]; static int32_t out[2*N];
  /* 1) small-signal sine response (speaker profile, -30 dBFS) */
  double fs[]={60,100,150,200,300,500,820,1500,2900,6000,10000};
  for(int k=0;k<11;k++){ MBDSP_Init(&d);
    for(int i=0;i<N;i++){float v=0.0316f*sinf(2*M_PI*fs[k]*i/48000.0); in[2*i]=v; in[2*i+1]=v;}
    MBDSP_Process(&d,in,out,N,1.0f,0);
    double pk=0; for(int i=N/2;i<N;i++){double v=fabs(out[2*i]/2147483647.0); if(v>pk)pk=v;}
    printf("%6.0f Hz  gain %+5.1f dB\n",fs[k],20*log10(pk/0.0316)); }
  /* 2) noise + very loud bass sine: ceiling, NaN */
  for(int prof=0;prof<2;prof++){ MBDSP_Init(&d); double mx=0; int nan=0; srand(1);
    for(int rep=0;rep<10;rep++){
      for(int i=0;i<N;i++){float v=(rand()/(float)RAND_MAX*2-1)*0.9f+0.9f*sinf(2*M_PI*80*i/48000.0); in[2*i]=v; in[2*i+1]=-v;}
      MBDSP_Process(&d,in,out,N,1.0f,prof);
      for(int i=0;i<2*N;i++){double v=fabs(out[i]/2147483647.0); if(v>mx)mx=v; if(v!=v)nan=1;}
    }
    printf("profile %d: output peak %.4f (%.2f dBFS) nan=%d\n",prof,mx,20*log10(mx),nan);}
  /* 3) headphones: transparency at -20 dBFS (latency excluded) */
  MBDSP_Init(&d);
  for(int i=0;i<N;i++){float v=0.1f*sinf(2*M_PI*1000*i/48000.0); in[2*i]=v; in[2*i+1]=v;}
  MBDSP_Process(&d,in,out,N,1.0f,1);
  double e=0; for(int i=1000;i<N;i++){double v=out[2*i]/2147483647.0-in[2*(i-(MBDSP_LA-1))]; if(fabs(v)>e)e=fabs(v);}
  printf("headphones: max error vs delayed input %.2e\n",e);
  /* perf */
  MBDSP_Init(&d); clock_t t=clock(); for(int r=0;r<100;r++) MBDSP_Process(&d,in,out,N,1.0f,0);
  printf("perf: %.2f %% of one core\n",100.0*(clock()-t)/CLOCKS_PER_SEC/100.0);
}

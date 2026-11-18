/* Four-stage gait with IMU attitude PD compensation; Linux/RK3566. */
#define _DEFAULT_SOURCE
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#define N 14
extern int open_icm(void);
extern int sample_icm(int16_t *data);
typedef struct{char name[48];int id,home,pos;} J;static J j[N];
static int load(void){FILE*f=fopen("jduck_tools/calibration.yaml","r");if(!f)return -1;char b[256];int n=0,c=-1,v;while(fgets(b,sizeof b,f)){if(b[0]==' '&&b[1]==' '&&sscanf(b,"  %47[^:]:",j[n].name)==1){if(n>=N)break;c=n++;}else if(c>=0&&sscanf(b,"    id: %d",&v)==1)j[c].id=v;else if(c>=0&&sscanf(b,"    adc_zero: %d",&v)==1)j[c].home=v;}fclose(f);return n==N?0:-1;}
static int ji(const char*n){for(int i=0;i<N;i++)if(!strcmp(j[i].name,n))return i;return -1;}
static int send(int fd,int id,int cmd,int p,int*out){unsigned char b[8]={250,175,id,cmd,p>>8,p&255,0,237};b[6]=(id+cmd+b[4]+b[5])&255;if(write(fd,b,8)!=8)return -1;unsigned char r[9];int n=0;struct timespec s,t;clock_gettime(CLOCK_MONOTONIC,&s);while(n<9){unsigned char x;if(read(fd,&x,1)==1){if(!n&&x!=250)continue;r[n++]=x;}else{clock_gettime(CLOCK_MONOTONIC,&t);if(t.tv_sec-s.tv_sec>0||t.tv_nsec-s.tv_nsec>100000000)return -1;usleep(500);}}if(r[1]!=175||r[2]!=id||r[8]!=237||r[7]!=((r[2]+r[3]+r[4]+r[5]+r[6])&255))return -1;*out=(r[3]<<8)|r[4];return 0;}
static double clamp(double x,double a,double b){return x<a?a:x>b?b:x;}
int main(int ac,char**av){if(load()){fprintf(stderr,"校准文件读取失败\n");return 2;}double kp=ac>1?atof(av[1]):0.25,kd=ac>2?atof(av[2]):0.025,limit=ac>3?atof(av[3]):5.0,period=ac>4?atof(av[4]):2.0;if(period<0.4||period>30)return 2;
 int fd=open("/dev/ttyS3",O_RDWR|O_NOCTTY|O_SYNC);if(fd<0){perror("ttyS3");return 1;}struct termios tt;tcgetattr(fd,&tt);cfmakeraw(&tt);cfsetispeed(&tt,B230400);cfsetospeed(&tt,B230400);tt.c_cflag|=CLOCAL|CREAD;tcsetattr(fd,TCSANOW,&tt);tcflush(fd,TCIOFLUSH);
 if(open_icm()<0){fprintf(stderr,"ICM42688P init failed\n");close(fd);return 1;}for(int i=0;i<N;i++)if(send(fd,j[i].id,3,0,&j[i].pos)){fprintf(stderr,"servo %d read failed\n",j[i].id);return 1;}
 int phase=0;FILE*f=fopen(".gait_phase","r");if(f){fscanf(f,"%d",&phase);fclose(f);}phase=(phase+1)%4;
 double pitch=0,roll=0,prevp=0,prevr=0;int16_t raw[6];const double dt=.02,deg=57.295779513;int iphl=ji("left_hip_pitch"),iphr=ji("right_hip_pitch"),ial=ji("left_ankle"),iar=ji("right_ankle"),irl=ji("left_hip_roll"),irr=ji("right_hip_roll");
 printf("IMU-PD gait stage %d (ctrl-C to stop) kp %.3f kd %.3f limit %.1f deg\n",phase+1,kp,kd,limit);for(;;){if(sample_icm(raw)<0){fprintf(stderr,"IMU sample failed\n");break;}
  /* configured board axes are inverted X/Y; accel complementary attitude, gyro dps */
  double ax=-raw[0]/4096.0,ay=-raw[1]/4096.0,az=raw[2]/4096.0;
  double ap=atan2(ax,sqrt(ay*ay+az*az))*deg,ar=atan2(ay,sqrt(ax*ax+az*az))*deg;
  double gp=raw[4]/32.8,gr=-raw[3]/32.8; pitch=.98*(pitch+gp*dt)+.02*ap;roll=.98*(roll+gr*dt)+.02*ar;
  double cp=clamp(-kp*pitch-kd*gp,-limit,limit),cr=clamp(-kp*roll-kd*gr,-limit,limit);
  int tgt[N];for(int i=0;i<N;i++)tgt[i]=j[i].home;
  /* Stage-specific swing preset, overlaid by bounded trunk stabilization. */
  int swing=phase==1?1:phase==3?-1:0;if(swing){char nm[48];int k;snprintf(nm,sizeof nm,"%s_hip_pitch",swing>0?"right":"left");if((k=ji(nm))>=0)tgt[k]+=swing*12;snprintf(nm,sizeof nm,"%s_knee",swing>0?"right":"left");if((k=ji(nm))>=0)tgt[k]+=swing*30;snprintf(nm,sizeof nm,"%s_ankle",swing>0?"right":"left");if((k=ji(nm))>=0)tgt[k]-=swing*15;}
  int dph=(int)lround(cp*deg/0.00613592315),dr=(int)lround(cr*deg/0.00613592315);if(iphl>=0)tgt[iphl]+=dph;if(iphr>=0)tgt[iphr]+=dph;if(ial>=0)tgt[ial]+=dph;if(iar>=0)tgt[iar]+=dph;if(irl>=0)tgt[irl]+=dr;if(irr>=0)tgt[irr]+=dr;
  for(int i=0;i<N;i++){tgt[i]=(int)clamp(tgt[i],50,950);int reply;if(send(fd,j[i].id,1,tgt[i],&reply)){fprintf(stderr,"servo write failed\n");close(fd);return 1;}}
  usleep((useconds_t)(dt*1e6));
 }
 close(fd);return 0;}

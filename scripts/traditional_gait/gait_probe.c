/* Manual, one-command-at-a-time stand/weight-shift/lift-foot probe. */
#define _DEFAULT_SOURCE
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#define N 14
typedef struct{char name[48];int id,home,now;}J;static J j[N];
static int load(void){FILE*f=fopen("jduck_tools/calibration.yaml","r");if(!f)return -1;char b[256];int n=0,c=-1,v;while(fgets(b,sizeof b,f)){if(b[0]==' '&&b[1]==' '&&n<N&&sscanf(b,"  %47[^:]:",j[n].name)==1)c=n++;else if(c>=0&&sscanf(b,"    id: %d",&v)==1)j[c].id=v;else if(c>=0&&sscanf(b,"    adc_zero: %d",&v)==1)j[c].home=v;}fclose(f);return n==N?0:-1;}
static int ix(const char*s){for(int i=0;i<N;i++)if(!strcmp(j[i].name,s))return i;return -1;}
static int tx(int fd,int id,int cmd,int p,int*out){unsigned char b[8]={250,175,id,cmd,p>>8,p&255,0,237};b[6]=(id+cmd+b[4]+b[5])&255;if(write(fd,b,8)!=8)return -1;unsigned char r[9];int n=0;struct timespec a,z;clock_gettime(CLOCK_MONOTONIC,&a);while(n<9){unsigned char x;if(read(fd,&x,1)==1){if(!n&&x!=250)continue;r[n++]=x;}else{clock_gettime(CLOCK_MONOTONIC,&z);if(z.tv_sec-a.tv_sec>0||z.tv_nsec-a.tv_nsec>100000000)return -1;usleep(500);}}if(r[1]!=175||r[2]!=id||r[8]!=237||r[7]!=((r[2]+r[3]+r[4]+r[5]+r[6])&255))return -1;*out=(r[3]<<8)|r[4];return 0;}
static void add(int*t,const char*n,double deg){int k=ix(n);if(k>=0)t[k]+=(int)lround(deg*0.01745329252/0.00613592315);}
int main(int ac,char**av){if(ac<2||!strcmp(av[1],"help")){puts("用法: gait_probe stand|shift_left|shift_right|lift_left|lift_right|home [秒 [偏移度]]\n依次建议 stand → shift_left → lift_right → home，再反向测试。默认转换2秒，偏移5度。参数只用于受控调试，不代表安全步幅。");return 0;}if(load()){fprintf(stderr,"读取 calibration.yaml 失败\n");return 2;}double sec=ac>2?atof(av[2]):2,mag=ac>3?atof(av[3]):5;if(sec<0.5||sec>30||mag<0||mag>15){fprintf(stderr,"时长须0.5..30秒，偏移须0..15度\n");return 2;}
 int target[N];for(int i=0;i<N;i++)target[i]=j[i].home;const char*c=av[1];
 if(!strcmp(c,"shift_left")){add(target,"left_hip_roll",-mag);add(target,"right_hip_roll",-mag);}else if(!strcmp(c,"shift_right")){add(target,"left_hip_roll",mag);add(target,"right_hip_roll",mag);}
 else if(!strcmp(c,"lift_left")){add(target,"left_knee",mag);add(target,"left_hip_pitch",mag/2);add(target,"left_ankle",-mag/2);}else if(!strcmp(c,"lift_right")){add(target,"right_knee",mag);add(target,"right_hip_pitch",-mag/2);add(target,"right_ankle",mag/2);}else if(strcmp(c,"stand")&&strcmp(c,"home")){fprintf(stderr,"未知动作 %s\n",c);return 2;}
 for(int i=0;i<N;i++)if(target[i]<50||target[i]>950){fprintf(stderr,"目标越界: %s=%d\n",j[i].name,target[i]);return 2;}
 int fd=open("/dev/ttyS3",O_RDWR|O_NOCTTY|O_SYNC);if(fd<0){perror("/dev/ttyS3");return 1;}struct termios t;if(tcgetattr(fd,&t)<0)return 1;cfmakeraw(&t);cfsetispeed(&t,B230400);cfsetospeed(&t,B230400);t.c_cflag|=CLOCAL|CREAD;tcsetattr(fd,TCSANOW,&t);tcflush(fd,TCIOFLUSH);
 for(int i=0;i<N;i++)if(tx(fd,j[i].id,3,0,&j[i].now)){fprintf(stderr,"读取舵机%d失败\n",j[i].id);close(fd);return 1;}printf("动作 %s，时长 %.1fs，偏移 %.1f°\n",c,sec,mag);
 struct timespec a,z;clock_gettime(CLOCK_MONOTONIC,&a);for(;;){clock_gettime(CLOCK_MONOTONIC,&z);double u=(z.tv_sec-a.tv_sec+(z.tv_nsec-a.tv_nsec)/1e9)/sec;if(u>1)u=1;double e=u*u*(3-2*u);for(int i=0;i<N;i++){int reply,p=(int)lround(j[i].now+(target[i]-j[i].now)*e);if(tx(fd,j[i].id,1,p,&reply)){fprintf(stderr,"舵机%d通信失败\n",j[i].id);close(fd);return 1;}}if(u>=1)break;usleep(20000);}close(fd);puts("完成");return 0;}

/* Interactive four-phase pose sequencer. See gait_step.md for use/config. */
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
typedef struct { const char *name; int id, home, current; } Joint;
static Joint j[N];
static int load_cal(void) {
 FILE *f=fopen("jduck_tools/calibration.yaml","r"); if(!f){perror("jduck_tools/calibration.yaml");return -1;}
 char line[256],name[64]; int n=0,cur=-1,v;
 while(fgets(line,sizeof line,f)) {
  if(line[0]==' '&&line[1]==' '&&sscanf(line,"  %63[^:]:",name)==1) { if(n>=N)break; j[n].name=strdup(name);cur=n++; }
  else if(cur>=0&&sscanf(line,"    id: %d",&v)==1)j[cur].id=v;
  else if(cur>=0&&sscanf(line,"    adc_zero: %d",&v)==1)j[cur].home=v;
 }
 fclose(f); return n==N?0:-1;
}
static int txn(int fd,int id,int cmd,int pos,int *actual){
 unsigned char b[8]={0xfa,0xaf,id,cmd,(pos>>8)&255,pos&255,0,0xed};b[6]=(id+cmd+b[4]+b[5])&255;
 if(write(fd,b,8)!=8)return -1;unsigned char r[9];int n=0;struct timespec st,now;clock_gettime(CLOCK_MONOTONIC,&st);
 while(n<9){unsigned char c;if(read(fd,&c,1)==1){if(!n&&c!=0xfa)continue;r[n++]=c;}else{clock_gettime(CLOCK_MONOTONIC,&now);if(now.tv_sec-st.tv_sec>0||now.tv_nsec-st.tv_nsec>100000000)return -1;usleep(1000);}}
 if(r[1]!=0xaf||r[2]!=id||r[8]!=0xed||r[7]!=((r[2]+r[3]+r[4]+r[5]+r[6])&255))return -1;*actual=(r[3]<<8)|r[4];return 0;
}
/* Phase order: double support, left support/right swing, double support,
   right support/left swing. hip_roll shifts weight; hip_pitch/knee/ankle
   form a small configurable swing pose around calibrated HOME. */
static const char *phase_name[4]={"双脚支撑","左支撑右摆","双脚支撑","右支撑左摆"};
static int idx(const char *s){for(int k=0;k<N;k++)if(!strcmp(j[k].name,s))return k;return -1;}
int main(int argc,char **argv){
 if(load_cal()){fprintf(stderr,"校准文件必须包含14个关节\n");return 2;}
 const char *cmd=argc>1?argv[1]:"help";double sec=argc>2?atof(argv[2]):1.5;
 double shift=argc>3?atof(argv[3]):12, hip=argc>4?atof(argv[4]):8,knee=argc>5?atof(argv[5]):18,ankle=argc>6?atof(argv[6]):-10;
 if(!strcmp(cmd,"help")){printf("用法: gait_step next [秒 [重心偏移deg [摆腿髋deg [膝deg [踝deg]]]]]\n       gait_step reset [秒]\n连续调用 next 依次进入四阶段；reset 回 calibration.yaml 初始位置。\n默认小幅关节偏移仅用于调试，真机使用前请吊起机器人并校验方向。\n");return 0;}
 if(sec<0.2||sec>30){fprintf(stderr,"时长需在0.2..30秒\n");return 2;}
 int fd=open("/dev/ttyS3",O_RDWR|O_NOCTTY|O_SYNC);if(fd<0){perror("/dev/ttyS3");return 1;}struct termios t;tcgetattr(fd,&t);cfmakeraw(&t);cfsetispeed(&t,B230400);cfsetospeed(&t,B230400);t.c_cflag|=CLOCAL|CREAD;tcsetattr(fd,TCSANOW,&t);tcflush(fd,TCIOFLUSH);
 for(int k=0;k<N;k++)if(txn(fd,j[k].id,3,0,&j[k].current)){fprintf(stderr,"读取舵机%d失败\n",j[k].id);close(fd);return 1;}
 int phase=0;if(!strcmp(cmd,"next")){FILE*f=fopen(".gait_phase","r");if(f){fscanf(f,"%d",&phase);fclose(f);}phase=(phase+1)%4;}else if(strcmp(cmd,"reset")){fprintf(stderr,"未知命令: %s\n",cmd);close(fd);return 2;}
 int target[N];for(int k=0;k<N;k++)target[k]=j[k].home;
 if(!strcmp(cmd,"next")){
  int side=(phase==1)?1:(phase==3)?-1:0;char *names[]={(char*)"left_hip_roll",(char*)"right_hip_roll"};
  for(int a=0;a<2;a++){int q=idx(names[a]);if(q>=0)target[q]+=(int)lround((a==0?1:-1)*shift*side*0.01745329252/0.00613592315);}
  if(side){const char *leg=side>0?"right":"left";char nm[64];int q;
   snprintf(nm,sizeof nm,"%s_hip_pitch",leg);if((q=idx(nm))>=0)target[q]+=(int)lround(hip*0.01745329252/0.00613592315);
   snprintf(nm,sizeof nm,"%s_knee",leg);if((q=idx(nm))>=0)target[q]+=(int)lround(knee*0.01745329252/0.00613592315);
   snprintf(nm,sizeof nm,"%s_ankle",leg);if((q=idx(nm))>=0)target[q]+=(int)lround(ankle*0.01745329252/0.00613592315);
  }
 }
 for(int k=0;k<N;k++){if(target[k]<50||target[k]>950){fprintf(stderr,"目标越界: %s=%d\n",j[k].name,target[k]);close(fd);return 2;}}
 printf("阶段: %s\n",!strcmp(cmd,"reset")?"初始姿态":phase_name[phase]);struct timespec st,now;clock_gettime(CLOCK_MONOTONIC,&st);
 for(;;){clock_gettime(CLOCK_MONOTONIC,&now);double u=(now.tv_sec-st.tv_sec+(now.tv_nsec-st.tv_nsec)/1e9)/sec;if(u>1)u=1;double e=u*u*(3-2*u);
  for(int k=0;k<N;k++){int a;if(txn(fd,j[k].id,1,(int)lround(j[k].current+(target[k]-j[k].current)*e),&a)){fprintf(stderr,"写入舵机%d失败\n",j[k].id);close(fd);return 1;}}
  if(u>=1)break;usleep(20000);
 }
 if(!strcmp(cmd,"next")){FILE*f=fopen(".gait_phase","w");if(f){fprintf(f,"%d\n",phase);fclose(f);}}
 close(fd);return 0;
}

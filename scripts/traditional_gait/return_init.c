/* Smoothly return the calibrated servos to adc_zero from calibration.yaml.
 * Linux example: cc -O2 -Wall return_init.c -o return_init
 * Run from repository root: ./return_init [serial-device] [duration-sec]
 */
#define _DEFAULT_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define N 15
typedef struct { int id, target, start; char name[48]; } Servo;
static Servo s[N];
static int load_yaml(const char *path) {
    FILE *f=fopen(path,"r"); if(!f){perror(path);return -1;}
    char line[256]; int count=0, cur=-1;
    while(fgets(line,sizeof line,f)) {
        char name[48]; int v;
        if(line[0]!=' ' && sscanf(line," %47[^:]:",name)==1 && strcmp(name,"joints")) {
            if(count<N){cur=count++; strncpy(s[cur].name,name,sizeof s[cur].name-1);}
        } else if(cur>=0 && sscanf(line," id: %d",&v)==1) s[cur].id=v;
        else if(cur>=0 && sscanf(line," adc_zero: %d",&v)==1) s[cur].target=v;
    }
    fclose(f); if(count!=14){fprintf(stderr,"Expected 14 calibrated joints, found %d\n",count);return -1;}
    /* Mouth servo is outside the 14-joint calibration; its init ADC is from joint_limit.txt. */
    strcpy(s[14].name,"mouth"); s[14].id=4; s[14].target=443;
    return N;
}
static int transact(int fd,int id,int cmd,int pos,int *reply) {
    unsigned char b[8]={0xfa,0xaf,id,cmd,(pos>>8)&255,pos&255,0,0xed}; b[6]=(id+cmd+b[4]+b[5])&255;
    if(write(fd,b,8)!=8)return -1;
    unsigned char r[9]; int n=0; struct timespec end; clock_gettime(CLOCK_MONOTONIC,&end); end.tv_nsec+=100000000; if(end.tv_nsec>=1000000000){end.tv_sec++;end.tv_nsec-=1000000000;}
    while(n<9){unsigned char c; if(read(fd,&c,1)==1){if(!n&&c!=0xfa)continue;r[n++]=c;} else {struct timespec now;clock_gettime(CLOCK_MONOTONIC,&now);if(now.tv_sec>end.tv_sec||(now.tv_sec==end.tv_sec&&now.tv_nsec>=end.tv_nsec))return -1; usleep(1000);}}
    int sum=(r[2]+r[3]+r[4]+r[5]+r[6])&255; if(r[0]!=0xfa||r[1]!=0xaf||r[8]!=0xed||r[7]!=sum||r[2]!=id)return -1;
    *reply=(r[3]<<8)|r[4]; return 0;
}
int main(int argc,char **argv) {
    const char *port="/dev/ttyS3"; double duration=argc>1?atof(argv[1]):3.0;
    if(duration<0.5||duration>60){fprintf(stderr,"duration must be 0.5..60 seconds\n");return 2;}
    if(load_yaml("jduck_tools/calibration.yaml")<0)return 2;
    int fd=open(port,O_RDWR|O_NOCTTY|O_SYNC); if(fd<0){perror(port);return 1;}
    struct termios t; if(tcgetattr(fd,&t)<0){perror("tcgetattr");return 1;}
    cfmakeraw(&t); cfsetispeed(&t,B230400); cfsetospeed(&t,B230400); t.c_cflag|=CLOCAL|CREAD; t.c_cflag&=~CSTOPB; t.c_cflag&=~CRTSCTS;
    if(tcsetattr(fd,TCSANOW,&t)<0){perror("tcsetattr");return 1;} tcflush(fd,TCIOFLUSH);
    for(int i=0;i<N;i++){ if(!s[i].id||!s[i].target){fprintf(stderr,"Bad calibration entry %s\n",s[i].name);return 2;}
        if(transact(fd,s[i].id,3,0,&s[i].start)<0){fprintf(stderr,"No reply from servo %d (%s)\n",s[i].id,s[i].name);return 1;}
        printf("%s id=%d current=%d target=%d\n",s[i].name,s[i].id,s[i].start,s[i].target);
    }
    fprintf(stderr,"Returning to init in %.1f seconds... Ctrl-C to stop.\n",duration);
    struct timespec begin,now; clock_gettime(CLOCK_MONOTONIC,&begin);
    for(;;){clock_gettime(CLOCK_MONOTONIC,&now); double elapsed=now.tv_sec-begin.tv_sec+(now.tv_nsec-begin.tv_nsec)/1e9; double u=elapsed/duration;if(u>1)u=1;
        double ease=u*u*(3-2*u);
        for(int i=0;i<N;i++){int p=(int)lround(s[i].start+(s[i].target-s[i].start)*ease),actual;
            if(transact(fd,s[i].id,1,p,&actual)<0){fprintf(stderr,"Move/read failed at servo %d\n",s[i].id);close(fd);return 1;}}
        if(u>=1)break; usleep(20000);
    }
    printf("Init pose reached.\n"); close(fd); return 0;
}

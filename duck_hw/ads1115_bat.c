#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stdlib.h>


#define I2C_DEV "/dev/i2c-2"
#define ADS1115_ADDR 0x48


// ADS1115参考电压
#define ADS1115_FSR 4.096f


// 电阻分压
// 22k 上拉，10k 下拉
// Vbat = Ain * (22+10)/10
#define DIV_RATIO 3.2f


// 电池电量范围
#define BAT_FULL 8.4f
#define BAT_EMPTY 3.5f



// 写ADS1115配置寄存器
int ads1115_start_conversion(int fd)
{
    uint8_t buf[3];

    /*
        Config register = 0xC383

        OS   = 1       start single conversion
        MUX  = 100     AIN0-GND
        PGA  = 001     ±4.096V
        MODE = 1       single shot
        DR   = 100     128SPS
    */

    buf[0] = 0x01;      // config register
    buf[1] = 0xC3;
    buf[2] = 0x83;


    if(write(fd, buf, 3) != 3)
        return -1;


    return 0;
}



//读取ADS1115转换寄存器
int ads1115_read_adc(int fd)
{
    uint8_t reg = 0x00;
    uint8_t data[2];


    // 指向conversion register
    if(write(fd, &reg, 1) != 1)
        return -1;


    //读取两个字节
    if(read(fd, data, 2) != 2)
        return -1;


    int16_t adc;


    // ADS1115数据高字节在前
    adc = ((int16_t)data[0] << 8) | data[1];


    return adc;
}



int main()
{
    int fd;


    fd = open(I2C_DEV, O_RDWR);

    if(fd < 0)
    {
        perror("open i2c");
        return -1;
    }



    if(ioctl(fd, I2C_SLAVE, ADS1115_ADDR) < 0)
    {
        perror("i2c slave");
        return -1;
    }



    while(1)
    {

        //启动一次转换
        if(ads1115_start_conversion(fd) < 0)
        {
            printf("ADS1115 write error\n");
            continue;
        }



        // ADS1115 128SPS，等待10ms足够
        usleep(10000);



        int adc = ads1115_read_adc(fd);


        if(adc < 0)
        {
            printf("ADS1115 read error\n");
            continue;
        }



        // ADC -> AIN0电压
        float ain_voltage;

        ain_voltage =
            adc * ADS1115_FSR / 32768.0f;



        // 分压还原电池电压
        float battery_voltage;

        battery_voltage =
            ain_voltage * DIV_RATIO;



        // 电量百分比
        float battery_percent;


        battery_percent =
            (battery_voltage - BAT_EMPTY)
            /
            (BAT_FULL - BAT_EMPTY)
            * 100.0f;



        //限制范围
        if(battery_percent > 100.0f)
            battery_percent = 100.0f;


        if(battery_percent < 0.0f)
            battery_percent = 0.0f;



        printf(
            "ADC=%d  AIN0=%.3fV  Battery=%.3fV  %.1f%%\n",
            adc,
            ain_voltage,
            battery_voltage,
            battery_percent
        );


        sleep(1);
    }



    close(fd);


    return 0;
}

#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define ADS1115_ADDR 0x48
#define REG_CONVERSION 0x00
#define REG_CONFIG 0x01

static int write_register(int fd, uint8_t reg, uint16_t value)
{
    uint8_t data[] = {reg, (uint8_t)(value >> 8), (uint8_t)value};
    struct i2c_msg msg = {.addr = ADS1115_ADDR, .len = sizeof(data), .buf = data};
    struct i2c_rdwr_ioctl_data transfer = {.msgs = &msg, .nmsgs = 1};
    return ioctl(fd, I2C_RDWR, &transfer) == 1 ? 0 : -1;
}

static int read_register(int fd, uint8_t reg, uint16_t *value)
{
    uint8_t data[2];
    struct i2c_msg msgs[] = {
        {.addr = ADS1115_ADDR, .len = 1, .buf = &reg},
        {.addr = ADS1115_ADDR, .flags = I2C_M_RD, .len = sizeof(data), .buf = data},
    };
    struct i2c_rdwr_ioctl_data transfer = {.msgs = msgs, .nmsgs = 2};
    if (ioctl(fd, I2C_RDWR, &transfer) != 2)
        return -1;
    *value = (uint16_t)((data[0] << 8) | data[1]);
    return 0;
}

int main(int argc, char **argv)
{
    const char *device = argc > 1 ? argv[1] : "/dev/i2c-2";
    int fd = open(device, O_RDWR);
    if (fd < 0) {
        perror(device);
        return 1;
    }

    uint16_t original_config;
    if (read_register(fd, REG_CONFIG, &original_config) < 0) {
        perror("read ADS1115 configuration");
        close(fd);
        return 1;
    }

    int status = 0;
    for (int channel = 0; channel < 4; ++channel) {
        // Single-ended AINx, +/-4.096 V range, single shot, 128 samples/s.
        uint16_t config = (uint16_t)(0x8383 | ((0x04 + channel) << 12));
        if (write_register(fd, REG_CONFIG, config) < 0) {
            perror("start ADS1115 conversion");
            status = 1;
            break;
        }

        uint16_t ready = 0;
        int attempts;
        for (attempts = 0; attempts < 50; ++attempts) {
            usleep(2000);
            if (read_register(fd, REG_CONFIG, &ready) < 0) {
                perror("poll ADS1115 conversion");
                status = 1;
                break;
            }
            if (ready & 0x8000)
                break;
        }
        if (status)
            break;
        if (attempts == 50) {
            fprintf(stderr, "AIN%d conversion timed out\n", channel);
            status = 1;
            break;
        }

        uint16_t raw;
        if (read_register(fd, REG_CONVERSION, &raw) < 0) {
            perror("read ADS1115 conversion");
            status = 1;
            break;
        }
        printf("AIN%d: %d (0x%04X)\n", channel, (int16_t)raw, raw);
    }

    if (write_register(fd, REG_CONFIG, original_config) < 0) {
        perror("restore ADS1115 configuration");
        status = 1;
    }
    close(fd);
    return status;
}

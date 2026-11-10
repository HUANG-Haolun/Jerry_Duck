#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <signal.h>
#include <math.h>

/* =========================
 * SPI configuration
 * ========================= */

#define SPI_DEV         "/dev/spidev3.0"
#define SPI_SPEED_HZ    1000000
#define SPI_MODE        SPI_MODE_0
#define SPI_BITS        8

static int spi_fd = -1;
static volatile int running = 1;


/* =========================
 * ICM42688P Registers
 * ========================= */

#define REG_DEVICE_CONFIG        0x11

#define REG_INT_CONFIG           0x14

#define REG_ACCEL_DATA_X1        0x1F
#define REG_ACCEL_DATA_X0        0x20
#define REG_ACCEL_DATA_Y1        0x21
#define REG_ACCEL_DATA_Y0        0x22
#define REG_ACCEL_DATA_Z1        0x23
#define REG_ACCEL_DATA_Z0        0x24

#define REG_GYRO_DATA_X1         0x25
#define REG_GYRO_DATA_X0         0x26
#define REG_GYRO_DATA_Y1         0x27
#define REG_GYRO_DATA_Y0         0x28
#define REG_GYRO_DATA_Z1         0x29
#define REG_GYRO_DATA_Z0         0x2A

#define REG_TEMP_DATA1           0x1D
#define REG_TEMP_DATA0           0x1E

#define REG_INT_STATUS           0x2D

#define REG_PWR_MGMT0            0x4E
#define REG_GYRO_CONFIG0         0x4F
#define REG_ACCEL_CONFIG0        0x50

#define REG_WHO_AM_I             0x75

#define REG_BANK_SEL             0x76


/* =========================
 * Expected WHO_AM_I
 * ========================= */

#define ICM42688_WHO_AM_I        0x47


/* =========================
 * Scale
 *
 * ACCEL ±4g:
 * 8192 LSB/g
 *
 * GYRO ±500 dps:
 * 65.5 LSB/dps
 * ========================= */

#define ACCEL_LSB_PER_G          8192.0f
#define GYRO_LSB_PER_DPS         65.5f


/* =========================
 * Signal handler
 * ========================= */

static void signal_handler(int sig)
{
    (void)sig;
    running = 0;
}


/* =========================
 * SPI transfer
 * ========================= */

static int spi_transfer(uint8_t *tx, uint8_t *rx, uint32_t len)
{
    struct spi_ioc_transfer tr;

    memset(&tr, 0, sizeof(tr));

    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;

    tr.len = len;

    tr.speed_hz = SPI_SPEED_HZ;
    tr.bits_per_word = SPI_BITS;

    int ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);

    if (ret < 0) {
        perror("SPI transfer");
        return -1;
    }

    return 0;
}


/* =========================
 * Write register
 * ========================= */

static int icm_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = reg & 0x7F;
    tx[1] = value;

    return spi_transfer(tx, rx, 2);
}


/* =========================
 * Read register
 * ========================= */

static int icm_read_reg(uint8_t reg, uint8_t *value)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = reg | 0x80;
    tx[1] = 0x00;

    if (spi_transfer(tx, rx, 2) < 0)
        return -1;

    *value = rx[1];

    return 0;
}


/* =========================
 * Read multiple registers
 * ========================= */

static int icm_read_regs(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t tx[32];
    uint8_t rx[32];

    if (len > 30)
        return -1;

    memset(tx, 0, sizeof(tx));
    memset(rx, 0, sizeof(rx));

    tx[0] = reg | 0x80;

    if (spi_transfer(tx, rx, len + 1) < 0)
        return -1;

    memcpy(data, &rx[1], len);

    return 0;
}


/* =========================
 * Open SPI
 * ========================= */

static int spi_init(void)
{
    spi_fd = open(SPI_DEV, O_RDWR);

    if (spi_fd < 0) {
        perror("open SPI device");
        return -1;
    }

    uint8_t mode = SPI_MODE;
    uint8_t bits = SPI_BITS;
    uint32_t speed = SPI_SPEED_HZ;

    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0) {
        perror("SPI_IOC_WR_MODE");
        return -1;
    }

    if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        perror("SPI_IOC_WR_BITS_PER_WORD");
        return -1;
    }

    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        perror("SPI_IOC_WR_MAX_SPEED_HZ");
        return -1;
    }

    printf("SPI device : %s\n", SPI_DEV);
    printf("SPI mode   : %d\n", mode);
    printf("SPI bits   : %d\n", bits);
    printf("SPI speed  : %u Hz\n\n", speed);

    return 0;
}


/* =========================
 * Check WHO_AM_I
 * ========================= */

static int icm_check_id(void)
{
    uint8_t id = 0;

    if (icm_read_reg(REG_WHO_AM_I, &id) < 0)
        return -1;

    printf("WHO_AM_I = 0x%02X\n", id);

    if (id != ICM42688_WHO_AM_I) {

        printf("ERROR: WHO_AM_I mismatch!\n");
        printf("Expected: 0x47\n");
        printf("Received: 0x%02X\n", id);

        return -1;
    }

    printf("ICM42688P detected OK!\n\n");

    return 0;
}


/* =========================
 * Software reset
 * ========================= */

static int icm_reset(void)
{
    printf("Reset ICM42688P...\n");

    /*
     * DEVICE_CONFIG:
     *
     * Bit 0 = SOFT_RESET_CONFIG
     */

    if (icm_write_reg(REG_DEVICE_CONFIG, 0x01) < 0)
        return -1;

    usleep(100000);

    printf("Reset done.\n\n");

    return 0;
}


/* =========================
 * Configure ICM42688P
 * ========================= */

static int icm_config(void)
{
    printf("Configure ICM42688P...\n");

    /*
     * GYRO_CONFIG0
     *
     * FS_SEL = 1 -> ±500 dps
     *
     * ODR = 100 Hz
     *
     * Register:
     *
     * FS_SEL[7:5]
     * ODR[3:0]
     *
     * 500 dps = 0x20
     * 100 Hz = 0x07
     *
     * => 0x27
     */

    if (icm_write_reg(REG_GYRO_CONFIG0, 0x27) < 0)
        return -1;


    /*
     * ACCEL_CONFIG0
     *
     * FS_SEL = 1 -> ±4g
     *
     * ODR = 100 Hz
     *
     * => 0x27
     */

    if (icm_write_reg(REG_ACCEL_CONFIG0, 0x27) < 0)
        return -1;


    /*
     * PWR_MGMT0
     *
     * GYRO_MODE  = 11 -> Low Noise
     * ACCEL_MODE = 11 -> Low Noise
     *
     * 0x0F
     */

    if (icm_write_reg(REG_PWR_MGMT0, 0x0F) < 0)
        return -1;

    /*
     * Sensor startup time
     */

    usleep(50000);

    printf("Configuration done.\n\n");

    return 0;
}


/* =========================
 * Read sensor data
 * ========================= */

static int icm_read_sensor(
    int16_t *ax,
    int16_t *ay,
    int16_t *az,
    int16_t *gx,
    int16_t *gy,
    int16_t *gz,
    int16_t *temp)
{
    uint8_t data[14];

    /*
     * Starting from:
     *
     * TEMP_DATA1 = 0x1D
     *
     * 14 bytes:
     *
     * 0x1D temperature
     * 0x1E temperature
     *
     * 0x1F accel X
     * 0x20 accel X
     *
     * 0x21 accel Y
     * 0x22 accel Y
     *
     * 0x23 accel Z
     * 0x24 accel Z
     *
     * 0x25 gyro X
     * 0x26 gyro X
     *
     * 0x27 gyro Y
     * 0x28 gyro Y
     *
     * 0x29 gyro Z
     * 0x2A gyro Z
     */

    if (icm_read_regs(REG_TEMP_DATA1, data, 14) < 0)
        return -1;

    *temp = (int16_t)((data[0] << 8) | data[1]);

    *ax = (int16_t)((data[2] << 8) | data[3]);
    *ay = (int16_t)((data[4] << 8) | data[5]);
    *az = (int16_t)((data[6] << 8) | data[7]);

    *gx = (int16_t)((data[8] << 8) | data[9]);
    *gy = (int16_t)((data[10] << 8) | data[11]);
    *gz = (int16_t)((data[12] << 8) | data[13]);

    return 0;
}


/* =========================
 * Main
 * ========================= */

int main(void)
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("\n");
    printf("========================================\n");
    printf("      ICM42688P RK3566 SPI TEST\n");
    printf("========================================\n\n");


    /* Initialize SPI */

    if (spi_init() < 0)
        return -1;


    /* Initial delay */

    usleep(100000);


    /* Check WHO_AM_I */

    if (icm_check_id() < 0) {

        printf("\nICM42688P communication failed!\n");

        close(spi_fd);

        return -1;
    }


    /* Software reset */

    if (icm_reset() < 0) {

        close(spi_fd);

        return -1;
    }


    /*
     * Check WHO_AM_I again after reset
     */

    if (icm_check_id() < 0) {

        close(spi_fd);

        return -1;
    }


    /* Configure sensor */

    if (icm_config() < 0) {

        close(spi_fd);

        return -1;
    }


    printf("Start sensor reading...\n");
    printf("Press Ctrl+C to stop.\n\n");


    printf(
        " AX(g)     AY(g)     AZ(g)   "
        " GX(dps)   GY(dps)   GZ(dps)   TEMP(C)\n"
    );

    printf(
        "-------------------------------------------------------------\n"
    );


    while (running) {

        int16_t ax, ay, az;
        int16_t gx, gy, gz;
        int16_t temp_raw;

        if (icm_read_sensor(
                &ax,
                &ay,
                &az,
                &gx,
                &gy,
                &gz,
                &temp_raw) < 0) {

            printf("Sensor read error!\n");

            break;
        }


        /*
         * Convert acceleration
         */

        float ax_g = (float)ax / ACCEL_LSB_PER_G;
        float ay_g = (float)ay / ACCEL_LSB_PER_G;
        float az_g = (float)az / ACCEL_LSB_PER_G;


        /*
         * Convert gyro
         */

        float gx_dps = (float)gx / GYRO_LSB_PER_DPS;
        float gy_dps = (float)gy / GYRO_LSB_PER_DPS;
        float gz_dps = (float)gz / GYRO_LSB_PER_DPS;


        /*
         * Temperature
         *
         * T = (TEMP / 132.48) + 25
         */

        float temperature =
            ((float)temp_raw / 132.48f) + 25.0f;


        printf(
            "%8.3f %8.3f %8.3f   "
            "%8.2f %8.2f %8.2f   "
            "%7.2f\n",
            ax_g,
            ay_g,
            az_g,
            gx_dps,
            gy_dps,
            gz_dps,
            temperature
        );


        /*
         * 100 Hz output
         */

        usleep(10000);
    }


    printf("\nStopping...\n");

    close(spi_fd);

    return 0;
}

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <signal.h>
#include <math.h>
#include <errno.h>

/* =========================================================
 * SPI
 * ========================================================= */

#define SPI_DEV             "/dev/spidev3.0"
#define SPI_SPEED_HZ        1000000
#define SPI_MODE            SPI_MODE_0
#define SPI_BITS            8

static int spi_fd = -1;
static volatile int running = 1;


/* =========================================================
 * ICM42688P Registers
 * ========================================================= */

#define REG_DEVICE_CONFIG       0x11
#define REG_INT_CONFIG           0x14

#define REG_TEMP_DATA1           0x1D
#define REG_TEMP_DATA0           0x1E

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

#define REG_INT_STATUS            0x2D

#define REG_PWR_MGMT0             0x4E
#define REG_GYRO_CONFIG0          0x4F
#define REG_ACCEL_CONFIG0         0x50

#define REG_WHO_AM_I              0x75
#define REG_BANK_SEL              0x76


/* =========================================================
 * Expected
 * ========================================================= */

#define WHO_AM_I_EXPECTED         0x47


/* =========================================================
 * Sensor configuration
 *
 * Gyro:
 *   ±500 dps
 *   100 Hz
 *
 * Accel:
 *   ±4 g
 *   100 Hz
 * ========================================================= */

#define ACCEL_LSB_PER_G            8192.0f
#define GYRO_LSB_PER_DPS           65.5f


/* =========================================================
 * Signal
 * ========================================================= */

static void signal_handler(int sig)
{
    (void)sig;
    running = 0;
}


/* =========================================================
 * SPI transfer
 * ========================================================= */

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


/* =========================================================
 * Write register
 * ========================================================= */

static int icm_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = reg & 0x7F;
    tx[1] = value;

    return spi_transfer(tx, rx, 2);
}


/* =========================================================
 * Read register
 * ========================================================= */

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


/* =========================================================
 * Read multiple registers
 * ========================================================= */

static int icm_read_regs(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t tx[64];
    uint8_t rx[64];

    if (len > 60)
        return -1;

    memset(tx, 0, sizeof(tx));
    memset(rx, 0, sizeof(rx));

    tx[0] = reg | 0x80;

    if (spi_transfer(tx, rx, len + 1) < 0)
        return -1;

    memcpy(data, &rx[1], len);

    return 0;
}


/* =========================================================
 * Open / configure SPI
 * ========================================================= */

static int spi_init(void)
{
    spi_fd = open(SPI_DEV, O_RDWR);

    if (spi_fd < 0) {
        perror("open /dev/spidev3.0");
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


/* =========================================================
 * Read / print one register
 * ========================================================= */

static void print_reg(const char *name, uint8_t reg)
{
    uint8_t value = 0;

    if (icm_read_reg(reg, &value) == 0) {
        printf("%-18s [0x%02X] = 0x%02X  (%3u)\n",
               name, reg, value, value);
    } else {
        printf("%-18s [0x%02X] = READ ERROR\n",
               name, reg);
    }
}


/* =========================================================
 * WHO_AM_I
 * ========================================================= */

static int check_who_am_i(void)
{
    uint8_t id = 0;

    if (icm_read_reg(REG_WHO_AM_I, &id) < 0)
        return -1;

    printf("WHO_AM_I           [0x75] = 0x%02X\n", id);

    if (id == WHO_AM_I_EXPECTED) {
        printf(">>> ICM42688P ID OK\n");
        return 0;
    }

    printf(">>> ERROR: WHO_AM_I mismatch!\n");
    printf(">>> Expected 0x47, received 0x%02X\n", id);

    return -1;
}


/* =========================================================
 * Software reset
 * ========================================================= */

static int icm_reset(void)
{
    printf("\nSoftware reset...\n");

    /*
     * DEVICE_CONFIG
     * Bit0 = SOFT_RESET_CONFIG
     */

    if (icm_write_reg(REG_DEVICE_CONFIG, 0x01) < 0)
        return -1;

    /*
     * Datasheet startup/reset delay
     */

    usleep(100000);

    printf("Reset delay complete.\n");

    return 0;
}


/* =========================================================
 * Configure sensor
 * ========================================================= */

static int icm_config(void)
{
    printf("\nConfiguring sensor...\n");

    /*
     * GYRO_CONFIG0
     *
     * FS_SEL = 1 -> ±500 dps
     * ODR    = 7 -> 100 Hz
     *
     * 0x27
     */

    if (icm_write_reg(REG_GYRO_CONFIG0, 0x27) < 0)
        return -1;


    /*
     * ACCEL_CONFIG0
     *
     * FS_SEL = 1 -> ±4 g
     * ODR    = 7 -> 100 Hz
     *
     * 0x27
     */

    if (icm_write_reg(REG_ACCEL_CONFIG0, 0x27) < 0)
        return -1;


    /*
     * PWR_MGMT0
     *
     * Gyro  = Low Noise
     * Accel = Low Noise
     *
     * 0x0F
     */

    if (icm_write_reg(REG_PWR_MGMT0, 0x0F) < 0)
        return -1;


    /*
     * Allow sensors to start
     */

    usleep(50000);

    printf("Configuration complete.\n");

    return 0;
}


/* =========================================================
 * Print important registers
 * ========================================================= */

static void print_configuration(void)
{
    printf("\n========================================\n");
    printf("ICM42688P CONFIGURATION REGISTERS\n");
    printf("========================================\n");

    print_reg("DEVICE_CONFIG", REG_DEVICE_CONFIG);
    print_reg("INT_CONFIG", REG_INT_CONFIG);
    print_reg("PWR_MGMT0", REG_PWR_MGMT0);
    print_reg("GYRO_CONFIG0", REG_GYRO_CONFIG0);
    print_reg("ACCEL_CONFIG0", REG_ACCEL_CONFIG0);
    print_reg("INT_STATUS", REG_INT_STATUS);
    print_reg("WHO_AM_I", REG_WHO_AM_I);
    print_reg("BANK_SEL", REG_BANK_SEL);

    printf("========================================\n\n");
}


/* =========================================================
 * Read sensor
 *
 * Starting from TEMP_DATA1:
 *
 * 1D TEMP_H
 * 1E TEMP_L
 *
 * 1F AX_H
 * 20 AX_L
 *
 * 21 AY_H
 * 22 AY_L
 *
 * 23 AZ_H
 * 24 AZ_L
 *
 * 25 GX_H
 * 26 GX_L
 *
 * 27 GY_H
 * 28 GY_L
 *
 * 29 GZ_H
 * 2A GZ_L
 * ========================================================= */

static int read_sensor(
    int16_t *ax,
    int16_t *ay,
    int16_t *az,
    int16_t *gx,
    int16_t *gy,
    int16_t *gz,
    int16_t *temp)
{
    uint8_t data[14];

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


/* =========================================================
 * Main
 * ========================================================= */

int main(void)
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("\n");
    printf("========================================\n");
    printf("   ICM42688P RK3566 SPI DIAGNOSTIC\n");
    printf("========================================\n\n");


    /* -----------------------------------------
     * SPI init
     * ----------------------------------------- */

    if (spi_init() < 0)
        return -1;


    usleep(100000);


    /* -----------------------------------------
     * Initial WHO_AM_I
     * ----------------------------------------- */

    printf("Initial WHO_AM_I test:\n");

    if (check_who_am_i() < 0) {
        close(spi_fd);
        return -1;
    }


    /* -----------------------------------------
     * Reset
     * ----------------------------------------- */

    if (icm_reset() < 0) {
        close(spi_fd);
        return -1;
    }


    /* -----------------------------------------
     * Check ID after reset
     * ----------------------------------------- */

    printf("\nWHO_AM_I after reset:\n");

    if (check_who_am_i() < 0) {
        printf("\n!!! WHO_AM_I failed after reset !!!\n");
        close(spi_fd);
        return -1;
    }


    /* -----------------------------------------
     * Configure
     * ----------------------------------------- */

    if (icm_config() < 0) {
        close(spi_fd);
        return -1;
    }


    /* -----------------------------------------
     * Read configuration back
     * ----------------------------------------- */

    print_configuration();


    /* -----------------------------------------
     * Start output
     * ----------------------------------------- */

    printf("Sensor output:\n\n");

    printf(
        "RAW ACCEL                 RAW GYRO"
        "                  CONVERTED\n"
    );

    printf(
        " AX       AY       AZ       "
        "GX       GY       GZ       "
        " AX(g)    AY(g)    AZ(g)    "
        " GX(dps)  GY(dps)  GZ(dps)  "
        "|ACC|\n"
    );

    printf(
        "--------------------------------------------------------------------------------------------------------\n"
    );


    while (running)
    {
        int16_t ax, ay, az;
        int16_t gx, gy, gz;
        int16_t temp_raw;

        if (read_sensor(
                &ax,
                &ay,
                &az,
                &gx,
                &gy,
                &gz,
                &temp_raw) < 0)
        {
            printf("Sensor read error!\n");
            break;
        }


        /* -------------------------
         * Convert acceleration
         * ------------------------- */

        float ax_g = (float)ax / ACCEL_LSB_PER_G;
        float ay_g = (float)ay / ACCEL_LSB_PER_G;
        float az_g = (float)az / ACCEL_LSB_PER_G;


        /* -------------------------
         * Convert gyro
         * ------------------------- */

        float gx_dps = (float)gx / GYRO_LSB_PER_DPS;
        float gy_dps = (float)gy / GYRO_LSB_PER_DPS;
        float gz_dps = (float)gz / GYRO_LSB_PER_DPS;


        /* -------------------------
         * Acceleration magnitude
         *
         * sqrt(X²+Y²+Z²)
         * ------------------------- */

        float acc_mag =
            sqrtf(
                ax_g * ax_g +
                ay_g * ay_g +
                az_g * az_g
            );


        /* -------------------------
         * Temperature
         *
         * T = TEMP / 132.48 + 25
         * ------------------------- */

        float temperature =
            (float)temp_raw / 132.48f + 25.0f;


        printf(
            "%7d %7d %7d   "
            "%7d %7d %7d   "
            "%7.3f %7.3f %7.3f   "
            "%8.2f %8.2f %8.2f   "
            "%5.3f   "
            "T=%5.2f\n",
            ax,
            ay,
            az,
            gx,
            gy,
            gz,
            ax_g,
            ay_g,
            az_g,
            gx_dps,
            gy_dps,
            gz_dps,
            acc_mag,
            temperature
        );


        /*
         * 100 Hz
         */

        usleep(10000);
    }


    printf("\nProgram stopped.\n");

    close(spi_fd);

    return 0;
}


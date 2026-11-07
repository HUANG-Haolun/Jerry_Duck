#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <sys/select.h>
#include <gpiod.h>

#define UART_DEV        "/dev/ttyS3"
#define BAUDRATE        B115200

#define GPIOCHIP        "gpiochip3"
#define TXEN_LINE       7       // GPIO3_A7
#define RXEN_LINE       8       // GPIO3_B0

#define SERVO_ID        6

static struct gpiod_chip *chip = NULL;
static struct gpiod_line *txen = NULL;
static struct gpiod_line *rxen = NULL;
static struct gpiod_line_bulk dir_lines;

/*
 * UBT-12HC 半双工方向：
 *
 * 发送：TXEN = 0，RXEN = 1
 * 接收：TXEN = 1，RXEN = 0
 */

static int servo_tx_mode(void)
{
    int values[] = { 0, 1 };
    return gpiod_line_set_value_bulk(&dir_lines, values);
}

static int servo_rx_mode(void)
{
    int values[] = { 1, 0 };
    return gpiod_line_set_value_bulk(&dir_lines, values);
}

static int servo_rx_prepare(void)
{
    return gpiod_line_set_value(rxen, 0);
}

static int gpio_init(void)
{
    chip = gpiod_chip_open_by_name(GPIOCHIP);
    if (!chip) {
        perror("gpiod_chip_open_by_name");
        return -1;
    }

    txen = gpiod_chip_get_line(chip, TXEN_LINE);
    if (!txen) {
        perror("get TXEN line");
        return -1;
    }

    rxen = gpiod_chip_get_line(chip, RXEN_LINE);
    if (!rxen) {
        perror("get RXEN line");
        return -1;
    }

    gpiod_line_bulk_init(&dir_lines);
    gpiod_line_bulk_add(&dir_lines, txen);
    gpiod_line_bulk_add(&dir_lines, rxen);

    int initial_values[] = { 1, 0 };
    if (gpiod_line_request_bulk_output(&dir_lines, "servo_dir",
                                       initial_values) < 0) {
        perror("request direction GPIOs");
        return -1;
    }

    /*
     * 默认进入接收状态
     */
    servo_rx_mode();

    return 0;
}

static void gpio_deinit(void)
{
    if (txen)
        gpiod_line_release(txen);

    if (rxen)
        gpiod_line_release(rxen);

    if (chip)
        gpiod_chip_close(chip);
}

static int uart_init(void)
{
    int fd = open(UART_DEV, O_RDWR | O_NOCTTY);

    if (fd < 0) {
        perror("open UART");
        return -1;
    }

    struct termios tty;

    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        close(fd);
        return -1;
    }

    cfsetispeed(&tty, BAUDRATE);
    cfsetospeed(&tty, BAUDRATE);

    tty.c_cflag &= ~PARENB;      // 无校验
    tty.c_cflag &= ~CSTOPB;      // 1 stop
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;          // 8 bit

    tty.c_cflag &= ~CRTSCTS;     // 无硬件流控
    tty.c_cflag |= CLOCAL;
    tty.c_cflag |= CREAD;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(INLCR | ICRNL | IGNCR);

    tty.c_oflag &= ~OPOST;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    /*
     * 非阻塞读取
     */
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    return fd;
}

/* Wait until the UART shift register has transmitted the final stop bit. */
static int uart_wait_tx_empty(int fd)
{
    int lsr = 0;
    const int timeout_us = 5000;

    for (int elapsed = 0; elapsed < timeout_us; elapsed += 10) {
        if (ioctl(fd, TIOCSERGETLSR, &lsr) == 0) {
            if (lsr & TIOCSER_TEMT)
                return 0;
        } else {
            /* Some UART drivers do not implement this ioctl. */
            if (errno == ENOTTY || errno == EINVAL)
                return 1;
            return -1;
        }
        usleep(10);
    }

    errno = ETIMEDOUT;
    return -1;
}

/*
 * 计算校验和
 *
 * SUM(Byte2 ~ Byte7)
 *
 * FA AF 和 ED 不参与
 */
/*
 * 读取舵机角度
 */
static int servo_read_angle(int fd,
                            int *target_angle,
                            int *actual_angle)
{
    /*
     * FA AF ID CMD P1H P1L P2H P2L CHECKSUM ED
     *
     * ID = 10 = 0x0A
     * CMD = 02
     *
     * checksum:
     * 0A + 02 = 0C
     */
    uint8_t tx[10] = {
        0xFA,
        0xAF,
        SERVO_ID,
        0x02,
        0x00,
        0x00,
        0x00,
        0x00,
        0x08,
        0xED
    };

    uint8_t rx[256];
    int rx_len = 0;

    printf("\nTX:");

    for (int i = 0; i < 10; i++)
        printf(" %02X", tx[i]);

    printf("\n");

    /*
     * 清空 UART 接收缓冲区
     */
    tcflush(fd, TCIFLUSH);

    /*
     * 切换到发送模式
     */
    if (servo_tx_mode() < 0) {
        perror("servo_tx_mode");
        return -1;
    }

    /*
     * 给收发器一点建立时间
     */
    usleep(1000);

    /*
     * 发送
     */
    int ret = write(fd, tx, sizeof(tx));

    if (ret != sizeof(tx)) {
        perror("UART write");
        servo_rx_mode();
        return -1;
    }

    /* Open RX near the end of the frame; keep TX enabled until TEMT. */
    usleep(850);
    if (servo_rx_prepare() < 0) {
        perror("servo_rx_prepare");
        servo_rx_mode();
        return -1;
    }

    /* Wait for the actual UART transmitter-empty indication, not an estimate. */
    int tx_wait = uart_wait_tx_empty(fd);
    if (tx_wait < 0) {
        perror("wait UART TX empty");
        servo_rx_mode();
        return -1;
    }
    if (tx_wait > 0)
        usleep(900);

    /*
     * 立刻切换到接收模式
     */
    if (servo_rx_mode() < 0) {
        perror("servo_rx_mode");
        return -1;
    }

    /*
     * 给收发器进入接收状态一点时间
     */
    //usleep(10);

    /*
     * 接收窗口 1500ms
     */
    int timeout_ms = 1500;

    while (timeout_ms > 0 && rx_len < (int)sizeof(rx)) {

        fd_set readfds;

        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);

        struct timeval tv;

        tv.tv_sec = 0;
        tv.tv_usec = 10000;   // 10ms

        int r = select(fd + 1, &readfds, NULL, NULL, &tv);

        if (r < 0) {
            if (errno == EINTR)
                continue;

            perror("select");
            break;
        }

        if (r == 0) {
            timeout_ms -= 10;
            continue;
        }

        if (FD_ISSET(fd, &readfds)) {

            int n = read(fd, rx + rx_len,
                         sizeof(rx) - rx_len);

            if (n > 0) {
                rx_len += n;

                /*
                 * 如果已经收到数据，再等待一点，
                 * 防止数据分批到达
                 */
                usleep(5000);
            }
        }
    }

    /*
     * 打印实际收到的全部数据
     */
    if (rx_len == 0) {

        printf("RX: no response\n");
        return -2;
    }

    printf("RX (%d bytes):", rx_len);

    for (int i = 0; i < rx_len; i++)
        printf(" %02X", rx[i]);

    printf("\n");

    /*
     * 找协议帧：
     *
     * FA AF ID STATUS TARGET_H TARGET_L ACTUAL_H ACTUAL_L CHECKSUM ED
     */
    for (int start = 0; start <= rx_len - 9; start++) {
        int af;

        if (rx[start] == 0xFA && rx[start + 1] == 0xAF)
            af = start + 1;
        else if (rx[start] == 0x00 && rx[start + 1] == 0xAF)
            af = start + 1; /* leading FA was lost */
        else
            continue;

        if (rx[af + 1] != SERVO_ID)
            continue;

        /*
         * 状态
         */
        uint8_t status = rx[af + 2];

        /* 0x02 is our read command echo, not a servo response. */
        if (status != 0xAA)
            continue;

        /*
         * ED
         */
        if (rx[af + 8] != 0xED)
            continue;

        /*
         * 校验
         */
        uint8_t cs = 0;
        for (int i = af + 1; i <= af + 6; i++)
            cs += rx[i];

        if (cs != rx[af + 7]) {

            printf("Checksum error: calculated=%02X received=%02X\n",
                   cs,
                   rx[af + 7]);

            continue;
        }

        /*
         * 状态 AA = 成功
         */
        if (status != 0xAA) {

            printf("Servo response status = %02X\n", status);

            return -3;
        }

        *target_angle =
            ((int)rx[af + 3] << 8) |
            rx[af + 4];

        *actual_angle =
            ((int)rx[af + 5] << 8) |
            rx[af + 6];

        printf("\n========================================\n");
        printf(" Servo ID       : %d\n", SERVO_ID);
        printf(" Status         : 0x%02X\n", status);
        printf(" Target angle   : %d deg\n", *target_angle);
        printf(" Actual angle   : %d deg\n", *actual_angle);
        printf("========================================\n");

        return 0;
    }

    printf("RX data received, but no valid UBT frame found.\n");

    return -4;
}

int main(void)
{
    printf("========================================\n");
    printf(" RK3566 UBT-12HC Servo Angle Reader\n");
    printf(" UART : %s\n", UART_DEV);
    printf(" Baud : 115200 8N1\n");
    printf(" Servo ID : %d\n", SERVO_ID);
    printf(" TXEN : gpiochip3 line 7\n");
    printf(" RXEN : gpiochip3 line 8\n");
    printf("========================================\n");

    /*
     * GPIO 初始化
     */
    if (gpio_init() < 0) {

        printf("GPIO initialization FAILED.\n");
        gpio_deinit();

        return 1;
    }

    printf("GPIO initialized.\n");

    /*
     * UART 初始化
     */
    int fd = uart_init();

    if (fd < 0) {

        printf("UART initialization FAILED.\n");
        gpio_deinit();

        return 1;
    }

    printf("UART opened successfully.\n");

    /*
     * 等待硬件稳定
     */
    usleep(10000);

    int target_angle = 0;
    int actual_angle = 0;

    int ret = servo_read_angle(fd,
                               &target_angle,
                               &actual_angle);

    if (ret == 0) {

        printf("\nRead servo angle SUCCESS.\n");

    } else {

        printf("\nRead servo angle FAILED, error=%d\n", ret);
    }

    /*
     * 回到接收状态
     */
    servo_rx_mode();

    close(fd);

    gpio_deinit();

    return (ret == 0) ? 0 : 1;
}

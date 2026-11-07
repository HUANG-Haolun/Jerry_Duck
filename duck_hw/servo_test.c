#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <gpiod.h>

#define UART_DEV "/dev/ttyS3"

#define TXEN_GPIO 7
#define RXEN_GPIO 8

#define SERVO_ID 6

int uart_fd;
struct gpiod_chip *chip;
struct gpiod_line *txen;
struct gpiod_line *rxen;


/* =========================
 * GPIO 初始化
 * ========================= */
int gpio_init(void)
{
    chip = gpiod_chip_open("/dev/gpiochip3");

    if (!chip) {
        perror("gpiod_chip_open");
        return -1;
    }

    txen = gpiod_chip_get_line(chip, TXEN_GPIO);
    rxen = gpiod_chip_get_line(chip, RXEN_GPIO);

    if (!txen || !rxen) {
        perror("gpiod_chip_get_line");
        return -1;
    }

    if (gpiod_line_request_output(txen, "ubt_txen", 0) < 0) {
        perror("TXEN");
        return -1;
    }

    if (gpiod_line_request_output(rxen, "ubt_rxen", 0) < 0) {
        perror("RXEN");
        return -1;
    }

    return 0;
}


/* =========================
 * UART 初始化
 * ========================= */
int uart_init(void)
{
    uart_fd = open(
        UART_DEV,
        O_RDWR | O_NOCTTY
    );

    if (uart_fd < 0) {
        perror("open uart");
        return -1;
    }

    struct termios tty;

    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(uart_fd, &tty) != 0) {
        perror("tcgetattr");
        return -1;
    }

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    /* 8N1 */
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    /* 无硬件流控 */
    tty.c_cflag &= ~CRTSCTS;

    tty.c_cflag |= CREAD;
    tty.c_cflag |= CLOCAL;

    /* RAW */
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    tcflush(uart_fd, TCIOFLUSH);

    if (tcsetattr(uart_fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        return -1;
    }

    return 0;
}


/* =========================
 * TX模式
 * ========================= */
void rx_mode(void)
{
    gpiod_line_set_value(txen, 1);
    gpiod_line_set_value(rxen, 0);

    usleep(20);
}


/* =========================
 * RX模式
 * ========================= */
void tx_mode(void)
{
    gpiod_line_set_value(txen, 0);
    gpiod_line_set_value(rxen, 1);

    usleep(20);
}


/* =========================
 * 发送舵机命令
 * ========================= */
void servo_move(
    uint8_t id,
    uint8_t position,
    uint8_t time
)
{
    uint8_t packet[10];

    packet[0] = 0xFA;
    packet[1] = 0xAF;

    packet[2] = id;
    packet[3] = 0x01;

    packet[4] = position;
    packet[5] = time;

    packet[6] = 0x00;
    packet[7] = 0x00;

    /*
     * checksum
     */
    packet[8] =
        (packet[2] +
         packet[3] +
         packet[4] +
         packet[5] +
         packet[6] +
         packet[7]) & 0xFF;

    packet[9] = 0xED;


    printf("TX:");

    for (int i = 0; i < 10; i++)
        printf(" %02X", packet[i]);

    printf("\n");


    /* 切换发送 */
    tx_mode();

    /* 发送 */
    write(
        uart_fd,
        packet,
        10
    );

    /*
     * 非常重要：
     * 等最后一个字节真正发完
     */
    tcdrain(uart_fd);

    /* 切换接收 */
    rx_mode();
}


/* =========================
 * main
 * ========================= */
int main(void)
{
    printf("\n");
    printf("============================\n");
    printf(" UBTECH UBT-12HC TEST\n");
    printf("============================\n");

    printf("UART : %s\n", UART_DEV);
    printf("TXEN : GPIO3_A7\n");
    printf("RXEN : GPIO3_B0\n");
    printf("Baud : 115200\n");
    printf("Servo ID : %d\n\n", SERVO_ID);


    /* GPIO */
    if (gpio_init() < 0)
        return -1;

    printf("GPIO OK\n");


    /* UART */
    if (uart_init() < 0)
        return -1;

    printf("UART OK\n");


    rx_mode();


    /*
     * ==========================
     * 移动到位置
     * ==========================
     *
     * position：
     *
     * 2   ~ 最小
     * 70  ~ 中间附近
     * 120 ~ 中间附近
     * 138 ~ 最大
     *
     * time：
     * 50 = 约 1 秒
     */

    printf("Move servo...\n");

    servo_move(
        SERVO_ID,
        150,
        50
    );


    printf("Command sent.\n");

    while (1)
    {
        sleep(1);
    }


    return 0;
}

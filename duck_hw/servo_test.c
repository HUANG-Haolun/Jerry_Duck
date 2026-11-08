#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>

#define UART_DEV "/dev/ttyS3"

#define SEND_INTERVAL_NS 100000000L /* 10 Hz */

int uart_fd;


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

    cfsetispeed(&tty, B230400);
    cfsetospeed(&tty, B230400);

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
 * 发送舵机命令
 * ========================= */
void servo_move(
    uint8_t id,
    uint16_t position
)
{
    uint8_t packet[8];

    packet[0] = 0xFA;
    packet[1] = 0xAF;

    packet[2] = id;
    packet[3] = 0x01;

    packet[4] = (uint8_t)(position >> 8);
    packet[5] = (uint8_t)position;

    /*
     * checksum
     */
    packet[6] =
        (packet[2] +
         packet[3] +
         packet[4] +
         packet[5]) & 0xFF;

    packet[7] = 0xED;


    printf("TX:");

    for (int i = 0; i < 8; i++)
        printf(" %02X", packet[i]);

    printf("\n");


    /* 发送 */
    write(
        uart_fd,
        packet,
        sizeof(packet)
    );

    /*
     * 非常重要：
     * 等最后一个字节真正发完
     */
    tcdrain(uart_fd);

}


/* =========================
 * main
 * ========================= */
static int parse_number(const char *text, long minimum, long maximum,
                        uint16_t *value)
{
    char *end;
    errno = 0;
    long parsed = strtol(text, &end, 10);
    if (errno || end == text || *end != '\0' || parsed < minimum || parsed > maximum)
        return -1;
    *value = (uint16_t)parsed;
    return 0;
}

int main(int argc, char *argv[])
{
    uint16_t servo_id;
    uint16_t position;

    if (argc != 3 || parse_number(argv[1], 0, 255, &servo_id) < 0 ||
        parse_number(argv[2], 100, 900, &position) < 0) {
        fprintf(stderr, "Usage: %s <ID: 0-255> <position: 100-900>\n", argv[0]);
        return EXIT_FAILURE;
    }

    printf("\n");
    printf("============================\n");
    printf(" UBTECH UBT-12HC TEST\n");
    printf("============================\n");

    printf("UART : %s\n", UART_DEV);
    printf("Baud : 230400\n");
    printf("Servo ID : %u\n", servo_id);
    printf("Position : %u\n", position);
    printf("Send rate : 10 Hz (100 ms)\n\n");


    /* UART */
    if (uart_init() < 0)
        return -1;

    printf("UART OK\n");


    /* 位置范围：100～900，按 8 字节协议发送。 */
    printf("Move servo...\n");

    struct timespec next_send;
    if (clock_gettime(CLOCK_MONOTONIC, &next_send) < 0) {
        perror("clock_gettime");
        return EXIT_FAILURE;
    }

    while (1)
    {
        servo_move((uint8_t)servo_id, position);

        next_send.tv_nsec += SEND_INTERVAL_NS;
        if (next_send.tv_nsec >= 1000000000L) {
            next_send.tv_sec++;
            next_send.tv_nsec -= 1000000000L;
        }

        int result;
        do {
            result = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                                     &next_send, NULL);
        } while (result == EINTR);
        if (result != 0) {
            errno = result;
            perror("clock_nanosleep");
            return EXIT_FAILURE;
        }
    }


    return 0;
}

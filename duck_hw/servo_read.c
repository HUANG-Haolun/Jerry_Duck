#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/select.h>
#include <signal.h>
#include <time.h>

#define UART_DEV "/dev/ttyS3"
#define BAUDRATE B230400
#define INTERVAL_NS 100000000LL /* 10 Hz */

static volatile sig_atomic_t stop_requested;

static void request_stop(int signo)
{
    (void)signo;
    stop_requested = 1;
}

static int64_t monotonic_ns(void)
{
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) < 0) {
        perror("clock_gettime");
        exit(EXIT_FAILURE);
    }
    return (int64_t)now.tv_sec * 1000000000LL + now.tv_nsec;
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

/* 自动收发：FA AF ID 03 00 00 CHECKSUM ED。 */
static int send_query(int fd, uint8_t id)
{
    uint8_t tx[8] = {0xFA, 0xAF, id, 0x03, 0, 0,
                     (uint8_t)(id + 0x03), 0xED};
    size_t sent = 0;
    while (sent < sizeof(tx) && !stop_requested) {
        ssize_t n = write(fd, tx + sent, sizeof(tx) - sent);
        if (n > 0)
            sent += (size_t)n;
        else if (n < 0 && errno == EINTR)
            continue;
        else {
            if (n == 0)
                errno = EIO;
            perror("UART write");
            return -1;
        }
    }
    if (sent != sizeof(tx))
        return -1;
    printf("TX:");
    for (size_t i = 0; i < sizeof(tx); ++i)
        printf(" %02X", tx[i]);
    fflush(stdout);
    return 0;
}

/* 保留未收完整的帧，处理分段回复、查询回显及噪声。 */
static void receive_byte(uint8_t byte, uint8_t id,
                         uint8_t pending[9], size_t *length, int *received)
{
    pending[(*length)++] = byte;
    while (*length > 0) {
        if (pending[0] != 0xFA ||
            (*length >= 2 && pending[1] != 0xAF)) {
            memmove(pending, pending + 1, --(*length));
            continue;
        }
        if (*length >= 8 && pending[2] == id && pending[3] == 0x03 &&
            pending[4] == 0 && pending[5] == 0 &&
            pending[6] == (uint8_t)(id + 0x03) && pending[7] == 0xED) {
            *length = 0;
            return;
        }
        if (*length < 9)
            return;

        /* 回复：FA AF ID POS_H POS_L SPEED_H SPEED_L CS ED。 */
        uint8_t checksum = 0;
        for (size_t i = 2; i <= 6; ++i)
            checksum += pending[i];
        if (pending[2] == id && pending[8] == 0xED &&
            pending[7] == checksum) {
            unsigned position = ((unsigned)pending[3] << 8) | pending[4];
            unsigned raw_speed = ((unsigned)pending[5] << 8) | pending[6];
            int speed = raw_speed >= 0x8000 ? (int)raw_speed - 0x10000 :
                                            (int)raw_speed;
            printf(" | RX: ID=%u Position=%u Speed=%d", id, position, speed);
            fflush(stdout);
            *received = 1;
            *length = 0;
            return;
        }
        memmove(pending, pending + 1, --(*length));
    }
}

int main(int argc, char *argv[])
{
    char *end;
    long parsed_id = -1;
    if (argc == 2) {
        errno = 0;
        parsed_id = strtol(argv[1], &end, 10);
        if (errno || end == argv[1] || *end != '\0')
            parsed_id = -1;
    }
    if (parsed_id < 0 || parsed_id > 255) {
        fprintf(stderr, "Usage: %s <ID: 0-255>\n", argv[0]);
        return EXIT_FAILURE;
    }
    uint8_t id = (uint8_t)parsed_id;
    struct sigaction action = {0};
    action.sa_handler = request_stop;
    sigemptyset(&action.sa_mask);
    if (sigaction(SIGINT, &action, NULL) < 0 ||
        sigaction(SIGTERM, &action, NULL) < 0) {
        perror("sigaction");
        return EXIT_FAILURE;
    }
    setvbuf(stdout, NULL, _IOLBF, 0);
    printf("UART: %s, Baud: 230400 8N1, ID: %u, CMD: 0x03, Rate: 10 Hz\n",
           UART_DEV, id);
    int fd = uart_init();
    if (fd < 0)
        return EXIT_FAILURE;

    uint8_t pending[9];
    size_t pending_len = 0;
    int result = EXIT_SUCCESS;
    int row_open = 0;
    int received = 0;
    int64_t next_send = monotonic_ns();
    while (!stop_requested) {
        int64_t now = monotonic_ns();
        if (now >= next_send) {
            if (row_open) {
                printf("%s\n", received ? "" : " | RX: no valid response");
                row_open = 0;
            }
            if (send_query(fd, id) < 0) {
                if (!stop_requested)
                    result = EXIT_FAILURE;
                break;
            }
            row_open = 1;
            received = 0;
            next_send += INTERVAL_NS;
            /* 调度延迟时跳过错过的周期，避免连续补发。 */
            now = monotonic_ns();
            if (next_send <= now)
                next_send += ((now - next_send) / INTERVAL_NS + 1) * INTERVAL_NS;
        }
        int64_t remaining = next_send - monotonic_ns();
        if (remaining <= 0)
            continue;
        struct timeval timeout = {
            .tv_sec = remaining / 1000000000LL,
            .tv_usec = (remaining % 1000000000LL) / 1000
        };
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);
        int ready = select(fd + 1, &readfds, NULL, NULL, &timeout);
        if (ready < 0) {
            if (errno == EINTR)
                continue;
            perror("select");
            result = EXIT_FAILURE;
            break;
        }
        if (ready > 0) {
            uint8_t data[256];
            ssize_t n = read(fd, data, sizeof(data));
            if (n < 0) {
                if (errno == EINTR)
                    continue;
                perror("UART read");
                result = EXIT_FAILURE;
                break;
            }
            for (ssize_t i = 0; i < n; ++i)
                receive_byte(data[i], id, pending, &pending_len, &received);
        }
    }
    if (row_open)
        printf("%s\n", received ? "" : " | RX: no valid response");
    close(fd);
    return result;
}

#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm/termbits.h>
#include <asm/ioctls.h>

#define UART_DEV "/dev/ttyS3"
#define BAUDRATE 230460
#define SERVO_ID 6
#define POSITION_A 100
#define POSITION_B 900
#define STEP_INTERVAL_NS 20000000L /* 20 ms */
#define STEPS_PER_LEG 125       /* 2.5 seconds each way */

static volatile sig_atomic_t stop_requested;

static void request_stop(int signal_number)
{
    (void)signal_number;
    stop_requested = 1;
}

static int uart_open(void)
{
    int fd = open(UART_DEV, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        perror("open UART");
        return -1;
    }

    struct termios2 tty;
    if (ioctl(fd, TCGETS2, &tty) < 0) {
        perror("TCGETS2");
        close(fd);
        return -1;
    }

    tty.c_cflag &= ~(CBAUD | PARENB | CSTOPB | CSIZE | CRTSCTS);
    tty.c_cflag |= BOTHER | CS8 | CLOCAL | CREAD;
    tty.c_ispeed = BAUDRATE;
    tty.c_ospeed = BAUDRATE;
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (ioctl(fd, TCSETS2, &tty) < 0) {
        perror("TCSETS2");
        close(fd);
        return -1;
    }
    if (ioctl(fd, TCGETS2, &tty) < 0 ||
        tty.c_ispeed != BAUDRATE || tty.c_ospeed != BAUDRATE) {
        fprintf(stderr, "Unable to set UART to %d baud\n", BAUDRATE);
        close(fd);
        return -1;
    }
    return fd;
}

static int send_position(int fd, uint16_t position)
{
    uint8_t frame[8] = {
        0xFA, 0xAF, SERVO_ID, 0x01,
        (uint8_t)(position >> 8), (uint8_t)position,
        0, 0xED
    };
    frame[6] = (uint8_t)(frame[2] + frame[3] + frame[4] + frame[5]);

    size_t sent = 0;
    while (sent < sizeof(frame)) {
        ssize_t n = write(fd, frame + sent, sizeof(frame) - sent);
        if (n > 0) {
            sent += (size_t)n;
        } else if (n < 0 && (errno == EINTR || errno == EAGAIN)) {
            if (stop_requested)
                return -1;
        } else {
            perror("UART write");
            return -1;
        }
    }
    if (ioctl(fd, TCSBRK, 1) < 0) {
        perror("UART drain");
        return -1;
    }

    return 0;
}

static int wait_until(const struct timespec *deadline)
{
    while (!stop_requested) {
        int ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                                  deadline, NULL);
        if (ret == 0)
            return 0;
        if (ret != EINTR) {
            errno = ret;
            perror("clock_nanosleep");
            return -1;
        }
    }
    return 1;
}

static int sweep_leg(int fd, uint16_t from, uint16_t to,
                     struct timespec *next)
{
    int delta = (int)to - (int)from;
    for (int step = 1; step <= STEPS_PER_LEG; ++step) {
        next->tv_nsec += STEP_INTERVAL_NS;
        if (next->tv_nsec >= 1000000000L) {
            next->tv_sec++;
            next->tv_nsec -= 1000000000L;
        }

        int wait_result = wait_until(next);
        if (wait_result != 0)
            return wait_result;

        /* Round to the nearest target unit; the time steps stay uniform. */
        int offset = (delta * step + (delta >= 0 ? STEPS_PER_LEG / 2
                                                   : -STEPS_PER_LEG / 2))
                     / STEPS_PER_LEG;
        if (send_position(fd, (uint16_t)((int)from + offset)) < 0)
            return -1;
    }
    return 0;
}

int main(int argc, char **argv)
{
    unsigned long cycles = 0; /* 0 means run until Ctrl+C. */
    if (argc > 2) {
        fprintf(stderr, "Usage: %s [cycles]\n", argv[0]);
        return 2;
    }
    if (argc == 2) {
        char *end;
        errno = 0;
        cycles = strtoul(argv[1], &end, 10);
        if (errno || end == argv[1] || *end || cycles == 0) {
            fprintf(stderr, "cycles must be a positive integer\n");
            return 2;
        }
    }

    struct sigaction action = {0};
    action.sa_handler = request_stop;
    sigemptyset(&action.sa_mask);
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);

    int fd = uart_open();
    if (fd < 0)
        return 1;

    printf("Servo %d: %d <-> %d, 5 seconds per cycle, 20 ms steps; Ctrl+C to stop.\n",
           SERVO_ID, POSITION_A, POSITION_B);

    struct timespec next;
    if (clock_gettime(CLOCK_MONOTONIC, &next) < 0) {
        perror("clock_gettime");
        close(fd);
        return 1;
    }

    int result = send_position(fd, POSITION_A) < 0;
    if (!result)
        printf("Position %d\n", POSITION_A);

    for (unsigned long cycle = 0; !stop_requested &&
         !result && (cycles == 0 || cycle < cycles); ++cycle) {
        int sweep_result = sweep_leg(fd, POSITION_A, POSITION_B, &next);
        if (sweep_result != 0) {
            result = sweep_result < 0;
            break;
        }
        printf("Position %d\n", POSITION_B);

        sweep_result = sweep_leg(fd, POSITION_B, POSITION_A, &next);
        if (sweep_result != 0) {
            result = sweep_result < 0;
            break;
        }
        printf("Position %d (cycle %lu complete)\n", POSITION_A, cycle + 1);
    }

    close(fd);
    return result;
}

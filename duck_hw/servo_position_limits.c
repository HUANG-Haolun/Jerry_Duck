#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>
#include <asm/termbits.h>
#include <asm/ioctls.h>

#define UART_DEV "/dev/ttyS3"
#define BAUDRATE 230460
#define CYCLES_PER_SECOND 50
#define GRACE_MS 100

static const uint8_t servo_ids[] = {
    1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16
};
#define SERVO_COUNT (sizeof(servo_ids) / sizeof(servo_ids[0]))
#define QUERY_HZ (CYCLES_PER_SECOND * SERVO_COUNT)

struct servo_state {
    uint16_t current;
    uint16_t minimum;
    uint16_t maximum;
    uint64_t replies;
    uint64_t shown_replies;
};

struct monitor {
    struct servo_state servo[SERVO_COUNT];
    uint8_t pending[16];
    size_t pending_len;
    uint64_t sent;
    uint64_t echoes;
    uint64_t malformed;
    uint64_t checksum_errors;
    uint64_t noise_bytes;
};

static volatile sig_atomic_t stop_requested;

static void request_stop(int signo)
{
    (void)signo;
    stop_requested = 1;
}

static uint64_t monotonic_ns(void)
{
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) < 0) {
        perror("clock_gettime");
        exit(1);
    }
    return (uint64_t)now.tv_sec * 1000000000ULL + now.tv_nsec;
}

static int servo_index(uint8_t id)
{
    for (size_t i = 0; i < SERVO_COUNT; ++i)
        if (servo_ids[i] == id)
            return (int)i;
    return -1;
}

static int open_uart(void)
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
    if (ioctl(fd, TCSETS2, &tty) < 0 ||
        ioctl(fd, TCGETS2, &tty) < 0 ||
        tty.c_ispeed != BAUDRATE || tty.c_ospeed != BAUDRATE) {
        fprintf(stderr, "Failed to configure UART at %d baud\n", BAUDRATE);
        close(fd);
        return -1;
    }
    if (ioctl(fd, TCFLSH, TCIOFLUSH) < 0) {
        perror("TCFLSH");
        close(fd);
        return -1;
    }
    return fd;
}

static int send_query(int fd, uint8_t id)
{
    const uint8_t frame[8] = {
        0xFA, 0xAF, id, 0x03, 0, 0, (uint8_t)(id + 0x03), 0xED
    };
    size_t offset = 0;
    while (offset < sizeof(frame)) {
        ssize_t n = write(fd, frame + offset, sizeof(frame) - offset);
        if (n > 0) {
            offset += (size_t)n;
        } else if (n < 0 && (errno == EINTR || errno == EAGAIN)) {
            if (stop_requested)
                return -1;
        } else {
            perror("UART write");
            return -1;
        }
    }
    return 0;
}

static void consume(struct monitor *m, size_t count)
{
    m->pending_len -= count;
    memmove(m->pending, m->pending + count, m->pending_len);
}

static void parse_pending(struct monitor *m)
{
    while (m->pending_len >= 2) {
        if (m->pending[0] != 0xFA || m->pending[1] != 0xAF) {
            m->noise_bytes++;
            consume(m, 1);
            continue;
        }
        if (m->pending_len < 8)
            return;

        int index = servo_index(m->pending[2]);
        if (index < 0) {
            m->malformed++;
            consume(m, 1);
            continue;
        }

        const uint8_t echo[8] = {
            0xFA, 0xAF, m->pending[2], 0x03, 0, 0,
            (uint8_t)(m->pending[2] + 0x03), 0xED
        };
        if (memcmp(m->pending, echo, sizeof(echo)) == 0) {
            m->echoes++;
            consume(m, sizeof(echo));
            continue;
        }
        if (m->pending_len < 9)
            return;
        if (m->pending[8] != 0xED) {
            m->malformed++;
            consume(m, 1);
            continue;
        }
        uint8_t checksum = (uint8_t)(m->pending[2] + m->pending[3] +
                                     m->pending[4] + m->pending[5] +
                                     m->pending[6]);
        if (m->pending[7] != checksum) {
            m->checksum_errors++;
            consume(m, 1);
            continue;
        }

        uint16_t position = ((uint16_t)m->pending[3] << 8) | m->pending[4];
        struct servo_state *s = &m->servo[index];
        if (s->replies == 0) {
            s->minimum = position;
            s->maximum = position;
        } else {
            if (position < s->minimum)
                s->minimum = position;
            if (position > s->maximum)
                s->maximum = position;
        }
        s->current = position;
        s->replies++;
        consume(m, 9);
    }
}

static int read_available(int fd, struct monitor *m)
{
    uint8_t buffer[512];
    for (;;) {
        ssize_t n = read(fd, buffer, sizeof(buffer));
        if (n < 0 && errno == EINTR)
            continue;
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
            return 0;
        if (n < 0) {
            perror("UART read");
            return -1;
        }
        if (n == 0)
            return 0;
        for (ssize_t i = 0; i < n; ++i) {
            if (m->pending_len == sizeof(m->pending)) {
                m->noise_bytes++;
                consume(m, 1);
            }
            m->pending[m->pending_len++] = buffer[i];
            parse_pending(m);
        }
    }
}

static int receive_until(int fd, uint64_t deadline_ns, struct monitor *m)
{
    while (!stop_requested) {
        uint64_t now = monotonic_ns();
        if (now >= deadline_ns)
            return 0;
        uint64_t remaining = deadline_ns - now;
        struct timespec timeout = {
            .tv_sec = (time_t)(remaining / 1000000000ULL),
            .tv_nsec = (long)(remaining % 1000000000ULL)
        };
        struct pollfd pfd = { .fd = fd, .events = POLLIN };
        int ready = ppoll(&pfd, 1, &timeout, NULL);
        if (ready < 0) {
            if (errno == EINTR)
                continue;
            perror("ppoll");
            return -1;
        }
        if (ready > 0) {
            if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
                fprintf(stderr, "UART poll error: %x\n", pfd.revents);
                return -1;
            }
            if ((pfd.revents & POLLIN) && read_available(fd, m) < 0)
                return -1;
        }
    }
    return 0;
}

static void print_positions(struct monitor *m, const char *label)
{
    printf("\n%s\n", label);
    printf(" ID  Current    Min    Max  New replies  Total replies\n");
    for (size_t i = 0; i < SERVO_COUNT; ++i) {
        struct servo_state *s = &m->servo[i];
        if (s->replies) {
            printf("%3u  %7u  %5u  %5u  %11" PRIu64 "  %13" PRIu64 "\n",
                   servo_ids[i], s->current, s->minimum, s->maximum,
                   s->replies - s->shown_replies, s->replies);
        } else {
            printf("%3u  %7s  %5s  %5s  %11" PRIu64 "  %13" PRIu64 "\n",
                   servo_ids[i], "--", "--", "--",
                   (uint64_t)0, (uint64_t)0);
        }
        s->shown_replies = s->replies;
    }
    printf("Sent=%" PRIu64 ", echoes=%" PRIu64 ", malformed=%" PRIu64
           ", checksum errors=%" PRIu64 ", noise bytes=%" PRIu64 "\n",
           m->sent, m->echoes, m->malformed,
           m->checksum_errors, m->noise_bytes);
    fflush(stdout);
}

int main(int argc, char **argv)
{
    unsigned long seconds = 0; /* Omit duration to run until Ctrl+C. */
    if (argc > 2) {
        fprintf(stderr, "Usage: %s [seconds]\n", argv[0]);
        return 2;
    }
    if (argc == 2) {
        char *end;
        errno = 0;
        seconds = strtoul(argv[1], &end, 10);
        if (errno || end == argv[1] || *end || seconds == 0 || seconds > 3600) {
            fprintf(stderr, "seconds must be an integer from 1 to 3600\n");
            return 2;
        }
    }

    struct sigaction action = {0};
    action.sa_handler = request_stop;
    sigemptyset(&action.sa_mask);
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);

    int fd = open_uart();
    if (fd < 0)
        return 1;

    struct monitor m = {0};
    uint64_t start_ns = monotonic_ns();
    uint64_t total_queries = seconds ? (uint64_t)seconds * QUERY_HZ : UINT64_MAX;
    printf("Monitoring %zu servos at %zu queries/s. Values are raw positions.\n",
           SERVO_COUNT, QUERY_HZ);
    fflush(stdout);

    int result = 0;
    uint64_t queries = 0;
    for (; queries < total_queries && !stop_requested; ++queries) {
        uint64_t due = start_ns + queries * 1000000000ULL / QUERY_HZ;
        if (receive_until(fd, due, &m) < 0) {
            result = 1;
            break;
        }
        if (stop_requested)
            break;
        if (send_query(fd, servo_ids[queries % SERVO_COUNT]) < 0) {
            result = 1;
            break;
        }
        m.sent++;

        if ((queries + 1) % QUERY_HZ == 0) {
            if (read_available(fd, &m) < 0) {
                result = 1;
                break;
            }
            print_positions(&m, "Current extrema:");
        }
    }

    if (!result && !stop_requested && seconds) {
        uint64_t end_ns = start_ns + total_queries * 1000000000ULL / QUERY_HZ;
        if (receive_until(fd, end_ns + GRACE_MS * 1000000ULL, &m) < 0)
            result = 1;
    }
    if (read_available(fd, &m) < 0)
        result = 1;
    print_positions(&m, "Final extrema:");
    close(fd);
    return result;
}

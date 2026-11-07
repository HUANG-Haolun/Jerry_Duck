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
#define SERVO_ID 6
#define QUERY_HZ 750
#define DEFAULT_SECONDS 10
#define RX_GRACE_MS 100

static const uint8_t query[8] = {
    0xFA, 0xAF, SERVO_ID, 0x03, 0x00, 0x00,
    (uint8_t)(SERVO_ID + 0x03), 0xED
};

static volatile sig_atomic_t stop_requested;

struct stats {
    uint64_t sent;
    uint64_t valid;
    uint64_t echoes;
    uint64_t checksum_errors;
    uint64_t other_frames;
    uint64_t noise_bytes;
    uint64_t received_bytes;
    uint64_t late_sends;
    uint64_t first_send_ns;
    uint64_t last_send_ns;
    uint16_t last_position;
    uint16_t min_position;
    uint16_t max_position;
    int16_t last_speed;
    int16_t min_speed;
    int16_t max_speed;
    uint8_t pending[16];
    size_t pending_len;
};

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
    if (ioctl(fd, TCSETS2, &tty) < 0) {
        perror("TCSETS2");
        close(fd);
        return -1;
    }
    if (ioctl(fd, TCGETS2, &tty) < 0 ||
        tty.c_ispeed != BAUDRATE || tty.c_ospeed != BAUDRATE) {
        fprintf(stderr, "Cannot set baud rate to %d\n", BAUDRATE);
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

static int send_query(int fd)
{
    size_t sent = 0;
    while (sent < sizeof(query)) {
        ssize_t n = write(fd, query + sent, sizeof(query) - sent);
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
    return 0;
}

static void remove_pending(struct stats *s, size_t count)
{
    s->pending_len -= count;
    memmove(s->pending, s->pending + count, s->pending_len);
}

static void parse_pending(struct stats *s)
{
    while (s->pending_len >= 2) {
        if (s->pending[0] != 0xFA || s->pending[1] != 0xAF) {
            s->noise_bytes++;
            remove_pending(s, 1);
            continue;
        }
        if (s->pending_len < 8)
            return;
        if (memcmp(s->pending, query, sizeof(query)) == 0) {
            s->echoes++;
            remove_pending(s, sizeof(query));
            continue;
        }
        if (s->pending_len < 9)
            return;
        if (s->pending[8] != 0xED || s->pending[2] != SERVO_ID) {
            s->other_frames++;
            if (s->other_frames <= 3) {
                printf("Malformed frame candidate:");
                for (size_t i = 0; i < 9; ++i)
                    printf(" %02X", s->pending[i]);
                putchar('\n');
            }
            remove_pending(s, 1);
            continue;
        }

        uint8_t checksum = (uint8_t)(s->pending[2] + s->pending[3] +
                                     s->pending[4] + s->pending[5] +
                                     s->pending[6]);
        if (checksum != s->pending[7]) {
            s->checksum_errors++;
            if (s->checksum_errors <= 5) {
                printf("Bad checksum frame:");
                for (size_t i = 0; i < 9; ++i)
                    printf(" %02X", s->pending[i]);
                putchar('\n');
            }
            remove_pending(s, 9);
            continue;
        }

        /* STM32 protocol: ID, position high/low, signed speed high/low. */
        uint16_t position = ((uint16_t)s->pending[3] << 8) |
                             s->pending[4];
        int16_t speed = (int16_t)(((uint16_t)s->pending[5] << 8) |
                                  s->pending[6]);
        s->valid++;
        s->last_position = position;
        s->last_speed = speed;
        if (position < s->min_position)
            s->min_position = position;
        if (position > s->max_position)
            s->max_position = position;
        if (speed < s->min_speed)
            s->min_speed = speed;
        if (speed > s->max_speed)
            s->max_speed = speed;
        remove_pending(s, 9);
    }
}

static int read_available(int fd, struct stats *s, int probe)
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

        s->received_bytes += (uint64_t)n;
        if (probe) {
            printf("RX (%zd bytes):", n);
            for (ssize_t i = 0; i < n; ++i)
                printf(" %02X", buffer[i]);
            putchar('\n');
        }
        for (ssize_t i = 0; i < n; ++i) {
            if (s->pending_len == sizeof(s->pending)) {
                s->noise_bytes++;
                remove_pending(s, 1);
            }
            s->pending[s->pending_len++] = buffer[i];
            parse_pending(s);
        }
    }
}

static int wait_for_rx(int fd, uint64_t deadline_ns, struct stats *s,
                       int probe)
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
            if ((pfd.revents & POLLIN) && read_available(fd, s, probe) < 0)
                return -1;
        }
    }
    return 0;
}

int main(int argc, char **argv)
{
    int probe = argc == 2 && strcmp(argv[1], "--probe") == 0;
    unsigned seconds = DEFAULT_SECONDS;
    if (!probe && argc == 2) {
        char *end;
        errno = 0;
        unsigned long value = strtoul(argv[1], &end, 10);
        if (errno || end == argv[1] || *end || value < 1 || value > 60) {
            fprintf(stderr, "Usage: %s [seconds: 1..60 | --probe]\n", argv[0]);
            return 2;
        }
        seconds = (unsigned)value;
    } else if (argc > 2) {
        fprintf(stderr, "Usage: %s [seconds: 1..60 | --probe]\n", argv[0]);
        return 2;
    }

    struct sigaction action = {0};
    action.sa_handler = request_stop;
    sigemptyset(&action.sa_mask);
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);

    int fd = open_uart();
    if (fd < 0)
        return 1;

    struct stats s = {
        .min_position = UINT16_MAX,
        .min_speed = INT16_MAX,
        .max_speed = INT16_MIN
    };
    uint64_t start_ns = monotonic_ns();
    uint64_t total_queries = probe ? 1 : (uint64_t)seconds * QUERY_HZ;
    printf("TX frame:");
    for (size_t i = 0; i < sizeof(query); ++i)
        printf(" %02X", query[i]);
    printf("\nRate: %d Hz; requests: %" PRIu64 "\n", QUERY_HZ, total_queries);

    int result = 0;
    for (uint64_t i = 0; i < total_queries && !stop_requested; ++i) {
        uint64_t due = start_ns + i * 1000000000ULL / QUERY_HZ;
        if (wait_for_rx(fd, due, &s, probe) < 0) {
            result = 1;
            break;
        }
        if (stop_requested)
            break;
        if (monotonic_ns() > due + 1000000ULL)
            s.late_sends++;
        if (send_query(fd) < 0) {
            result = 1;
            break;
        }
        uint64_t sent_at = monotonic_ns();
        if (s.sent == 0)
            s.first_send_ns = sent_at;
        s.last_send_ns = sent_at;
        s.sent++;
    }

    if (!result && !stop_requested) {
        uint64_t end_ns = start_ns + total_queries * 1000000000ULL / QUERY_HZ;
        if (wait_for_rx(fd, end_ns + RX_GRACE_MS * 1000000ULL,
                        &s, probe) < 0)
            result = 1;
    }
    if (read_available(fd, &s, probe) < 0)
        result = 1;

    uint64_t unmatched = s.sent > s.valid ? s.sent - s.valid : 0;
    printf("Sent=%" PRIu64 ", checksum-valid replies=%" PRIu64
           ", unmatched=%" PRIu64 ", echoes=%" PRIu64 "\n",
           s.sent, s.valid, unmatched, s.echoes);
    printf("Checksum errors=%" PRIu64 ", other frames=%" PRIu64
           ", noise bytes=%" PRIu64 ", pending bytes=%zu\n",
           s.checksum_errors, s.other_frames, s.noise_bytes, s.pending_len);
    printf("Received bytes=%" PRIu64 ", sends >1 ms late=%" PRIu64 "\n",
           s.received_bytes, s.late_sends);
    if (s.sent > 1 && s.last_send_ns > s.first_send_ns)
        printf("Measured send rate=%.2f Hz\n",
               (double)(s.sent - 1) * 1000000000.0 /
               (double)(s.last_send_ns - s.first_send_ns));
    if (s.valid)
        printf("Position range=%u..%u (last=%u); "
               "speed range=%d..%d (last=%d)\n",
               s.min_position, s.max_position, s.last_position,
               s.min_speed, s.max_speed, s.last_speed);
    if (s.sent)
        printf("Checksum-valid reply ratio=%.3f%%\n",
               100.0 * (double)s.valid / (double)s.sent);

    if (!stop_requested &&
        (s.sent != s.valid || s.checksum_errors || s.other_frames || s.noise_bytes ||
         s.pending_len)) {
        printf("Frame integrity check: FAIL\n");
        result = 1;
    } else if (!stop_requested && !result) {
        printf("Frame integrity check: PASS\n");
    }

    close(fd);
    return result;
}

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
#define READ_GRACE_MS 100

static const uint8_t servo_ids[] = {
    1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16
};
#define SERVO_COUNT (sizeof(servo_ids) / sizeof(servo_ids[0]))
#define QUERY_HZ (CYCLES_PER_SECOND * SERVO_COUNT)

struct servo_state {
    uint64_t sent;
    uint64_t received;
    uint64_t shown_received;
    uint16_t position;
    int16_t speed;
};

struct receiver {
    struct servo_state servo[SERVO_COUNT];
    uint64_t echoes;
    uint64_t checksum_errors;
    uint64_t malformed;
    uint64_t noise_bytes;
    uint64_t received_bytes;
    uint64_t late_sends;
    uint64_t first_send_ns;
    uint64_t last_send_ns;
    uint8_t pending[16];
    size_t pending_len;
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

static int send_query(int fd, uint8_t id)
{
    uint8_t frame[8] = { 0xFA, 0xAF, id, 0x03, 0, 0,
                         (uint8_t)(id + 0x03), 0xED };
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
    return 0;
}

static void remove_pending(struct receiver *rx, size_t count)
{
    rx->pending_len -= count;
    memmove(rx->pending, rx->pending + count, rx->pending_len);
}

static void parse_pending(struct receiver *rx)
{
    while (rx->pending_len >= 2) {
        if (rx->pending[0] != 0xFA || rx->pending[1] != 0xAF) {
            rx->noise_bytes++;
            remove_pending(rx, 1);
            continue;
        }
        if (rx->pending_len < 8)
            return;

        int index = servo_index(rx->pending[2]);
        if (index < 0) {
            rx->malformed++;
            remove_pending(rx, 1);
            continue;
        }

        uint8_t expected_echo[8] = {
            0xFA, 0xAF, rx->pending[2], 0x03, 0, 0,
            (uint8_t)(rx->pending[2] + 0x03), 0xED
        };
        if (memcmp(rx->pending, expected_echo, sizeof(expected_echo)) == 0) {
            rx->echoes++;
            remove_pending(rx, sizeof(expected_echo));
            continue;
        }

        if (rx->pending_len < 9)
            return;
        if (rx->pending[8] != 0xED) {
            rx->malformed++;
            remove_pending(rx, 1);
            continue;
        }
        uint8_t checksum = (uint8_t)(rx->pending[2] + rx->pending[3] +
                                     rx->pending[4] + rx->pending[5] +
                                     rx->pending[6]);
        if (rx->pending[7] != checksum) {
            rx->checksum_errors++;
            remove_pending(rx, 1);
            continue;
        }

        struct servo_state *servo = &rx->servo[index];
        servo->position = ((uint16_t)rx->pending[3] << 8) | rx->pending[4];
        servo->speed = (int16_t)(((uint16_t)rx->pending[5] << 8) |
                                 rx->pending[6]);
        servo->received++;
        remove_pending(rx, 9);
    }
}

static int read_available(int fd, struct receiver *rx)
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

        rx->received_bytes += (uint64_t)n;
        for (ssize_t i = 0; i < n; ++i) {
            if (rx->pending_len == sizeof(rx->pending)) {
                rx->noise_bytes++;
                remove_pending(rx, 1);
            }
            rx->pending[rx->pending_len++] = buffer[i];
            parse_pending(rx);
        }
    }
}

static int wait_for_rx(int fd, uint64_t deadline_ns, struct receiver *rx)
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
            if ((pfd.revents & POLLIN) && read_available(fd, rx) < 0)
                return -1;
        }
    }
    return 0;
}

static void display(uint64_t cycles, struct receiver *rx)
{
    printf("\nAfter %" PRIu64 " cycles (%.2f s):\n",
           cycles, (double)cycles / CYCLES_PER_SECOND);
    printf(" ID  Position  Speed   Replies/new  Replies/total  Sent\n");
    for (size_t i = 0; i < SERVO_COUNT; ++i) {
        struct servo_state *s = &rx->servo[i];
        uint64_t recent = s->received - s->shown_received;
        if (recent) {
            printf("%3u  %8u  %5d   %11" PRIu64 "  %13" PRIu64
                   "  %4" PRIu64 "\n",
                   servo_ids[i], s->position, s->speed,
                   recent, s->received, s->sent);
        } else {
            printf("%3u  %8s  %5s   %11" PRIu64 "  %13" PRIu64
                   "  %4" PRIu64 "\n",
                   servo_ids[i], "--", "--", recent, s->received, s->sent);
        }
        s->shown_received = s->received;
    }
    printf("Echoes=%" PRIu64 ", checksum errors=%" PRIu64
           ", malformed=%" PRIu64 ", noise bytes=%" PRIu64 "\n",
           rx->echoes, rx->checksum_errors, rx->malformed, rx->noise_bytes);
    fflush(stdout);
}

int main(int argc, char **argv)
{
    unsigned long seconds = 0; /* 0: continuous until Ctrl+C. */
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
    struct receiver rx = {0};
    uint64_t start_ns = monotonic_ns();
    uint64_t total_queries = seconds ? (uint64_t)seconds * QUERY_HZ : UINT64_MAX;
    printf("UART %s at %d baud; %zu IDs, %d cycles/s, %zu queries/s.\n",
           UART_DEV, BAUDRATE, SERVO_COUNT, CYCLES_PER_SECOND, QUERY_HZ);
    printf("CMD 03 only; position and signed speed are raw protocol values.\n");
    fflush(stdout);

    int result = 0;
    uint64_t queries = 0;
    for (; queries < total_queries && !stop_requested; ++queries) {
        uint64_t due = start_ns + queries * 1000000000ULL / QUERY_HZ;
        if (wait_for_rx(fd, due, &rx) < 0) {
            result = 1;
            break;
        }
        if (stop_requested)
            break;

        size_t index = (size_t)(queries % SERVO_COUNT);
        if (monotonic_ns() > due + 1000000ULL)
            rx.late_sends++;
        if (send_query(fd, servo_ids[index]) < 0) {
            result = 1;
            break;
        }
        uint64_t sent_at = monotonic_ns();
        if (queries == 0)
            rx.first_send_ns = sent_at;
        rx.last_send_ns = sent_at;
        rx.servo[index].sent++;

        if ((queries + 1) % QUERY_HZ == 0) {
            if (read_available(fd, &rx) < 0) {
                result = 1;
                break;
            }
            display((queries + 1) / SERVO_COUNT, &rx);
        }
    }

    if (!result && !stop_requested && seconds) {
        uint64_t end_ns = start_ns + total_queries * 1000000000ULL / QUERY_HZ;
        if (wait_for_rx(fd, end_ns + READ_GRACE_MS * 1000000ULL,
                        &rx) < 0)
            result = 1;
    }
    if (read_available(fd, &rx) < 0)
        result = 1;

    uint64_t sent = 0;
    uint64_t received = 0;
    for (size_t i = 0; i < SERVO_COUNT; ++i) {
        sent += rx.servo[i].sent;
        received += rx.servo[i].received;
    }
    printf("\nFinal: sent=%" PRIu64 ", valid replies=%" PRIu64
           ", echoes=%" PRIu64 ", checksum errors=%" PRIu64
           ", malformed=%" PRIu64 ", noise bytes=%" PRIu64 "\n",
           sent, received, rx.echoes, rx.checksum_errors,
           rx.malformed, rx.noise_bytes);
    if (sent > 1 && rx.last_send_ns > rx.first_send_ns)
        printf("Measured send rate=%.2f packets/s; sends >1 ms late=%" PRIu64
               "\n", (double)(sent - 1) * 1000000000.0 /
               (double)(rx.last_send_ns - rx.first_send_ns), rx.late_sends);
    if (seconds && received == 0) {
        fprintf(stderr, "No position/speed replies from the requested IDs.\n");
        result = 1;
    }
    close(fd);
    return result;
}

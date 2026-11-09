#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <asm/termbits.h>
#include <asm/ioctls.h>
#include <sys/select.h>

#define UART_DEV        "/dev/ttyS3"
#define SERVO_ID        15
#define BAUDRATE        230460


/*
 * ============================================================
 * UART 初始化
 * ============================================================
 */

static int uart_init(void)
{
    int fd;

    fd = open(
        UART_DEV,
        O_RDWR | O_NOCTTY | O_NONBLOCK
    );

    if (fd < 0) {
        perror("open UART");
        return -1;
    }

    struct termios2 tty;

    if (ioctl(fd, TCGETS2, &tty) != 0) {
        perror("TCGETS2");
        close(fd);
        return -1;
    }

    tty.c_cflag &= ~(CBAUD | PARENB | CSTOPB | CSIZE);
    tty.c_cflag |= BOTHER | CS8;
    tty.c_ispeed = BAUDRATE;
    tty.c_ospeed = BAUDRATE;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (ioctl(fd, TCSETS2, &tty) != 0) {
        perror("TCSETS2");
        close(fd);
        return -1;
    }

    if (ioctl(fd, TCGETS2, &tty) != 0 ||
        tty.c_ispeed != BAUDRATE || tty.c_ospeed != BAUDRATE) {
        fprintf(stderr, "UART cannot set baud rate to %d\n", BAUDRATE);
        close(fd);
        return -1;
    }

    ioctl(fd, TCFLSH, TCIOFLUSH);

    return fd;
}


/*
 * ============================================================
 * 打印数据
 * ============================================================
 */

static void print_frame(
    const char *name,
    uint8_t *data,
    int len)
{
    printf("%s (%d bytes):", name, len);

    for (int i = 0; i < len; i++)
        printf(" %02X", data[i]);

    printf("\n");
}


/*
 * ============================================================
 * 8 字节协议校验和：ID + CMD/STATUS + DATA_H + DATA_L
 * ============================================================
 */

static uint8_t calc_checksum(const uint8_t *buf)
{
    return (uint8_t)(buf[2] + buf[3] + buf[4] + buf[5]);
}


/*
 * ============================================================
 * 直接通过串口发送数据，并等待 UART 完成发送。
 * ============================================================
 */

static int servo_send(
    int fd,
    const uint8_t *data,
    size_t len)
{
    size_t sent = 0;
    while (sent < len) {
        ssize_t ret = write(fd, data + sent, len - sent);
        if (ret > 0) {
            sent += (size_t)ret;
            continue;
        }
        if (ret < 0 && (errno == EINTR || errno == EAGAIN))
            continue;
        perror("UART write");
        return -1;
    }

    /*
     * 等待 UART 硬件真正发送完成
     */
    if (ioctl(fd, TCSBRK, 1) < 0) {
        perror("UART drain");
        return -1;
    }

    return 0;
}


/*
 * ============================================================
 * FA AF ID CMD DATA_H DATA_L CHECKSUM ED
 * CMD 01 使用目标位置；CMD 02/03 的数据必须为 0。
 * ============================================================
 */

static int servo_send_command(int fd, uint8_t id, uint8_t command,
                              uint16_t target_position)
{
    uint8_t frame[8];

    frame[0] = 0xFA;
    frame[1] = 0xAF;

    frame[2] = id;
    frame[3] = command;
    frame[4] = command == 0x01 ? (uint8_t)(target_position >> 8) : 0;
    frame[5] = command == 0x01 ? (uint8_t)target_position : 0;
    frame[6] = calc_checksum(frame);
    frame[7] = 0xED;

    print_frame("TX", frame, sizeof(frame));

    /*
     * 清除之前可能残留的数据
     */
    ioctl(fd, TCFLSH, TCIFLUSH);

    return servo_send(fd, frame, sizeof(frame));
}


/*
 * ============================================================
 * 读取角度
 *
 * FA AF ID 02 00 00 CHECKSUM ED
 * ============================================================
 */

static int servo_read_angle(int fd, uint16_t *position, int16_t *speed)
{
    if (servo_send_command(fd, SERVO_ID, 0x02, 0) < 0)
        return -1;

    /*
     * ========================================================
     * 连续监听 500ms
     *
     * 接收缓冲中可能同时包含查询命令的回显和舵机回复。
     * 收到什么就打印什么。
     * ========================================================
     */

    uint8_t rx[256];

    int rx_len = 0;

    int elapsed_ms = 0;

    while (elapsed_ms < 500 &&
           rx_len < (int)sizeof(rx)) {

        fd_set readfds;

        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);

        struct timeval tv;

        tv.tv_sec = 0;
        tv.tv_usec = 5000;   // 5ms

        int ret = select(
            fd + 1,
            &readfds,
            NULL,
            NULL,
            &tv
        );

        if (ret < 0) {

            if (errno == EINTR)
                continue;

            perror("select");
            break;
        }

        if (ret == 0) {

            elapsed_ms += 5;
            continue;
        }

        if (FD_ISSET(fd, &readfds)) {

            int n = read(
                fd,
                rx + rx_len,
                sizeof(rx) - rx_len
            );

            if (n > 0) {

                rx_len += n;

                /*
                 * 等一下，允许后续字节进入
                 */
                usleep(1000);
            }
        }
    }


    /*
     * ========================================================
     * 打印所有收到的数据
     * ========================================================
     */

    if (rx_len == 0) {

        printf("READ RX: no response\n");

        return -2;
    }

    print_frame("READ RX", rx, rx_len);


    /*
     * ========================================================
     * 回复为 9 字节：FA AF ID POS_H POS_L SPEED_H SPEED_L CS ED。
     * 校验和为 ID 到 SPEED_L 的字节和；8 字节查询回显会被跳过。
     * ========================================================
     */

    for (int i = 0; i <= rx_len - 9; i++) {

        if (rx[i] != 0xFA)
            continue;

        if (rx[i + 1] != 0xAF)
            continue;

        if (rx[i + 2] != SERVO_ID)
            continue;

        if (rx[i + 8] != 0xED)
            continue;


        /*
         * 校验
         */

        uint8_t cs = (uint8_t)(rx[i + 2] + rx[i + 3] +
                               rx[i + 4] + rx[i + 5] + rx[i + 6]);

        if (cs != rx[i + 7]) {

            printf(
                "Checksum ERROR: calc=%02X recv=%02X\n",
                cs,
                rx[i + 7]
            );

            continue;
        }


        *position = ((uint16_t)rx[i + 3] << 8) | rx[i + 4];
        *speed = (int16_t)(((uint16_t)rx[i + 5] << 8) | rx[i + 6]);


        printf("\n");
        printf("========================================\n");
        printf(" Servo ID      : %d\n", SERVO_ID);
        printf(" Position      : %u\n", *position);
        printf(" Speed         : %d\n", *speed);
        printf("========================================\n");

        return 0;
    }


    printf(
        "Received data, but no valid UBT response frame found.\n"
    );

    return -4;
}


/*
 * ============================================================
 * MAIN
 * ============================================================
 */

int main(void)
{
    printf("\n");
    printf("========================================\n");
    printf(" UBT-12HC MOVE + ANGLE READ TEST\n");
    printf("========================================\n");

    printf("UART      : %s\n", UART_DEV);
    printf("Baud      : %d 8N1\n", BAUDRATE);
    printf("Servo ID  : %d\n", SERVO_ID);

    printf("========================================\n");


    /*
     * UART
     */

    int fd = uart_init();

    if (fd < 0) {

        printf("UART initialization FAILED\n");

        return 1;
    }

    printf("UART opened successfully.\n");


    /*
     * ========================================================
     * 第一步：动作
     *
     * ID 6
     * 目标位置 120
     * ========================================================
     */

    printf("\n");
    printf(">>> STEP 1: MOVE SERVO\n");

    if (servo_send_command(fd, SERVO_ID, 0x01, 550) < 0) {

        printf("Move command FAILED\n");

        close(fd);
        return 1;
    }

    printf("Move command sent.\n");


    /*
     * 等待舵机移动；新协议不含动作时间字段。
     */

    usleep(1100000);


    /*
     * ========================================================
     * 第二步：读取角度
     * ========================================================
     */

    printf("\n");
    printf(">>> STEP 2: READ SERVO ANGLE\n");

    uint16_t position = 0;
    int16_t speed = 0;
    int ret = servo_read_angle(fd, &position, &speed);


    /*
     * ========================================================
     * 结果
     * ========================================================
     */

    if (ret == 0) {

        printf("\n");
        printf("READ SUCCESS!\n");
        printf("Position = %u\n", position);
        printf("Speed = %d\n", speed);

    } else {

        printf("\n");
        printf("READ FAILED, error = %d\n", ret);
    }


    close(fd);

    printf("\nTest finished.\n");

    return ret == 0 ? 0 : 1;
}

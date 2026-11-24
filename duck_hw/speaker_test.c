#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static volatile sig_atomic_t running = 1;
static void stop(int sig) { (void)sig; running = 0; }

int main(int argc, char **argv) {
    double seconds = 5.0, frequency = 880.0;
    if (argc > 1) seconds = strtod(argv[1], NULL);
    if (argc > 2) frequency = strtod(argv[2], NULL);
    if (seconds <= 0 || seconds > 3600 || frequency < 100 || frequency > 4000) {
        fprintf(stderr, "用法: %s [时长秒数(默认5)] [频率Hz(默认880)]\n", argv[0]);
        return 2;
    }
    if (system("command -v aplay >/dev/null 2>&1") != 0) {
        fprintf(stderr, "未找到 aplay。请安装 alsa-utils，或使用系统自带播放工具。\n");
        return 1;
    }
    signal(SIGINT, stop);
    signal(SIGTERM, stop);
    fprintf(stderr, "播放 %.0f Hz 测试音 %.1f 秒；按 Ctrl+C 停止。\n", frequency, seconds);
    char command[128];
    snprintf(command, sizeof(command), "aplay -q -t raw -f S16_LE -r 48000 -c 2");
    FILE *player = popen(command, "w");
    if (!player) { perror("启动 aplay"); return 1; }
    const unsigned rate = 48000;
    long frames = (long)(seconds * rate);
    for (long i = 0; i < frames && running; ++i) {
        double envelope = fmin(1.0, fmin((double)i / 1200.0, (double)(frames-i) / 1200.0));
        int16_t sample = (int16_t)(8000.0 * envelope * sin(2.0 * M_PI * frequency * i / rate));
        int16_t stereo[2] = {sample, sample};
        if (fwrite(stereo, sizeof(stereo), 1, player) != 1) break;
    }
    int status = pclose(player);
    if (status != 0) { fprintf(stderr, "aplay 播放失败（状态 %d）；检查声卡/音量设置。\n", status); return 1; }
    puts("测试音播放结束。");
    return 0;
}

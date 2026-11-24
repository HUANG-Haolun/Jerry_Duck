#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#ifndef DUCK_WAV_DIR
#define DUCK_WAV_DIR "duck_sounds"
#endif

int main(int argc, char **argv) {
    if (argc != 2) {
        fprintf(stderr, "用法: %s <wav文件名>\n例如: %s chirp_a.wav\n", argv[0], argv[0]);
        return 2;
    }
    const char *name = argv[1];
    if (strchr(name, '/') || strchr(name, '\\') || strstr(name, "..") ||
        strlen(name) < 5 || strcmp(name + strlen(name) - 4, ".wav") != 0) {
        fprintf(stderr, "参数必须是 duck_sounds 中的 WAV 文件名（仅支持 .wav）。\n");
        return 2;
    }
    const char *separator = strchr(name, '_');
    size_t tag_len = separator ? (size_t)(separator - name) : strlen(name) - 4;
    if (tag_len == 0 || tag_len >= 32) {
        fprintf(stderr, "无法从 WAV 文件名确定音效目录。\n");
        return 2;
    }
    char tag[32];
    memcpy(tag, name, tag_len);
    tag[tag_len] = '\0';
    char path[512];
    int n = snprintf(path, sizeof(path), "%s/%s/%s", DUCK_WAV_DIR, tag, name);
    if (n < 0 || (size_t)n >= sizeof(path)) {
        fprintf(stderr, "WAV 文件路径过长。\n");
        return 2;
    }
    execlp("aplay", "aplay", "-q", path, (char *)NULL);
    fprintf(stderr, "无法启动 aplay: %s（请安装 alsa-utils）\n", strerror(errno));
    return 1;
}

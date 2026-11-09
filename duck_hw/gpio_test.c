#include <stdio.h>
#include <unistd.h>
#include <gpiod.h>

int main(void)
{
    struct gpiod_chip *chip;
    struct gpiod_line *line;

    chip = gpiod_chip_open("/dev/gpiochip3");
    if (!chip) {
        perror("gpiod_chip_open");
        return 1;
    }

    // GPIO3_A7 = line 7
    line = gpiod_chip_get_line(chip, 8);
    if (!line) {
        perror("gpiod_chip_get_line");
        return 1;
    }

    // 设置为输出，初始 LOW
    if (gpiod_line_request_output(line, "gpio_test", 0) < 0) {
        perror("gpiod_line_request_output");
        return 1;
    }

    printf("GPIO3_A7 test start\n");

    while (1)
    {
        gpiod_line_set_value(line, 1);
        printf("GPIO3_A7 = HIGH\n");
        fflush(stdout);

        sleep(1);

        gpiod_line_set_value(line, 0);
        printf("GPIO3_A7 = LOW\n");
        fflush(stdout);

        sleep(1);
    }

    gpiod_line_release(line);
    gpiod_chip_close(chip);

    return 0;
}

#define main board_diagnostic_main
#include "icm42688_test.c"
#undef main

int open_icm(void) {
    if (spi_init() < 0 || icm_check_id() < 0 || icm_reset() < 0 ||
        icm_check_id() < 0 || icm_config() < 0) return -1;
    uint8_t accel, gyro;
    if (icm_read_reg(REG_ACCEL_CONFIG0, &accel) < 0 ||
        icm_read_reg(REG_GYRO_CONFIG0, &gyro) < 0) return -1;
    return (accel == 0x27 && gyro == 0x27) ? 0 : -1;
}

int sample_icm(int16_t *data) {
    int16_t temp;
    return icm_read_sensor(data, data+1, data+2, data+3, data+4, data+5, &temp);
}

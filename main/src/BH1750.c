#include "driver/i2c.h"

#define BH1750_POWER_ON       0x01
#define BH1750_POWER_DOWN     0x00

int bh1750_power_on(i2c_port_t i2c_num, uint8_t addr) {
    uint8_t cmd = BH1750_POWER_ON;
    return i2c_master_write_to_device(i2c_num, addr, &cmd, 1, 1000 / portTICK_PERIOD_MS);
}

int bh1750_power_off(i2c_port_t i2c_num, uint8_t addr) {
    uint8_t cmd = BH1750_POWER_DOWN;
    return i2c_master_write_to_device(i2c_num, addr, &cmd, 1, 1000 / portTICK_PERIOD_MS);
}

int bh1750_set_mode(i2c_port_t i2c_num, uint8_t addr, uint8_t mode) {
    return i2c_master_write_to_device(i2c_num, addr, &mode, 1, 1000 / portTICK_PERIOD_MS);
}

int bh1750_read_light(i2c_port_t i2c_num, uint8_t addr, float *lux) {
    uint8_t data[2];
    int ret = i2c_master_read_from_device(i2c_num, addr, data, 2, 1000 / portTICK_PERIOD_MS);
    if (ret != 0) return ret;
    uint16_t raw = ((uint16_t)data[0] << 8) | data[1];
    *lux = (float)raw / 1.2f;
    return 0;
}

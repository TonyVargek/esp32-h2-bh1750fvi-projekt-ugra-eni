#ifndef _BH1750_H_
#define _BH1750_H_

#include "driver/i2c.h"

typedef enum {
    CONT_HIGH_RES_MODE = 0x10,
    CONT_LOW_RES_MODE = 0x13,
    ONE_TIME_HIGH_RES_MODE = 0x20,
    ONE_TIME_LOW_RES_MODE = 0x23
} bh1750_mode_t;

int bh1750_power_on(i2c_port_t i2c_num, uint8_t addr);
int bh1750_set_mode(i2c_port_t i2c_num, uint8_t addr, bh1750_mode_t mode);
int bh1750_read_light(i2c_port_t i2c_num, uint8_t addr, float *lux);
int bh1750_power_off(i2c_port_t i2c_num, uint8_t addr);

#endif

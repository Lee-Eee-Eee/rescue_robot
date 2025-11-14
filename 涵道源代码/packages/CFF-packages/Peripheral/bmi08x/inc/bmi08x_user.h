#ifndef __BMI08X_USER_H__
#define __BMI08X_USER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "bmi08x_defs.h"
#include "bmi08x.h"

typedef struct bmi08x_dev* bmi08x_dev_t;

int8_t bmi08x_interface_init(bmi08x_dev_t bmi08x, uint8_t intf, uint8_t variant);
void bmi08x_error_codes_print_result(const char api_name[], int8_t rslt);
int8_t bmi08x_enable_interrupt(bmi08x_dev_t bmi08x);
int8_t bmi08x_disable_interrupt(bmi08x_dev_t bmi08x);
int8_t bmi08x_enable_data_synchronization_interrupt(bmi08x_dev_t bmi08x);
int8_t bmi08x_disable_data_synchronization_interrupt(bmi08x_dev_t bmi08x);
int8_t bmi08x_init(bmi08x_dev_t bmi08x);

float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width);
float lsb_to_G(int16_t val, int8_t g_range, uint8_t bit_width);
float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);

#ifdef __cplusplus
}
#endif

#endif // __BMI08X_USER_H__
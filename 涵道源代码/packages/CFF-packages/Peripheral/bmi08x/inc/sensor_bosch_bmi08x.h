#ifndef __SENSOR_BOSCH_BMI08X_H__
#define __SENSOR_BOSCH_BMI08X_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "bmi08x_defs.h"
#include "bmi08x_user.h"
    
int rt_hw_bmi08x_init(const char *name, struct bmi08x_dev *bmi08x);

#ifdef __cplusplus
}
#endif

#endif // __SENSOR_BOSCH_BMI08X_H__
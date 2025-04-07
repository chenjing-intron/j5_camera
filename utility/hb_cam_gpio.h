/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef __HB_CAM_GPIO_H__
#define __HB_CAM_GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

int32_t vin_gpio_export(uint32_t gpio);
int32_t vin_gpio_unexport(uint32_t gpio);
int32_t vin_gpio_set_dir(uint32_t gpio, uint32_t out_flag);
int32_t vin_gpio_set_value(uint32_t gpio, uint32_t value);
int32_t vin_gpio_get_value(uint32_t gpio, uint32_t *value);
int32_t vin_gpio_set_edge(uint32_t gpio, const char *edge);

#ifdef __cplusplus
}
#endif

#endif

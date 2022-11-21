#ifndef CST816_H
#define CST816_H

#include <driver/i2c.h>
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

void cst816_init(void);
bool cst816_touch_read(lv_indev_drv_t * drv, lv_indev_data_t * data);
#endif
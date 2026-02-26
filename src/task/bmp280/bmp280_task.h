#pragma once
#ifndef __BMP280_TASK_H__
#define __BMP280_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#include "errcode.h"
#include "osal_task.h"

extern osal_task *g_bmp280_task_handle;

float bmp280_task_read_pressure(void);
errcode_t bmp280_task_entry(void);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif //__BMP280_TASK_H__
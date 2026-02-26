#pragma once
#ifndef __AHT20_TASK_H__
#define __AHT20_TASK_H__
#ifdef __cplusplus
extern "C"
{
#endif //__cplusplus

#include <stdint.h>

#include "errcode.h"

    extern osal_task *g_aht20_task_handle;

    void aht20_read_temperature_and_humidity(float *out_t, uint8_t *out_h);
    errcode_t
    aht20_task_entry(void);

#ifdef __cplusplus
}
#endif //__cplusplus
#endif //__AHT20_TASK_H__
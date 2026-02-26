#pragma once
#ifndef __SH1106_TASK_H__
#define __SH1106_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#include "errcode.h"
#include "osal_task.h"

extern osal_task *g_sh1106_tash_handle;

errcode_t sh1106_task_entry(void);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif //__SH1106_TASK_H__
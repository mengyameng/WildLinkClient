#pragma once
#ifndef __SLE_CLIENT_TASK_H__
#define __SLE_CLIENT_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#include "errcode.h"
#include "osal_task.h"

extern osal_task *g_sle_client_task_handle;

errcode_t sle_client_task_entry(void);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif //__SLE_CLIENT_TASK_H__
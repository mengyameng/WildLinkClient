#pragma once
#ifndef __BLE_SERVER_TASK_H__
#define __BLE_SERVER_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#include "errcode.h"
#include "osal_task.h"

extern osal_task *g_ble_server_task_handle;

errcode_t ble_server_task_entry(void);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif //__BLE_SERVER_TASK_H__

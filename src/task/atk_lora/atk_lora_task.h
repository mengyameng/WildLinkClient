#pragma once
#ifndef __ATK_LORA_TASK_H__
#define __ATK_LORA_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#include "errcode.h"
#include "osal_task.h"

extern osal_task *g_atk_lora_task_handle;

errcode_t atk_lora_task_entry(void);
errcode_t atk_lora_task_send_msg(uint16_t dest_addr, const void *data, size_t data_len);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif //__ATK_LORA_TASK_H__
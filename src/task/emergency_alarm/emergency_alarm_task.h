#pragma once
#ifndef __EMERGENCY_ALARM_TASK_H__
#define __EMERGENCY_ALARM_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#include "errcode.h"
#include "osal_task.h"

extern osal_task *g_emergency_alarm_task_handle;

void emergency_alarm_task_trigger_sos(void);
void emergency_alarm_task_trigger_buzzer_beep_once(void);
void emergency_alarm_task_trigger_led(void);
void emergency_alarm_task_trigger_alarm(void);

errcode_t emergency_alarm_task_entry(void);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif //__EMERGENCY_ALARM_TASK_H__
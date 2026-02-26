#pragma once
#ifndef __WLID_LINK_CLIENT_LOG_CONFIG_H__
#define __WLID_LINK_CLIENT_LOG_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#include "wlid_link_client_log_levels.h"

#define WLID_LINK_CLIENT_CURRENT_LOG_LEVEL CONFIG_WLID_LINK_CLIENT_LOG_LEVEL
#define WLID_LINK_CLIENT_LOG_PRINT(fmt, ...) osal_printk((fmt), ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // __WLID_LINK_CLIENT_LOG_CONFIG_H__
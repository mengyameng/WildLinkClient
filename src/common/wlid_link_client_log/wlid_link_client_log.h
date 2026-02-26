#pragma once
#ifndef __WLID_LINK_CLIENT_LOG_H__
#define __WLID_LINK_CLIENT_LOG_H__

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#include "wlid_link_client_log_levels.h"

#ifdef __WLID_LINK_CLIENT_STRINGIFY
#undef __WLID_LINK_CLIENT_STRINGIFY
#endif //__WLID_LINK_CLIENT_STRINGIFY

#ifdef __WLID_LINK_CLIENT_STRINGIFY_1
#undef __WLID_LINK_CLIENT_STRINGIFY_1
#endif //__WLID_LINK_CLIENT_STRINGIFY_1

#define __WLID_LINK_CLIENT_STRINGIFY(x) #x
#define __WLID_LINK_CLIENT_STRINGIFY_1(x) __WLID_LINK_CLIENT_STRINGIFY(x)

#ifndef WLID_LINK_CLIENT_IGNORE_USER_CONFIG
#include "wlid_link_client_log_config.h"
#endif // WLID_LINK_CLIENT_IGNORE_USER_CONFIG

#ifndef WLID_LINK_CLIENT_LOG_PRINT
#define WLID_LINK_CLIENT_LOG_PRINT(fmt, ...)
#warning "WLID_LINK_CLIENT_LOG_PRINT not defined! Log will not be printed!"
#endif // WLID_LINK_CLIENT_LOG_PRINT

#ifndef WLID_LINK_CLIENT_CURRENT_LOG_LEVEL
#define WLID_LINK_CLIENT_CURRENT_LOG_LEVEL WLID_LINK_CLIENT_LOG_LEVEL_NONE
#endif // WLID_LINK_CLIENT_CURRENT_LOG_LEVEL

#if WLID_LINK_CLIENT_CURRENT_LOG_LEVEL == WLID_LINK_CLIENT_LOG_LEVEL_NONE
#define WLID_LINK_CLIENT_LOG_PRINT_DEBUG(fmt, ...)
#define WLID_LINK_CLIENT_LOG_DEBUG(fmt, ...)
#define WLID_LINK_CLIENT_LOG_PRINT_INFO(fmt, ...)
#define WLID_LINK_CLIENT_LOG_INFO(fmt, ...)
#define WLID_LINK_CLIENT_LOG_PRINT_WARN(fmt, ...)
#define WLID_LINK_CLIENT_LOG_WARN(fmt, ...)
#define WLID_LINK_CLIENT_LOG_PRINT_ERROR(fmt, ...)
#define WLID_LINK_CLIENT_LOG_ERROR(fmt, ...)
#else
#if WLID_LINK_CLIENT_CURRENT_LOG_LEVEL <= WLID_LINK_CLIENT_LOG_LEVEL_DEBUG
#define WLID_LINK_CLIENT_LOG_PRINT_DEBUG(fmt, ...)                                     \
    WLID_LINK_CLIENT_LOG_PRINT((fmt), ##__VA_ARGS__)
#define WLID_LINK_CLIENT_LOG_DEBUG(fmt, ...)                                           \
    WLID_LINK_CLIENT_LOG_PRINT_DEBUG(                                                  \
        ("[WLC DEBUG] %s:" __WLID_LINK_CLIENT_STRINGIFY_1(__LINE__) ": " fmt),         \
        __func__, ##__VA_ARGS__)
#else
#define WLID_LINK_CLIENT_LOG_PRINT_DEBUG(fmt, ...)
#define WLID_LINK_CLIENT_LOG_DEBUG(fmt, ...)
#endif // WLID_LINK_CLIENT_CURRENT_LOG_LEVEL <= WLID_LINK_CLIENT_LOG_LEVEL_DEBUG

#if WLID_LINK_CLIENT_CURRENT_LOG_LEVEL <= WLID_LINK_CLIENT_LOG_LEVEL_INFO
#define WLID_LINK_CLIENT_LOG_PRINT_INFO(fmt, ...)                                      \
    WLID_LINK_CLIENT_LOG_PRINT((fmt), ##__VA_ARGS__)
#define WLID_LINK_CLIENT_LOG_INFO(fmt, ...)                                            \
    WLID_LINK_CLIENT_LOG_PRINT_INFO(                                                   \
        ("[WLC INFO] %s:" __WLID_LINK_CLIENT_STRINGIFY_1(__LINE__) ": " fmt),          \
        __func__, ##__VA_ARGS__)
#else
#define WLID_LINK_CLIENT_LOG_PRINT_INFO(fmt, ...)
#define WLID_LINK_CLIENT_LOG_INFO(fmt, ...)
#endif // WLID_LINK_CLIENT_CURRENT_LOG_LEVEL <= WLID_LINK_CLIENT_LOG_LEVEL_INFO

#if WLID_LINK_CLIENT_CURRENT_LOG_LEVEL <= WLID_LINK_CLIENT_LOG_LEVEL_WARN
#define WLID_LINK_CLIENT_LOG_PRINT_WARN(fmt, ...)                                      \
    WLID_LINK_CLIENT_LOG_PRINT((fmt), ##__VA_ARGS__)
#define WLID_LINK_CLIENT_LOG_WARN(fmt, ...)                                            \
    WLID_LINK_CLIENT_LOG_PRINT_WARN(                                                   \
        ("[WLC WARN] %s:" __WLID_LINK_CLIENT_STRINGIFY_1(__LINE__) ": " fmt),          \
        __func__, ##__VA_ARGS__)
#else
#define WLID_LINK_CLIENT_LOG_PRINT_WARN(fmt, ...)
#define WLID_LINK_CLIENT_LOG_WARN(fmt, ...)
#endif // WLID_LINK_CLIENT_CURRENT_LOG_LEVEL <= WLID_LINK_CLIENT_LOG_LEVEL_WARN

#if WLID_LINK_CLIENT_CURRENT_LOG_LEVEL <= WLID_LINK_CLIENT_LOG_LEVEL_ERROR
#define WLID_LINK_CLIENT_LOG_PRINT_ERROR(fmt, ...)                                     \
    WLID_LINK_CLIENT_LOG_PRINT((fmt), ##__VA_ARGS__)
#define WLID_LINK_CLIENT_LOG_ERROR(fmt, ...)                                           \
    WLID_LINK_CLIENT_LOG_PRINT_ERROR(                                                  \
        ("[WLC ERROR] %s:" __WLID_LINK_CLIENT_STRINGIFY_1(__LINE__) ": " fmt),         \
        __func__, ##__VA_ARGS__)
#else
#define WLID_LINK_CLIENT_LOG_PRINT_ERROR(fmt, ...)
#define WLID_LINK_CLIENT_LOG_ERROR(fmt, ...)
#endif // WLID_LINK_CLIENT_CURRENT_LOG_LEVEL <= WLID_LINK_CLIENT_LOG_LEVEL_ERROR

#endif // WLID_LINK_CLIENT_CURRENT_LOG_LEVEL != WLID_LINK_CLIENT_LOG_LEVEL_NONE

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // __WLID_LINK_CLIENT_LOG_H__
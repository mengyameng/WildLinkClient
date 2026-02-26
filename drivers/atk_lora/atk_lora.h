#pragma once
#ifndef __ATK_LORA_H__
#define __ATK_LORA_H__

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#include <stdint.h>
#include <stdio.h>
#include <string.h> //for memset

typedef struct atk_lora_handle_s {
    uint8_t (*uart_init)(void);
    uint8_t (*uart_deinit)(void);
    uint8_t (*uart_write)(const uint8_t *buffer, uint16_t len);
    uint8_t (*uart_read)(uint8_t *buffer, uint16_t len);
    uint8_t (*md0_write)(uint8_t v);
    void (*delay_ms)(uint32_t ms);
    void (*debug_print)(const char *fmt, ...);
    uint8_t _initialized;
} atk_lora_handle_t;

typedef enum {
    ATK_LORA_UART_RATE_1200 = 0,  // 1200bps
    ATK_LORA_UART_RATE_2400 = 1,  // 2400bps
    ATK_LORA_UART_RATE_4800 = 2,  // 4800bps
    ATK_LORA_UART_RATE_9600 = 3,  // 9600bps（默认）
    ATK_LORA_UART_RATE_19200 = 4, // 19200bps
    ATK_LORA_UART_RATE_38400 = 5, // 38400bps
    ATK_LORA_UART_RATE_57600 = 6, // 57600bps
    ATK_LORA_UART_RATE_115200 = 7 // 115200bps
} atk_lora_uart_rate_t;

typedef enum {
    ATK_LORA_PARITY_NONE = 0, // 无校验（默认）
    ATK_LORA_PARITY_EVEN = 1, // 偶校验
    ATK_LORA_PARITY_ODD = 2   // 奇校验
} atk_lora_parity_t;

typedef enum {
    ATK_LORA_CHANNEL_0 = 0,
    ATK_LORA_CHANNEL_1 = 1,
    ATK_LORA_CHANNEL_2 = 2,
    ATK_LORA_CHANNEL_3 = 3,
    ATK_LORA_CHANNEL_4 = 4,
    ATK_LORA_CHANNEL_5 = 5,
    ATK_LORA_CHANNEL_6 = 6,
    ATK_LORA_CHANNEL_7 = 7,
    ATK_LORA_CHANNEL_8 = 8,
    ATK_LORA_CHANNEL_9 = 9,
    ATK_LORA_CHANNEL_10 = 10,
    ATK_LORA_CHANNEL_11 = 11,
    ATK_LORA_CHANNEL_12 = 12,
    ATK_LORA_CHANNEL_13 = 13,
    ATK_LORA_CHANNEL_14 = 14,
    ATK_LORA_CHANNEL_15 = 15,
    ATK_LORA_CHANNEL_16 = 16,
    ATK_LORA_CHANNEL_17 = 17,
    ATK_LORA_CHANNEL_18 = 18,
    ATK_LORA_CHANNEL_19 = 19,
    ATK_LORA_CHANNEL_20 = 20,
    ATK_LORA_CHANNEL_21 = 21,
    ATK_LORA_CHANNEL_22 = 22,
    ATK_LORA_CHANNEL_23 = 23,
    ATK_LORA_CHANNEL_24 = 24,
    ATK_LORA_CHANNEL_25 = 25,
    ATK_LORA_CHANNEL_26 = 26,
    ATK_LORA_CHANNEL_27 = 27,
    ATK_LORA_CHANNEL_28 = 28,
    ATK_LORA_CHANNEL_29 = 29,
    ATK_LORA_CHANNEL_30 = 30,
    ATK_LORA_CHANNEL_31 = 31
} atk_lora_channel_t;

typedef enum {
    ATK_LORA_AIR_RATE_0_3K = 0, // 0.3Kbps
    ATK_LORA_AIR_RATE_1_2K = 1, // 1.2Kbps
    ATK_LORA_AIR_RATE_2_4K = 2, // 2.4Kbps
    ATK_LORA_AIR_RATE_4_8K = 3, // 4.8Kbps
    ATK_LORA_AIR_RATE_9_6K = 4, // 9.6Kbps
    ATK_LORA_AIR_RATE_19_2K = 5 // 19.2Kbps（默认）
} atk_lora_air_rate_t;

typedef enum {
    ATK_LORA_SLEEP_TIME_1S = 0, // 1秒（默认）
    ATK_LORA_SLEEP_TIME_2S = 1  // 2秒
} atk_lora_sleep_time_t;

typedef enum {
    ATK_LORA_TX_POWER_11DBM = 0, // 11dBm
    ATK_LORA_TX_POWER_14DBM = 1, // 14dBm
    ATK_LORA_TX_POWER_17DBM = 2, // 17dBm
    ATK_LORA_TX_POWER_20DBM = 3  // 20dBm（默认）
} atk_lora_tx_power_t;

typedef enum {
    ATK_LORA_WORK_MODE_NORMAL = 0,         // 一般模式（默认）
    ATK_LORA_WORK_MODE_WAKUP = 1,          // 唤醒模式
    ATK_LORA_WORK_MODE_POWER_SAVE = 2,     // 省电模式
    ATK_LORA_WORK_MODE_SIGNAL_STRENGTH = 3 // 信号强度模式
} atk_lora_work_mode_t;

typedef enum {
    ATK_LORA_TX_MODE_TRANSPARENT = 0, // 透明传输（默认）
    ATK_LORA_TX_MODE_DIRECTED = 1     // 定向传输
} atk_lora_tx_mode_t;

#define ATK_LORA_LINK_INIT(pHANDLE, STRUCT) memset(pHANDLE, 0, sizeof(STRUCT))
#define ATK_LORA_LINK_INIT_2(pHANDLE, STRUCT) memset(pHANDLE, 0, sizeof(*(pHANDLE)))
#define ATK_LORA_LINK_UART_INIT(pHANDLE, FUNC) ((pHANDLE)->uart_init = FUNC)
#define ATK_LORA_LINK_UART_DEINIT(pHANDLE, FUNC) ((pHANDLE)->uart_deinit = FUNC)
#define ATK_LORA_LINK_UART_WRITE(pHANDLE, FUNC) ((pHANDLE)->uart_write = FUNC)
#define ATK_LORA_LINK_UART_READ(pHANDLE, FUNC) ((pHANDLE)->uart_read = FUNC)
#define ATK_LORA_LINK_MD0_WRITE(pHANDLE, FUNC) ((pHANDLE)->md0_write = FUNC)
#define ATK_LORA_LINK_DELAY_MS(pHANDLE, FUNC) ((pHANDLE)->delay_ms = FUNC)
#define ATK_LORA_LINK_DEBUG_PRINT(pHANDLE, FUNC) ((pHANDLE)->debug_print = FUNC)

#define ATK_LORA_ERR_NONE 0
#define ATK_LORA_ERR_FAILED 1
#define ATK_LORA_ERR_HANDLE_IS_NULL 2
#define ATK_LORA_ERR_MEMBER_FUNC_IS_NULL 3
#define ATK_LORA_ERR_NOT_INITIALIZED 4
#define ATK_LORA_ERR_SEND_AT_COMMAND 5
#define ATK_LORA_ERR_RECV_AT_COMMAND 6
#define ATK_LORA_ERR_AT_COMMAND_FAILED 7
#define ATK_LORA_ERR_MD0_OPERATION 8
#define ATK_LORA_ERR_INVALID_PARAMS 9
#define ATK_LORA_ERR_PARSE_AT_CONFIG 10

uint8_t atk_lora_init(atk_lora_handle_t *handle);
uint8_t atk_lora_deinit(atk_lora_handle_t *handle);

uint8_t atk_lora_send_buf(atk_lora_handle_t *handle, const void *buf, size_t buf_size);
uint8_t atk_lora_send_at_cmd(atk_lora_handle_t *handle, const char *at_cmd);
uint8_t atk_lora_send_at_cmd_fmt(atk_lora_handle_t *handle, const char *at_fmt, ...);
uint8_t atk_lora_send_at_cmd_buf(atk_lora_handle_t *handle, char *at_buf,
                                 size_t at_buf_size, const char *at_fmt, ...);
uint8_t atk_lora_read_at_resp(atk_lora_handle_t *handle);
uint8_t atk_lora_read_at_resp_buf(atk_lora_handle_t *handle, char *resp_buf,
                                  size_t resp_buf_size);
uint8_t atk_lora_check_at_resp(atk_lora_handle_t *handle, const char *expected_resp);
uint8_t atk_lora_check_at_resp_buf(atk_lora_handle_t *handle, const char *resp_buf,
                                   const char *expected_resp);

uint8_t _atk_lora_parse_at_resp(atk_lora_handle_t *handle, const char *parse_fmt, ...);
uint8_t _atk_lora_parse_at_resp_buf(atk_lora_handle_t *handle, const char *resp_buf,
                                    const char *parse_fmt, ...);
uint8_t _atk_lora_vparse_at_resp_buf(atk_lora_handle_t *handle, const char *resp_buf,
                                     const char *parse_fmt, va_list parse_fmt_args);
#define atk_lora_parse_at_resp(pHANDLE, PARSE_FMT, ...)                                \
    _atk_lora_parse_at_resp((pHANDLE), ("%*[^+]%*[^:]%*c" PARSE_FMT), ##__VA_ARGS__)
#define atk_lora_parse_at_resp_buf(pHANDLE, RESP_BUF, PARSE_FMT, ...)                  \
    _atk_lora_parse_at_resp_buf((pHANDLE), (RESP_BUF), ("%*[^+]%*[^:]%*c" PARSE_FMT),  \
                                ##__VA_ARGS__)

uint8_t atk_lora_set_communicate(atk_lora_handle_t *handle, uint8_t state);
#define atk_lora_start_communicate(pHANDLE) atk_lora_set_communicate(pHANDLE, 0)
#define atk_lora_end_communicate(pHANDLE) atk_lora_set_communicate(pHANDLE, 1)

#ifdef __cplusplus
}
#endif //__cplusplus

#endif //__ATK_LORA_H__
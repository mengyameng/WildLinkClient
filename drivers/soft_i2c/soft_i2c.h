#ifndef __SOFT_I2C_H__
#define __SOFT_I2C_H__

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifndef SOFT_I2C_IGNORE_USER_CONFIG
#include "soft_i2c_config.h"
#endif // SOFT_I2C_IGNORE_USER_CONFIG

#ifndef SOFT_I2C_SUPPORT_CONCURRENCY
#define SOFT_I2C_SUPPORT_CONCURRENCY 0
#endif // SOFT_I2C_SUPPORT_CONCURRENCY

typedef enum {
    SOFT_I2C_ADDR_7BIT,
    SOFT_I2C_ADDR_10BIT
} soft_i2c_addr_t;

typedef struct soft_i2c_handle_s {
    uint8_t (*pin_init)(void);
    void (*pin_deinit)(void);

    uint8_t (*scl_write)(uint8_t val);
    uint8_t (*scl_read)(void);
    uint8_t (*sda_write)(uint8_t val);
    uint8_t (*sda_read)(void);

#if SOFT_I2C_SUPPORT_CONCURRENCY
    uint8_t (*mutex_acquire)(void);
    void (*mutex_release)(void);
#endif // SOFT_I2C_SUPPORT_CONCURRENCY

    void (*delay_ms)(uint32_t ms);
    void (*debug_print)(const char *fmt, ...);

    bool _initialized;
} soft_i2c_handle_t;

#define SOFT_I2C_INIT(pHANDLE) memset(pHANDLE, 0, sizeof(*(pHANDLE)))
#define SOFT_I2C_LINK_PIN_INIT(pHANDLE, FUNC) ((pHANDLE)->pin_init = (FUNC))
#define SOFT_I2C_LINK_PIN_DEINIT(pHANDLE, FUNC) ((pHANDLE)->pin_deinit = (FUNC))
#define SOFT_I2C_LINK_SCL_WRITE(pHANDLE, FUNC) ((pHANDLE)->scl_write = (FUNC))
#define SOFT_I2C_LINK_SCL_READ(pHANDLE, FUNC) ((pHANDLE)->scl_read = (FUNC))
#define SOFT_I2C_LINK_SDA_WRITE(pHANDLE, FUNC) ((pHANDLE)->sda_write = (FUNC))
#define SOFT_I2C_LINK_SDA_READ(pHANDLE, FUNC) ((pHANDLE)->sda_read = (FUNC))
#if SOFT_I2C_SUPPORT_CONCURRENCY
#define SOFT_I2C_LINK_MUTEX_ACQUIRE(pHANDLE, FUNC) ((pHANDLE)->mutex_acquire = (FUNC))
#define SOFT_I2C_LINK_MUTEX_RELEASE(pHANDLE, FUNC) ((pHANDLE)->mutex_release = (FUNC))
#else
#define SOFT_I2C_LINK_MUTEX_ACQUIRE(pHANDLE, FUNC)
#define SOFT_I2C_LINK_MUTEX_RELEASE(pHANDLE, FUNC)
#endif // SOFT_I2C_SUPPORT_CONCURRENCY
#define SOFT_I2C_LINK_DELAY_MS(pHANDLE, FUNC) ((pHANDLE)->delay_ms = (FUNC))
#define SOFT_I2C_LINK_DEBUG_PRINT(pHANDLE, FUNC) ((pHANDLE)->debug_print = (FUNC))

#define SOFT_I2C_ERR_NONE 0
#define SOFT_I2C_ERR_FAILED 1
#define SOFT_I2C_ERR_HANDLE_IS_NULL 2
#define SOFT_I2C_ERR_MEMBER_FUNC_IS_NULL 3
#define SOFT_I2C_ERR_NOT_INITIALIZED 4
#define SOFT_I2C_ERR_INVALID_PARAMS 5
#define SOFT_I2C_ERR_MUTEX_ACQUIRE 6
#define SOFT_I2C_ERR_ADDR_NACK 7
#define SOFT_I2C_ERR_DATA_NACK 8

uint8_t soft_i2c_init(soft_i2c_handle_t *handle);
void soft_i2c_deinit(soft_i2c_handle_t *handle);

uint8_t soft_i2c_swap_lines(soft_i2c_handle_t *handle);

uint8_t soft_i2c_mem_write(soft_i2c_handle_t *handle, uint16_t dev_addr,
                           soft_i2c_addr_t addr_type, const void *reg_addr,
                           size_t reg_addr_size, const void *data, size_t data_size);
uint8_t soft_i2c_mem_read(soft_i2c_handle_t *handle, uint16_t dev_addr,
                          soft_i2c_addr_t addr_type, const void *reg_addr,
                          size_t reg_addr_size, void *data, size_t data_size);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif //__SOFT_I2C_H__
#include "soft_i2c.h"

#if SOFT_I2C_SUPPORT_CONCURRENCY
#define SOFT_I2C_ENTER_CRITICAL(pHandle) (pHandle)->mutex_acquire()
#define SOFT_I2C_EXIT_CRITICAL(pHandle) (pHandle)->mutex_release()
#else
#define SOFT_I2C_ENTER_CRITICAL(pHandle) SOFT_I2C_ERR_NONE
#define SOFT_I2C_EXIT_CRITICAL(pHandle)
#endif // SOFT_I2C_SUPPORT_CONCURRENCY

// After function execution, both scl and sda are set to low level,
// which facilitates concatenating timing units
static uint8_t soft_i2c_start(soft_i2c_handle_t *handle);
static uint8_t soft_i2c_stop(soft_i2c_handle_t *handle);
static uint8_t soft_i2c_write_byte(soft_i2c_handle_t *handle, uint8_t data);
static uint8_t soft_i2c_read_byte(soft_i2c_handle_t *handle, uint8_t *data);
static uint8_t soft_i2c_check_ack(soft_i2c_handle_t *handle);
/**
 * @brief
 *
 * @param handle
 * @param ack if true, send ack. Otherwise, send nack
 * @return uint8_t
 */
static uint8_t soft_i2c_send_ack(soft_i2c_handle_t *handle, bool ack);

uint8_t soft_i2c_init(soft_i2c_handle_t *handle) {
    uint8_t ret;

    if (handle == NULL) {
        ret = SOFT_I2C_ERR_HANDLE_IS_NULL;
        goto error;
    }
    else if (handle->pin_init == NULL) {
        ret = SOFT_I2C_ERR_MEMBER_FUNC_IS_NULL;
        goto error;
    }
    else if (handle->pin_deinit == NULL) {
        ret = SOFT_I2C_ERR_MEMBER_FUNC_IS_NULL;
        goto error;
    }
    else if (handle->scl_write == NULL) {
        ret = SOFT_I2C_ERR_MEMBER_FUNC_IS_NULL;
        goto error;
    }
    else if (handle->scl_read == NULL) {
        ret = SOFT_I2C_ERR_MEMBER_FUNC_IS_NULL;
        goto error;
    }
    else if (handle->sda_write == NULL) {
        ret = SOFT_I2C_ERR_MEMBER_FUNC_IS_NULL;
        goto error;
    }
    else if (handle->sda_read == NULL) {
        ret = SOFT_I2C_ERR_MEMBER_FUNC_IS_NULL;
        goto error;
    }
#if SOFT_I2C_SUPPORT_CONCURRENCY
    else if (handle->mutex_acquire == NULL) {
        ret = SOFT_I2C_ERR_MEMBER_FUNC_IS_NULL;
        goto error;
    }
    else if (handle->mutex_release == NULL) {
        ret = SOFT_I2C_ERR_MEMBER_FUNC_IS_NULL;
        goto error;
    }
#endif // SOFT_I2C_SUPPORT_CONCURRENCY
    else if (handle->delay_ms == NULL) {
        ret = SOFT_I2C_ERR_MEMBER_FUNC_IS_NULL;
        goto error;
    }
    else if (handle->debug_print == NULL) {
        ret = SOFT_I2C_ERR_MEMBER_FUNC_IS_NULL;
        goto error;
    }

    ret = handle->pin_init();
    if (ret != SOFT_I2C_ERR_NONE) {
        goto error;
    }

    ret = handle->scl_write(1);
    if (ret != SOFT_I2C_ERR_NONE) {
        handle->pin_deinit();
        goto error;
    }

    ret = handle->sda_write(1);
    if (ret != SOFT_I2C_ERR_NONE) {
        handle->pin_deinit();
        goto error;
    }

    handle->_initialized = true;

    return SOFT_I2C_ERR_NONE;
error:
    return ret;
}

void soft_i2c_deinit(soft_i2c_handle_t *handle) {
    if (handle == NULL) {
        return; // SOFT_I2C_ERR_HANDLE_IS_NULL;
    }
    else if (!(handle->_initialized)) {
        return; // SOFT_I2C_ERR_NOT_INITIALIZED; // no need to deinit
    }

    handle->pin_deinit();
    handle->_initialized = false;

    return; // SOFT_I2C_ERR_NONE;
}

uint8_t soft_i2c_swap_lines(soft_i2c_handle_t *handle) {
    if (handle == NULL) {
        return SOFT_I2C_ERR_HANDLE_IS_NULL;
    }
    else if (!(handle->_initialized)) {
        return SOFT_I2C_ERR_NOT_INITIALIZED;
    }

    uint8_t ret;
    ret = SOFT_I2C_ENTER_CRITICAL(handle);
    if (ret != SOFT_I2C_ERR_NONE) {
        goto error;
    }

    uint8_t (*temp_write)(uint8_t);
    temp_write = handle->scl_write;
    handle->scl_write = handle->sda_write;
    handle->sda_write = temp_write;

    uint8_t (*temp_read)(void);
    temp_read = handle->scl_read;
    handle->scl_read = handle->sda_read;
    handle->sda_read = temp_read;

    SOFT_I2C_EXIT_CRITICAL(handle);
    return SOFT_I2C_ERR_NONE;
error:
    if (ret != SOFT_I2C_ERR_MUTEX_ACQUIRE) {
        SOFT_I2C_EXIT_CRITICAL(handle);
    }
    return ret;
}

uint8_t soft_i2c_mem_write(soft_i2c_handle_t *handle, uint16_t dev_addr,
                           soft_i2c_addr_t addr_type, const void *reg_addr,
                           size_t reg_addr_size, const void *data, size_t data_size) {
    if (handle == NULL) {
        return SOFT_I2C_ERR_HANDLE_IS_NULL;
    }
    else if (!(handle->_initialized)) {
        return SOFT_I2C_ERR_NOT_INITIALIZED;
    }
    else if (addr_type == SOFT_I2C_ADDR_10BIT) {
        return SOFT_I2C_ERR_FAILED;
    }

    uint8_t ret;
    const uint8_t *reg_addr_bytes = (const uint8_t *)reg_addr;
    const uint8_t *data_bytes = (const uint8_t *)data;

    ret = SOFT_I2C_ENTER_CRITICAL(handle);
    if (ret != SOFT_I2C_ERR_NONE) {
        goto error;
    }

    ret = soft_i2c_start(handle);
    if (ret != SOFT_I2C_ERR_NONE) {
        goto error;
    }

    ret = soft_i2c_write_byte(handle, (dev_addr << 1) & (~0x01));
    if (ret != SOFT_I2C_ERR_NONE) {
        goto error;
    }

    ret = soft_i2c_check_ack(handle);
    if (ret != SOFT_I2C_ERR_NONE) {
        if (ret == SOFT_I2C_ERR_DATA_NACK) {
            ret = SOFT_I2C_ERR_ADDR_NACK;
        }
        goto error;
    }

    if (reg_addr_bytes != NULL) {
        for (size_t i = 0; i < reg_addr_size; i++) {
            ret = soft_i2c_write_byte(handle, reg_addr_bytes[i]);
            if (ret != SOFT_I2C_ERR_NONE) {
                goto error;
            }

            ret = soft_i2c_check_ack(handle);
            if (ret != SOFT_I2C_ERR_NONE) {
                goto error;
            }
        }
    }

    if (data_bytes != NULL) {
        for (size_t i = 0; i < data_size; i++) {
            ret = soft_i2c_write_byte(handle, data_bytes[i]);
            if (ret != SOFT_I2C_ERR_NONE) {
                goto error;
            }

            ret = soft_i2c_check_ack(handle);
            if (ret != SOFT_I2C_ERR_NONE) {
                goto error;
            }
        }
    }

    soft_i2c_stop(handle);
    SOFT_I2C_EXIT_CRITICAL(handle);
    return SOFT_I2C_ERR_NONE;
error:
    soft_i2c_stop(handle);
    if (ret != SOFT_I2C_ERR_MUTEX_ACQUIRE) {
        SOFT_I2C_EXIT_CRITICAL(handle);
    }
    return ret;
}

uint8_t soft_i2c_mem_read(soft_i2c_handle_t *handle, uint16_t dev_addr,
                          soft_i2c_addr_t addr_type, const void *reg_addr,
                          size_t reg_addr_size, void *data, size_t data_size) {
    if (handle == NULL) {
        return SOFT_I2C_ERR_HANDLE_IS_NULL;
    }
    else if (!(handle->_initialized)) {
        return SOFT_I2C_ERR_NOT_INITIALIZED;
    }
    else if (addr_type == SOFT_I2C_ADDR_10BIT) {
        return SOFT_I2C_ERR_FAILED;
    }

    uint8_t ret;
    const uint8_t *reg_addr_bytes = (const uint8_t *)reg_addr;
    uint8_t *data_bytes = (uint8_t *)data;

    ret = SOFT_I2C_ENTER_CRITICAL(handle);
    if (ret != SOFT_I2C_ERR_NONE) {
        goto error;
    }

    ret = soft_i2c_start(handle);
    if (ret != SOFT_I2C_ERR_NONE) {
        goto error;
    }

    ret = soft_i2c_write_byte(handle, (dev_addr << 1) & (~0x01));
    if (ret != SOFT_I2C_ERR_NONE) {
        goto error;
    }

    ret = soft_i2c_check_ack(handle);
    if (ret != SOFT_I2C_ERR_NONE) {
        if (ret == SOFT_I2C_ERR_DATA_NACK) {
            ret = SOFT_I2C_ERR_ADDR_NACK;
        }
        goto error;
    }

    if (reg_addr_bytes != NULL) {
        for (size_t i = 0; i < reg_addr_size; i++) {
            ret = soft_i2c_write_byte(handle, reg_addr_bytes[i]);
            if (ret != SOFT_I2C_ERR_NONE) {
                goto error;
            }

            ret = soft_i2c_check_ack(handle);
            if (ret != SOFT_I2C_ERR_NONE) {
                goto error;
            }
        }
    }

    if (data_bytes != NULL) {
        ret = soft_i2c_start(handle);
        if (ret != SOFT_I2C_ERR_NONE) {
            goto error;
        }

        ret = soft_i2c_write_byte(handle, (dev_addr << 1) | (0x01));
        if (ret != SOFT_I2C_ERR_NONE) {
            goto error;
        }

        ret = soft_i2c_check_ack(handle);
        if (ret != SOFT_I2C_ERR_NONE) {
            if (ret == SOFT_I2C_ERR_DATA_NACK) {
                ret = SOFT_I2C_ERR_ADDR_NACK;
            }
            goto error;
        }

        for (size_t i = 0; i < data_size - 1; i++) {
            ret = soft_i2c_read_byte(handle, &data_bytes[i]);
            if (ret != SOFT_I2C_ERR_NONE) {
                goto error;
            }

            ret = soft_i2c_send_ack(handle, true);
            if (ret != SOFT_I2C_ERR_NONE) {
                goto error;
            }
        }
        ret = soft_i2c_read_byte(handle, &data_bytes[data_size - 1]);
        if (ret != SOFT_I2C_ERR_NONE) {
            goto error;
        }

        ret = soft_i2c_send_ack(handle, false);
        if (ret != SOFT_I2C_ERR_NONE) {
            goto error;
        }
    }

    soft_i2c_stop(handle);
    SOFT_I2C_EXIT_CRITICAL(handle);
    return SOFT_I2C_ERR_NONE;
error:
    soft_i2c_stop(handle);
    if (ret != SOFT_I2C_ERR_MUTEX_ACQUIRE) {
        SOFT_I2C_EXIT_CRITICAL(handle);
    }
    return ret;
}

static uint8_t soft_i2c_start(soft_i2c_handle_t *handle) {
    if (handle == NULL) {
        return SOFT_I2C_ERR_HANDLE_IS_NULL;
    }
    else if (!(handle->_initialized)) {
        return SOFT_I2C_ERR_NOT_INITIALIZED;
    }

    uint8_t ret;
    ret = handle->sda_write(1);
    if (ret != SOFT_I2C_ERR_NONE) {
        goto error;
    }

    ret = handle->scl_write(1);
    if (ret != SOFT_I2C_ERR_NONE) {
        goto error;
    }

    // generate start condition
    ret = handle->sda_write(0);
    if (ret != SOFT_I2C_ERR_NONE) {
        goto error;
    }

    ret = handle->scl_write(0);
    if (ret != SOFT_I2C_ERR_NONE) {
        goto error;
    }

    return SOFT_I2C_ERR_NONE;
error:
    return ret;
}

static uint8_t soft_i2c_stop(soft_i2c_handle_t *handle) {
    if (handle == NULL) {
        return SOFT_I2C_ERR_HANDLE_IS_NULL;
    }
    else if (!(handle->_initialized)) {
        return SOFT_I2C_ERR_NOT_INITIALIZED;
    }

    uint8_t ret;
    ret = handle->sda_write(0);
    if (ret != SOFT_I2C_ERR_NONE) {
        goto error;
    }

    ret = handle->scl_write(1);
    if (ret != SOFT_I2C_ERR_NONE) {
        goto error;
    }

    // generate stop condition
    ret = handle->sda_write(1);
    if (ret != SOFT_I2C_ERR_NONE) {
        goto error;
    }

    return SOFT_I2C_ERR_NONE;
error:
    return ret;
}

static uint8_t soft_i2c_write_byte(soft_i2c_handle_t *handle, uint8_t data) {
    if (handle == NULL) {
        return SOFT_I2C_ERR_HANDLE_IS_NULL;
    }
    else if (!(handle->_initialized)) {
        return SOFT_I2C_ERR_NOT_INITIALIZED;
    }

    uint8_t ret;
    for (uint8_t i = 0; i < 8 * sizeof(data); i++) {
        // At this moment, scl is already at low level
        ret = handle->sda_write(data & 0x80);
        if (ret != SOFT_I2C_ERR_NONE) {
            goto error;
        }

        ret = handle->scl_write(1);
        if (ret != SOFT_I2C_ERR_NONE) {
            goto error;
        }

        ret = handle->scl_write(0);
        if (ret != SOFT_I2C_ERR_NONE) {
            goto error;
        }

        data <<= 1;
    }

    return SOFT_I2C_ERR_NONE;
error:
    return ret;
}

static uint8_t soft_i2c_read_byte(soft_i2c_handle_t *handle, uint8_t *data) {
    if (data == NULL) {
        return SOFT_I2C_ERR_INVALID_PARAMS;
    }
    else if (handle == NULL) {
        return SOFT_I2C_ERR_HANDLE_IS_NULL;
    }
    else if (!(handle->_initialized)) {
        return SOFT_I2C_ERR_NOT_INITIALIZED;
    }

    uint8_t ret;
    uint8_t read_byte = 0xff;

    ret = handle->sda_write(1);
    if (ret != SOFT_I2C_ERR_NONE) {
        goto error;
    }

    for (uint8_t i = 0; i < 8; i++) {
        ret = handle->scl_write(1);
        if (ret != SOFT_I2C_ERR_NONE) {
            goto error;
        }

        if (!(handle->sda_read())) {
            read_byte &= (~(0x80 >> i));
        }

        ret = handle->scl_write(0);
        if (ret != SOFT_I2C_ERR_NONE) {
            goto error;
        }
    }
    *data = read_byte;

    return SOFT_I2C_ERR_NONE;
error:
    return ret;
}

static uint8_t soft_i2c_check_ack(soft_i2c_handle_t *handle) {
    if (handle == NULL) {
        return SOFT_I2C_ERR_HANDLE_IS_NULL;
    }
    else if (!(handle->_initialized)) {
        return SOFT_I2C_ERR_NOT_INITIALIZED;
    }

    uint8_t ret, ack_state = 1;
    ret = handle->sda_write(1);
    if (ret != SOFT_I2C_ERR_NONE) {
        goto error;
    }

    ret = handle->scl_write(1);
    if (ret != SOFT_I2C_ERR_NONE) {
        goto error;
    }

    ack_state = handle->sda_read();

    ret = handle->scl_write(0);
    if (ret != SOFT_I2C_ERR_NONE) {
        goto error;
    }

    // sda is high level, nack received
    if (ack_state) {
        ret = SOFT_I2C_ERR_DATA_NACK;
        goto error;
    }

    return SOFT_I2C_ERR_NONE;
error:
    return ret;
}

static uint8_t soft_i2c_send_ack(soft_i2c_handle_t *handle, bool ack) {
    if (handle == NULL) {
        return SOFT_I2C_ERR_HANDLE_IS_NULL;
    }
    else if (!(handle->_initialized)) {
        return SOFT_I2C_ERR_NOT_INITIALIZED;
    }

    uint8_t ret;
    ret = handle->sda_write(!ack);
    if (ret != SOFT_I2C_ERR_NONE) {
        goto error;
    }

    ret = handle->scl_write(1);
    if (ret != SOFT_I2C_ERR_NONE) {
        goto error;
    }

    ret = handle->scl_write(0);
    if (ret != SOFT_I2C_ERR_NONE) {
        goto error;
    }

    return SOFT_I2C_ERR_NONE;
error:
    return ret;
}

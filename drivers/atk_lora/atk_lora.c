#include <stdarg.h>
#include <stdio.h>

#include "atk_lora.h"

#define AT_CONFIG_RX_BUFFER_SIZE 80

static uint8_t at_cmd_inner_buf[CONFIG_ATK_LORA_CMD_BUFFER_SIZE];
static uint8_t at_resp_inner_buf[CONFIG_ATK_LORA_RESP_BUFFER_SIZE];

uint8_t atk_lora_init(atk_lora_handle_t *handle) {
    uint8_t ret;

    if (handle == NULL) {
        ret = ATK_LORA_ERR_HANDLE_IS_NULL;
        goto error;
    }
    else if (handle->uart_init == NULL) {
        ret = ATK_LORA_ERR_MEMBER_FUNC_IS_NULL;
        goto error;
    }
    else if (handle->uart_deinit == NULL) {
        ret = ATK_LORA_ERR_MEMBER_FUNC_IS_NULL;
        goto error;
    }
    else if (handle->uart_write == NULL) {
        ret = ATK_LORA_ERR_MEMBER_FUNC_IS_NULL;
        goto error;
    }
    else if (handle->uart_read == NULL) {
        ret = ATK_LORA_ERR_MEMBER_FUNC_IS_NULL;
        goto error;
    }
    else if (handle->md0_write == NULL) {
        ret = ATK_LORA_ERR_MEMBER_FUNC_IS_NULL;
        goto error;
    }
    else if (handle->delay_ms == NULL) {
        ret = ATK_LORA_ERR_MEMBER_FUNC_IS_NULL;
        goto error;
    }
    else if (handle->debug_print == NULL) {
        ret = ATK_LORA_ERR_MEMBER_FUNC_IS_NULL;
        goto error;
    }

    ret = handle->uart_init();
    if (ret != ATK_LORA_ERR_NONE) {
        goto error;
    }

    handle->_initialized = 1;

    return ATK_LORA_ERR_NONE;
error:
    return ret;
}

uint8_t atk_lora_deinit(atk_lora_handle_t *handle) {
    if (handle == NULL) {
        return ATK_LORA_ERR_HANDLE_IS_NULL;
    }
    else if (!(handle->_initialized)) {
        return ATK_LORA_ERR_NOT_INITIALIZED;
    }

    uint8_t ret = handle->uart_deinit();
    if (ret != ATK_LORA_ERR_NONE) {
        return ret;
    }

    handle->_initialized = 0;
    return ATK_LORA_ERR_NONE;
}

uint8_t atk_lora_send_buf(atk_lora_handle_t *handle, const void *buf, size_t buf_size) {
    if (handle == NULL) {
        return ATK_LORA_ERR_HANDLE_IS_NULL;
    }
    else if (!(handle->_initialized)) {
        return ATK_LORA_ERR_NOT_INITIALIZED;
    }
    else if (buf == NULL) {
        return ATK_LORA_ERR_INVALID_PARAMS;
    }

    uint8_t ret;
    ret = handle->uart_write((const uint8_t *)buf, buf_size);
    if (ret != ATK_LORA_ERR_NONE) {
        goto error;
    }

    return ATK_LORA_ERR_NONE;
error:
    return ret;
}

uint8_t atk_lora_send_at_cmd(atk_lora_handle_t *handle, const char *at_cmd) {
    if (handle == NULL) {
        return ATK_LORA_ERR_HANDLE_IS_NULL;
    }
    else if (!(handle->_initialized)) {
        return ATK_LORA_ERR_NOT_INITIALIZED;
    }
    else if (at_cmd == NULL) {
        return ATK_LORA_ERR_INVALID_PARAMS;
    }

    uint8_t ret;
    ret = handle->md0_write(1);
    if (ret != ATK_LORA_ERR_NONE) {
        goto error;
    }
    handle->delay_ms(50);

    ret = handle->uart_write((uint8_t *)at_cmd, strlen(at_cmd));
    if (ret != ATK_LORA_ERR_NONE) {
        goto error;
    }

    return ATK_LORA_ERR_NONE;
error:
    return ret;
}

uint8_t atk_lora_send_at_cmd_fmt(atk_lora_handle_t *handle, const char *at_fmt, ...) {
    if (at_fmt == NULL) {
        return ATK_LORA_ERR_INVALID_PARAMS;
    }

    va_list at_args;
    va_start(at_args, at_fmt);

    vsnprintf((char *)at_cmd_inner_buf, sizeof(at_cmd_inner_buf), at_fmt, at_args);

    va_end(at_args);

    return atk_lora_send_at_cmd(handle, (const char *)at_cmd_inner_buf);
}

uint8_t atk_lora_send_at_cmd_buf(atk_lora_handle_t *handle, char *at_buf,
                                 size_t at_buf_size, const char *at_fmt, ...) {
    if (at_buf == NULL || at_fmt == NULL) {
        return ATK_LORA_ERR_INVALID_PARAMS;
    }

    va_list at_args;
    va_start(at_args, at_fmt);

    vsnprintf(at_buf, at_buf_size, at_fmt, at_args);

    va_end(at_args);

    return atk_lora_send_at_cmd(handle, at_buf);
}

uint8_t atk_lora_read_at_resp(atk_lora_handle_t *handle) {
    return atk_lora_read_at_resp_buf(handle, (char *)at_resp_inner_buf,
                                     sizeof(at_resp_inner_buf));
}

uint8_t atk_lora_read_at_resp_buf(atk_lora_handle_t *handle, char *resp_buf,
                                  size_t resp_buf_size) {
    if (handle == NULL) {
        return ATK_LORA_ERR_HANDLE_IS_NULL;
    }
    else if (!(handle->_initialized)) {
        return ATK_LORA_ERR_NOT_INITIALIZED;
    }
    else if (resp_buf == NULL) {
        return ATK_LORA_ERR_INVALID_PARAMS;
    }

    uint8_t ret;
    ret = handle->md0_write(1);
    if (ret != ATK_LORA_ERR_NONE) {
        goto error;
    }
    handle->delay_ms(50);

    memset(resp_buf, 0, resp_buf_size);
    ret = handle->uart_read((uint8_t *)resp_buf, resp_buf_size);
    if (ret != ATK_LORA_ERR_NONE) {
        goto error;
    }
    else {
        resp_buf[resp_buf_size - 1] = 0;
    }

    return ATK_LORA_ERR_NONE;
error:
    return ret;
}

uint8_t atk_lora_check_at_resp(atk_lora_handle_t *handle, const char *expected_resp) {
    return atk_lora_check_at_resp_buf(handle, (char *)at_resp_inner_buf, expected_resp);
}

uint8_t atk_lora_check_at_resp_buf(atk_lora_handle_t *handle, const char *resp_buf,
                                   const char *expected_resp) {
    if (handle == NULL) {
        return ATK_LORA_ERR_HANDLE_IS_NULL;
    }
    else if (!(handle->_initialized)) {
        return ATK_LORA_ERR_NOT_INITIALIZED;
    }
    else if (resp_buf == NULL || expected_resp == NULL) {
        return ATK_LORA_ERR_INVALID_PARAMS;
    }

    uint8_t ret;
    if (strstr((const char *)resp_buf, expected_resp) == NULL) {
        ret = ATK_LORA_ERR_AT_COMMAND_FAILED;
        goto error;
    }

    return ATK_LORA_ERR_NONE;
error:
    return ret;
}

uint8_t _atk_lora_parse_at_resp(atk_lora_handle_t *handle, const char *parse_fmt, ...) {
    va_list parse_fmt_args;
    va_start(parse_fmt_args, parse_fmt);

    uint8_t parse_cnt = _atk_lora_vparse_at_resp_buf(
        handle, (const char *)at_resp_inner_buf, parse_fmt, parse_fmt_args);

    va_end(parse_fmt_args);

    return parse_cnt;
}

uint8_t _atk_lora_parse_at_resp_buf(atk_lora_handle_t *handle, const char *resp_buf,
                                    const char *parse_fmt, ...) {

    va_list parse_fmt_args;
    va_start(parse_fmt_args, parse_fmt);

    uint8_t parse_cnt =
        _atk_lora_vparse_at_resp_buf(handle, resp_buf, parse_fmt, parse_fmt_args);

    va_end(parse_fmt_args);

    return parse_cnt;
}

uint8_t _atk_lora_vparse_at_resp_buf(atk_lora_handle_t *handle, const char *resp_buf,
                                     const char *parse_fmt, va_list parse_fmt_args) {
    if (handle == NULL) {
        return ATK_LORA_ERR_HANDLE_IS_NULL;
    }
    else if (!(handle->_initialized)) {
        return ATK_LORA_ERR_NOT_INITIALIZED;
    }

    return (uint8_t)vsscanf(resp_buf, parse_fmt, parse_fmt_args);
}

uint8_t atk_lora_set_communicate(atk_lora_handle_t *handle, uint8_t state) {
    uint8_t ret;

    if (handle == NULL) {
        ret = ATK_LORA_ERR_HANDLE_IS_NULL;
        goto error;
    }
    else if (!(handle->_initialized)) {
        ret = ATK_LORA_ERR_NOT_INITIALIZED;
        goto error;
    }

    ret = handle->md0_write(state);
    if (ret != ATK_LORA_ERR_NONE) {
        goto error;
    }

    return ATK_LORA_ERR_NONE;
error:
    return ret;
}

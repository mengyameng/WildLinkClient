#include <inttypes.h>
#include <stdarg.h>

#include "common_def.h"
#if 0
#include "i2c.h"
#endif
#include "osal_task.h"
#include "soc_osal.h"

#include "aht20.h"
#include "node_telemetry.h"
#include "soft_i2c.h"
#include "wlid_link_client_log.h"

#include "aht20_task.h"

#ifndef STRINGIFY
#define STRINGIFY(x) #x
#endif // STRINGIFY

static aht20_handle_t g_aht20_handle;
osal_task *g_aht20_task_handle;
extern soft_i2c_handle_t g_soft_i2c_handle;

static uint32_t g_t_raw; // temperature raw
static float g_t_cal;    // temperature
static uint32_t g_h_raw; // humidity raw
static uint8_t g_h_cal;  // humidity

static uint8_t aht20_interface_iic_init(void);
static uint8_t aht20_interface_iic_deinit(void);
static uint8_t aht20_interface_iic_read_cmd(uint8_t addr, uint8_t *buf, uint16_t len);
static uint8_t aht20_interface_iic_write_cmd(uint8_t addr, uint8_t *buf, uint16_t len);
static void aht20_interface_delay_ms(uint32_t ms);
static void aht20_interface_debug_print(const char *const fmt, ...);

static int aht20_task(void *args);

void aht20_read_temperature_and_humidity(float *out_t, uint8_t *out_h) {
    if (out_t != NULL) {
        *out_t = g_t_cal;
    }
    if (out_h != NULL) {
        *out_h = g_h_cal;
    }
}

errcode_t aht20_task_entry(void) {
    g_aht20_task_handle = osal_kthread_create(aht20_task, NULL, STRINGIFY(aht20_task),
                                              CONFIG_AHT20_TASK_STACK_SIZE);
    if (g_aht20_task_handle == NULL) {
        WLID_LINK_CLIENT_LOG_ERROR("failed to create aht20 task\r\n");
        goto _error_exit;
    }

    if (osal_kthread_set_priority(g_aht20_task_handle, CONFIG_AHT20_TASK_PRIORITY)
        != OSAL_SUCCESS)
    {
        WLID_LINK_CLIENT_LOG_ERROR("failed to set aht20 task priority\r\n");
        goto _clean_thread;
    }

    return ERRCODE_SUCC;
_clean_thread:
    osal_kthread_destroy(g_aht20_task_handle, 0);
_error_exit:
    return ERRCODE_FAIL;
}

static int aht20_task(void *args) {
    unused(args);

    /* delay 1000 ms for ready */
    WLID_LINK_CLIENT_LOG_DEBUG("wait 1000 ms for ready\r\n");
    osal_msleep(1000);
    WLID_LINK_CLIENT_LOG_INFO("start\r\n");

    // DRIVER_AHT20_LINK_INIT(&g_aht20_handle, aht20_handle_t);
    DRIVER_AHT20_LINK_IIC_INIT(&g_aht20_handle, aht20_interface_iic_init);
    DRIVER_AHT20_LINK_IIC_DEINIT(&g_aht20_handle, aht20_interface_iic_deinit);
    DRIVER_AHT20_LINK_IIC_READ_CMD(&g_aht20_handle, aht20_interface_iic_read_cmd);
    DRIVER_AHT20_LINK_IIC_WRITE_CMD(&g_aht20_handle, aht20_interface_iic_write_cmd);
    DRIVER_AHT20_LINK_DELAY_MS(&g_aht20_handle, aht20_interface_delay_ms);
    DRIVER_AHT20_LINK_DEBUG_PRINT(&g_aht20_handle, aht20_interface_debug_print);

    uint8_t ret;

    /* aht20 init */
    ret = aht20_init(&g_aht20_handle);
    if (ret != 0) {
        WLID_LINK_CLIENT_LOG_ERROR("failed to init aht20, ret = %" PRIu8 "\n");
    }

    for (;;) {
        osal_msleep(1000);

        /* read temperature and humidity */
        ret = aht20_read_temperature_humidity(&g_aht20_handle, &g_t_raw, &g_t_cal,
                                              &g_h_raw, &g_h_cal);
        if (ret != 0) {
            WLID_LINK_CLIENT_LOG_ERROR("failed to read aht20, ret = %" PRIu8 "\n", ret);
            continue;
        }
        WLID_LINK_CLIENT_LOG_DEBUG("t: %d, h: %d\r\n", (uint32_t)g_t_cal,
                                   (uint32_t)g_h_cal);

        NodeTelemetry_t *const local_node = nodeTelemetry_getLocalNode();
        if (local_node == NULL) {
            WLID_LINK_CLIENT_LOG_WARN("local node is NULL\r\n");
        }
        else {
            local_node->air_temp = g_t_cal;
            local_node->air_humidity = g_h_cal;
        }
    }

    osal_kthread_destroy(g_aht20_task_handle, 0);
    return 0;
}

static uint8_t aht20_interface_iic_init(void) {
    return 0;
}

static uint8_t aht20_interface_iic_deinit(void) {
    return 0;
}

static uint8_t aht20_interface_iic_read_cmd(uint8_t addr, uint8_t *buf, uint16_t len) {
#if 0
    errcode_t ret;

    i2c_data_t i2c_data = {0};
    i2c_data.receive_buf = buf;
    i2c_data.receive_len = len;

    ret = uapi_i2c_master_read(1, addr, &i2c_data);
    if (ret != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR("failed to read aht20, ret = %#08" PRIx32 "\n", ret);
        return -1;
    }

    return 0;
#else
    uint8_t ret;
    ret = soft_i2c_mem_read(&g_soft_i2c_handle, addr, SOFT_I2C_ADDR_7BIT, NULL, 0, buf,
                            len);
    if (ret != SOFT_I2C_ERR_NONE) {
        WLID_LINK_CLIENT_LOG_ERROR("failed to read aht20, ret = %" PRIu8 "\n", ret);
        return 1;
    }
    return 0;
#endif // 0
}

static uint8_t aht20_interface_iic_write_cmd(uint8_t addr, uint8_t *buf, uint16_t len) {
#if 0
    errcode_t ret;

    i2c_data_t i2c_data = {0};
    i2c_data.send_buf = buf;
    i2c_data.send_len = len;

    ret = uapi_i2c_master_write(1, addr, &i2c_data);
    if (ret != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR("failed to write aht20, ret = %#08" PRIx32 "\n", ret);
        return -1;
    }

    return 0;
#else
    uint8_t ret;
    ret = soft_i2c_mem_write(&g_soft_i2c_handle, addr, SOFT_I2C_ADDR_7BIT, NULL, 0, buf,
                             len);
    if (ret != SOFT_I2C_ERR_NONE) {
        WLID_LINK_CLIENT_LOG_ERROR("failed to write aht20, ret = %" PRIu8 "\n", ret);
        return 1;
    }
    return 0;
#endif // 0
}

static void aht20_interface_delay_ms(uint32_t ms) {
    osal_msleep(ms);
}

static void aht20_interface_debug_print(const char *const fmt, ...) {
    va_list args;
    va_start(args, fmt);
    osal_vprintk(fmt, args);
    va_end(args);
}

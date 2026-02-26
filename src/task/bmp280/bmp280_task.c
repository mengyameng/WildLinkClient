#include <inttypes.h>
#include <stdarg.h>

#include "common_def.h"
#include "soc_osal.h"

#include "bmp280.h"
#include "node_telemetry.h"
#include "soft_i2c.h"
#include "wlid_link_client_log.h"

#include "bmp280_task.h"

#ifndef STRINGIFY
#define STRINGIFY(x) #x
#endif // STRINGIFY

osal_task *g_bmp280_task_handle;
static bmp280_handle_t g_bmp280_handle;
extern soft_i2c_handle_t g_soft_i2c_handle;
static float g_temperature_c;
static float g_pressure_c;

static uint8_t bmp280_interface_iic_init(void);
static uint8_t bmp280_interface_iic_deinit(void);
static uint8_t bmp280_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf,
                                         uint16_t len);
static uint8_t bmp280_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf,
                                          uint16_t len);
static uint8_t bmp280_interface_spi_init(void);
static uint8_t bmp280_interface_spi_deinit(void);
static uint8_t bmp280_interface_spi_read(uint8_t reg, uint8_t *buf, uint16_t len);
static uint8_t bmp280_interface_spi_write(uint8_t reg, uint8_t *buf, uint16_t len);
static void bmp280_interface_delay_ms(uint32_t ms);
static void bmp280_interface_debug_print(const char *const fmt, ...);

static int bmp280_task(void *args);

float bmp280_task_read_pressure(void) {
    return g_pressure_c;
}

errcode_t bmp280_task_entry(void) {
    g_bmp280_task_handle = osal_kthread_create(
        bmp280_task, NULL, STRINGIFY(bmp280_task), CONFIG_BMP280_TASK_STACK_SIZE);
    if (g_bmp280_task_handle == NULL) {
        WLID_LINK_CLIENT_LOG_ERROR("failed to create bmp280 task\r\n");
        return ERRCODE_FAIL;
    }

    if (osal_kthread_set_priority(g_bmp280_task_handle, CONFIG_BMP280_TASK_PRIORITY)
        != OSAL_SUCCESS)
    {
        WLID_LINK_CLIENT_LOG_ERROR("failed to set bmp280 task priority\r\n");
        return ERRCODE_FAIL;
    }

    return ERRCODE_SUCC;
}

static int bmp280_task(void *args) {
    unused(args);
    osal_msleep(1000);
    WLID_LINK_CLIENT_LOG_INFO("start\r\n");

    uint8_t ret;

    DRIVER_BMP280_LINK_INIT(&g_bmp280_handle, bmp280_handle_t);
    DRIVER_BMP280_LINK_IIC_INIT(&g_bmp280_handle, bmp280_interface_iic_init);
    DRIVER_BMP280_LINK_IIC_DEINIT(&g_bmp280_handle, bmp280_interface_iic_deinit);
    DRIVER_BMP280_LINK_IIC_READ(&g_bmp280_handle, bmp280_interface_iic_read);
    DRIVER_BMP280_LINK_IIC_WRITE(&g_bmp280_handle, bmp280_interface_iic_write);
    DRIVER_BMP280_LINK_SPI_INIT(&g_bmp280_handle, bmp280_interface_spi_init);
    DRIVER_BMP280_LINK_SPI_DEINIT(&g_bmp280_handle, bmp280_interface_spi_deinit);
    DRIVER_BMP280_LINK_SPI_READ(&g_bmp280_handle, bmp280_interface_spi_read);
    DRIVER_BMP280_LINK_SPI_WRITE(&g_bmp280_handle, bmp280_interface_spi_write);
    DRIVER_BMP280_LINK_DELAY_MS(&g_bmp280_handle, bmp280_interface_delay_ms);
    DRIVER_BMP280_LINK_DEBUG_PRINT(&g_bmp280_handle, bmp280_interface_debug_print);

    ret = bmp280_set_interface(&g_bmp280_handle, BMP280_INTERFACE_IIC);
    if (ret != 0) {
        WLID_LINK_CLIENT_LOG_ERROR("ret = %" PRIu8 "\r\n", ret);
    }

    ret = bmp280_set_addr_pin(&g_bmp280_handle, BMP280_ADDRESS_ADO_HIGH);
    if (ret != 0) {
        WLID_LINK_CLIENT_LOG_ERROR("ret = %" PRIu8 "\r\n", ret);
    }

    ret = bmp280_init(&g_bmp280_handle);
    if (ret != 0) {
        WLID_LINK_CLIENT_LOG_ERROR("ret = %" PRIu8 "\r\n", ret);
    }

    ret = bmp280_set_temperatue_oversampling(&g_bmp280_handle, BMP280_OVERSAMPLING_x2);
    if (ret != 0) {
        WLID_LINK_CLIENT_LOG_ERROR("ret = %" PRIu8 "\r\n", ret);
    }

    ret = bmp280_set_pressure_oversampling(&g_bmp280_handle, BMP280_OVERSAMPLING_x2);
    if (ret != 0) {
        WLID_LINK_CLIENT_LOG_ERROR("ret = %" PRIu8 "\r\n", ret);
    }

    ret = bmp280_set_standby_time(&g_bmp280_handle, BMP280_STANDBY_TIME_0P5_MS);
    if (ret != 0) {
        WLID_LINK_CLIENT_LOG_ERROR("ret = %" PRIu8 "\r\n", ret);
    }

    ret = bmp280_set_filter(&g_bmp280_handle, BMP280_FILTER_COEFF_16);
    if (ret != 0) {
        WLID_LINK_CLIENT_LOG_ERROR("ret = %" PRIu8 "\r\n", ret);
    }

    ret = bmp280_set_mode(&g_bmp280_handle, BMP280_MODE_NORMAL);
    if (ret != 0) {
        WLID_LINK_CLIENT_LOG_ERROR("ret = %" PRIu8 "\r\n", ret);
    }

    osal_msleep(1000);

    uint32_t temperature_raw = 0, pressure_raw = 0;
    for (;;) {
        osal_msleep(1000);

        ret = bmp280_read_temperature_pressure(&g_bmp280_handle, &temperature_raw,
                                               &g_temperature_c, &pressure_raw,
                                               &g_pressure_c);
        if (ret != 0) {
            WLID_LINK_CLIENT_LOG_ERROR("ret = %" PRIu8 "\r\n", ret);
            continue;
        }
        WLID_LINK_CLIENT_LOG_DEBUG("t: %" PRId32 ", p: %" PRId32 "\r\n",
                                   (uint32_t)g_temperature_c, (uint32_t)g_pressure_c);

        NodeTelemetry_t *const local_node = nodeTelemetry_getLocalNode();
        if (local_node == NULL) {
            WLID_LINK_CLIENT_LOG_WARN("local node is NULL\r\n");
        }
        else {
            local_node->air_pressure = g_pressure_c;
        }
    }

    osal_kthread_destroy(g_bmp280_task_handle, 0);
    return 0;
}

static uint8_t bmp280_interface_iic_init(void) {
    return 0;
}

static uint8_t bmp280_interface_iic_deinit(void) {
    return 0;
}

static uint8_t bmp280_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf,
                                         uint16_t len) {
#if 0
    errcode_t ret;

    i2c_data_t i2c_data = {0};
    i2c_data.send_buf = &reg;
    i2c_data.send_len = sizeof(reg);
    i2c_data.receive_buf = buf;
    i2c_data.receive_len = len;

    ret = uapi_i2c_master_writeread(1, addr, &i2c_data);
    if (ret != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR("failed to read bmp280 reg, ret = %#08x\r\n", ret);
        return -1;
    }
    return 0;
#else
    uint8_t ret;
    ret = soft_i2c_mem_read(&g_soft_i2c_handle, addr, SOFT_I2C_ADDR_7BIT, &reg,
                            sizeof(reg), buf, len);
    if (ret != SOFT_I2C_ERR_NONE) {
        WLID_LINK_CLIENT_LOG_ERROR("failed to read bmp280 reg, ret = %" PRIu8 "\n",
                                   ret);
        return 1;
    }

    return 0;
#endif
}

static uint8_t bmp280_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf,
                                          uint16_t len) {
#if 0
    errcode_t ret;

    uint8_t *write_data = osal_vmalloc(len + 1);
    if (write_data == NULL) {
        osal_printk("%s:%d: malloc failed\r\n", __func__, __LINE__);
        return -1;
    }
    write_data[0] = reg;
    memcpy_s(write_data + 1, len, buf, len * sizeof(uint8_t));

    i2c_data_t i2c_data = {0};
    i2c_data.send_buf = write_data;
    i2c_data.send_len = len + 1;

    ret = uapi_i2c_master_write(1, addr, &i2c_data);
    if (ret != ERRCODE_SUCC) {
        osal_vfree(write_data);
        WLID_LINK_CLIENT_LOG_ERROR("failed to write bmp280 reg, ret = %#08x\r\n", ret);
        return -1;
    }

    osal_vfree(write_data);
    return 0;
#else
    uint8_t ret;
    ret = soft_i2c_mem_write(&g_soft_i2c_handle, addr, SOFT_I2C_ADDR_7BIT, &reg,
                             sizeof(reg), buf, len);
    if (ret != SOFT_I2C_ERR_NONE) {
        WLID_LINK_CLIENT_LOG_ERROR("failed to write bmp280 reg, ret = %" PRIu8 "\n",
                                   ret);
        return 1;
    }
    return 0;
#endif // 0
}

static uint8_t bmp280_interface_spi_init(void) {
    return 1;
}

static uint8_t bmp280_interface_spi_deinit(void) {
    return 1;
}

static uint8_t bmp280_interface_spi_read(uint8_t reg, uint8_t *buf, uint16_t len) {
    unused(reg);
    unused(buf);
    unused(len);

    return 1;
}

static uint8_t bmp280_interface_spi_write(uint8_t reg, uint8_t *buf, uint16_t len) {
    unused(reg);
    unused(buf);
    unused(len);

    return 1;
}

static void bmp280_interface_delay_ms(uint32_t ms) {
    osal_msleep(ms);
}

static void bmp280_interface_debug_print(const char *const fmt, ...) {
    va_list args;
    va_start(args, fmt);
    osal_vprintk(fmt, args);
    va_end(args);
}

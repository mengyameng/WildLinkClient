#include "app_init.h"
#include "common_def.h"
#include "errcode.h"
#include "gpio.h"
#include "osal_debug.h"
#include "osal_task.h"
#include "pinctrl.h"
#include "pinctrl_porting.h"
#include "soc_osal.h"

#include "soft_i2c.h"

#ifdef CONFIG_AHT20_TASK_ENABLED
#include "aht20_task.h"
#endif // CONFIG_AHT20_TASK_ENABLED

#ifdef CONFIG_BMP280_TASK_ENABLED
#include "bmp280_task.h"
#endif // CONFIG_BMP280_TASK_ENABLED

#ifdef CONFIG_ATK_LORA_TASK_ENABLED
#include "atk_lora_task.h"
#endif // CONFIG_ATK_LORA_TASK_ENABLED

#if CONFIG_SLE_CLIENT_TASK_ENABLED
#include "sle_client_task.h"
#endif // CONFIG_SLE_CLIENT_TASK_ENABLED

#if CONFIG_EMERGENCY_ALARM_TASK_ENABLED
#include "emergency_alarm_task.h"
#endif // CONFIG_EMERGENCY_ALARM_TASK_ENABLED

#if CONFIG_BLE_SERVER_TASK_ENABLED
#include "ble_server_task.h"
#endif // CONFIG_BLE_SERVER_TASK_ENABLED

osal_task *g_app_init_task_handle;

#if SOFT_I2C_SUPPORT_CONCURRENCY
static osal_mutex g_soft_i2c_mutex;
#endif // SOFT_I2C_SUPPORT_CONCURRENCY
soft_i2c_handle_t g_soft_i2c_handle;

static uint8_t soft_i2c_interface_pin_init(void);
static void soft_i2c_interface_pin_deinit(void);
static uint8_t soft_i2c_interface_scl_write(uint8_t val);
static uint8_t soft_i2c_interface_scl_read(void);
static uint8_t soft_i2c_interface_sda_write(uint8_t val);
static uint8_t soft_i2c_interface_sda_read(void);
#if SOFT_I2C_SUPPORT_CONCURRENCY
static uint8_t soft_i2c_interface_mutex_acquire(void);
static void soft_i2c_interface_mutex_release(void);
#endif // SOFT_I2C_SUPPORT_CONCURRENCY
static void soft_i2c_interface_debug_print(const char *fmt, ...);
static void soft_i2c_interface_delay_ms(uint32_t ms);

static int app_init_task(void *args);

static void app_init_entry(void) {
    osal_kthread_lock();

    g_app_init_task_handle = osal_kthread_create(app_init_task, NULL, "app_init_task",
                                                 CONFIG_APP_INIT_TASK_STACK_SIZE);
    osal_kthread_set_priority(g_app_init_task_handle, CONFIG_APP_INIT_TASK_PRIORITY);

    osal_kthread_unlock();
}

app_run(app_init_entry);

static int app_init_task(void *args) {
    unused(args);
    errcode_t ret;

    osal_printk("%s:%d: in\r\n", __func__, __LINE__);

#if SOFT_I2C_SUPPORT_CONCURRENCY
    if (osal_mutex_init(&g_soft_i2c_mutex)) {
        osal_printk("%s:%d soft i2c mutex init failed\r\n", __func__, __LINE__);
        goto exit;
    }
#endif // SOFT_I2C_SUPPORT_CONCURRENCY

    SOFT_I2C_INIT(&g_soft_i2c_handle);
    SOFT_I2C_LINK_PIN_INIT(&g_soft_i2c_handle, soft_i2c_interface_pin_init);
    SOFT_I2C_LINK_PIN_DEINIT(&g_soft_i2c_handle, soft_i2c_interface_pin_deinit);
    SOFT_I2C_LINK_SCL_WRITE(&g_soft_i2c_handle, soft_i2c_interface_scl_write);
    SOFT_I2C_LINK_SCL_READ(&g_soft_i2c_handle, soft_i2c_interface_scl_read);
    SOFT_I2C_LINK_SDA_WRITE(&g_soft_i2c_handle, soft_i2c_interface_sda_write);
    SOFT_I2C_LINK_SDA_READ(&g_soft_i2c_handle, soft_i2c_interface_sda_read);
    SOFT_I2C_LINK_MUTEX_ACQUIRE(&g_soft_i2c_handle, soft_i2c_interface_mutex_acquire);
    SOFT_I2C_LINK_MUTEX_RELEASE(&g_soft_i2c_handle, soft_i2c_interface_mutex_release);
    SOFT_I2C_LINK_DELAY_MS(&g_soft_i2c_handle, soft_i2c_interface_delay_ms);
    SOFT_I2C_LINK_DEBUG_PRINT(&g_soft_i2c_handle, soft_i2c_interface_debug_print);
    {
        uint8_t soft_i2c_ret;
        soft_i2c_ret = soft_i2c_init(&g_soft_i2c_handle);
        if (soft_i2c_ret != SOFT_I2C_ERR_NONE) {
            osal_printk("%s:%d: soft i2c init failed, ret = %d\r\n", __func__, __LINE__,
                        soft_i2c_ret);
            goto exit;
        }
    }

#if CONFIG_SLE_CLIENT_TASK_ENABLED
    ret = sle_client_task_entry();
    if (ret != ERRCODE_SUCC) {
        osal_printk("%s:%d: sle client task entry error, ret = %#08x\r\n", __func__,
                    __LINE__, ret);
        goto exit;
    }
#endif // CONFIG_SLE_CLIENT_TASK_ENABLED

#if CONFIG_BLE_SERVER_TASK_ENABLED
    ret = ble_server_task_entry();
    if (ret != ERRCODE_SUCC) {
        osal_printk("%s:%d: ble server task entry error, ret = %#08x\r\n", __func__,
                    __LINE__, ret);
        goto exit;
    }
#endif // CONFIG_BLE_SERVER_TASK_ENABLED

#ifdef CONFIG_ATK_LORA_DRIVER_ENABLED
    ret = atk_lora_task_entry();
    if (ret != ERRCODE_SUCC) {
        osal_printk("%s:%d: atk lora task entry error, ret = %#08x\r\n", __func__,
                    __LINE__, ret);
        goto exit;
    }
#endif // CONFIG_ATK_LORA_DRIVER_ENABLED

#ifdef CONFIG_AHT20_DRIVER_ENABLED
    ret = aht20_task_entry();
    if (ret != ERRCODE_SUCC) {
        osal_printk("%s:%d: aht20 task entry error, ret = %#08x\r\n", __func__,
                    __LINE__, ret);
        goto exit;
    }
#endif // CONFIG_AHT20_DRIVER_ENABLED

#ifdef CONFIG_BMP280_DRIVER_ENABLED
    ret = bmp280_task_entry();
    if (ret != ERRCODE_SUCC) {
        osal_printk("%s:%d: bmp280 task entry error, ret = %#08x\r\n", __func__,
                    __LINE__, ret);
        goto exit;
    }
#endif // CONFIG_BMP280_DRIVER_ENABLED

#if CONFIG_EMERGENCY_ALARM_TASK_ENABLED
    ret = emergency_alarm_task_entry();
    if (ret != ERRCODE_SUCC) {
        osal_printk("%s:%d: emergency alarm task entry error, ret = %#08x\r\n",
                    __func__, __LINE__, ret);
        goto exit;
    }
#endif // CONFIG_EMERGENCY_ALARM_TASK_ENABLED

    unused(ret);

exit:
    osal_kthread_destroy(g_app_init_task_handle, 0);
    return 0;
}

static uint8_t soft_i2c_interface_pin_init(void) {
#if defined(CONFIG_SOFT_I2C_SDA_PIN)
#if CONFIG_SOFT_I2C_SDA_PIN == 4
    uapi_pin_set_mode(CONFIG_SOFT_I2C_SDA_PIN, PIN_MODE_2); // gpio
#elif CONFIG_SOFT_I2C_SDA_PIN == 5
    uapi_pin_set_mode(CONFIG_SOFT_I2C_SDA_PIN, PIN_MODE_4); // gpio
#else
    uapi_pin_set_mode(CONFIG_SOFT_I2C_SDA_PIN, HAL_PIO_FUNC_GPIO);
#endif // CONFIG_SOFT_I2C_SDA_PIN==4
#endif // defined(CONFIG_SOFT_I2C_SDA_PIN)
    uapi_pin_set_pull(CONFIG_SOFT_I2C_SDA_PIN, PIN_PULL_TYPE_UP);
    uapi_gpio_set_dir(CONFIG_SOFT_I2C_SDA_PIN, GPIO_DIRECTION_OUTPUT);

#if defined(CONFIG_SOFT_I2C_SCL_PIN)
#if CONFIG_SOFT_I2C_SCL_PIN == 4
    uapi_pin_set_mode(CONFIG_SOFT_I2C_SCL_PIN, PIN_MODE_2);
#elif CONFIG_SOFT_I2C_SCL_PIN == 5
    uapi_pin_set_mode(CONFIG_SOFT_I2C_SCL_PIN, PIN_MODE_4);
#else
    uapi_pin_set_mode(CONFIG_SOFT_I2C_SCL_PIN, HAL_PIO_FUNC_GPIO);
#endif // CONFIG_SOFT_I2C_SCL_PIN==4
#endif // defined(CONFIG_SOFT_I2C_SCL_PIN)
    uapi_pin_set_pull(CONFIG_SOFT_I2C_SCL_PIN, PIN_PULL_TYPE_UP);
    uapi_gpio_set_dir(CONFIG_SOFT_I2C_SCL_PIN, GPIO_DIRECTION_OUTPUT);

    return SOFT_I2C_ERR_NONE;
}

static void soft_i2c_interface_pin_deinit(void) {
    return;
}

static uint8_t soft_i2c_interface_scl_write(uint8_t val) {
    errcode_t ret;

    uapi_gpio_set_dir(CONFIG_SOFT_I2C_SCL_PIN, GPIO_DIRECTION_OUTPUT);
    ret = uapi_gpio_set_val(CONFIG_SOFT_I2C_SCL_PIN,
                            val ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW);
    if (ret != ERRCODE_SUCC) {
        return SOFT_I2C_ERR_FAILED;
    }

    return SOFT_I2C_ERR_NONE;
}

static uint8_t soft_i2c_interface_scl_read(void) {
    uapi_gpio_set_dir(CONFIG_SOFT_I2C_SCL_PIN, GPIO_DIRECTION_INPUT);
    return uapi_gpio_get_val(CONFIG_SOFT_I2C_SCL_PIN) ? 1 : 0;
}

static uint8_t soft_i2c_interface_sda_write(uint8_t val) {
    errcode_t ret;

    uapi_gpio_set_dir(CONFIG_SOFT_I2C_SDA_PIN, GPIO_DIRECTION_OUTPUT);
    ret = uapi_gpio_set_val(CONFIG_SOFT_I2C_SDA_PIN,
                            val ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW);
    if (ret != ERRCODE_SUCC) {
        return SOFT_I2C_ERR_FAILED;
    }

    return SOFT_I2C_ERR_NONE;
}

static uint8_t soft_i2c_interface_sda_read(void) {
    uapi_gpio_set_dir(CONFIG_SOFT_I2C_SDA_PIN, GPIO_DIRECTION_INPUT);
    return uapi_gpio_get_val(CONFIG_SOFT_I2C_SDA_PIN) ? 1 : 0;
}

#if SOFT_I2C_SUPPORT_CONCURRENCY
static uint8_t soft_i2c_interface_mutex_acquire(void) {
    int ret;

    ret = osal_mutex_lock_timeout(&g_soft_i2c_mutex, 2000);
    if (ret != OSAL_SUCCESS) {
        return SOFT_I2C_ERR_MUTEX_ACQUIRE;
    }

    return SOFT_I2C_ERR_NONE;
}

static void soft_i2c_interface_mutex_release(void) {
    osal_mutex_unlock(&g_soft_i2c_mutex);
}
#endif // SOFT_I2C_SUPPORT_CONCURRENCY

static void soft_i2c_interface_delay_ms(uint32_t ms) {
    osal_mdelay(ms);
}

static void soft_i2c_interface_debug_print(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    osal_vprintk(fmt, args);
    va_end(args);
}
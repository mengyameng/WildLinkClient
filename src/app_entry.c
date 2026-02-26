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

#ifdef CONFIG_ATK_LORA_TASK_ENABLED
#include "atk_lora_task.h"
#endif // CONFIG_ATK_LORA_TASK_ENABLED

// #ifdef CONFIG_SC16IS752_TASK_ENABLED
// #include "sc16is752.h"
// #include "sc16is752_task.h"
// #endif // CONFIG_SC16IS752_TASK_ENABLED

#if CONFIG_SH1106_TASK_ENABLED
#include "sh1106_task.h"
#endif // CONFIG_SH1106_TASK_ENABLED

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

// static void peripheral_init(void);
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

#if CONFIG_SH1106_TASK_ENABLED
    ret = sh1106_task_entry();
    if (ret != ERRCODE_SUCC) {
        osal_printk("%s:%d: sh1106 task entry error, ret = %#08x\r\n", __func__,
                    __LINE__, ret);
        goto exit;
    }
#endif // CONFIG_SH1106_TASK_ENABLED

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

// static void peripheral_init(void) {
// #if defined(CONFIG_AHT20_I2C_BUS_ID) || defined(CONFIG_BMP280_I2C_BUS_ID)
//     // i2c master pin init
//     uapi_pin_set_mode(15, 2);
//     uapi_pin_set_mode(16, 2);
//     uapi_pin_set_pull(15, PIN_PULL_TYPE_UP);
//     uapi_pin_set_pull(16, PIN_PULL_TYPE_UP);

// #if defined(CONFIG_AHT20_I2C_BUS_ID) && defined(CONFIG_BMP280_I2C_BUS_ID)
// #if CONFIG_AHT20_I2C_BUS_ID != CONFIG_BMP280_I2C_BUS_ID
// #error
//     "WS63 has only one I2C interface available, both macros should be equal when
//     enabled"
// #else
//     uapi_i2c_master_init(CONFIG_AHT20_I2C_BUS_ID, 400000, 0x0);
// #endif // CONFIG_AHT20_I2C_BUS_ID != CONFIG_BMP280_I2C_BUS_ID
// #else
// #ifdef CONFIG_AHT20_I2C_BUS_ID
//     uapi_i2c_master_init(CONFIG_AHT20_I2C_BUS_ID, 400000, 0x0);
// #elif CONFIG_BMP280_I2C_BUS_ID
//     uapi_i2c_master_init(CONFIG_BMP280_I2C_BUS_ID, 400000, 0x0);
// #endif // CONFIG_AHT20_I2C_BUS_ID
// #endif // defined(CONFIG_AHT20_I2C_BUS_ID) && defined(CONFIG_BMP280_I2C_BUS_ID)
// #endif // defined(CONFIG_AHT20_I2C_BUS_ID) || defined(CONFIG_BMP280_I2C_BUS_ID)

//     // // uart init
//     // uapi_pin_set_mode(7, 2);
//     // uapi_pin_set_mode(8, 2);
//     // uart_attr_t attr = {
//     //     .baud_rate = 115200,
//     //     .data_bits = UART_DATA_BIT_8,
//     //     .stop_bits = UART_STOP_BIT_1,
//     //     .parity = UART_PARITY_NONE};

//     // uart_pin_config_t pin_config = {
//     //     .tx_pin = 8,
//     //     .rx_pin = 7,
//     //     .cts_pin = PIN_NONE,
//     //     .rts_pin = PIN_NONE};

//     // uart_buffer_config_t buffer_config = {
//     //     .rx_buffer = uart_buffer,
//     //     .rx_buffer_size = sizeof(uart_buffer)};

//     // uapi_uart_deinit(2);
//     // uapi_uart_init(2, &pin_config, &attr, NULL, &buffer_config);

//     // // gpio init
//     // uapi_pin_set_mode(0, HAL_PIO_FUNC_GPIO);
//     // uapi_pin_set_pull(0, PIN_PULL_TYPE_UP);
//     // uapi_gpio_set_dir(0, GPIO_DIRECTION_OUTPUT);

// #ifdef CONFIG_SC16IS752_SPI_BUS_ID
// #if CONFIG_SC16IS752_SPI_BUS_ID >= 2
// #error "WS63 has only two SPI interfaces available"
// #else
//     uapi_pin_set_mode(CONFIG_SC16IS752_SPI_MISO_PIN, 3); // MISO
//     uapi_pin_set_mode(CONFIG_SC16IS752_SPI_MOSI_PIN, 3); // MOSI
//     uapi_pin_set_mode(CONFIG_SC16IS752_SPI_SCK_PIN, 3);  // SCK
//     // CS in manually controlled
//     uapi_pin_set_mode(CONFIG_SC16IS752_SPI_CS_PIN, HAL_PIO_FUNC_GPIO);     // CS
//     uapi_gpio_set_dir(CONFIG_SC16IS752_SPI_CS_PIN, GPIO_DIRECTION_OUTPUT); // CS
//     uapi_gpio_set_val(CONFIG_SC16IS752_SPI_CS_PIN, GPIO_LEVEL_HIGH);       // CS
// #endif // CONFIG_SC16IS752_SPI_BUS_ID>=2

//     spi_attr_t config = {0};
//     spi_extra_attr_t ext_config = {0};

//     config.is_slave = false;
//     config.slave_num = 1;
//     config.bus_clk = 32000000;
//     config.freq_mhz = 2;
//     config.clk_polarity = SPI_CFG_CLK_CPOL_0;
//     config.clk_phase = SPI_CFG_CLK_CPHA_0;
//     config.frame_format = SPI_CFG_FRAME_FORMAT_MOTOROLA_SPI;
//     config.spi_frame_format = HAL_SPI_FRAME_FORMAT_STANDARD;
//     config.frame_size = HAL_SPI_FRAME_SIZE_8;
//     config.tmod = HAL_SPI_TRANS_MODE_TXRX;
//     config.sste = SPI_CFG_SSTE_ENABLE;

//     // ext_config.tx_use_dma = false;
//     // ext_config.rx_use_dma = false;
//     // ext_config.qspi_param.wait_cycles = 0x10;
//     // ext_config.qspi_param.wait_cycles = 0x10;
//     uapi_spi_init(CONFIG_SC16IS752_SPI_BUS_ID, &config, &ext_config);

// #if CONFIG_SC16IS752_TASK_ENABLED
//     uapi_pin_set_mode(CONFIG_SC16IS752_INT_PIN, HAL_PIO_FUNC_GPIO);
//     uapi_pin_set_pull(CONFIG_SC16IS752_INT_PIN, PIN_PULL_TYPE_UP);
//     uapi_gpio_set_dir(CONFIG_SC16IS752_INT_PIN, GPIO_DIRECTION_INPUT);
//     uapi_gpio_register_isr_func(CONFIG_SC16IS752_INT_PIN,
//     GPIO_INTERRUPT_FALLING_EDGE,
//                                 sc16is752_isr_callback);
// #endif // CONFIG_SC16IS752_TASK_ENABLED
// #endif // CONFIG_SC16IS752_SPI_BUS_ID
// }

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
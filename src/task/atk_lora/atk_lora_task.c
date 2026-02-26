#include <assert.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdint.h>

#include "common_def.h"
#include "gpio.h"
#include "osal_event.h"
#include "osal_mutex.h"
#include "osal_semaphore.h"
#include "osal_timer.h"
#include "pinctrl.h"
#include "pinctrl_porting.h"
#include "soc_osal.h"
#include "trng.h"
#include "uart.h"

#include "atk_lora.h"
#include "node_telemetry.h"
#include "wlid_link_client_log.h"

#include "atk_lora_task.h"

#if !(defined(__cplusplus) || defined(static_assert))
#define static_assert _Static_assert
#endif //!(defined( __cplusplus) || defined(static_assert))

#ifndef STRINGIFY
#define STRINGIFY(x) #x
#endif // STRINGIFY

#ifndef STRINGIFY_1
#define STRINGIFY_1(x) STRINGIFY(x)
#endif // STRINGIFY_1

// Frame synchronization byte (chosen as 0xAA, a common sync pattern)
#define ATK_LORA_FRAME_SYNC_BYTE 0xaau
static bool lora_frame_sync_byte_found = false;

static_assert(CONFIG_ATK_LORA_TASK_ADDR <= 10,
              "CONFIG_ATK_LORA_TASK_ADDR must be less than or equal to 10");

#define ATK_LORA_PACKET_RECEIVED 0x01U
#define ATK_LORA_PACKET_READY_TO_SEND 0x02U

#pragma pack(1)
typedef struct atk_lora_packet_s {
    struct header_t {
        uint16_t src_addr;
        uint16_t dest_addr;
        uint8_t pack_idx;
    } header;
    struct payload_t {
        uint8_t data_len;
        uint8_t data[CONFIG_ATK_LORA_TASK_PAYLOAD_MAX_SIZE];
    } payload;
    uint8_t crc_8;
} atk_lora_packet_t;
#pragma pack()

static atk_lora_packet_t packet_buffers[2] = {0};
static atk_lora_packet_t *packet_ready_buf = &packet_buffers[0];
static atk_lora_packet_t *packet_active_buf = &packet_buffers[1];

typedef struct {
    uint16_t src_addr;
    uint8_t pack_idx;
} atk_lora_recv_cache_t;
static atk_lora_recv_cache_t recv_cache[10] = {0};

osal_task *g_atk_lora_task_handle;
static osal_mutex g_atk_lora_task_send_mutex;
static osal_event g_atk_lora_event;
static atk_lora_handle_t g_atk_lora_handle;

static uint8_t atk_lora_interface_uart_init(void);
static uint8_t atk_lora_interface_uart_deinit(void);
static uint8_t atk_lora_interface_uart_write(const uint8_t *buffer, uint16_t len);
static uint8_t atk_lora_interface_uart_read(uint8_t *buffer, uint16_t len);
static uint8_t atk_lora_interface_md0_write(uint8_t v);
static void atk_lora_interface_delay_ms(uint32_t ms);
static void atk_lora_interface_debug_print(const char *fmt, ...);
static void atk_lora_recv_cb(const void *buffer, uint16_t length, bool error);

static int atk_lora_task(void *args);
static uint8_t atk_lora_task_calc_crc8(const void *data, size_t data_len,
                                       uint8_t init_crc);

#define ATK_LORA_UART_TIMEOUT_MS 1000

errcode_t atk_lora_task_entry(void) {
    nodeTelemetry_setLocalNodeId(CONFIG_ATK_LORA_TASK_ADDR);
    NodeTelemetry_t *local_node = nodeTelemetry_getLocalNode();
    if (local_node == NULL) {
        WLID_LINK_CLIENT_LOG_ERROR("local node is null\r\n");
        goto _error_return;
    }
    else {
        local_node->heart_rate_min = 0;
        local_node->heart_rate_max = 200;
        local_node->body_temp_min = 0;
        local_node->body_temp_max = 100;
        local_node->blood_oxygen_low = 0;
    }

    if (osal_mutex_init(&g_atk_lora_task_send_mutex) != OSAL_SUCCESS) {
        WLID_LINK_CLIENT_LOG_ERROR("failed to init atk lora task send mutex\r\n");
        goto _error_return;
    }

    if (osal_event_init(&g_atk_lora_event) != OSAL_SUCCESS) {
        WLID_LINK_CLIENT_LOG_ERROR("failed to init atk lora task event\r\n");
        goto mutex_destory;
    }

    g_atk_lora_task_handle = osal_kthread_create(
        atk_lora_task, NULL, STRINGIFY(atk_lora_task), CONFIG_ATK_LORA_TASK_STACK_SIZE);
    if (g_atk_lora_task_handle == NULL) {
        WLID_LINK_CLIENT_LOG_ERROR("failed to create atk lora task\r\n");
        goto event_destory;
    }

    if (osal_kthread_set_priority(g_atk_lora_task_handle, CONFIG_ATK_LORA_TASK_PRIORITY)
        != OSAL_SUCCESS)
    {
        WLID_LINK_CLIENT_LOG_ERROR("failed to set atk lora task priority\r\n");
        goto kthread_destory;
    }

    return ERRCODE_SUCC;
kthread_destory:
    osal_kthread_destroy(g_atk_lora_task_handle, 0);
event_destory:
    osal_event_destroy(&g_atk_lora_event);
mutex_destory:
    osal_mutex_destroy(&g_atk_lora_task_send_mutex);
_error_return:
    return ERRCODE_FAIL;
}

errcode_t atk_lora_task_send_packet(const atk_lora_packet_t *packet) {
    uint8_t ret;

    uint8_t sync_byte = ATK_LORA_FRAME_SYNC_BYTE;
    uint8_t packet_len;

    ret = atk_lora_send_buf(&g_atk_lora_handle, &sync_byte, sizeof(sync_byte));
    if (ret != ATK_LORA_ERR_NONE) {
        WLID_LINK_CLIENT_LOG_ERROR("ret = %" PRIu8 "\r\n", ret);
        goto error;
    }

    packet_len = sizeof(packet->header) + sizeof(packet->payload.data_len)
                 + packet->payload.data_len + sizeof(packet->crc_8);
    ret = atk_lora_send_buf(&g_atk_lora_handle, &packet_len, sizeof(packet_len));
    if (ret != ATK_LORA_ERR_NONE) {
        WLID_LINK_CLIENT_LOG_ERROR("ret = %" PRIu8 "\r\n", ret);
        goto error;
    }

    ret =
        atk_lora_send_buf(&g_atk_lora_handle, &packet->header, sizeof(packet->header));
    if (ret != ATK_LORA_ERR_NONE) {
        WLID_LINK_CLIENT_LOG_ERROR("ret = %" PRIu8 "\r\n", ret);
        goto error;
    }

    ret = atk_lora_send_buf(&g_atk_lora_handle, &packet->payload.data_len,
                            sizeof(packet->payload.data_len));
    if (ret != ATK_LORA_ERR_NONE) {
        WLID_LINK_CLIENT_LOG_ERROR("ret = %" PRIu8 "\r\n", ret);
        goto error;
    }

    ret = atk_lora_send_buf(&g_atk_lora_handle, packet->payload.data,
                            packet->payload.data_len);
    if (ret != ATK_LORA_ERR_NONE) {
        WLID_LINK_CLIENT_LOG_ERROR("ret = %" PRIu8 "\r\n", ret);
        goto error;
    }

    ret = atk_lora_send_buf(&g_atk_lora_handle, &packet->crc_8, sizeof(packet->crc_8));
    if (ret != ATK_LORA_ERR_NONE) {
        WLID_LINK_CLIENT_LOG_ERROR("ret = %" PRIu8 "\r\n", ret);
        goto error;
    }

    return ERRCODE_SUCC;
error:
    return ERRCODE_FAIL;
}

errcode_t atk_lora_task_send_msg(uint16_t dest_addr, const void *data,
                                 size_t data_len) {
    static atk_lora_packet_t packet = {0};
    static uint8_t pack_idx = 0;

    if (data_len > CONFIG_ATK_LORA_TASK_PAYLOAD_MAX_SIZE) {
        WLID_LINK_CLIENT_LOG_ERROR(
            "data length exceeds maximum payload size limit\r\n");
        return ERRCODE_FAIL;
    }

    if (osal_mutex_lock_timeout(&g_atk_lora_task_send_mutex, 5000) != OSAL_SUCCESS) {
        WLID_LINK_CLIENT_LOG_ERROR("lora send mutex lock failed\r\n");
        return ERRCODE_FAIL;
    }

    packet.header.src_addr = CONFIG_ATK_LORA_TASK_ADDR;
    packet.header.dest_addr = dest_addr;
    packet.header.pack_idx = pack_idx++;
    packet.payload.data_len = data_len;
    memcpy(packet.payload.data, data, data_len);

    uint8_t crc8 = 0x00;
    crc8 = atk_lora_task_calc_crc8(&packet.header, sizeof(packet.header), crc8);
    crc8 = atk_lora_task_calc_crc8(&packet.payload.data_len,
                                   sizeof(packet.payload.data_len), crc8);
    crc8 = atk_lora_task_calc_crc8(packet.payload.data, packet.payload.data_len, crc8);
    packet.crc_8 = crc8;

    if (atk_lora_task_send_packet(&packet) != ERRCODE_SUCC) {
        goto failed;
    }

    osal_mutex_unlock(&g_atk_lora_task_send_mutex);
    return ERRCODE_SUCC;
failed:
    osal_mutex_unlock(&g_atk_lora_task_send_mutex);
    return ERRCODE_FAIL;
}

static int atk_lora_task(void *args) {
    unused(args);

    WLID_LINK_CLIENT_LOG_INFO("start\r\n");
    osal_msleep(1000);

    ATK_LORA_LINK_INIT(&g_atk_lora_handle, atk_lora_handle_t);
    ATK_LORA_LINK_UART_INIT(&g_atk_lora_handle, atk_lora_interface_uart_init);
    ATK_LORA_LINK_UART_DEINIT(&g_atk_lora_handle, atk_lora_interface_uart_deinit);
    ATK_LORA_LINK_UART_WRITE(&g_atk_lora_handle, atk_lora_interface_uart_write);
    ATK_LORA_LINK_UART_READ(&g_atk_lora_handle, atk_lora_interface_uart_read);
    ATK_LORA_LINK_MD0_WRITE(&g_atk_lora_handle, atk_lora_interface_md0_write);
    ATK_LORA_LINK_DELAY_MS(&g_atk_lora_handle, atk_lora_interface_delay_ms);
    ATK_LORA_LINK_DEBUG_PRINT(&g_atk_lora_handle, atk_lora_interface_debug_print);

    uint8_t ret;

    ret = atk_lora_init(&g_atk_lora_handle);
    if (ret != 0) {
        WLID_LINK_CLIENT_LOG_ERROR("failed to init lora, ret = %" PRIu8 "\r\n", ret);
    }

    uapi_uart_register_rx_callback(CONFIG_ATK_LORA_TASK_UART_BUS_ID,
                                   UART_RX_CONDITION_FULL_OR_SUFFICIENT_DATA_OR_IDLE,
                                   sizeof(atk_lora_packet_t)
                                       + 1 /*sync byte*/ + 1 /*packet len byte*/,
                                   atk_lora_recv_cb);

    for (;;) {
        osal_msleep(1);

        uint32_t rand_val = 0;
        const errcode_t trng_ret = uapi_drv_cipher_trng_get_random(&rand_val);
        if (trng_ret != ERRCODE_SUCC) {
            WLID_LINK_CLIENT_LOG_ERROR(
                "failed to get trng random number, ret = %" PRIu32 "\r\n", trng_ret);
            rand_val = UINT32_MAX / 2;
        }

        const unsigned int rand_delay_ms =
            1000 + (20000 * (rand_val / (UINT32_MAX + 1.0f)));
        WLID_LINK_CLIENT_LOG_DEBUG("wait %" PRIu32 " ms...\r\n", rand_delay_ms);

        const int event = osal_event_read(
            &g_atk_lora_event, ATK_LORA_PACKET_RECEIVED | ATK_LORA_PACKET_READY_TO_SEND,
            rand_delay_ms, OSAL_WAITMODE_OR | OSAL_WAITMODE_CLR);

        if (event != OSAL_FAILURE) {
            if (event & ATK_LORA_PACKET_RECEIVED) {
                // 校验crc
                uint8_t expected_crc =
                    atk_lora_task_calc_crc8(packet_ready_buf,
                                            sizeof(packet_ready_buf->header)
                                                + packet_ready_buf->payload.data_len,
                                            0x00);
                if (expected_crc != packet_ready_buf->crc_8) {
                    WLID_LINK_CLIENT_LOG_ERROR(
                        "crc error, rx crc = %#02x, expected crc = %#02x\r\n",
                        packet_ready_buf->crc_8, expected_crc);
                    continue;
                }

                // 检查是否已经收到过包了
                bool received = false;
                for (uint8_t i = 0; i < sizeof(recv_cache) / sizeof(recv_cache[0]); i++)
                {
                    if ((packet_ready_buf->header.src_addr == recv_cache[i].src_addr)
                        && (packet_ready_buf->header.pack_idx
                            == recv_cache[i].pack_idx))
                    {
                        received = true;
                    }
                }
                if (received) {
                    // 跳过已经受到过了的包
                    WLID_LINK_CLIENT_LOG_WARN(
                        "Duplicate packet detected, skipping...\r\n");
                    continue;
                }
                else {
                    // 记录已经收到过了
                    static uint8_t recv_cache_wr_ptr = 0;
                    recv_cache[recv_cache_wr_ptr].src_addr =
                        packet_ready_buf->header.src_addr;
                    recv_cache[recv_cache_wr_ptr].pack_idx =
                        packet_ready_buf->header.pack_idx;
                    recv_cache_wr_ptr++;
                    recv_cache_wr_ptr %= (sizeof(recv_cache) / sizeof(recv_cache[0]));
                }

                if (packet_ready_buf->header.src_addr == CONFIG_ATK_LORA_TASK_ADDR) {
                    // 其它节点转发给自己的，不处理
                    WLID_LINK_CLIENT_LOG_DEBUG("Packet from self, ignoring...\r\n");
                    continue;
                }
                else if (packet_ready_buf->header.dest_addr == 0xffff) {
                    // 广播地址需要转发
                    WLID_LINK_CLIENT_LOG_DEBUG(
                        "Broadcast packet received, forwarding...\r\n");
                    osal_event_write(&g_atk_lora_event, ATK_LORA_PACKET_READY_TO_SEND);
                }
                else if (packet_ready_buf->header.dest_addr
                         != CONFIG_ATK_LORA_TASK_ADDR)
                {
                    // 不是发给自己的，无脑转发
                    WLID_LINK_CLIENT_LOG_DEBUG(
                        "Packet not destined for this node, forwarding...\r\n");
                    osal_event_write(&g_atk_lora_event, ATK_LORA_PACKET_READY_TO_SEND);
                    continue;
                }

                WLID_LINK_CLIENT_LOG_DEBUG("new packet received from %" PRIu16
                                           ", data len = %" PRIu8 ", data = ",
                                           packet_ready_buf->header.src_addr,
                                           packet_ready_buf->payload.data_len);
                for (uint8_t i = 0; i < packet_ready_buf->payload.data_len; i++) {
                    WLID_LINK_CLIENT_LOG_PRINT_DEBUG("%#02" PRIx8 " ",
                                                     packet_ready_buf->payload.data[i]);
                }
                WLID_LINK_CLIENT_LOG_PRINT_DEBUG("\r\n");

                NodeTelemetry_t *node;
                node = nodeTelemetry_getNode(packet_ready_buf->header.src_addr);
                if (node == NULL) {
                    nodeTelemetry_newNode(packet_ready_buf->header.src_addr);
                    node = nodeTelemetry_getNode(packet_ready_buf->header.src_addr);
                    if (node == NULL) {
                        WLID_LINK_CLIENT_LOG_DEBUG(
                            "failed to allocate buffer for this node\r\n");
                        continue;
                    }
                }

                memcpy(node, packet_ready_buf->payload.data,
                       packet_ready_buf->payload.data_len);
                // The timestamp is not only used to record the collection time, but
                // also helps determine whether a node has not updated its data for an
                // extended period (which may indicate that the node is offline or this
                // node cannot receive packets from it). Normally, the UTC timestamp
                // should be filled by the sender, but for implementation convenience,
                // it is currently populated with the number of seconds elapsed since
                // the system started until this packet was received.
                node->timestamp = osal_jiffies_to_msecs(osal_get_jiffies());
            }
            if (event & ATK_LORA_PACKET_READY_TO_SEND) {
                atk_lora_task_send_packet(packet_ready_buf);
            }
        }
        else {
            WLID_LINK_CLIENT_LOG_DEBUG("send local node data...\r\n");
            NodeTelemetry_t *const local_node = nodeTelemetry_getLocalNode();
            if (local_node == NULL) {
                WLID_LINK_CLIENT_LOG_WARN("local node is NULL\r\n");
            }
            else {
                local_node->timestamp = osal_jiffies_to_msecs(osal_get_jiffies());

                errcode_t send_ret =
                    atk_lora_task_send_msg(0xffff, local_node, sizeof(*local_node));
                if (send_ret != ERRCODE_SUCC) {
                    WLID_LINK_CLIENT_LOG_WARN(
                        "send local node data failed, ret = %#" PRIx32 "\r\n",
                        send_ret);
                }
            }
        }
    }

    return 0;
}

static uint8_t atk_lora_task_calc_crc8(const void *data, size_t data_len,
                                       uint8_t init_crc) {
    static const uint8_t crc8_table[] = {
        0x00, 0x2f, 0x5e, 0x71, 0xbc, 0x93, 0xe2, 0xcd, 0x57, 0x78, 0x09, 0x26, 0xeb,
        0xc4, 0xb5, 0x9a, 0xae, 0x81, 0xf0, 0xdf, 0x12, 0x3d, 0x4c, 0x63, 0xf9, 0xd6,
        0xa7, 0x88, 0x45, 0x6a, 0x1b, 0x34, 0x73, 0x5c, 0x2d, 0x02, 0xcf, 0xe0, 0x91,
        0xbe, 0x24, 0x0b, 0x7a, 0x55, 0x98, 0xb7, 0xc6, 0xe9, 0xdd, 0xf2, 0x83, 0xac,
        0x61, 0x4e, 0x3f, 0x10, 0x8a, 0xa5, 0xd4, 0xfb, 0x36, 0x19, 0x68, 0x47, 0xe6,
        0xc9, 0xb8, 0x97, 0x5a, 0x75, 0x04, 0x2b, 0xb1, 0x9e, 0xef, 0xc0, 0x0d, 0x22,
        0x53, 0x7c, 0x48, 0x67, 0x16, 0x39, 0xf4, 0xdb, 0xaa, 0x85, 0x1f, 0x30, 0x41,
        0x6e, 0xa3, 0x8c, 0xfd, 0xd2, 0x95, 0xba, 0xcb, 0xe4, 0x29, 0x06, 0x77, 0x58,
        0xc2, 0xed, 0x9c, 0xb3, 0x7e, 0x51, 0x20, 0x0f, 0x3b, 0x14, 0x65, 0x4a, 0x87,
        0xa8, 0xd9, 0xf6, 0x6c, 0x43, 0x32, 0x1d, 0xd0, 0xff, 0x8e, 0xa1, 0xe3, 0xcc,
        0xbd, 0x92, 0x5f, 0x70, 0x01, 0x2e, 0xb4, 0x9b, 0xea, 0xc5, 0x08, 0x27, 0x56,
        0x79, 0x4d, 0x62, 0x13, 0x3c, 0xf1, 0xde, 0xaf, 0x80, 0x1a, 0x35, 0x44, 0x6b,
        0xa6, 0x89, 0xf8, 0xd7, 0x90, 0xbf, 0xce, 0xe1, 0x2c, 0x03, 0x72, 0x5d, 0xc7,
        0xe8, 0x99, 0xb6, 0x7b, 0x54, 0x25, 0x0a, 0x3e, 0x11, 0x60, 0x4f, 0x82, 0xad,
        0xdc, 0xf3, 0x69, 0x46, 0x37, 0x18, 0xd5, 0xfa, 0x8b, 0xa4, 0x05, 0x2a, 0x5b,
        0x74, 0xb9, 0x96, 0xe7, 0xc8, 0x52, 0x7d, 0x0c, 0x23, 0xee, 0xc1, 0xb0, 0x9f,
        0xab, 0x84, 0xf5, 0xda, 0x17, 0x38, 0x49, 0x66, 0xfc, 0xd3, 0xa2, 0x8d, 0x40,
        0x6f, 0x1e, 0x31, 0x76, 0x59, 0x28, 0x07, 0xca, 0xe5, 0x94, 0xbb, 0x21, 0x0e,
        0x7f, 0x50, 0x9d, 0xb2, 0xc3, 0xec, 0xd8, 0xf7, 0x86, 0xa9, 0x64, 0x4b, 0x3a,
        0x15, 0x8f, 0xa0, 0xd1, 0xfe, 0x33, 0x1c, 0x6d, 0x42,
    };

    const uint8_t *data_bytes = (const uint8_t *)data;
    while (data_len--) {
        init_crc = crc8_table[init_crc ^ (*data_bytes++)];
    }

    return init_crc;
}

static uint8_t atk_lora_interface_uart_init(void) {
    osal_printk("%s:%d: in\r\n", __func__, __LINE__);
    static uint8_t uart_rx_buffer[64];
    errcode_t ret;

#if CONFIG_ATK_LORA_TASK_UART_BUS_ID == 1
    uapi_pin_set_mode(GPIO_15, PIN_MODE_1); // uart1 txd
    uapi_pin_set_mode(GPIO_16, PIN_MODE_1); // uart1 rxd
#elif CONFIG_ATK_LORA_TASK_UART_BUS_ID == 2
    uapi_pin_set_mode(GPIO_08, PIN_MODE_2); // uart2 txd
    uapi_pin_set_mode(GPIO_07, PIN_MODE_2); // uart2 rxd
#else
#error "ATK_LORA_TASK_UART_BUS_ID must be 1 or 2"
#endif // ATK_LORA_TASK_UART_BUS_ID==1

    uart_attr_t uart_attr = {.baud_rate = 115200,
                             .data_bits = UART_DATA_BIT_8,
                             .stop_bits = UART_STOP_BIT_1,
                             .parity = UART_PARITY_NONE,
                             .flow_ctrl = UART_FLOW_CTRL_NONE};

    uart_pin_config_t uart_pin_config = {
#if CONFIG_ATK_LORA_TASK_UART_BUS_ID == 1
        .tx_pin = GPIO_15,
        .rx_pin = GPIO_16,
#elif CONFIG_ATK_LORA_TASK_UART_BUS_ID == 2
        .tx_pin = GPIO_08,
        .rx_pin = GPIO_07,
#endif // CONFIG_ATK_LORA_TASK_UART_BUS_ID == 1
        .cts_pin = PIN_NONE,
        .rts_pin = PIN_NONE};

    uart_buffer_config_t uart_buffer_config = {
        .rx_buffer = uart_rx_buffer, .rx_buffer_size = sizeof(uart_rx_buffer)};

    uapi_uart_deinit(CONFIG_ATK_LORA_TASK_UART_BUS_ID);
    ret = uapi_uart_init(CONFIG_ATK_LORA_TASK_UART_BUS_ID, &uart_pin_config, &uart_attr,
                         NULL, &uart_buffer_config);
    if (ret != ERRCODE_SUCC) {
        return ATK_LORA_ERR_FAILED;
    }

    return ATK_LORA_ERR_NONE;
}

static uint8_t atk_lora_interface_uart_deinit(void) {
    uapi_uart_deinit(CONFIG_ATK_LORA_TASK_UART_BUS_ID);
    return ATK_LORA_ERR_NONE;
}

static uint8_t atk_lora_interface_uart_write(const uint8_t *buffer, uint16_t len) {
    uint32_t cnt;

    cnt = uapi_uart_write(CONFIG_ATK_LORA_TASK_UART_BUS_ID, buffer, len,
                          ATK_LORA_UART_TIMEOUT_MS);
    // WLID_LINK_CLIENT_LOG_DEBUG("write cnt = %d\r\n", cnt);

    unused(cnt);

    return ATK_LORA_ERR_NONE;
}

static uint8_t atk_lora_interface_uart_read(uint8_t *buffer, uint16_t len) {
    len = uapi_uart_read(CONFIG_ATK_LORA_TASK_UART_BUS_ID, buffer, len,
                         ATK_LORA_UART_TIMEOUT_MS);
    // WLID_LINK_CLIENT_LOG_DEBUG("read len = %" PRIu16 "\r\n", len);
    // for (uint16_t i = 0; i < len; i++) {
    //     WLID_LINK_CLIENT_LOG_PRINT_DEBUG("%#02" PRIx16 " ", buffer[i]);
    // }
    // WLID_LINK_CLIENT_LOG_PRINT_DEBUG("\r\n");

    return ATK_LORA_ERR_NONE;
}

static uint8_t atk_lora_interface_md0_write(uint8_t v) {
    // uapi_gpio_set_val(CONFIG_ATK_LORA_TASK_MD0_PIN,
    //                   v ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW);
    return ATK_LORA_ERR_NONE;
}

static void atk_lora_interface_delay_ms(uint32_t ms) {
    osal_msleep(ms);
}

static void atk_lora_interface_debug_print(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    osal_vprintk(fmt, args);
    va_end(args);
}

static void atk_lora_recv_cb(const void *buffer, uint16_t length, bool error) {
#if 0
    unused(error);

    static uint8_t packet_len = 0;
    static uint8_t packet_data_idx = 0;
    const uint8_t *const buf_bytes = (const uint8_t *)buffer;

    uint16_t i = 0;
    if (packet_len == 0) {
        packet_len = buf_bytes[0];

        WLID_LINK_CLIENT_LOG_DEBUG("packet_len = %" PRIu8 "\r\n", packet_len);
        if (packet_len > sizeof(atk_lora_packet_t)) {
            WLID_LINK_CLIENT_LOG_DEBUG("invalid lora packet len = %" PRIu16 "\r\n",
                                       packet_len);
            packet_len = 0;
            return;
        }

        i = 1;
        packet_data_idx = 0;
    }
    uint8_t *active_buf_bytes = (uint8_t *)packet_active_buf;
    for (; i < length && packet_data_idx < packet_len; i++) {
        active_buf_bytes[packet_data_idx++] = buf_bytes[i];
    }

    if (packet_data_idx >= packet_len) {
        if (length > i) {
            WLID_LINK_CLIENT_LOG_DEBUG("warn: received bytes exceed expected packet "
                                       "length, expected_packet_len = %" PRIu8
                                       ", excess_bytes = %" PRIu8 "\r\n",
                                       packet_len, length - i);
        }

        uint16_t crc8_offset = sizeof(packet_active_buf->header)
                               + sizeof(packet_active_buf->payload.data_len)
                               + packet_active_buf->payload.data_len;
        WLID_LINK_CLIENT_LOG_DEBUG("payload data len = %" PRIu8 "\r\n",
                                   packet_active_buf->payload.data_len);

        if (crc8_offset > packet_len - 1) {
            WLID_LINK_CLIENT_LOG_DEBUG("invalid crc8 offset = %" PRIu8 "\r\n",
                                       crc8_offset);
            packet_len = 0;
            packet_data_idx = 0;
            return;
        }
        else {
            packet_active_buf->crc_8 =
                packet_active_buf->payload.data[packet_active_buf->payload.data_len];
        }

        atk_lora_packet_t *temp = packet_active_buf;
        packet_active_buf = packet_ready_buf;
        packet_ready_buf = temp;

        WLID_LINK_CLIENT_LOG_DEBUG("new packet received\r\n");
        osal_event_write(&g_atk_lora_event, ATK_LORA_PACKET_RECEIVED);

        packet_len = 0;
        packet_data_idx = 0;
    }
#else
    unused(error);

    // Frame synchronization state
    static bool sync_byte_found = false;

    bool packet_processed = false;

    // Packet reception state
    static uint8_t packet_len = 0;
    static uint8_t packet_data_idx = 0;

    const uint8_t *const buf_bytes = (const uint8_t *)buffer;
    uint16_t i = 0;

    while (i < length) {
        // Step 1: Search for frame sync byte
        if (!sync_byte_found) {
            if (buf_bytes[i] == ATK_LORA_FRAME_SYNC_BYTE) {
                WLID_LINK_CLIENT_LOG_INFO(
                    "Frame sync byte " STRINGIFY_1(
                        ATK_LORA_FRAME_SYNC_BYTE) " found at position %" PRIu16 "\r\n",
                    i);
                sync_byte_found = true;
                i++;
            }
            else {
                // Skip bytes until sync byte is found
                WLID_LINK_CLIENT_LOG_DEBUG(
                    "Skipping byte 0x%02" PRIu8 ", waiting for sync\r\n", buf_bytes[i]);
                i++;
                continue;
            }
        }

        // Step 2: Parse packet length
        if (packet_len == 0) {
            if (i >= length) {
                // Need more data for packet length
                WLID_LINK_CLIENT_LOG_DEBUG("Need more data for packet length\r\n");
                break;
            }

            packet_len = buf_bytes[i];
            WLID_LINK_CLIENT_LOG_DEBUG("Packet length = %" PRIu8 " bytes\r\n",
                                       packet_len);

            // Validate packet length
            if (packet_len > sizeof(atk_lora_packet_t)) {
                WLID_LINK_CLIENT_LOG_ERROR("Invalid packet length = %" PRIu8
                                           " (max: %zu)\r\n",
                                           packet_len, sizeof(atk_lora_packet_t));
                packet_len = 0;
                sync_byte_found = false;
                break;
            }

            i++;
            packet_data_idx = 0;
            WLID_LINK_CLIENT_LOG_DEBUG("Ready to receive packet data\r\n");
        }

        // Step 3: Receive packet data
        uint8_t *active_buf_bytes = (uint8_t *)packet_active_buf;
        for (; i < length && packet_data_idx < packet_len; i++) {
            active_buf_bytes[packet_data_idx++] = buf_bytes[i];
        }

        // Step 4: Process complete packet
        if (packet_data_idx >= packet_len) {
            WLID_LINK_CLIENT_LOG_DEBUG("Packet reception complete\r\n");

            // Check for excess bytes
            if (length > i) {
                WLID_LINK_CLIENT_LOG_WARN(
                    "Excess bytes detected: %" PRIu16 " bytes\r\n", length - i);
            }

            // Calculate CRC offset
            uint16_t crc8_offset = sizeof(packet_active_buf->header)
                                   + sizeof(packet_active_buf->payload.data_len)
                                   + packet_active_buf->payload.data_len;

            WLID_LINK_CLIENT_LOG_DEBUG("Payload data length = %" PRIu8 " bytes\r\n",
                                       packet_active_buf->payload.data_len);
            WLID_LINK_CLIENT_LOG_DEBUG("CRC offset = %" PRIu16 "\r\n", crc8_offset);

            // Validate CRC offset
            if (crc8_offset > packet_len - 1) {
                WLID_LINK_CLIENT_LOG_ERROR("Invalid CRC offset = %" PRIu16
                                           " (packet len = %" PRIu8 ")\r\n",
                                           crc8_offset, packet_len);
                packet_len = 0;
                packet_data_idx = 0;
                sync_byte_found = false;
                break;
            }

            // Extract CRC
            packet_active_buf->crc_8 =
                packet_active_buf->payload.data[packet_active_buf->payload.data_len];
            WLID_LINK_CLIENT_LOG_DEBUG("Packet CRC = 0x%02" PRIx8 "\r\n",
                                       packet_active_buf->crc_8);

            // Swap buffers
            atk_lora_packet_t *temp = packet_active_buf;
            packet_active_buf = packet_ready_buf;
            packet_ready_buf = temp;

            packet_processed = true;
            // Notify task
            WLID_LINK_CLIENT_LOG_INFO("New packet received successfully\r\n");
            osal_event_write(&g_atk_lora_event, ATK_LORA_PACKET_RECEIVED);

            // Reset state for next packet
            packet_len = 0;
            packet_data_idx = 0;
            sync_byte_found = false;
        }
    }

    if (!sync_byte_found && !packet_processed) {
        WLID_LINK_CLIENT_LOG_WARN(
            "During a function call, the frame synchronization byte was not found\r\n");
    }

#endif
}
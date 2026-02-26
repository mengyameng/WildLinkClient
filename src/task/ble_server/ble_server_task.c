#include <inttypes.h>
#include <stdbool.h>
#include <string.h>

#include "bts_gatt_server.h"
#include "bts_le_gap.h"
#include "common_def.h"
#include "soc_osal.h"

#if CONFIG_EMERGENCY_ALARM_TASK_ENABLED
#include "emergency_alarm_task.h"
#endif // CONFIG_EMERGENCY_ALARM_TASK_ENABLED
#include "node_telemetry.h"
#include "wlid_link_client_log.h"

#include "ble_server_task.h"

#ifndef STRINGIFY
#define STRINGIFY(x) #x
#endif // STRINGIFY

#ifndef CONFIG_BLE_SERVER_TASK_GAP_ADV_ID
#define CONFIG_BLE_SERVER_TASK_GAP_ADV_ID 0x01
#endif // CONFIG_BLE_SERVER_TASK_GAP_ADV_ID

osal_task *g_ble_server_task_handle;

static const uint8_t g_ble_server_name[] = "WildLinkClient";
static const bd_addr_t g_ble_server_addr = {
    .type = BT_ADDRESS_TYPE_PUBLIC_DEVICE_ADDRESS,
    .addr = {0x02, 0x00, 0x00, 0x00, 0x00, CONFIG_BLE_SERVER_MAC_ADDR_LAST_OCTET}};
static uint8_t g_ble_server_id = 0;
static uint16_t g_ble_service_handle = 0;
static uint16_t g_ble_server_tx_characteristic_value_handle = 0;
static uint16_t g_ble_server_rx_characteristic_value_handle = 0;
static uint16_t g_ble_server_conn_id = 0;
static bool g_ble_server_is_connected = false;

static void ble_server_task_for_each_teammte_node_cb(NodeTelemetry_t *node);

static bt_uuid_t *ble_server_task_set_uuid16(bt_uuid_t *buf, uint16_t uuid);
static errcode_t
ble_server_task_add_tx_characteristic(uint8_t server_id, uint16_t service_handle,
                                      uint16_t tx_charateristic_uuid,
                                      uint16_t *out_tx_charateristic_value_handle);
static errcode_t
ble_server_task_add_rx_characteristic(uint8_t server_id, uint16_t service_handle,
                                      uint16_t rx_charateristic_uuid,
                                      uint16_t *out_rx_charateristic_value_handle);
static errcode_t ble_server_task_set_adv_data(void);

// gap
static void ble_server_task_gap_ble_start_adv_callback(uint8_t adv_id,
                                                       adv_status_t status);
static void ble_server_task_gap_ble_connect_state_changed_callback(
    uint16_t conn_id, bd_addr_t *addr, gap_ble_conn_state_t conn_state,
    gap_ble_pair_state_t pair_state, gap_ble_disc_reason_t disc_reason);
static void ble_server_task_gap_ble_stop_adv_callback(uint8_t adv_id,
                                                      adv_status_t status);
static void ble_server_task_gap_ble_pair_result_callback(uint16_t conn_id,
                                                         const bd_addr_t *addr,
                                                         errcode_t status);

// gatts
static void ble_server_task_gatts_add_service_callback(uint8_t server_id,
                                                       bt_uuid_t *uuid, uint16_t handle,
                                                       errcode_t status); // TODO
static void ble_server_task_gatts_add_characteristic_callback(
    uint8_t server_id, bt_uuid_t *uuid, uint16_t service_handle,
    gatts_add_character_result_t *result, errcode_t status); // TODO
static void ble_server_task_gatts_add_descriptor_callback(uint8_t server_id,
                                                          bt_uuid_t *uuid,
                                                          uint16_t service_handle,
                                                          uint16_t handle,
                                                          errcode_t status); // TODO
static void ble_server_task_gatts_start_service_callback(uint8_t server_id,
                                                         uint16_t handle,
                                                         errcode_t status);
static void
ble_server_task_gatts_read_request_callback(uint8_t server_id, uint16_t conn_id,
                                            gatts_req_read_cb_t *read_cb_para,
                                            errcode_t status);
static void
ble_server_task_gatts_write_request_callback(uint8_t server_id, uint16_t conn_id,
                                             gatts_req_write_cb_t *write_cb_para,
                                             errcode_t status);
static void ble_server_task_gatts_mtu_changed_callback(uint8_t server_id,
                                                       uint16_t conn_id,
                                                       uint16_t mtu_size,
                                                       errcode_t status);

static int ble_server_task(void *args);

errcode_t ble_server_task_entry(void) {
    g_ble_server_task_handle =
        osal_kthread_create(ble_server_task, NULL, STRINGIFY(ble_server_task),
                            CONFIG_BLE_SERVER_TASK_STACK_SIZE);
    if (g_ble_server_task_handle == NULL) {
        WLID_LINK_CLIENT_LOG_ERROR("failed to create ble server task\r\n");
        goto _error_return;
    }

    if (osal_kthread_set_priority(g_ble_server_task_handle,
                                  CONFIG_BLE_SERVER_TASK_PRIORITY)
        != OSAL_SUCCESS)
    {
        WLID_LINK_CLIENT_LOG_ERROR("failed to set ble server task priority\r\n");
        goto _clean_kthread;
    }

    return ERRCODE_SUCC;
_clean_kthread:
    osal_kthread_destroy(g_ble_server_task_handle, 0);
_error_return:
    return ERRCODE_FAIL;
}

static void ble_server_task_gatts_start_service_callback(uint8_t server_id,
                                                         uint16_t handle,
                                                         errcode_t status) {
    if (status != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR("status = %#" PRIx32 "\r\n", status);
    }

    WLID_LINK_CLIENT_LOG_INFO("server id = %" PRIu8 ", handle = %" PRIu16 "\r\n",
                              server_id, handle);

    unused(server_id);
    unused(handle);
}

static void
ble_server_task_gatts_read_request_callback(uint8_t server_id, uint16_t conn_id,
                                            gatts_req_read_cb_t *read_cb_para,
                                            errcode_t status) {
    if (status != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR("status = %#" PRIx32 "\r\n", status);
    }

    WLID_LINK_CLIENT_LOG_DEBUG("server_id = %" PRIu8 ", conn_id = %" PRIu16, server_id,
                               conn_id);
    if (read_cb_para != NULL) {
        WLID_LINK_CLIENT_LOG_PRINT_DEBUG(
            ", request_id = %" PRIu16 ", handle = %" PRIu16 ", offset = %" PRIu16
            ", need_rsp = %" PRIu8 ", need_authorize = %" PRIu8 ", long read = %" PRIu8,
            read_cb_para->request_id, read_cb_para->handle, read_cb_para->offset,
            read_cb_para->need_rsp, read_cb_para->need_authorize,
            read_cb_para->is_long);
    }
    WLID_LINK_CLIENT_LOG_PRINT_DEBUG("\r\n");

    unused(server_id);
    unused(conn_id);
    unused(read_cb_para);
}

static void
ble_server_task_gatts_write_request_callback(uint8_t server_id, uint16_t conn_id,
                                             gatts_req_write_cb_t *write_cb_para,
                                             errcode_t status) {
    if (status != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR("status = %#" PRIx32 "\r\n", status);
    }

    WLID_LINK_CLIENT_LOG_DEBUG(
        "server_id = %" PRIu8 ", conn_id = %" PRIu16 ", status = %#" PRIx32
        ", request_id = %" PRIu16 ", handle = %" PRIu16 ", offset = %" PRIu16
        ", need_rsp = %" PRIu8 ", need_authorize = %" PRIu8 ", prepare_write = %" PRIu8,
        server_id, conn_id, status, write_cb_para->request_id, write_cb_para->handle,
        write_cb_para->offset, write_cb_para->need_rsp, write_cb_para->need_authorize,
        write_cb_para->is_prep);

    if (write_cb_para != NULL) {
        WLID_LINK_CLIENT_LOG_PRINT_DEBUG(", length = %" PRIu16 ", value = ",
                                         write_cb_para->length);

        if (write_cb_para->value != NULL) {
            for (uint16_t i = 0; i < write_cb_para->length; i++) {
                WLID_LINK_CLIENT_LOG_PRINT_DEBUG("0x%02x ", write_cb_para->value[i]);
            }
        }
        WLID_LINK_CLIENT_LOG_PRINT_DEBUG("\r\n");

        if (write_cb_para->handle == g_ble_server_rx_characteristic_value_handle) {
            WLID_LINK_CLIENT_LOG_DEBUG(
                "rx characteristic value write, length = %" PRIu16 "\r\n",
                write_cb_para->length);
            if (write_cb_para->length < sizeof(NodeTelemetry_t)) {
                WLID_LINK_CLIENT_LOG_ERROR("invalid write length! skip it\r\n");
            }
            else {
                NodeTelemetry_t *write_node = (NodeTelemetry_t *)write_cb_para->value;
                NodeTelemetry_t *const teammate_node =
                    nodeTelemetry_getNode(write_node->id);
                if (teammate_node == NULL) {
                    WLID_LINK_CLIENT_LOG_ERROR(
                        "invalid node id = %" PRIu8 "! skip it\r\n", write_node->id);
                }
                else {
                    memcpy(teammate_node, write_node, sizeof(NodeTelemetry_t));
                }
            }
        }
    }
    else {
        WLID_LINK_CLIENT_LOG_PRINT_DEBUG("\r\n");
    }

    unused(server_id);
    unused(conn_id);
    unused(write_cb_para);
}

static void ble_server_task_gatts_mtu_changed_callback(uint8_t server_id,
                                                       uint16_t conn_id,
                                                       uint16_t mtu_size,
                                                       errcode_t status) {
    if (status != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR("status = %#" PRIx32 "\r\n", status);
    }

    WLID_LINK_CLIENT_LOG_INFO("server_id = %" PRIu8 ", conn_id = %" PRIu16
                              ", mtu_size = %" PRIu16 "\r\n",
                              server_id, conn_id, mtu_size);

    unused(server_id);
    unused(conn_id);
    unused(mtu_size);
}

static int ble_server_task(void *args) {
    unused(args);

    WLID_LINK_CLIENT_LOG_DEBUG("wait a few seconds for ble to be ready\r\n");
    osal_msleep(2000);
    WLID_LINK_CLIENT_LOG_INFO("start\r\n");

    errcode_t ret;
    gap_ble_callbacks_t gap_ble_cbs = {0};
    gatts_callbacks_t gatts_cbs = {0};
    bt_uuid_t uuid = {0};
    gap_ble_adv_params_t adv_params = {0};

    ret = enable_ble();
    if (ret != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR("enable_ble failed, ret = %#" PRIx32 "\r\n", ret);
        goto _error_return;
    }

    // register gap ble callbacks
    gap_ble_cbs.start_adv_cb = ble_server_task_gap_ble_start_adv_callback;
    gap_ble_cbs.stop_adv_cb = ble_server_task_gap_ble_stop_adv_callback;
    gap_ble_cbs.conn_state_change_cb =
        ble_server_task_gap_ble_connect_state_changed_callback;
    gap_ble_cbs.pair_result_cb = ble_server_task_gap_ble_pair_result_callback;
    ret = gap_ble_register_callbacks(&gap_ble_cbs);
    if (ret != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR(
            "gap_ble_register_callbacks failed, ret = %#" PRIx32 "\r\n", ret);
        goto _error_return;
    }

    // register gatts callbacks
    // gatts_cbs.add_service_cb = ble_server_task_gatts_add_service_callback;
    // gatts_cbs.add_characteristic_cb =
    // ble_server_task_gatts_add_characteristic_callback; gatts_cbs.add_descriptor_cb =
    // ble_server_task_gatts_add_descriptor_callback;
    gatts_cbs.start_service_cb = ble_server_task_gatts_start_service_callback;
    gatts_cbs.mtu_changed_cb = ble_server_task_gatts_mtu_changed_callback;
    gatts_cbs.write_request_cb = ble_server_task_gatts_write_request_callback;
    gatts_cbs.read_request_cb = ble_server_task_gatts_read_request_callback;
    ret = gatts_register_callbacks(&gatts_cbs);
    if (ret != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR(
            "gatts_register_callbacks failed, ret = %#" PRIx32 "\r\n", ret);
        goto _error_return;
    }

    // set local name
    ret = gap_ble_set_local_name(g_ble_server_name, array_size(g_ble_server_name));
    if (ret != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR(
            "gap_ble_set_local_name failed, ret = %#" PRIx32 "\r\n", ret);
        goto _error_return;
    }

    // set local addr
    ret = gap_ble_set_local_addr(&g_ble_server_addr);
    if (ret != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR(
            "gap_ble_set_local_addr failed, ret = %#" PRIx32 "\r\n", ret);
        goto _error_return;
    }

    // register server
    ble_server_task_set_uuid16(&uuid, CONFIG_BLE_SERVER_APP_UUID);
    ret = gatts_register_server(&uuid, &g_ble_server_id);
    if (ret != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR(
            "gatts_register_server failed, ret = %#" PRIx32 "\r\n", ret);
        goto _error_return;
    }

    // register service
    ble_server_task_set_uuid16(&uuid, CONFIG_BLE_SERVER_SERVICE_UUID);
    ret = gatts_add_service_sync(g_ble_server_id, &uuid, true, &g_ble_service_handle);
    if (ret != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR(
            "gatts_add_service_sync failed, ret = %#" PRIx32 "\r\n", ret);
        goto _error_return;
    }

    // add tx characteristic
    ret = ble_server_task_add_tx_characteristic(
        g_ble_server_id, g_ble_service_handle, CONFIG_BLE_SERVER_TX_CHARACTERISTIC_UUID,
        &g_ble_server_tx_characteristic_value_handle);
    if (ret != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR(
            "ble_server_task_add_tx_characteristic failed, ret = %#" PRIx32 "\r\n",
            ret);
        goto _error_return;
    }

    // add rx characteristic
    ret = ble_server_task_add_rx_characteristic(
        g_ble_server_id, g_ble_service_handle, CONFIG_BLE_SERVER_RX_CHARACTERISTIC_UUID,
        &g_ble_server_rx_characteristic_value_handle);
    if (ret != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR(
            "ble_server_task_add_rx_characteristic failed, ret = %#" PRIx32 "\r\n",
            ret);
        goto _error_return;
    }

    // start service
    ret = gatts_start_service(g_ble_server_id, g_ble_service_handle);
    if (ret != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR("gatts_start_service failed, ret = %#" PRIx32 "\r\n",
                                   ret);
        goto _error_return;
    }

    ret = gatts_set_mtu_size(g_ble_server_id, sizeof(NodeTelemetry_t)
                                                  + 2 /*l2cap head*/ + 1 /*att head*/);
    if (ret != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR(
            "failed to set gatts mtu size, ret = %#" PRIx32 "\r\n", ret);
    }

    // set adv data
    ret = ble_server_task_set_adv_data();
    if (ret != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR(
            "ble_server_task_set_adv_data failed, ret = %#" PRIx32 "\r\n", ret);
        goto _error_return;
    }

    adv_params.adv_filter_policy = GAP_BLE_ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
    adv_params.adv_type = GAP_BLE_ADV_CONN_SCAN_UNDIR;
    adv_params.channel_map = 0x07;
    adv_params.duration = 0;
    adv_params.max_interval = 0x60 /* [* 0.625ms] */;
    adv_params.min_interval = 0x30; /* [* 0.625ms] */
    // adv_params.own_addr = 0;
    adv_params.peer_addr.type = BT_ADDRESS_TYPE_PUBLIC_DEVICE_ADDRESS;
    // adv_params.tx_power = 0;
    ret = gap_ble_set_adv_param(CONFIG_BLE_SERVER_TASK_GAP_ADV_ID, &adv_params);
    if (ret != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR(
            "gap_ble_set_adv_params failed, ret = %#" PRIx32 "\r\n", ret);
        goto _error_return;
    }

    ret = gap_ble_start_adv(CONFIG_BLE_SERVER_TASK_GAP_ADV_ID);
    if (ret != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR("gap_ble_start_adv failed, ret = %#" PRIx32 "\r\n",
                                   ret);
        goto _error_return;
    }

    for (;;) {
        osal_msleep(1);
        nodeTelemetry_forEach(ble_server_task_for_each_teammte_node_cb);
    }

_error_return:
    osal_kthread_destroy(g_ble_server_task_handle, 0);
    return 0;
}

static void ble_server_task_for_each_teammte_node_cb(NodeTelemetry_t *node) {
    osal_msleep(1000);

    if (!g_ble_server_is_connected) {
        WLID_LINK_CLIENT_LOG_WARN("client is not connected\r\n");
        return;
    }

    errcode_t ret;

    gatts_ntf_ind_t param = {0};
    param.attr_handle = g_ble_server_tx_characteristic_value_handle;
    param.value = (uint8_t *)node;
    param.value_len = sizeof(NodeTelemetry_t);
    ret = gatts_notify_indicate(g_ble_server_id, g_ble_server_conn_id, &param);
    if (ret != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR("notify ble client failed, ret = %#" PRIx32 "\r\n",
                                   ret);
    }
}

bt_uuid_t *ble_server_task_set_uuid16(bt_uuid_t *buf, uint16_t uuid) {
    if (buf == NULL) {
        return NULL;
    }

    buf->uuid_len = sizeof(uuid);
    buf->uuid[0] = (uuid >> 8) & 0xff;
    buf->uuid[1] = uuid & 0xff;

    return buf;
}

errcode_t
ble_server_task_add_tx_characteristic(uint8_t server_id, uint16_t service_handle,
                                      uint16_t tx_charateristic_uuid,
                                      uint16_t *out_tx_charateristic_value_handle) {
    if (out_tx_charateristic_value_handle == NULL) {
        return ERRCODE_BT_PARAM_ERR;
    }

    errcode_bt_t ret;
    gatts_add_chara_info_t chara_info = {0};
    gatts_add_desc_info_t desc_info = {0};
    uint16_t desc_handle = 0;
    gatts_add_character_result_t chara_result = {0};
    uint8_t chara_val[2] = {0};
    uint8_t ccc_val[2] = {0x00, 0x00};

    ble_server_task_set_uuid16(&chara_info.chara_uuid, tx_charateristic_uuid);
    chara_info.permissions = GATT_ATTRIBUTE_PERMISSION_READ;
    chara_info.properties =
        GATT_CHARACTER_PROPERTY_BIT_READ | GATT_CHARACTER_PROPERTY_BIT_NOTIFY;
    chara_info.value = chara_val;
    chara_info.value_len = array_size(chara_val);

    ret = gatts_add_characteristic_sync(server_id, service_handle, &chara_info,
                                        &chara_result);
    if (ret != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR(
            "gatts_add_characteristic_sync failed, ret = %#" PRIx32 "\r\n", ret);
        goto _error_return;
    }

    *out_tx_charateristic_value_handle = chara_result.value_handle;

    WLID_LINK_CLIENT_LOG_INFO("tx charateristic: handle = %" PRIu16
                              ", value handle = %" PRIu16 "\r\n",
                              chara_result.handle, chara_result.value_handle);

    ble_server_task_set_uuid16(&desc_info.desc_uuid, 0x2902);
    desc_info.permissions =
        GATT_ATTRIBUTE_PERMISSION_READ | GATT_ATTRIBUTE_PERMISSION_WRITE;
    desc_info.value = ccc_val;
    desc_info.value_len = array_size(ccc_val);

    ret =
        gatts_add_descriptor_sync(server_id, service_handle, &desc_info, &desc_handle);
    if (ret != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR(
            "gatts_add_descriptor_sync failed, ret = %#" PRIx32 "\r\n", ret);
        goto _error_return;
    }

    WLID_LINK_CLIENT_LOG_DEBUG("ccc desc handle = %" PRIu16 "\r\n", desc_handle);

    return ERRCODE_SUCC;
_error_return:
    return ret;
}

errcode_t
ble_server_task_add_rx_characteristic(uint8_t server_id, uint16_t service_handle,
                                      uint16_t rx_charateristic_uuid,
                                      uint16_t *out_rx_charateristic_value_handle) {
    if (out_rx_charateristic_value_handle == NULL) {
        return ERRCODE_BT_PARAM_ERR;
    }

    errcode_bt_t ret;
    gatts_add_chara_info_t chara_info = {0};
    gatts_add_desc_info_t desc_info = {0};
    uint16_t desc_handle = 0;
    gatts_add_character_result_t chara_result = {0};
    uint8_t ccc_val[2] = {0x00, 0x00};

    uint8_t chara_val = 0;
    NodeTelemetry_t *local_node = nodeTelemetry_getLocalNode();
    if (local_node != NULL) {
        chara_val = local_node->id;
    }
    else {
        WLID_LINK_CLIENT_LOG_ERROR("local node is NULL\r\n");
    }

    ble_server_task_set_uuid16(&chara_info.chara_uuid, rx_charateristic_uuid);
    chara_info.permissions =
        GATT_ATTRIBUTE_PERMISSION_READ | GATT_ATTRIBUTE_PERMISSION_WRITE;
    chara_info.properties =
        GATT_CHARACTER_PROPERTY_BIT_READ | GATT_CHARACTER_PROPERTY_BIT_WRITE_NO_RSP;
    chara_info.value = &chara_val;
    chara_info.value_len = sizeof(chara_val);

    ret = gatts_add_characteristic_sync(server_id, service_handle, &chara_info,
                                        &chara_result);
    if (ret != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR(
            "gatts_add_characteristic_sync failed, ret = %#" PRIx32 "\r\n", ret);
        goto _error_return;
    }
    *out_rx_charateristic_value_handle = chara_result.value_handle;

    WLID_LINK_CLIENT_LOG_INFO("rx charateristic: handle = %" PRIu16
                              ", value handle = %" PRIu16 "\r\n",
                              chara_result.handle, chara_result.value_handle);

    ble_server_task_set_uuid16(&desc_info.desc_uuid, 0x2902);
    desc_info.permissions =
        GATT_ATTRIBUTE_PERMISSION_READ | GATT_ATTRIBUTE_PERMISSION_WRITE;
    desc_info.value = ccc_val;
    desc_info.value_len = array_size(ccc_val);

    ret =
        gatts_add_descriptor_sync(server_id, service_handle, &desc_info, &desc_handle);
    if (ret != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR(
            "gatts_add_descriptor_sync failed, ret = %#" PRIx32 "\r\n", ret);
        goto _error_return;
    }

    WLID_LINK_CLIENT_LOG_DEBUG("ccc desc handle = %" PRIu16 "\r\n", desc_handle);

    return ERRCODE_SUCC;
_error_return:
    return ret;
}

errcode_t ble_server_task_set_adv_data(void) {
    static uint8_t adv_bytes[] = {
        /* 1. Advertising Flags*/
        0x02, // Length: 2 bytes (type + data)
        0x01, // Type: 0x01 (Flags)
        0x05, // Data: 0x05 (General Discoverable Mode + BR/EDR Not Supported)
              // 0x01: General Discoverable Mode
              // 0x04: BR/EDR Not Supported (BLE only mode)
              // 0x05 = 0x01 + 0x04

        /* 2. Device Appearance */
        0x03, // Length: 2 bytes (type + data)
        0x19, // Type: 0x19 (Appearance)
        0x80,
        0x00, // Data: 0x0080 (UART Device Category) - Little-endian storage
              // Low byte first, high byte last

        // /* 2. 新增：完整的 16 位服务 UUID 列表（Web Bluetooth 必须）*/
        // 0x03, // Length: 3 bytes (type + 2字节UUID)
        // 0x03, // Type: 0x03 (Complete List of 16-bit Service UUIDs)
        // (CONFIG_BLE_SERVER_SERVICE_UUID & 0xFF),      // UUID 低字节（0xE0）
        // (CONFIG_BLE_SERVER_SERVICE_UUID >> 8) & 0xFF, // UUID 高字节（0xFF）

        /* 3. Complete Local Name */
        0x0f, // Length: 15 bytes (type + data)
        0x09, // Type: 0x09 (Complete Local Name)
        'W',
        'i',
        'd',
        'L',
        'n',
        'k',
        'C',
        'l',
        'i',
        'e',
        'n',
        't',
    };

    gap_ble_config_adv_data_t adv_data = {0};

    adv_data.adv_data = adv_bytes;
    adv_data.adv_length = array_size(adv_bytes);
    adv_data.scan_rsp_data = NULL;
    adv_data.scan_rsp_length = 0;

    return gap_ble_set_adv_data(CONFIG_BLE_SERVER_TASK_GAP_ADV_ID, &adv_data);
}

static void ble_server_task_gap_ble_start_adv_callback(uint8_t adv_id,
                                                       adv_status_t status) {
    WLID_LINK_CLIENT_LOG_INFO("adv id = %" PRIu8 ", status = %" PRId32 "\r\n", adv_id,
                              status);

    unused(adv_id);
    unused(status);
}

static void ble_server_task_gap_ble_connect_state_changed_callback(
    uint16_t conn_id, bd_addr_t *addr, gap_ble_conn_state_t conn_state,
    gap_ble_pair_state_t pair_state, gap_ble_disc_reason_t disc_reason) {
    WLID_LINK_CLIENT_LOG_DEBUG(
        "conn id = %" PRIu16 ", conn state = %" PRId32 ", pair state = %" PRId32
        ", disc reason = %" PRId32 ", bd addr: type = %" PRIu8 ", addr = ",
        conn_id, conn_state, pair_state, disc_reason, addr->type);
    for (uint8_t i = 0; i < array_size(addr->addr); i++) {
        WLID_LINK_CLIENT_LOG_PRINT_DEBUG("0x%02x ", addr->addr[i]);
    }
    WLID_LINK_CLIENT_LOG_PRINT_DEBUG("\r\n");

    if (conn_state == GAP_BLE_STATE_CONNECTED) {
#if CONFIG_EMERGENCY_ALARM_TASK_ENABLED
        emergency_alarm_task_trigger_buzzer_beep_once();
#endif // CONFIG_EMERGENCY_ALARM_TASK_ENABLED

        g_ble_server_is_connected = true;
        g_ble_server_conn_id = conn_id;
    }
    else {
        g_ble_server_is_connected = false;
        g_ble_server_conn_id = 0;

        errcode_t ret = gap_ble_start_adv(CONFIG_BLE_SERVER_TASK_GAP_ADV_ID);
        if (ret != ERRCODE_SUCC) {
            WLID_LINK_CLIENT_LOG_ERROR("ble restart adv failed, ret = %#" PRIx32 "\r\n",
                                       ret);
        }
    }

    unused(addr);
    unused(pair_state);
    unused(disc_reason);
}

static void ble_server_task_gap_ble_stop_adv_callback(uint8_t adv_id,
                                                      adv_status_t status) {
    WLID_LINK_CLIENT_LOG_INFO("adv id = %" PRIu8 ", adv status = %" PRId32 "\r\n",
                              adv_id, status);

    unused(adv_id);
    unused(status);
}

static void ble_server_task_gap_ble_pair_result_callback(uint16_t conn_id,
                                                         const bd_addr_t *addr,
                                                         errcode_t status) {
    if (status != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR("status = %#" PRIx32 "\r\n", status);
    }

    WLID_LINK_CLIENT_LOG_DEBUG(
        "conn id = %" PRIu16 ", bd addr: type = %" PRIu8 ", addr = ", conn_id,
        addr->type);
    for (uint8_t i = 0; i < array_size(addr->addr); i++) {
        WLID_LINK_CLIENT_LOG_PRINT_DEBUG("0x%02x ", addr->addr[i]);
    }
    WLID_LINK_CLIENT_LOG_PRINT_DEBUG("\r\n");

    unused(conn_id);
    unused(addr);
}

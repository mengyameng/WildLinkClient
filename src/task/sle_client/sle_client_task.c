#include <inttypes.h>
#include <string.h>

#include "common_def.h"
#include "sle_connection_manager.h"
#include "sle_device_discovery.h"
#include "sle_ssap_client.h"
#include "soc_osal.h"

#include "node_telemetry.h"
#include "wlid_link_client_log.h"

#include "sle_client_task.h"

#ifndef STRINGIFY
#define STRINGIFY(x) #x
#endif // STRINGIFY

// #ifdef IS_EMPTY_ARGS_HELPER
// #undef IS_EMPTY_ARGS_HELPER
// #endif // IS_EMPTY_ARGS_HELPER

// #ifdef IS_EMPTY_ARGS
// #undef IS_EMPTY_ARGS
// #endif // IS_EMPTY_ARGS

// #ifdef IS_EMPTY_MACRO
// #undef IS_EMPTY_MACRO
// #endif // IS_EMPTY_MACRO

// #define IS_EMPTY_ARGS_HELPER(_0, _1, _2, ...) _2
// #define IS_EMPTY_ARGS(...) IS_EMPTY_ARGS_HELPER(1, ##__VA_ARGS__, 0, 1)
// #define IS_EMPTY_MACRO(x) IS_EMPTY_ARGS((x))

osal_task *g_sle_client_task_handle;

static sle_announce_seek_callbacks_t g_sle_announce_seek_cbks = {0};
static sle_connection_callbacks_t g_sle_connect_cbk = {0};
static ssapc_callbacks_t g_sle_ssapc_cbk = {0};

static const sle_addr_t g_sle_target_server_addr = {
    .type = SLE_ADDRESS_TYPE_PUBLIC,
    .addr = {CONFIG_SLE_TARGET_SERVER_ADDR_LOW_BYTE, 0x02, 0x03, 0x04, 0x05, 0x06}};
static const uint8_t g_sle_target_server_uuid_base[SLE_UUID_LEN] = {
    0x37, 0xBE, 0xA8, 0x80, 0xFC, 0x70, 0x11, 0xEA,
    0xB7, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint16_t g_sle_conn_id = 0;
static const uint16_t g_sle_server_property_max30102_uuid16 =
    CONFIG_SLE_SERVER_PROPERTY_MAX30102_UUID16;
static const uint16_t g_sle_server_property_max30205_uuid16 =
    CONFIG_SLE_SERVER_PROPERTY_MAX30205_UUID16;
static uint16_t g_sle_server_property_max30102_handle = 0;
static uint16_t g_sle_server_property_max30205_handle = 0;

// seek
static void sle_client_task_sle_enable_cbk(errcode_t status);
static void sle_client_task_seek_enable_cbk(errcode_t status);
static void
sle_client_task_seek_result_info_cbk(sle_seek_result_info_t *seek_result_data);
static void sle_client_task_seek_disable_cbk(errcode_t status);

// connect
static void sle_client_task_connect_state_changed_cbk(uint16_t conn_id,
                                                      const sle_addr_t *addr,
                                                      sle_acb_state_t conn_state,
                                                      sle_pair_state_t pair_state,
                                                      sle_disc_reason_t disc_reason);
static void sle_client_task_pair_complete_cbk(uint16_t conn_id, const sle_addr_t *addr,
                                              errcode_t status);

// ssapc
static void sle_client_task_ssapc_exchange_info_cbk(uint8_t client_id, uint16_t conn_id,
                                                    ssap_exchange_info_t *param,
                                                    errcode_t status);
static void
sle_client_task_ssapc_find_structure_cbk(uint8_t client_id, uint16_t conn_id,
                                         ssapc_find_service_result_t *service,
                                         errcode_t status);
static void
sle_client_task_ssapc_find_property_cbk(uint8_t client_id, uint16_t conn_id,
                                        ssapc_find_property_result_t *property,
                                        errcode_t status);
static void sle_client_task_ssapc_find_structure_cmp_cbk(
    uint8_t client_id, uint16_t conn_id,
    ssapc_find_structure_result_t *structure_result, errcode_t status);
static void sle_client_task_ssapc_write_cfm_cb(uint8_t client_id, uint16_t conn_id,
                                               ssapc_write_result_t *write_result,
                                               errcode_t status);
static void sle_client_task_ssapc_notification_cb(uint8_t client_id, uint16_t conn_id,
                                                  ssapc_handle_value_t *data,
                                                  errcode_t status);
static void sle_client_task_ssapc_indication_cb(uint8_t client_id, uint16_t conn_id,
                                                ssapc_handle_value_t *data,
                                                errcode_t status);

static int sle_client_task(void *args);

errcode_t sle_client_task_entry(void) {
    g_sle_client_task_handle =
        osal_kthread_create(sle_client_task, NULL, STRINGIFY(sle_client_task),
                            CONFIG_SLE_CLIENT_TASK_STACK_SIZE);
    if (g_sle_client_task_handle == NULL) {
        WLID_LINK_CLIENT_LOG_ERROR("failed to create sle client task\r\n");
        goto failed;
    }

    if (osal_kthread_set_priority(g_sle_client_task_handle,
                                  CONFIG_SLE_CLIENT_TASK_PRIORITY)
        != OSAL_SUCCESS)
    {
        WLID_LINK_CLIENT_LOG_ERROR("failed to set sle client task priority\r\n");
        goto kthread_destory;
    }

    return ERRCODE_SUCC;
kthread_destory:
    osal_kthread_destroy(g_sle_client_task_handle, 0);
failed:
    return ERRCODE_FAIL;
}

static int sle_client_task(void *args) {
    unused(args);
    WLID_LINK_CLIENT_LOG_INFO("start\r\n");
    osal_msleep(2000);

    errcode_t ret;

    // register sle announce seek callbacks
    g_sle_announce_seek_cbks.sle_enable_cb = sle_client_task_sle_enable_cbk;
    g_sle_announce_seek_cbks.seek_enable_cb = sle_client_task_seek_enable_cbk;
    g_sle_announce_seek_cbks.seek_result_cb = sle_client_task_seek_result_info_cbk;
    g_sle_announce_seek_cbks.seek_disable_cb = sle_client_task_seek_disable_cbk;
    ret = sle_announce_seek_register_callbacks(&g_sle_announce_seek_cbks);
    if (ret != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR(
            "failed to register sle announce seek callbacks, ret = %#" PRIx32 "\r\n",
            ret);
        goto _error_exit;
    }

    // register sle connect callbacks
    g_sle_connect_cbk.connect_state_changed_cb =
        sle_client_task_connect_state_changed_cbk;
    g_sle_connect_cbk.pair_complete_cb = sle_client_task_pair_complete_cbk;
    ret = sle_connection_register_callbacks(&g_sle_connect_cbk);
    if (ret != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR(
            "failed to register sle connection callbacks, ret = %#" PRIx32 "\r\n", ret);
        goto _error_exit;
    }

    // register sle ssapc callbacks
    g_sle_ssapc_cbk.exchange_info_cb = sle_client_task_ssapc_exchange_info_cbk;
    g_sle_ssapc_cbk.find_structure_cb = sle_client_task_ssapc_find_structure_cbk;
    g_sle_ssapc_cbk.ssapc_find_property_cbk = sle_client_task_ssapc_find_property_cbk;
    g_sle_ssapc_cbk.find_structure_cmp_cb =
        sle_client_task_ssapc_find_structure_cmp_cbk;
    g_sle_ssapc_cbk.write_cfm_cb = sle_client_task_ssapc_write_cfm_cb;
    g_sle_ssapc_cbk.notification_cb = sle_client_task_ssapc_notification_cb;
    g_sle_ssapc_cbk.indication_cb = sle_client_task_ssapc_indication_cb;
    ret = ssapc_register_callbacks(&g_sle_ssapc_cbk);
    if (ret != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR(
            "failed to register ssapc callbacks, ret = %#" PRIx32 "\r\n", ret);
        goto _error_exit;
    }

    // enable sle
    ret = enable_sle();
    if (ret != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR("failed to enable sle, ret = %#" PRIx32 "\r\n", ret);
        goto _error_exit;
    }

    for (;;) {
        osal_msleep(1000);
    }

_error_exit:
    osal_kthread_destroy(g_sle_client_task_handle, 0);
    return 0;
}

static void sle_client_task_sle_enable_cbk(errcode_t status) {
    errcode_t ret;
    // in order to start scan
    sle_seek_param_t seek_parma = {.filter_duplicates = 0,
                                   .own_addr_type = 0,
                                   .seek_filter_policy = SLE_SEEK_FILTER_ALLOW_ALL,
                                   .seek_interval[0] = 100 /*100*0.125ms*/,
                                   .seek_phys = SLE_SEEK_PHY_1M,
                                   .seek_type[0] = SLE_SEEK_ACTIVE,
                                   .seek_window[0] = 100 /*100*0.125ms*/};
    ret = sle_set_seek_param(&seek_parma);
    if (ret != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR("failed to set seek param, ret = %#" PRIx32 "\r\n",
                                   ret);
    }

    ret = sle_start_seek();
    if (ret != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR("failed to start seek, ret = %#" PRIx32 "\r\n", ret);
    }
}

static void sle_client_task_seek_enable_cbk(errcode_t status) {
    if (status != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR("failed to enable seek, status = %#" PRIx32 "\r\n",
                                   status);
    }
}

static void
sle_client_task_seek_result_info_cbk(sle_seek_result_info_t *seek_result_data) {
    if (seek_result_data == NULL) {
        WLID_LINK_CLIENT_LOG_ERROR("seek result data is null\r\n");
        return;
    }

    WLID_LINK_CLIENT_LOG_DEBUG("found device\r\n");
    // find the specified sle server
    if (osal_memcmp(seek_result_data->addr.addr, g_sle_target_server_addr.addr,
                    sizeof(seek_result_data->addr.addr))
        == 0)
    {
        WLID_LINK_CLIENT_LOG_INFO("specified sle server found\r\n");
        sle_stop_seek();
    }
}

static void sle_client_task_seek_disable_cbk(errcode_t status) {
    if (status != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR("failed to disable seek, status = %#" PRIx32 "\r\n",
                                   status);
    }
    else {
        // connect to server
        sle_remove_paired_remote_device(&g_sle_target_server_addr);
        sle_connect_remote_device(&g_sle_target_server_addr);
    }
}

void sle_client_task_connect_state_changed_cbk(uint16_t conn_id, const sle_addr_t *addr,
                                               sle_acb_state_t conn_state,
                                               sle_pair_state_t pair_state,
                                               sle_disc_reason_t disc_reason) {
    unused(addr);
    unused(disc_reason);
    WLID_LINK_CLIENT_LOG_DEBUG("disconnect reason = %" PRIu32 "\r\n", disc_reason);

    g_sle_conn_id = conn_id;
    if (conn_state == SLE_ACB_STATE_CONNECTED) {
        if (pair_state == SLE_PAIR_NONE) {
            sle_pair_remote_device(&g_sle_target_server_addr);
        }
    }
    else if (conn_state == SLE_ACB_STATE_DISCONNECTED) {
        sle_remove_paired_remote_device(&g_sle_target_server_addr);
        sle_start_seek();
    }
}

static void sle_client_task_pair_complete_cbk(uint16_t conn_id, const sle_addr_t *addr,
                                              errcode_t status) {
    if (status != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR("pair status = %" PRIu32 "\r\n", status);
    }
    else {
        ssap_exchange_info_t info = {.mtu_size = 520, .version = 1};
        ssapc_exchange_info_req(0, g_sle_conn_id, &info);
    }
}

static void sle_client_task_ssapc_exchange_info_cbk(uint8_t client_id, uint16_t conn_id,
                                                    ssap_exchange_info_t *param,
                                                    errcode_t status) {
    WLID_LINK_CLIENT_LOG_DEBUG("client id = %" PRIu8 ", connect id = %" PRIu16
                               ", mtu size = %" PRIu32 ", mtu version = %" PRIu16
                               "\r\n",
                               client_id, conn_id, param->mtu_size, param->version);
    if (status != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR("status = %#" PRIx32 "\r\n", status);
    }
    else {
        ssapc_find_structure_param_t find_param = {.type = SSAP_FIND_TYPE_PROPERTY,
                                                   find_param.start_hdl = 1,
                                                   find_param.end_hdl = 0xffff};
        ssapc_find_structure(client_id, conn_id, &find_param);
    }

    unused(param);
}

static void
sle_client_task_ssapc_find_structure_cbk(uint8_t client_id, uint16_t conn_id,
                                         ssapc_find_service_result_t *service,
                                         errcode_t status) {

    WLID_LINK_CLIENT_LOG_DEBUG(
        "client id = %" PRIu8 ", connect id = %" PRIu16
        ", service start handle = %" PRIu16 ", service end handle = %" PRIu16
        ", uuid len = %" PRIu8 ", uuid(full) = ",
        client_id, conn_id, service->start_hdl, service->end_hdl, service->uuid.len);
    for (uint8_t i = 0; i < sizeof(service->uuid.uuid); i++) {
        WLID_LINK_CLIENT_LOG_PRINT_DEBUG("%#02x ", service->uuid.uuid[i]);
    }
    WLID_LINK_CLIENT_LOG_PRINT_DEBUG("\r\n");

    if (status) {
        WLID_LINK_CLIENT_LOG_ERROR("status = %#" PRIx32 "\r\n", status);
    }
}

static void
sle_client_task_ssapc_find_property_cbk(uint8_t client_id, uint16_t conn_id,
                                        ssapc_find_property_result_t *property,
                                        errcode_t status) {
    WLID_LINK_CLIENT_LOG_DEBUG(
        "client id = %" PRIu8 ", connect id = %" PRIu16 ", property handle = %" PRIu16
        ", property uuid len = %" PRIu8 ", property uuid(full) = ",
        client_id, conn_id, property->handle, property->uuid.len);
    for (uint8_t i = 0; i < sizeof(property->uuid.uuid); i++) {
        WLID_LINK_CLIENT_LOG_PRINT_DEBUG("%#02" PRIu8 " ", property->uuid.uuid[i]);
    }
    WLID_LINK_CLIENT_LOG_PRINT_DEBUG("\r\n");

    if (status != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR("status = %#" PRIx32 "\r\n", status);
    }
    else {
        if (osal_memncmp(property->uuid.uuid, g_sle_target_server_uuid_base,
                         sizeof(property->uuid.uuid) - property->uuid.len)
            != 0)
        {
            WLID_LINK_CLIENT_LOG_INFO("uuid base not match, skip it\r\n");
            return;
        }

        if (osal_memncmp(
                &property->uuid.uuid[sizeof(property->uuid.uuid) - property->uuid.len],
                &g_sle_server_property_max30102_uuid16,
                sizeof(g_sle_server_property_max30102_uuid16))
            == 0)
        {
            WLID_LINK_CLIENT_LOG_INFO(
                "found max30102 property, handle = %" PRIu16 "\r\n", property->handle);
            g_sle_server_property_max30102_handle = property->handle;
        }
        else if (osal_memncmp(&property->uuid.uuid[sizeof(property->uuid.uuid)
                                                   - property->uuid.len],
                              &g_sle_server_property_max30205_uuid16,
                              sizeof(g_sle_server_property_max30205_uuid16))
                 == 0)
        {
            WLID_LINK_CLIENT_LOG_INFO(
                "found max30205 property, handle = %" PRIu16 "\r\n", property->handle);
            g_sle_server_property_max30205_handle = property->handle;
        }
    }
}

static void sle_client_task_ssapc_find_structure_cmp_cbk(
    uint8_t client_id, uint16_t conn_id,
    ssapc_find_structure_result_t *structure_result, errcode_t status) {
    WLID_LINK_CLIENT_LOG_DEBUG(
        "client id = %" PRIu8 ", connect id = %" PRIu16 ", type = %" PRIu8
        ", uuid len = %" PRIu8 ", uuid(full) = ",
        client_id, conn_id, structure_result->type, structure_result->uuid.len);
    for (uint8_t i = 0; i < sizeof(structure_result->uuid.uuid); i++) {
        WLID_LINK_CLIENT_LOG_PRINT_DEBUG("%#02" PRIx8 " ",
                                         structure_result->uuid.uuid[i]);
    }
    WLID_LINK_CLIENT_LOG_PRINT_DEBUG("\r\n");

    if (status != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR("status = %#" PRIx32 "\r\n", status);
    }
}

static void sle_client_task_ssapc_write_cfm_cb(uint8_t client_id, uint16_t conn_id,
                                               ssapc_write_result_t *write_result,
                                               errcode_t status) {
    WLID_LINK_CLIENT_LOG_DEBUG(
        "client id = %" PRIu8 ", connect id = %" PRIu16 ", type = " PRIu8
        ", handle = %" PRIu16 ", status = %#" PRIx32 "\r\n",
        client_id, conn_id, write_result->type, write_result->handle, status);

    if (status != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR("status = %#" PRIx32 "\r\n", status);
    }

    unused(client_id);
    unused(conn_id);
    unused(write_result);
}

static void sle_client_task_ssapc_notification_cb(uint8_t client_id, uint16_t conn_id,
                                                  ssapc_handle_value_t *data,
                                                  errcode_t status) {
    WLID_LINK_CLIENT_LOG_DEBUG(
        "client id = %" PRIu8 ", connect id = %" PRIu16 ", data type = %" PRIu8
        ", data handle = %" PRIu16 ", data len = %" PRIu16 ", data = ",
        client_id, conn_id, data->type, data->handle, data->data_len);
    for (uint16_t i = 0; i < data->data_len; i++) {
        WLID_LINK_CLIENT_LOG_PRINT_DEBUG("%#02" PRIx8 " ", data->data[i]);
    }
    WLID_LINK_CLIENT_LOG_PRINT_DEBUG("\r\n");

    if (status != ERRCODE_SUCC) {
        WLID_LINK_CLIENT_LOG_ERROR("status = %#" PRIx32 "\r\n", status);
        return;
    }

    if (data->handle == g_sle_server_property_max30102_handle) {
        if (data->data_len >= 4) {
            NodeTelemetry_t *const local_node = nodeTelemetry_getLocalNode();

            if (local_node == NULL) {
                WLID_LINK_CLIENT_LOG_WARN("local node is NULL\r\n");
                return;
            }
            else {
                if (data->data[1]) { // spo2 valid
                    local_node->blood_oxygen = data->data[0];
                }
                if (data->data[3]) { // heart rate valid
                    local_node->heart_rate = data->data[2];
                }
            }
        }
    }
    else if (data->handle == g_sle_server_property_max30205_handle) {
        if (data->data_len >= sizeof(float)) {
            NodeTelemetry_t *local_node = nodeTelemetry_getLocalNode();
            if (local_node == NULL) {
                WLID_LINK_CLIENT_LOG_WARN("local node is NULL\r\n");
            }
            else {
                memcpy(&local_node->body_temp, data->data, sizeof(float));
            }
        }
    }
    else {
        WLID_LINK_CLIENT_LOG_WARN("unknown handle = %" PRIu16 "\r\n", data->handle);
    }
}

static void sle_client_task_ssapc_indication_cb(uint8_t client_id, uint16_t conn_id,
                                                ssapc_handle_value_t *data,
                                                errcode_t status) {
    WLID_LINK_CLIENT_LOG_DEBUG("call notification callback\r\n");
    sle_client_task_ssapc_notification_cb(client_id, conn_id, data, status);
}

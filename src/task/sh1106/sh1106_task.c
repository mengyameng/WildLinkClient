#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>

#include "common_def.h"
#include "gpio.h"
#include "osal_event.h"
#include "osal_timer.h"
#include "pinctrl.h"
#include "pinctrl_porting.h"
#include "soc_osal.h"
#include "u8g2.h"

#if CONFIG_EMERGENCY_ALARM_TASK_ENABLED
#include "emergency_alarm_task.h"
#endif // CONFIG_EMERGENCY_ALARM_TASK_ENABLED
#include "free_menu/free_menu.h"
#include "node_telemetry.h"
#include "soft_i2c.h"
#include "wlid_link_client_log.h"

#include "sh1106_task.h"

#ifndef STRINGIFY
#define STRINGIFY(x) #x
#endif // STRINGIFY

#define USE_OLD_TEAMMATE_DISPLAY 0

#define ENCODER_CLK_PIN CONFIG_SH1106_TASK_ENCODER_PIN_A
#define ENCODER_DT_PIN CONFIG_SH1106_TASK_ENCODER_PIN_B
#define ENCODER_KEY_PIN CONFIG_SH1106_TASK_ENCODER_KEY_PIN

#define ENCODER_EVENT_KEY_PRESSED 0x01u      // 按键有效按下事件
#define ENCODER_EVENT_KEY_LONG_PRESSED 0x02u // 按键长按事件
#define ENCODER_EVENT_CW_ONE_ROUND 0x04u     // 顺时针旋转一圈事件
#define ENCODER_EVENT_CCW_ONE_ROUND 0x08u    // 逆时针旋转一圈事件
#define ENCODER_EVENT_ALL                                                              \
    (ENCODER_EVENT_KEY_PRESSED | ENCODER_EVENT_KEY_LONG_PRESSED                        \
     | ENCODER_EVENT_CW_ONE_ROUND | ENCODER_EVENT_CCW_ONE_ROUND)

static void sh1106_task_menu_show_teammates_page(void);
static void sh1106_task_menu_show_teammate_info(void);
#if USE_OLD_TEAMMATE_DISPLAY
#else
static void *menu_item_handler_teammate(void *items_, int index, int cmd);
#endif
static void *sh1106_task_menu_teammate_info_item_handler(void *items, int index,
                                                         int cmd);

typedef enum {
    MENU_MAIN_PAGE_ITEM_TITLE = 0,
    MENU_MAIN_PAGE_ITEM_HEART_RATE,
    MENU_MAIN_PAGE_ITEM_BLOOD_OX,
    MENU_MAIN_PAGE_ITEM_BODY_TEMP,
    MENU_MAIN_PAGE_ITEM_AIR_TEMP,
    MENU_MAIN_PAGE_ITEM_AIR_HUMIDITY,
    MENU_MAIN_PAGE_ITEM_AIR_PRESS,
    MENU_MAIN_PAGE_ITEM_GPS,
    MENU_MAIN_PAGE_ITEM_TEAMMATES,
} MenuTeamMemberInfoItemId_t;

typedef struct {
    MenuTeamMemberInfoItemId_t id;
    void (*item_handler)(void);
} MenuTeamMemberInfoItem_t;

static MenuTeamMemberInfoItem_t g_main_page_items[] = {
    {.id = MENU_MAIN_PAGE_ITEM_TITLE, .item_handler = NULL},
    {.id = MENU_MAIN_PAGE_ITEM_HEART_RATE, .item_handler = NULL},
    {.id = MENU_MAIN_PAGE_ITEM_BLOOD_OX, .item_handler = NULL},
    {.id = MENU_MAIN_PAGE_ITEM_BODY_TEMP, .item_handler = NULL},
    {.id = MENU_MAIN_PAGE_ITEM_AIR_TEMP, .item_handler = NULL},
    {.id = MENU_MAIN_PAGE_ITEM_AIR_HUMIDITY, .item_handler = NULL},
    {.id = MENU_MAIN_PAGE_ITEM_AIR_PRESS, .item_handler = NULL},
    {.id = MENU_MAIN_PAGE_ITEM_GPS, .item_handler = NULL},
    {.id = MENU_MAIN_PAGE_ITEM_TEAMMATES,
     .item_handler = sh1106_task_menu_show_teammates_page},
};

static MenuTeamMemberInfoItem_t g_team_member_info_items[] = {
    {.id = MENU_MAIN_PAGE_ITEM_TITLE, .item_handler = NULL},
    {.id = MENU_MAIN_PAGE_ITEM_HEART_RATE, .item_handler = NULL},
    {.id = MENU_MAIN_PAGE_ITEM_BLOOD_OX, .item_handler = NULL},
    {.id = MENU_MAIN_PAGE_ITEM_BODY_TEMP, .item_handler = NULL},
    {.id = MENU_MAIN_PAGE_ITEM_AIR_TEMP, .item_handler = NULL},
    {.id = MENU_MAIN_PAGE_ITEM_AIR_HUMIDITY, .item_handler = NULL},
    {.id = MENU_MAIN_PAGE_ITEM_AIR_PRESS, .item_handler = NULL},
    {.id = MENU_MAIN_PAGE_ITEM_GPS, .item_handler = NULL},
};
static Menu g_main_menu;

static Menu g_teammate_menu;

static Menu g_teammate_info_menu;

#define MAX_DISPLAYED_TEAMMATE_COUNT 10
#if USE_OLD_TEAMMATE_DISPLAY
static uint8_t displayed_teammate[MAX_DISPLAYED_TEAMMATE_COUNT] = {0};
static uint8_t displayed_teammate_wr_idx = 0;
#else
typedef struct {
    MenuItemTypeStru teammate_item;
    uint8_t teammate_node_id;
} MenuTeammateItem_t;
static MenuTeammateItem_t g_teammate_items[MAX_DISPLAYED_TEAMMATE_COUNT + 2] = {0};
static uint8_t g_teammate_item_wr_idx = 0;
#endif // USE_OLD_TEAMMATE_DISPLAY

static const unsigned int g_strong_alarm_intervals_ms[] = {
    10 * 1000 /*s*/,     20 * 1000 /*s*/,      30 * 1000 /*s*/, 60 * 1000 /*1min*/,
    300 * 1000 /*5min*/, 600 * 1000 /*10min*/, 1800 * 1000 /*30min*/};
static uint8_t g_strong_alarm_interval_rd_idx = 0;
static bool g_in_alarming = false;

osal_task *g_sh1106_tash_handle;
static osal_event g_encoder_event;
u8g2_t sh1106_u8g2;

extern soft_i2c_handle_t g_soft_i2c_handle;

static uint8_t u8x8_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
static void encoder_clk_pin_cb(pin_t pin, uintptr_t param);
static void encoder_key_pin_cb(pin_t pin, uintptr_t param);

static void node_telemetry_for_each_cb(NodeTelemetry_t *node);

static int sh1106_task(void *args);

errcode_t sh1106_task_entry(void) {
    if (osal_event_init(&g_encoder_event) != OSAL_SUCCESS) {
        WLID_LINK_CLIENT_LOG_ERROR("failed to init osal event\r\n");
        goto failed;
    }

    g_sh1106_tash_handle = osal_kthread_create(
        sh1106_task, NULL, STRINGIFY(sh1106_task), CONFIG_SH1106_TASK_STACK_SIZE);
    if (g_sh1106_tash_handle == NULL) {
        WLID_LINK_CLIENT_LOG_ERROR("failed to create sh1106 task\r\n");
        goto event_destory;
    }

    if (osal_kthread_set_priority(g_sh1106_tash_handle, CONFIG_SH1106_TASK_PRIORITY)
        != OSAL_SUCCESS)
    {
        WLID_LINK_CLIENT_LOG_ERROR("failed to set sh1106 task priority\r\n");
        goto kthread_destory;
    }

    return ERRCODE_SUCC;
kthread_destory:
    osal_kthread_destroy(g_sh1106_tash_handle, 0);
event_destory:
    osal_event_destroy(&g_encoder_event);
failed:
    return ERRCODE_FAIL;
}

static void encoder_clk_pin_cb(pin_t pin, uintptr_t param) {
    unused(param);
    // 所有状态变量均为函数内static局部变量（非全局），跨中断保持状态，初始化为0
    static uint8_t flag = 0; // 核心状态标记：0=空闲 1=已捕获CLK下降沿，等待上升沿
    static uint8_t dt_level_when_clk_fall = 0; // 暂存CLK下降沿时的DT电平

    uint8_t clk_level = uapi_gpio_get_val(pin);           // 读取CLK当前电平（判边沿）
    uint8_t dt_level = uapi_gpio_get_val(ENCODER_DT_PIN); // 读取DT当前电平

    // 条件1：flag=0（空闲） + CLK低电平（下降沿）→ 仅此时暂存DT电平，置flag等待上升沿
    if (flag == 0 && clk_level == GPIO_LEVEL_LOW) {
        dt_level_when_clk_fall = dt_level;
        flag = 1; // 标记已捕获下降沿，后续仅处理上升沿
    }

    // 条件2：flag=1（已等上升沿） + CLK高电平（上升沿）→ 仅此时校验电平，判断方向
    if (flag == 1 && clk_level == GPIO_LEVEL_HIGH) {
        if (dt_level_when_clk_fall == GPIO_LEVEL_LOW && dt_level == GPIO_LEVEL_HIGH) {
            // 顺时针旋转：触发顺时针一圈事件（你的原有规则，不变）
            osal_event_write(&g_encoder_event, ENCODER_EVENT_CW_ONE_ROUND);
        }
        else if (dt_level_when_clk_fall == GPIO_LEVEL_HIGH
                 && dt_level == GPIO_LEVEL_LOW)
        {
            // 逆时针旋转：触发逆时针一圈事件（你的原有规则，不变）
            osal_event_write(&g_encoder_event, ENCODER_EVENT_CCW_ONE_ROUND);
        }
        // 其余情况为噪声/无效电平，直接跳过

        flag = 0; // 完成一次边沿配对，重置flag为空闲，准备下一次旋转
    }
}

static void encoder_key_pin_cb(pin_t pin, uintptr_t param) {
#define ENCODER_KEY_PRESS_THRESHOLD_MS 15
#define ENCODER_KEY_LONG_PRESS_THRESHOLD_MS 2000
    unused(param);

    static int fall_ms = 0;
    static bool key_pressed = false;

    gpio_level_t pin_level = uapi_gpio_get_val(pin);
    if (pin_level == GPIO_LEVEL_LOW && !key_pressed) {
        fall_ms = osal_jiffies_to_msecs(osal_get_jiffies());
        key_pressed = true;
    }
    else if (pin_level == GPIO_LEVEL_HIGH && key_pressed) {
        const int duration_ms = osal_jiffies_to_msecs(osal_get_jiffies()) - fall_ms;
        if (duration_ms > ENCODER_KEY_PRESS_THRESHOLD_MS) {
            if (duration_ms > ENCODER_KEY_LONG_PRESS_THRESHOLD_MS) {
                osal_event_write(&g_encoder_event, ENCODER_EVENT_KEY_LONG_PRESSED);
            }
            else {
                osal_event_write(&g_encoder_event, ENCODER_EVENT_KEY_PRESSED);
            }
        }
        key_pressed = false;
    }
#undef ENCODER_KEY_PRESS_THRESHOLD_MS
#undef ENCODER_KEY_LONG_PRESS_THRESHOLD_MS
}

static void sh1106_task_menu_show_teammates_page(void) {
#if USE_OLD_TEAMMATE_DISPLAY
    static char teammate_name_buf[MAX_DISPLAYED_TEAMMATE_COUNT][10];
    static MenuItemTypeStru teammate_items[MAX_DISPLAYED_TEAMMATE_COUNT + 2];

    const uint8_t displayed_teammate_count = displayed_teammate_wr_idx;

    teammate_items[0].name = "Teammates";
    teammate_items[0].func = NULL;
    uint8_t i = 0;
    for (; i < displayed_teammate_count; i++) {
        const NodeTelemetry_t *const teammate_node =
            nodeTelemetry_getNode(displayed_teammate[i]);
        if (teammate_node == NULL) {
            teammate_items[i + 1].name = "null";
            continue;
        }

        snprintf(teammate_name_buf[i], sizeof(teammate_name_buf[i]), "0x%x",
                 teammate_node->id);
        teammate_items[i + 1].name = teammate_name_buf[i];
        teammate_items[i + 1].func = sh1106_task_menu_show_teammate_info;
    }
    teammate_items[i + 1].name = NULL;
    menu_init(&g_teammate_menu, &teammate_items, menu_item_handler_stru);
#else
    menu_init(&g_teammate_menu, &g_teammate_items, menu_item_handler_teammate);
#endif
    menu_set_focus(&g_teammate_menu);

    WLID_LINK_CLIENT_LOG_DEBUG("teammate menu item count = %" PRIi16 "\r\n",
                               g_teammate_menu.item_count);
}

static void sh1106_task_menu_show_teammate_info() {
    menu_init(&g_teammate_info_menu, &g_team_member_info_items,
              sh1106_task_menu_teammate_info_item_handler);
    menu_set_focus(&g_teammate_info_menu);
}

static void *sh1106_task_menu_teammate_info_item_handler(void *items, int index,
                                                         int cmd) {
    if (items == NULL || index < 0) {
        return "null";
    }

    const MenuTeamMemberInfoItem_t *const teammate_info_item =
        (MenuTeamMemberInfoItem_t *)items;
    const NodeTelemetry_t *teammate_node = NULL;

    if (menu_get_focus() == &g_main_menu) {
        teammate_node = nodeTelemetry_getLocalNode();
    }
    else {
#if USE_OLD_TEAMMATE_DISPLAY
        const uint8_t teammate_node_id =
            displayed_teammate[g_teammate_menu.item_index - 1];
#else
        const uint8_t teammate_node_id =
            g_teammate_items[g_teammate_menu.item_index].teammate_node_id;
#endif
        teammate_node = nodeTelemetry_getNode(teammate_node_id);
    }

    if (teammate_node == NULL) {
        return "teammate is null";
    }

    void *ret = NULL;

    switch (cmd) {
    case MENU_ITEM_CMD_NAME: {
        static char buffer[50];
        switch (teammate_info_item[index].id) {
        case MENU_MAIN_PAGE_ITEM_TITLE: {
            const char *teammate_name = NULL;
            if (menu_get_focus() == &g_main_menu) {
                teammate_name = "MySelf";
            }
            else {
#if USE_OLD_TEAMMATE_DISPLAY
                snprintf(buffer, sizeof(buffer), "Teammate 0x%x", teammate_node->id);
                ret = (void *)buffer;
#else
                teammate_name = teammate_node->name;
#endif
            }
            if (teammate_node->need_help) {
                snprintf(buffer, sizeof(buffer), "%.10s(sos)", teammate_name);
            }
            else if (g_in_alarming) {
                snprintf(buffer, sizeof(buffer), "%.10s(alarming)", teammate_name);
            }
            else {
                snprintf(buffer, sizeof(buffer), "%.10s", teammate_name);
            }
            ret = (void *)buffer;
            break;
        }
        case MENU_MAIN_PAGE_ITEM_HEART_RATE: {
            if (teammate_node->heart_rate < teammate_node->heart_rate_min
                || teammate_node->heart_rate > teammate_node->heart_rate_max)
            {
                snprintf(buffer, sizeof(buffer),
                         "Heart Rate: %" PRIu8 " (%" PRIu8 "-%" PRIu8 ")",
                         teammate_node->heart_rate, teammate_node->heart_rate_min,
                         teammate_node->heart_rate_max);
            }
            else {
                snprintf(buffer, sizeof(buffer), "Heart Rate: %" PRIu8,
                         teammate_node->heart_rate);
            }
            ret = (void *)buffer;
            break;
        }
        case MENU_MAIN_PAGE_ITEM_BLOOD_OX: {
            if (teammate_node->blood_oxygen < teammate_node->blood_oxygen_low) {
                snprintf(buffer, sizeof(buffer), "Blood Oxygen: %" PRIu8 "%%(Low)",
                         teammate_node->blood_oxygen);
            }
            else {
                snprintf(buffer, sizeof(buffer), "Blood Oxygen: %" PRIu8 "%%",
                         teammate_node->blood_oxygen);
            }
            ret = (void *)buffer;
            break;
        }
        case MENU_MAIN_PAGE_ITEM_BODY_TEMP: {
            if (teammate_node->body_temp < teammate_node->body_temp_min
                || teammate_node->body_temp > teammate_node->body_temp_max)
            {
                snprintf(buffer, sizeof(buffer),
                         "Body Temp: %" PRId32 ".%02" PRId32 "°C (%" PRId32
                         ".%02" PRId32 "-%" PRId32 ".%02" PRId32 ")",
                         ((int)teammate_node->body_temp),
                         ((int)(((int)(teammate_node->body_temp * 100)) % 100)),
                         ((int)teammate_node->body_temp_min),
                         ((int)(((int)(teammate_node->body_temp_min * 100)) % 100)),
                         ((int)teammate_node->body_temp_max),
                         ((int)(((int)(teammate_node->body_temp_max * 100)) % 100)));
            }
            else {
                snprintf(buffer, sizeof(buffer), "Body Temp: %d.%02d°C",
                         ((int)teammate_node->body_temp),
                         ((int)(((int)(teammate_node->body_temp * 100)) % 100)));
            }
            ret = (void *)buffer;
            break;
        }
        case MENU_MAIN_PAGE_ITEM_AIR_TEMP: {
            snprintf(buffer, sizeof(buffer), "Air Temp: %d.%02d°C",
                     ((int)teammate_node->air_temp),
                     ((int)(((int)(teammate_node->air_temp * 100)) % 100)));
            ret = (void *)buffer;
            break;
        }
        case MENU_MAIN_PAGE_ITEM_AIR_HUMIDITY: {
            snprintf(buffer, sizeof(buffer), "Air Humidity: %" PRIu8 "%%",
                     teammate_node->air_humidity);
            ret = (void *)buffer;
            break;
        }
        case MENU_MAIN_PAGE_ITEM_AIR_PRESS: {
            snprintf(buffer, sizeof(buffer), "Air Press: %d.%02dhPa",
                     ((int)teammate_node->air_pressure),
                     ((int)((int)(teammate_node->air_pressure * 100)) % 100));
            ret = (void *)buffer;
            break;
        }
        case MENU_MAIN_PAGE_ITEM_GPS: {
            snprintf(buffer, sizeof(buffer), "GPS: %d.%d,%d.%d",
                     ((int)teammate_node->gps.longitude),
                     ((int)((int)(teammate_node->gps.longitude * 1000000)) % 1000000),
                     ((int)teammate_node->gps.latitude),
                     ((int)((int)(teammate_node->gps.latitude * 1000000)) % 1000000));
            ret = (void *)buffer;
            break;
        }
        case MENU_MAIN_PAGE_ITEM_TEAMMATES: {
            ret = (void *)("Teammates >>>");
            break;
        }

        default:
            ret = (void *)("unknown id");
            break;
        }
        break;
    }
    case MENU_ITEM_CMD_ENTER: {
        if (teammate_info_item[index].item_handler != NULL) {
            teammate_info_item[index].item_handler();
        }
        break;
    }
    case MENU_ITEM_CMD_BACK: {
        menu_back_focus();
        break;
    }
    case MENU_ITEM_CMD_COUNT: {
        if (menu_get_focus() == &g_main_menu) {
            ret = (void *)(array_size(g_main_page_items));
        }
        else {
            ret = (void *)(array_size(g_team_member_info_items));
        }
        break;
    }
    default: {
        ret = (void *)("unknown cmd");
        break;
    }
    }

    return ret;
}

#if USE_OLD_TEAMMATE_DISPLAY
#else
static void *menu_item_handler_teammate(void *items_, int index, int cmd) {
    MenuTeammateItem_t *items = (MenuTeammateItem_t *)items_;
    if (items == NULL || index < 0)
        return "null";

    void *ret = NULL;
    switch (cmd) {
    case MENU_ITEM_CMD_NAME: {
        ret = items[index].teammate_item.name;
        break;
    }
    case MENU_ITEM_CMD_ENTER: {
        if (items[index].teammate_item.func)
            items[index].teammate_item.func();
        break;
    }
    case MENU_ITEM_CMD_BACK: {
        menu_back_focus();
        break;
    }
    case MENU_ITEM_CMD_COUNT: {
        int count = 0;
        // while (items[count].teammate_item.name)
        //     count++;
        count = g_teammate_item_wr_idx;

        ret = (void *)count;
        break;
    }
    }
    return ret;
}
#endif

static void node_telemetry_for_each_cb(NodeTelemetry_t *node) {
    const NodeTelemetry_t *const local_node = nodeTelemetry_getLocalNode();
    if (local_node == NULL || local_node->id != node->id) {
#if USE_OLD_TEAMMATE_DISPLAY
        displayed_teammate[displayed_teammate_wr_idx++] = node->id;
#else
        g_teammate_items[g_teammate_item_wr_idx].teammate_item.name = node->name;
        g_teammate_items[g_teammate_item_wr_idx++].teammate_node_id = node->id;
#endif
    }

    static int16_t last_focus_teammate_node_id = -1;
    const unsigned int curr_ms = osal_jiffies_to_msecs(osal_get_jiffies());
    static unsigned int last_strong_alarm_ms = 0;

    bool should_jump_for_strong_alarm = false;
    if (node->need_help) {
#if CONFIG_EMERGENCY_ALARM_TASK_ENABLED
        emergency_alarm_task_trigger_sos();
#endif // CONFIG_EMERGENCY_ALARM_TASK_ENABLED

        if (curr_ms - last_strong_alarm_ms
            >= g_strong_alarm_intervals_ms[g_strong_alarm_interval_rd_idx])
        {
            WLID_LINK_CLIENT_LOG_INFO(
                "Node %" PRIu8
                " (%.10s) sent sos signal! - %u ms since last strong alarm\r\n",
                node->id, node->name,
                g_strong_alarm_intervals_ms[g_strong_alarm_interval_rd_idx]);
            should_jump_for_strong_alarm = true;
            last_strong_alarm_ms = curr_ms;
        }
        else {
            should_jump_for_strong_alarm = false;
        }
    }
    else if ((curr_ms - node->timestamp < 30000)
             && ((node->heart_rate < node->heart_rate_min)
                 || (node->heart_rate > node->heart_rate_max)
                 || (node->body_temp < node->body_temp_min)
                 || (node->body_temp > node->body_temp_max)
                 || (node->blood_oxygen < node->blood_oxygen_low)))
    {
        if (curr_ms - last_strong_alarm_ms
            >= g_strong_alarm_intervals_ms[g_strong_alarm_interval_rd_idx])
        {
#if CONFIG_EMERGENCY_ALARM_TASK_ENABLED
            emergency_alarm_task_trigger_alarm();
#endif // CONFIG_EMERGENCY_ALARM_TASK_ENABLED

            WLID_LINK_CLIENT_LOG_INFO(
                "Node %" PRIu8
                " (%.10s) has abnormal vital signs - %u ms since last strong alarm\r\n",
                node->id, node->name,
                g_strong_alarm_intervals_ms[g_strong_alarm_interval_rd_idx]);

            if (node->heart_rate < node->heart_rate_min) {
                WLID_LINK_CLIENT_LOG_DEBUG("Heart rate %" PRIu8 " bpm (min: %" PRIu8
                                           ")\r\n",
                                           node->heart_rate, node->heart_rate_min);
            }
            if (node->heart_rate > node->heart_rate_max) {
                WLID_LINK_CLIENT_LOG_DEBUG("Heart rate %" PRIu8 " bpm (max: %" PRIu8
                                           ")\r\n",
                                           node->heart_rate, node->heart_rate_max);
            }
            if (node->body_temp < node->body_temp_min) {
                WLID_LINK_CLIENT_LOG_DEBUG("Body temp %" PRIu8 " °C (min: %" PRIu8
                                           ")\r\n",
                                           node->body_temp, node->body_temp_min);
            }
            if (node->body_temp > node->body_temp_max) {
                WLID_LINK_CLIENT_LOG_DEBUG("Body temp %" PRIu8 " °C (max: %" PRIu8
                                           ")\r\n",
                                           node->body_temp, node->body_temp_max);
            }
            if (node->blood_oxygen < node->blood_oxygen_low) {
                WLID_LINK_CLIENT_LOG_DEBUG("Blood oxygen %" PRIu8 " %% (min: %" PRIu8
                                           ")\r\n",
                                           node->blood_oxygen, node->blood_oxygen_low);
            }

            should_jump_for_strong_alarm = true;
            last_strong_alarm_ms = curr_ms;
        }
        else {
#if CONFIG_EMERGENCY_ALARM_TASK_ENABLED
            emergency_alarm_task_trigger_led();
#endif // CONFIG_EMERGENCY_ALARM_TASK_ENABLED

            should_jump_for_strong_alarm = false;
        }
    }

    if (should_jump_for_strong_alarm) {
        g_in_alarming = true;

        if ((local_node == NULL || local_node->id != node->id)
            && (last_focus_teammate_node_id <= -1
                || ((uint8_t)last_focus_teammate_node_id) != node->id))
        {
            menu_init(&g_teammate_menu, &g_teammate_items, menu_item_handler_teammate);
            menu_event_wheel(&g_teammate_menu, g_teammate_item_wr_idx - 1);
            sh1106_task_menu_show_teammate_info();

            WLID_LINK_CLIENT_LOG_DEBUG("Jumping to teammate %" PRIu8
                                       " (%.10s) info menu - "
                                       "alarm triggered\r\n",
                                       node->id, node->name);

            menu_jump_to(&g_teammate_info_menu);

            last_focus_teammate_node_id = node->id;
        }
        else {
            WLID_LINK_CLIENT_LOG_INFO(
                "Node %" PRIu8 " (%.10s) alarm triggered, "
                "but skipping menu jump (already focused or local node)\r\n",
                node->id, node->name);
        }
    }
}

static uint8_t u8x8_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    static uint8_t buffer[128];
    static uint8_t buf_idx;

    switch (msg) {
    case U8X8_MSG_BYTE_INIT: {
        break;
    }
    case U8X8_MSG_BYTE_START_TRANSFER: {
        buf_idx = 0;
        break;
    }
    case U8X8_MSG_BYTE_END_TRANSFER: {
        soft_i2c_mem_write(&g_soft_i2c_handle, 0x3c, SOFT_I2C_ADDR_7BIT, NULL, 0,
                           buffer, buf_idx);
        break;
    }
    case U8X8_MSG_BYTE_SET_DC: {
        break;
    }
    case U8X8_MSG_BYTE_SEND: {
        memcpy(buffer + buf_idx, arg_ptr, arg_int);
        buf_idx += arg_int;
        break;
    }

    default: {
        return 0;
    }
    }

    return 1;
}

static int sh1106_task(void *args) {
    unused(args);
    WLID_LINK_CLIENT_LOG_INFO("start\r\n");
    osal_msleep(1000);

#if ENCODER_CLK_PIN == 4
    uapi_pin_set_mode(ENCODER_CLK_PIN, PIN_MODE_2); // gpio
#elif ENCODER_CLK_PIN == 5
    uapi_pin_set_mode(ENCODER_CLK_PIN, PIN_MODE_4); // gpio
#else
    uapi_pin_set_mode(ENCODER_CLK_PIN, HAL_PIO_FUNC_GPIO);
#endif // ENCODER_CLK_PIN==4
    uapi_pin_set_pull(ENCODER_CLK_PIN, PIN_PULL_TYPE_DISABLE);
    uapi_gpio_set_dir(ENCODER_CLK_PIN, GPIO_DIRECTION_INPUT);
    uapi_gpio_register_isr_func(ENCODER_CLK_PIN, GPIO_INTERRUPT_DEDGE,
                                encoder_clk_pin_cb);

#if ENCODER_DT_PIN == 4
    uapi_pin_set_mode(ENCODER_DT_PIN, PIN_MODE_2); // gpio
#elif ENCODER_DT_PIN == 5
    uapi_pin_set_mode(ENCODER_DT_PIN, PIN_MODE_4); // gpio
#else
    uapi_pin_set_mode(ENCODER_DT_PIN, HAL_PIO_FUNC_GPIO);
#endif // ENCODER_DT_PIN==4
    uapi_pin_set_pull(ENCODER_DT_PIN, PIN_PULL_TYPE_DISABLE);
    uapi_gpio_set_dir(ENCODER_DT_PIN, GPIO_DIRECTION_INPUT);

#if ENCODER_KEY_PIN == 4
    uapi_pin_set_mode(ENCODER_KEY_PIN, PIN_MODE_2);
#elif ENCODER_KEY_PIN == 5
    uapi_pin_set_mode(ENCODER_KEY_PIN, PIN_MODE_4);
#else
    uapi_pin_set_mode(ENCODER_KEY_PIN, HAL_PIO_FUNC_GPIO);
#endif // ENCODER_KEY_PIN==4
    uapi_pin_set_pull(ENCODER_KEY_PIN, PIN_PULL_TYPE_DISABLE);
    uapi_gpio_set_dir(ENCODER_KEY_PIN, GPIO_DIRECTION_INPUT);
    uapi_gpio_register_isr_func(ENCODER_KEY_PIN, GPIO_INTERRUPT_DEDGE,
                                encoder_key_pin_cb);

    u8g2_Setup_sh1106_i2c_128x64_noname_f(&sh1106_u8g2, U8G2_R0, u8x8_byte_cb,
                                          u8x8_byte_empty);
    u8g2_InitDisplay(&sh1106_u8g2);
    u8g2_SetPowerSave(&sh1106_u8g2, 0);
    u8g2_SetFontMode(&sh1106_u8g2, 1);
    u8g2_SetFontDirection(&sh1106_u8g2, 0);
    u8g2_SetFont(&sh1106_u8g2, u8g2_font_6x10_tf);

    g_teammate_items[0].teammate_item.name = "Teammates";
    g_teammate_items[0].teammate_item.func = NULL;
    for (uint8_t i = 1; i < array_size(g_teammate_items); i++) {
        g_teammate_items[i].teammate_item.func = sh1106_task_menu_show_teammate_info;
    }

    menu_set_focus(&g_main_menu);
    menu_init(&g_main_menu, &g_main_page_items,
              sh1106_task_menu_teammate_info_item_handler);

    int event;
    for (;;) {
        osal_msleep(1);

        u8g2_ClearBuffer(&sh1106_u8g2);

        event = osal_event_read(&g_encoder_event, ENCODER_EVENT_ALL, 69,
                                OSAL_WAITMODE_OR | OSAL_WAITMODE_CLR);
        if (event != OSAL_FAILURE) {
            if (event & ENCODER_EVENT_KEY_PRESSED) {
                WLID_LINK_CLIENT_LOG_INFO("key pressed\r\n");
                if (g_in_alarming) {
                    g_in_alarming = false;
                    g_strong_alarm_interval_rd_idx = 0;
                }
                else {
                    menu_event_inject(menu_get_focus(), MENU_EVENT_ENTER);
                }
            }
            if (event & ENCODER_EVENT_KEY_LONG_PRESSED) {
                WLID_LINK_CLIENT_LOG_INFO("key long pressed\r\n");
                if (g_in_alarming) {
                    g_strong_alarm_interval_rd_idx++;
                    g_strong_alarm_interval_rd_idx %=
                        array_size(g_strong_alarm_intervals_ms);
                    WLID_LINK_CLIENT_LOG_DEBUG(
                        "Strong alarm interval read index = %" PRIu8
                        ", interval: %u ms\r\n",
                        g_strong_alarm_interval_rd_idx,
                        g_strong_alarm_intervals_ms[g_strong_alarm_interval_rd_idx]);

                    g_in_alarming = false;
                }
                else {
                    NodeTelemetry_t *const local_node = nodeTelemetry_getLocalNode();
                    if (local_node != NULL) {
                        local_node->need_help = !local_node->need_help;
                        WLID_LINK_CLIENT_LOG_INFO("need help = %" PRIu8 "\r\n",
                                                  local_node->need_help);
                    }
                    else {
                        WLID_LINK_CLIENT_LOG_WARN("local node is NULL\r\n");
                    }
                }
            }
            if (event & ENCODER_EVENT_CW_ONE_ROUND) {
                WLID_LINK_CLIENT_LOG_INFO("wheel down\r\n");
                menu_event_inject(menu_get_focus(), MENU_EVENT_WHEEL_DOWN);
            }
            if (event & ENCODER_EVENT_CCW_ONE_ROUND) {
                WLID_LINK_CLIENT_LOG_INFO("wheel up\r\n");
                menu_event_inject(menu_get_focus(), MENU_EVENT_WHEEL_UP);
            }
        }
#if USE_OLD_TEAMMATE_DISPLAY
        displayed_teammate_wr_idx = 0;
        nodeTelemetry_forEach(node_telemetry_for_each_cb);
#else
        g_teammate_item_wr_idx = 1;
        nodeTelemetry_forEach(node_telemetry_for_each_cb);
        g_teammate_items[g_teammate_item_wr_idx].teammate_item.name = NULL;
#endif

        menu_show_focus();
        u8g2_SendBuffer(&sh1106_u8g2);
    }

    osal_kthread_destroy(g_sh1106_tash_handle, 0);
    return 0;
}
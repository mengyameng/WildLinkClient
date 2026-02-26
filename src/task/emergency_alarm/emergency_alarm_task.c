#include <inttypes.h>
#include <stdbool.h>

#include "common_def.h"
#include "gpio.h"
#include "osal_event.h"
#include "osal_timer.h"
#include "pinctrl.h"
#include "pinctrl_porting.h"
#include "soc_osal.h"

#include "wlid_link_client_log.h"

#include "emergency_alarm_task.h"

#ifndef STRINGIFY
#define STRINGIFY(x) #x
#endif

#define EMERGENCY_ALARM_EVENT_SOS 0x01u
#define EMERGENCY_ALARM_EVENT_BUZZER_BEEP_ONCE 0x02u
#define EMERGENCY_ALARM_EVENT_LED_ALARM 0x04u
#define EMERGENCY_ALARM_EVENT_ACOUSTIC_ALARM 0x08u
#define EMERGENCY_ALARM_EVENT_ALL                                                      \
    (EMERGENCY_ALARM_EVENT_SOS | EMERGENCY_ALARM_EVENT_BUZZER_BEEP_ONCE                \
     | EMERGENCY_ALARM_EVENT_LED_ALARM | EMERGENCY_ALARM_EVENT_ACOUSTIC_ALARM)

osal_task *g_emergency_alarm_task_handle;
static osal_event g_emergency_alarm_event;

static int emergency_alarm_task(void *args);

errcode_t emergency_alarm_task_entry(void) {
    if (osal_event_init(&g_emergency_alarm_event) != OSAL_SUCCESS) {
        WLID_LINK_CLIENT_LOG_ERROR("failed to init emergency alarm task event\r\n");
        goto _error_return;
    }
    g_emergency_alarm_task_handle =
        osal_kthread_create(emergency_alarm_task, NULL, STRINGIFY(emergency_alarm_task),
                            CONFIG_EMERGENCY_ALARM_TASK_STACK_SIZE);
    if (g_emergency_alarm_task_handle == NULL) {
        WLID_LINK_CLIENT_LOG_ERROR("failed to create emergency alarm task\r\n");
        goto _event_destroy;
    }

    if (osal_kthread_set_priority(g_emergency_alarm_task_handle,
                                  CONFIG_EMERGENCY_ALARM_TASK_PRIORITY)
        != OSAL_SUCCESS)
    {
        WLID_LINK_CLIENT_LOG_ERROR("failed to set emergency alarm task priority\r\n");
        goto _kthread_destory;
    }

    return ERRCODE_SUCC;
_kthread_destory:
    osal_kthread_destroy(g_emergency_alarm_task_handle, 0);
_event_destroy:
    osal_event_destroy(&g_emergency_alarm_event);
_error_return:
    return ERRCODE_FAIL;
}

// Trigger emergency alarm (acoustic + LED)
void emergency_alarm_task_trigger_alarm(void) {
    osal_event_write(&g_emergency_alarm_event, EMERGENCY_ALARM_EVENT_ACOUSTIC_ALARM);
}

// Trigger LED only alarm
void emergency_alarm_task_trigger_led(void) {
    osal_event_write(&g_emergency_alarm_event, EMERGENCY_ALARM_EVENT_LED_ALARM);
}

// Trigger buzzer short beep (device connected)
void emergency_alarm_task_trigger_buzzer_beep_once(void) {
    osal_event_write(&g_emergency_alarm_event, EMERGENCY_ALARM_EVENT_BUZZER_BEEP_ONCE);
}

// SOS signal functions
void emergency_alarm_task_trigger_sos(void) {
    osal_event_write(&g_emergency_alarm_event, EMERGENCY_ALARM_EVENT_SOS);
}

static int emergency_alarm_task(void *args) {
    unused(args);

#if CONFIG_EMERGENCY_ALARM_TASK_BUZZER_PIN == 4
    uapi_pin_set_mode(CONFIG_EMERGENCY_ALARM_TASK_BUZZER_PIN, PIN_MODE_2); // gpio
#elif CONFIG_EMERGENCY_ALARM_TASK_BUZZER_PIN == 5
    uapi_pin_set_mode(CONFIG_EMERGENCY_ALARM_TASK_BUZZER_PIN, PIN_MODE_4); // gpio
#else
    uapi_pin_set_mode(CONFIG_EMERGENCY_ALARM_TASK_BUZZER_PIN, HAL_PIO_FUNC_GPIO);
#endif // CONFIG_EMERGENCY_ALARM_TASK_BUZZER_PIN==4
    uapi_pin_set_pull(CONFIG_EMERGENCY_ALARM_TASK_BUZZER_PIN, PIN_PULL_TYPE_DOWN);
    uapi_gpio_set_dir(CONFIG_EMERGENCY_ALARM_TASK_BUZZER_PIN, GPIO_DIRECTION_OUTPUT);

#if CONFIG_EMERGENCY_ALARM_TASK_LED_PIN == 4
    uapi_pin_set_mode(CONFIG_EMERGENCY_ALARM_TASK_LED_PIN, PIN_MODE_2); // gpio
#elif CONFIG_EMERGENCY_ALARM_TASK_LED_PIN == 5
    uapi_pin_set_mode(CONFIG_EMERGENCY_ALARM_TASK_LED_PIN, PIN_MODE_4); // gpio
#else
    uapi_pin_set_mode(CONFIG_EMERGENCY_ALARM_TASK_LED_PIN, HAL_PIO_FUNC_GPIO);
#endif // CONFIG_EMERGENCY_ALARM_TASK_LED_PIN==4
    uapi_pin_set_pull(CONFIG_EMERGENCY_ALARM_TASK_LED_PIN, PIN_PULL_TYPE_DOWN);
    uapi_gpio_set_dir(CONFIG_EMERGENCY_ALARM_TASK_LED_PIN, GPIO_DIRECTION_OUTPUT);

#define BUZZER_ON()                                                                    \
    uapi_gpio_set_val(CONFIG_EMERGENCY_ALARM_TASK_BUZZER_PIN, GPIO_LEVEL_LOW)
#define BUZZER_OFF()                                                                   \
    uapi_gpio_set_val(CONFIG_EMERGENCY_ALARM_TASK_BUZZER_PIN, GPIO_LEVEL_HIGH)
#define LED_ON() uapi_gpio_set_val(CONFIG_EMERGENCY_ALARM_TASK_LED_PIN, GPIO_LEVEL_HIGH)
#define LED_OFF() uapi_gpio_set_val(CONFIG_EMERGENCY_ALARM_TASK_LED_PIN, GPIO_LEVEL_LOW)

    for (;;) {
        BUZZER_OFF();
        LED_OFF();

        osal_msleep(1);

        const int event =
            osal_event_read(&g_emergency_alarm_event, EMERGENCY_ALARM_EVENT_ALL,
                            OSAL_WAIT_FOREVER, OSAL_WAITMODE_OR | OSAL_WAITMODE_CLR);
        if (event == OSAL_FAILURE) {
            continue;
        }

        WLID_LINK_CLIENT_LOG_INFO("event = 0x%" PRIx32 "\r\n", event);

        // Priority handling: SOS > BUZZER_CONNECT > LED_ALARM > ACOUSTIC_ALARM
        if (event & EMERGENCY_ALARM_EVENT_SOS) {
            // 处理SOS信号 - 国际通用SOS信号模式：三短、三长、三短
            for (int i = 0; i < 3; i++) { // 三短
                BUZZER_ON();
                LED_ON();
                osal_msleep(100);
                BUZZER_OFF();
                LED_OFF();
                osal_msleep(100);
            }

            osal_msleep(200); // 间隔

            for (int i = 0; i < 3; i++) { // 三长
                BUZZER_ON();
                LED_ON();
                osal_msleep(300);
                BUZZER_OFF();
                LED_OFF();
                osal_msleep(100);
            }

            osal_msleep(200); // 间隔

            for (int i = 0; i < 3; i++) { // 三短
                BUZZER_ON();
                LED_ON();
                osal_msleep(100);
                BUZZER_OFF();
                LED_OFF();
                osal_msleep(100);
            }
        }
        else if (event & EMERGENCY_ALARM_EVENT_BUZZER_BEEP_ONCE) {
            // Buzzer short beep
            BUZZER_ON();
            osal_msleep(100);
            BUZZER_OFF();
        }
        else if (event & EMERGENCY_ALARM_EVENT_ACOUSTIC_ALARM) {
            // Acoustic + LED alarm
            for (int i = 0; i < 3; i++) {
                BUZZER_ON();
                LED_ON();
                osal_msleep(200);
                BUZZER_OFF();
                LED_OFF();
                osal_msleep(200);
            }
        }
        else if (event & EMERGENCY_ALARM_EVENT_LED_ALARM) {
            // LED only alarm
            for (int i = 0; i < 3; i++) {
                LED_ON();
                osal_msleep(200);
                LED_OFF();
                osal_msleep(200);
            }
        }
    }

    // _exit:
    osal_kthread_destroy(g_emergency_alarm_task_handle, 0);
    return 0;
}
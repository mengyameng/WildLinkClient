set(PUBLIC_HEADER_LIST
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/common/wlid_link_client_log
    ${CMAKE_CURRENT_SOURCE_DIR}/src/task
)

set(SOURCES_LIST
    ${CMAKE_CURRENT_SOURCE_DIR}/src/app_entry.c
)

list(APPEND PUBLIC_HEADER_LIST
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/soft_i2c
    ${CMAKE_CURRENT_SOURCE_DIR}/shared/telemetry
)
list(APPEND SOURCES_LIST
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/soft_i2c/soft_i2c.c
    ${CMAKE_CURRENT_SOURCE_DIR}/shared/telemetry/node_telemetry.c
)

if ((DEFINED CONFIG_AHT20_DRIVER_ENABLED) AND ("${CONFIG_AHT20_DRIVER_ENABLED}" STREQUAL "y"))
    list(APPEND PUBLIC_HEADER_LIST
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/aht20
    )
    list(APPEND SOURCES_LIST
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/aht20/aht20.c
    )
    if ((DEFINED CONFIG_AHT20_TASK_ENABLED) AND ("${CONFIG_AHT20_TASK_ENABLED}" STREQUAL "y"))
        list(APPEND PUBLIC_HEADER_LIST
            ${CMAKE_CURRENT_SOURCE_DIR}/src/task/ath20
        )
        list(APPEND SOURCES_LIST
            ${CMAKE_CURRENT_SOURCE_DIR}/src/task/ath20/aht20_task.c
        )
    endif() # ((DEFINED CONFIG_AHT20_DRIVER_ENABLED) AND ("${CONFIG_AHT20_DRIVER_ENABLED}" STREQUAL "y"))
endif() # ((DEFINED CONFIG_AHT20_TASK_ENABLED) AND ("${CONFIG_AHT20_TASK_ENABLED}" STREQUAL "y"))

if ((DEFINED CONFIG_BMP280_DRIVER_ENABLED) AND ("${CONFIG_BMP280_DRIVER_ENABLED}" STREQUAL "y"))
    list(APPEND PUBLIC_HEADER_LIST
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/bmp280
    )
    list(APPEND SOURCES_LIST
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/bmp280/bmp280.c
    )
    if ((DEFINED CONFIG_BMP280_TASK_ENABLED) AND ("${CONFIG_BMP280_TASK_ENABLED}" STREQUAL "y"))
        list(APPEND PUBLIC_HEADER_LIST
            ${CMAKE_CURRENT_SOURCE_DIR}/src/task/bmp280
        )
        list(APPEND SOURCES_LIST
            ${CMAKE_CURRENT_SOURCE_DIR}/src/task/bmp280/bmp280_task.c
        )
    endif() # ((DEFINED CONFIG_BMP280_DRIVER_ENABLED) AND ("${CONFIG_BMP280_DRIVER_ENABLED}" STREQUAL "y"))
endif() # ((DEFINED CONFIG_BMP280_TASK_ENABLED) AND ("${CONFIG_BMP280_TASK_ENABLED}" STREQUAL "y"))

if ((DEFINED CONFIG_ATK_LORA_DRIVER_ENABLED) AND ("${CONFIG_ATK_LORA_DRIVER_ENABLED}" STREQUAL "y"))
    list(APPEND PUBLIC_HEADER_LIST
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/atk_lora
    )
    list(APPEND SOURCES_LIST
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/atk_lora/atk_lora.c
    )
    if ((DEFINED CONFIG_ATK_LORA_TASK_ENABLED) AND ("${CONFIG_ATK_LORA_TASK_ENABLED}" STREQUAL "y"))
        list(APPEND PUBLIC_HEADER_LIST
            ${CMAKE_CURRENT_SOURCE_DIR}/src/task/atk_lora
        )
        list(APPEND SOURCES_LIST
            ${CMAKE_CURRENT_SOURCE_DIR}/src/task/atk_lora/atk_lora_task.c
        )
    endif() # ((DEFINED CONFIG_ATK_LORA_TASK_ENABLED) AND ("${CONFIG_ATK_LORA_TASK_ENABLED}" STREQUAL "y"))
endif() # ((DEFINED CONFIG_ATK_LORA_DRIVER_ENABLED) AND ("${CONFIG_ATK_LORA_DRIVER_ENABLED}" STREQUAL "y"))

if ((DEFINED CONFIG_SLE_CLIENT_TASK_ENABLED) AND ("${CONFIG_SLE_CLIENT_TASK_ENABLED}" STREQUAL "y"))
    list(APPEND PUBLIC_HEADER_LIST
        ${CMAKE_CURRENT_SOURCE_DIR}/src/task/sle_client
    )
    list(APPEND SOURCES_LIST
        ${CMAKE_CURRENT_SOURCE_DIR}/src/task/sle_client/sle_client_task.c
    )
endif() # ((DEFINED CONFIG_SLE_CLIENT_TASK_ENABLED) AND ("${CONFIG_SLE_CLIENT_TASK_ENABLED}" STREQUAL "y"))

if ((DEFINED CONFIG_EMERGENCY_ALARM_TASK_ENABLED) AND ("${CONFIG_EMERGENCY_ALARM_TASK_ENABLED}" STREQUAL "y"))
    list(APPEND PUBLIC_HEADER_LIST
        ${CMAKE_CURRENT_SOURCE_DIR}/src/task/emergency_alarm
    )
    list(APPEND SOURCES_LIST
        ${CMAKE_CURRENT_SOURCE_DIR}/src/task/emergency_alarm/emergency_alarm_task.c
    )
endif() # ((DEFINED CONFIG_EMERGENCY_ALARM_TASK_ENABLED) AND ("${CONFIG_EMERGENCY_ALARM_TASK_ENABLED}" STREQUAL "y"))

if ((DEFINED CONFIG_BLE_SERVER_TASK_ENABLED) AND ("${CONFIG_BLE_SERVER_TASK_ENABLED}" STREQUAL "y"))
    list(APPEND PUBLIC_HEADER_LIST
        ${CMAKE_CURRENT_SOURCE_DIR}/src/task/ble_server
    )
    list(APPEND SOURCES_LIST
        ${CMAKE_CURRENT_SOURCE_DIR}/src/task/ble_server/ble_server_task.c
    )
endif() # ((DEFINED CONFIG_BLE_SERVER_TASK_ENABLED) AND ("${CONFIG_BLE_SERVER_TASK_ENABLED}" STREQUAL "y"))

if ((DEFINED CONFIG_U8G2_DRIVER_ENABLED) AND ("${CONFIG_U8G2_DRIVER_ENABLED}" STREQUAL "y"))
    list(APPEND PUBLIC_HEADER_LIST
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc
    )
    list(APPEND SOURCES_LIST
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/mui_u8g2.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/mui.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8g2_arc.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8g2_bitmap.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8g2_box.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8g2_buffer.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8g2_button.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8g2_circle.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8g2_cleardisplay.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8g2_d_memory.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8g2_d_setup.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8g2_font.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8g2_fonts.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8g2_hvline.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8g2_input_value.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8g2_intersection.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8g2_kerning.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8g2_line.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8g2_ll_hvline.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8g2_message.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8g2_polygon.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8g2_selection_list.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8g2_setup.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8log_u8g2.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8log_u8x8.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8log.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_8x8.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_byte.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_cad.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_capture.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_a2printer.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ch1120.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_gp1247ai.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_gp1287ai.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_gp1294ai.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_gu800.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_hd44102.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_jl3820_296x128.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_jst3020.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_jst3088.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_jst7920.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ks0108.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_lc7981.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ld7032_60x32.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ls013b7dh03.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_max7219.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_pcd8544_84x48.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_pcf8812.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_pcf8814_lx1230.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_s1d15e06.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_s1d15300.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_s1d15721.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_sbn1661.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_sed1330.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_sh1106_64x32.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_sh1106_72x40.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_sh1107.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_sh1108.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_sh1122.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ssd1305.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ssd1306_48x64.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ssd1306_64x32.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ssd1306_64x48.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ssd1306_72x40.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ssd1306_96x16.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ssd1306_96x40.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ssd1306_128x32.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ssd1306_128x64_noname.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ssd1306_2040x16.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ssd1309.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ssd1312.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ssd1315_128x64_noname.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ssd1316.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ssd1317.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ssd1318.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ssd1320.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ssd1322.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ssd1325.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ssd1326.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ssd1327.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ssd1329.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ssd1362.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ssd1363.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ssd1606_172x72.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_ssd1607_200x200.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_st7302.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_st7305.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_st7511.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_st7528.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_st7539.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_st7565.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_st7567.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_st7571.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_st7586s_erc240160.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_st7586s_jlx320160.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_st7586s_jlx384160.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_st7586s_md240128.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_st7586s_s028hn118a.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_st7586s_ymc240160.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_st7588.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_st7920.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_st75160.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_st75161.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_st75256.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_st75320.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_stdio.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_t6963.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_uc1601.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_uc1604.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_uc1608.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_uc1609.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_uc1610.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_uc1611.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_uc1617.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_uc1628.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_uc1638.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_uc1701_dogs102.c
        # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_d_uc1701_mini12864.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_debounce.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_display.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_fonts.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_gpio.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_input_value.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_message.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_selection_list.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_setup.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_string.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_u8toa.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/u8g2/csrc/u8x8_u16toa.c
    )
    if ((DEFINED CONFIG_SH1106_TASK_ENABLED) AND ("${CONFIG_SH1106_TASK_ENABLED}" STREQUAL "y"))
        list(APPEND PUBLIC_HEADER_LIST
            ${CMAKE_CURRENT_SOURCE_DIR}/src/task/sh1106
        )
        list(APPEND SOURCES_LIST
            ${CMAKE_CURRENT_SOURCE_DIR}/src/task/sh1106/free_menu/free_menu.c
            ${CMAKE_CURRENT_SOURCE_DIR}/src/task/sh1106/sh1106_task.c
        )
    endif() # ((DEFINED CONFIG_SH1106_TASK_ENABLED) AND ("${CONFIG_SH1106_TASK_ENABLED}" STREQUAL "y"))
endif() # ((DEFINED CONFIG_U8G2_DRIVER_ENABLED)  AND ("${CONFIG_U8G2_DRIVER_ENABLED}" STREQUAL "y"))


set(PUBLIC_HEADER "${PUBLIC_HEADER}" ${PUBLIC_HEADER_LIST} PARENT_SCOPE)
set(SOURCES "${SOURCES}" "${SOURCES_LIST}" PARENT_SCOPE)
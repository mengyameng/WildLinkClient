/*
    Author:     Adam
    Github:     https://gitee.com/AdamLoong

    Version:    4.0.0

    MIT License

    Copyright (c) 2025 AdamLoong

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef _FREE_MENU_H_
#define _FREE_MENU_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

/*移植开始*/

/*移植需实现绘制字符串函数*/
void menu_hal_draw_string(int x, int y, const char *str);               // 绘制字符串函数
void menu_hal_draw_cursor(int x, int y, int w, int h, int cursor_type); // 绘制光标函数
void menu_hal_draw_scroll(int numerator, int denominator);              // 绘制滚动条函数

/*配置参数*/
#define MENU_PANEL_WIDTH (128)      // 菜单面板宽度
#define MENU_PANEL_HEIGHT (64)      // 菜单面板高度
#define MENU_ITEM_XOFFSET (4)       // 菜单项X轴偏移
#define MENU_ITEM_HEIGHT (13)       // 菜单项高度
#define MENU_FONT_WIDTH (6)         // 字体宽度
#define MENU_FONT_HEIGHT (10)       // 字体高度
#define MENU_ANIMATION_SPEED (0.2f) // 动画速度[0,1], 为1时无动画效果
#define MENU_TITLE (1)              // 是否显示标题

/*移植结束*/

#define MENU_EVENT_ENTER 1
#define MENU_EVENT_BACK 2
#define MENU_EVENT_WHEEL_UP 3
#define MENU_EVENT_WHEEL_DOWN 4

#define MENU_ITEM_CMD_NAME 0  // 读名
#define MENU_ITEM_CMD_ENTER 1 // 执行
#define MENU_ITEM_CMD_BACK 2  // 退出
#define MENU_ITEM_CMD_COUNT 3 // 获取项数量

#define MENU_CURSOR_TYPE_NORMAL 0 // 正常
#define MENU_CURSOR_TYPE_EDIT 1   // 编辑光标
#define MENU_CURSOR_TYPE_NONE 2   // 无光标

typedef void *(*MenuItemHandler)(void *items, int index, int cmd);

typedef char *(*MenuItemTypeFunc)(bool action);

typedef struct _MenuItemTypeStru
{
    char *name;
    void (*func)(void);
} MenuItemTypeStru;

typedef struct _MenuItemTypeList
{
    struct _MenuItemTypeList *next;
    char *name;
    void (*func)(void);
} MenuItemTypeList;

typedef struct _Menu
{
    struct _Menu *next;
    void *items;
    MenuItemHandler item_cb;

    int16_t item_count;
    int16_t item_index;

    int16_t cursor_width;
    int8_t cursor_limit;
    int8_t cursor_index;
} Menu;

extern Menu *hmenu;

void menu_init(Menu *menu, void *items, MenuItemHandler item_cb);
void menu_jump_to(Menu *menu);
void menu_event_inject(Menu *menu, uint32_t event);
void menu_event_wheel(Menu *menu, int wheel);
Menu *menu_show(Menu *menu);
Menu *menu_show_edit_cursor(Menu *menu, int edit_offset, int edit_w);
void menu_select_item(Menu *menu, int item_index);

void menu_set_focus(Menu *menu); //
void menu_back_focus(void);      // 返回上一级菜单
#define menu_get_focus() (hmenu)
#define menu_show_focus() (menu_show(menu_get_focus()))
#define menu_show_focus_edit_cursor(edit_offset, edit_w) (menu_show_edit_cursor(menu_get_focus(), edit_offset, edit_w))
#define menu_event_inject_focus(event) (menu_event_inject(menu_get_focus(), event))
#define menu_event_wheel_focus(wheel) (menu_event_wheel(menu_get_focus(), wheel))

//
void *menu_item_handler_func(void *items, int index, int cmd);
void *menu_item_handler_stru(void *items, int index, int cmd);
void *menu_item_handler_char(void *items, int index, int cmd);
void *menu_item_handler_list(void *items, int index, int cmd);
void menu_item_list_add(Menu *menu, char *name, void (*func)(void));

#endif // _FREE_MENU_H_

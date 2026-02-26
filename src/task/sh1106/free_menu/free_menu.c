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

#include "free_menu.h"

#include "osal_addr.h"
#include "u8g2.h"
#define malloc osal_vmalloc

Menu *hmenu = NULL;

// Circular stack for menu navigation
#define MENU_STACK_SIZE 5
static Menu *g_menu_stack[MENU_STACK_SIZE];
static int g_menu_stack_top = -1;
static int g_menu_stack_count = 0;

#define MENU_CAL_ITEM_X(str) ((MENU_PANEL_WIDTH - (strlen(str) * MENU_FONT_WIDTH)) / 2)
/*
#define MENU_CAL_ITEM_Y(index)                                                         \
    ((index) * MENU_ITEM_HEIGHT + ((MENU_ITEM_HEIGHT - MENU_FONT_HEIGHT) / 2)          \
     + MENU_FONT_HEIGHT)*/
#define MENU_CAL_ITEM_Y(index)                                                         \
    ((index) * MENU_ITEM_HEIGHT + ((MENU_ITEM_HEIGHT + MENU_FONT_HEIGHT) / 2))
static void smooth_transition(int *(value), int target) {
    if (*(value) != target) {
        (*(value)) += (target > *(value)) ? 1 : -1;
        (*(value)) += (target - *(value)) * MENU_ANIMATION_SPEED;
    }
}

void menu_hal_draw_string(int x, int y, const char *str) {
    extern u8g2_t sh1106_u8g2;
    u8g2_DrawUTF8(&sh1106_u8g2, x, y, str);
}

void menu_hal_draw_cursor(int x, int y, int w, int h, int cursor_type) {
    extern u8g2_t sh1106_u8g2;
    // u8g2_SetDrawColor(&sh1106_u8g2, 2);
    u8g2_DrawRFrame(&sh1106_u8g2, x, y, w, h, 5);
    // u8g2_SetDrawColor(&sh1106_u8g2, 1);
}

void menu_hal_draw_scroll(int numerator, int denominator) {
    return;
}

void menu_init(Menu *menu, void *items, MenuItemHandler item_cb) {
    if (menu == NULL)
        return;

    menu->items = items;
    menu->item_cb = item_cb;
    menu->item_count = (int)menu->item_cb(menu->items, 1, MENU_ITEM_CMD_COUNT);

    menu->item_index = 0;
    menu->cursor_index = 0; // cursor_index Cannot be greater than item_index

    menu->cursor_limit = MENU_PANEL_HEIGHT / MENU_ITEM_HEIGHT;

    if (menu->cursor_limit > menu->item_count) {
        menu->cursor_limit = menu->item_count;
    }

    menu_event_wheel(menu, 1);
}

// Push menu to circular stack
static void menu_stack_push(Menu *menu) {
    if (menu == NULL) {
        return;
    }

    // Calculate next top index (circular)
    g_menu_stack_top = (g_menu_stack_top + 1) % MENU_STACK_SIZE;
    g_menu_stack[g_menu_stack_top] = menu;

    // Update stack count (max MENU_STACK_SIZE)
    if (g_menu_stack_count < MENU_STACK_SIZE) {
        g_menu_stack_count++;
    }
}

// Pop menu from circular stack
static Menu *menu_stack_pop(void) {
    if (g_menu_stack_count == 0) {
        return NULL;
    }

    Menu *menu = g_menu_stack[g_menu_stack_top];

    // Calculate previous top index (circular)
    g_menu_stack_top = (g_menu_stack_top - 1 + MENU_STACK_SIZE) % MENU_STACK_SIZE;
    g_menu_stack_count--;

    return menu;
}

// Jump to specific menu (push current to stack)
void menu_jump_to(Menu *menu) {
    if (menu == NULL) {
        return;
    }

    // Push current menu to stack if it's not the same as target
    if (hmenu && hmenu != menu) {
        menu_stack_push(hmenu);
        // Set new menu as focus
        hmenu = menu;
    }
}

void menu_event_inject(Menu *menu, uint32_t event) {
    if (menu == NULL)
        return;

    switch (event) {
    case MENU_EVENT_ENTER:
        if (menu->item_index < MENU_TITLE || menu->item_index >= menu->item_count) {
            menu->item_cb(menu->items, 0, MENU_ITEM_CMD_BACK);
        }
        else {
            menu->item_cb(menu->items, menu->item_index, MENU_ITEM_CMD_ENTER);
        }
        break;

    case MENU_EVENT_BACK:
        menu->item_cb(menu->items, 0, MENU_ITEM_CMD_BACK);
        break;

    case MENU_EVENT_WHEEL_UP:
        menu_event_wheel(menu, -1);
        break;

    case MENU_EVENT_WHEEL_DOWN:
        menu_event_wheel(menu, +1);
        break;

    default:
        break;
    }
}

void menu_event_wheel(Menu *menu, int wheel) {
    if (menu == NULL)
        return;

    menu->item_index += wheel;
    menu->cursor_index += wheel;

    if (menu->item_index >= menu->item_count)
        menu->item_index = menu->item_count - 1;
    else if (menu->item_index < MENU_TITLE)
        menu->item_index = menu->cursor_index < MENU_TITLE ? 0 : MENU_TITLE;

    if (menu->cursor_index >= menu->cursor_limit)
        menu->cursor_index = menu->cursor_limit - 1;
    else if (menu->cursor_index < MENU_TITLE)
        menu->cursor_index = MENU_TITLE;

    if (menu->cursor_index > menu->item_index)
        menu->cursor_index = menu->item_index;
}

static void menu_show_items(Menu *menu) {
    if (menu == NULL)
        return;

    static int item_smooth;
    static int16_t ibase_prev = -1;
    static int scroll_offset = 0;
    static int scroll_target = 0;
    static uint8_t scroll_timer = 0;
    static uint8_t reset_timer = 0;
    static uint8_t scroll_state =
        0; // 0: 初始状态, 1: 滚动中, 2: 已滚动到最右侧, 3: 重置中
    int16_t ibase, inum, icnt;

    inum = MENU_TITLE;
    ibase = menu->item_index - menu->cursor_index;
    if (ibase < 0) {
        ibase = 0, menu->cursor_index = menu->item_index;
    }
    icnt = (ibase + menu->cursor_limit > menu->item_count) ? (menu->item_count - ibase)
                                                           : menu->cursor_limit;

    if (ibase != ibase_prev) {
        item_smooth = (ibase - ibase_prev) > 0 ? MENU_ITEM_HEIGHT : -MENU_ITEM_HEIGHT;
        ibase_prev = ibase;
        // 重置滚动状态
        scroll_offset = 0;
        scroll_target = 0;
        scroll_timer = 0;
        reset_timer = 0;
        scroll_state = 0;
    }

    smooth_transition(&item_smooth, 0);

    if (item_smooth != 0) {
        if (item_smooth < 0)
            inum++;
        else
            icnt--;
    }

    while (inum < icnt) {
        char *name =
            (char *)menu->item_cb(menu->items, ibase + inum, MENU_ITEM_CMD_NAME);

        if (ibase + inum == menu->item_index) {
            menu->cursor_width = strlen(name) * MENU_FONT_WIDTH + MENU_ITEM_XOFFSET * 2;

            // 计算文本宽度
            int16_t text_width = strlen(name) * MENU_FONT_WIDTH;
            // 计算可用宽度
            int16_t available_width = MENU_PANEL_WIDTH - MENU_ITEM_XOFFSET * 2;

            // 如果文本宽度超过可用宽度，设置滚动目标
            if (text_width > available_width) {
                switch (scroll_state) {
                case 0: // 初始状态
                    scroll_target = 0;
                    scroll_offset = 0;
                    scroll_state = 1;
                    break;
                case 1: // 滚动中
                    scroll_target = available_width - text_width;

                    // 滚动动画控制
                    scroll_timer++;
                    if (scroll_timer > 10) { // 延迟开始滚动
                        smooth_transition(&scroll_offset, scroll_target);
                        // 当滚动到最右侧时，改变滚动状态
                        if (scroll_offset <= scroll_target) {
                            scroll_state = 2;
                            reset_timer = 0;
                        }
                    }
                    break;
                case 2: // 已滚动到最右侧
                    // 停留一段时间
                    reset_timer++;
                    if (reset_timer > 30) { // 停留时间
                        scroll_state = 3;
                        scroll_target = 0;
                    }
                    break;
                case 3: // 重置中
                    smooth_transition(&scroll_offset, scroll_target);
                    if (scroll_offset >= scroll_target) {
                        scroll_state = 1;
                        scroll_timer = 0;
                    }
                    break;
                }
            }
            else {
                // 文本宽度不超过可用宽度，重置滚动
                scroll_offset = 0;
                scroll_target = 0;
                scroll_timer = 0;
                reset_timer = 0;
                scroll_state = 0;
            }

            // 绘制选中的菜单项（带滚动效果）
            menu_hal_draw_string(MENU_ITEM_XOFFSET + scroll_offset,
                                 MENU_CAL_ITEM_Y(inum) + item_smooth, name);
        }
        else {
            // 绘制未选中的菜单项（无滚动效果）
            menu_hal_draw_string(MENU_ITEM_XOFFSET, MENU_CAL_ITEM_Y(inum) + item_smooth,
                                 name);
        }

        inum++;
    }

    if (MENU_TITLE) {
        char *name = (char *)menu->item_cb(menu->items, 0, MENU_ITEM_CMD_NAME);
        menu_hal_draw_string(MENU_CAL_ITEM_X(name), MENU_CAL_ITEM_Y(0), name);
        if (menu->cursor_index == 0)
            menu_hal_draw_string(0, MENU_CAL_ITEM_Y(0), "<<");
    }
}

static void menu_show_cursor(int x, int y, int w, int h, int cursor_style) {
    static int x_smooth = 0, y_smooth = 0, w_smooth = 0, h_smooth = 0;
    smooth_transition(&x_smooth, x);
    smooth_transition(&y_smooth, y);
    smooth_transition(&w_smooth, w);
    h_smooth = h; // smooth_transition(&h_smooth, h);

    menu_hal_draw_cursor(x_smooth, y_smooth, w_smooth, h_smooth, cursor_style);
}

Menu *menu_show(Menu *menu) {
    if (menu == NULL)
        return hmenu;

    menu_show_items(menu);
    menu_show_cursor(0,                                     // x
                     menu->cursor_index * MENU_ITEM_HEIGHT, // y
                     (MENU_TITLE && menu->item_index == 0) ? MENU_PANEL_WIDTH
                                                           : menu->cursor_width, // w
                     MENU_ITEM_HEIGHT,                                           // h
                     MENU_CURSOR_TYPE_NORMAL);

    menu_hal_draw_scroll(menu->item_index > 1 ? menu->item_index : 1,
                         menu->item_count - 1);

    return hmenu;
}

Menu *menu_show_edit_cursor(Menu *menu, int edit_offset, int edit_w) {
    if (menu == NULL)
        return hmenu;

    menu_show_items(menu);

    menu_show_cursor(MENU_ITEM_XOFFSET + edit_offset * MENU_FONT_WIDTH, // x
                     menu->cursor_index * MENU_ITEM_HEIGHT,             // y
                     (MENU_TITLE && menu->item_index == 0)
                         ? MENU_PANEL_WIDTH
                         : edit_w * MENU_FONT_WIDTH, // w
                     MENU_ITEM_HEIGHT,               // h
                     MENU_CURSOR_TYPE_EDIT);

    menu_hal_draw_scroll(menu->item_index > 1 ? menu->item_index : 1,
                         menu->item_count - 1);

    return hmenu;
}

void menu_select_item(Menu *menu, int item_index) {
    if (menu == NULL)
        return;

    menu->item_index = 0;
    menu_event_wheel(menu, item_index);
}

void menu_set_focus(Menu *menu) {
    if (menu == NULL) {
        hmenu = NULL;
        return;
    }

    // ???????????????????
    Menu *p = hmenu;
    while (p) {
        if (p == menu) {
            break;
        }
        p = p->next;
    }
    if (p == NULL) {
        menu->next = hmenu;
    }
    hmenu = menu;
}

void menu_back_focus(void) // ?????????
{
    // Try to pop from stack first
    Menu *prev_menu = menu_stack_pop();
    if (prev_menu != NULL) {
        hmenu = prev_menu;
        return;
    }

    // Fallback to original implementation if stack is empty
    if (hmenu && hmenu->next) {
        hmenu = hmenu->next;
    }
}

void *menu_item_handler_func(void *items_, int index, int cmd) {
    MenuItemTypeFunc *items = (MenuItemTypeFunc *)items_;
    if (items == NULL || index < 0)
        return "null";

    void *ret = NULL;
    switch (cmd) {
    case MENU_ITEM_CMD_NAME: {
        ret = items[index](false);
        break;
    }
    case MENU_ITEM_CMD_ENTER: {
        if (index == 0)
            menu_back_focus();
        else
            items[index](true);
        break;
    }
    case MENU_ITEM_CMD_BACK: {
        menu_back_focus();
        break;
    }
    case MENU_ITEM_CMD_COUNT: {
        int count = 0;
        while (items[count])
            count++;
        ret = (void *)count;
        break;
    }
    }
    return ret;
}

void *menu_item_handler_stru(void *items_, int index, int cmd) {
    MenuItemTypeStru *items = (MenuItemTypeStru *)items_;
    if (items == NULL || index < 0)
        return "null";

    void *ret = NULL;
    switch (cmd) {
    case MENU_ITEM_CMD_NAME: {
        ret = items[index].name;
        break;
    }
    case MENU_ITEM_CMD_ENTER: {
        if (items[index].func)
            items[index].func();
        break;
    }
    case MENU_ITEM_CMD_BACK: {
        menu_back_focus();
        break;
    }
    case MENU_ITEM_CMD_COUNT: {
        int count = 0;
        while (items[count].name)
            count++;
        ret = (void *)count;
        break;
    }
    }
    return ret;
}

void *menu_item_handler_list(void *items_, int index, int cmd) {
    volatile MenuItemTypeList *items = (MenuItemTypeList *)items_;
    if (items == NULL || index < 0)
        return "null";

    void *ret = NULL;
    switch (cmd) {
    case MENU_ITEM_CMD_NAME: {
        while (items && index--)
            items = items->next;
        ret = items->name;
        break;
    }
    case MENU_ITEM_CMD_ENTER: {
        while (items && index--)
            items = items->next;

        if (items->func)
            items->func();
        break;
    }
    case MENU_ITEM_CMD_BACK: {
        menu_back_focus();
        break;
    }
    case MENU_ITEM_CMD_COUNT: {
        int count = 0;
        while (items) {
            items = items->next;
            count++;
        }
        ret = (void *)count;
        break;
    }
    }
    return ret;
}

void *menu_item_handler_char(void *items_, int index, int cmd) {
    char **items = (char **)items_;

    if (items == NULL || index < 0)
        return "null";

    void *ret = NULL;
    switch (cmd) {
    case MENU_ITEM_CMD_NAME: {
        ret = items[index];
        break;
    }
    case MENU_ITEM_CMD_ENTER: {
        break;
    }
    case MENU_ITEM_CMD_BACK: {
        menu_back_focus();
        break;
    }
    case MENU_ITEM_CMD_COUNT: {
        int count = 0;
        while (items[count])
            count++;
        ret = (void *)count;
        break;
    }
    }
    return ret;
}

void menu_item_list_add(Menu *menu, char *name, void (*func)(void)) {
    if (menu == NULL)
        return;
    MenuItemTypeList *item = (MenuItemTypeList *)malloc(sizeof(MenuItemTypeList));
    item->name = name;
    item->func = func;
    item->next = NULL;

    MenuItemTypeList *p = menu->items;

    if (p == NULL) {
        menu->items = item;
    }
    else {
        while (p->next)
            p = p->next;
        p->next = item;
    }

    menu_init(menu, menu->items, menu_item_handler_list);
}
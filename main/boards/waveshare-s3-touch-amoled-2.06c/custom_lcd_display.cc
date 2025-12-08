#include <vector>
#include <esp_log.h>
#include <cstring>
#include <esp_lcd_panel_io.h>
#include <freertos/FreeRTOS.h>
#include "custom_lcd_display.h"
#include "esp_lvgl_port.h"
#include "settings.h"
#include "assets/lang_config.h"
#include "config.h"
#include "board.h"

#define TAG "CustomLcdDisplay"

#define BYTES_PER_PIXEL (LV_COLOR_FORMAT_GET_SIZE(LV_COLOR_FORMAT_RGB565))

static uint8_t *dest_map = NULL;

void CustomLcdDisplay::lvgl_port_flush_callback(lv_display_t *drv, const lv_area_t *area, uint8_t *color_map) {
    assert(drv != NULL);
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)lv_display_get_user_data(drv);
    assert(panel_handle != NULL);
    lv_draw_sw_rgb565_swap(color_map, lv_area_get_width(area) * lv_area_get_height(area));

    lv_display_rotation_t rotation = lv_display_get_rotation(drv);
    lv_area_t rotated_area;
    if(rotation != LV_DISPLAY_ROTATION_0) {
        lv_color_format_t cf = lv_display_get_color_format(drv);
        /*Calculate the position of the rotated area*/
        rotated_area = *area;
        lv_display_rotate_area(drv, &rotated_area);
        /*Calculate the source stride (bytes in a line) from the width of the area*/
        uint32_t src_stride = lv_draw_buf_width_to_stride(lv_area_get_width(area), cf);
        /*Calculate the stride of the destination (rotated) area too*/
        uint32_t dest_stride = lv_draw_buf_width_to_stride(lv_area_get_width(&rotated_area), cf);
        /*Have a buffer to store the rotated area and perform the rotation*/

        int32_t src_w = lv_area_get_width(area);
        int32_t src_h = lv_area_get_height(area);
        lv_draw_sw_rotate(color_map, dest_map, src_w, src_h, src_stride, dest_stride, rotation, cf);
        /*Use the rotated area and rotated buffer from now on*/
        area = &rotated_area;
    }
    esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, dest_map);
    lv_disp_flush_ready(drv);
}

void CustomLcdDisplay::lvgl_rounder_cb(lv_event_t *e) {
    lv_area_t *area = (lv_area_t *)lv_event_get_param(e);

    uint16_t x1 = area->x1;
    uint16_t x2 = area->x2;

    uint16_t y1 = area->y1;
    uint16_t y2 = area->y2;

    area->x1 = (x1 >> 1) << 1;
    area->y1 = (y1 >> 1) << 1;

    area->x2 = ((x2 >> 1) << 1) + 1;
    area->y2 = ((y2 >> 1) << 1) + 1;
}

CustomLcdDisplay::CustomLcdDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                  int width, int height, int offset_x, int offset_y,
                  bool mirror_x, bool mirror_y, bool swap_xy)
    : LcdDisplay(panel_io, panel, width, height) {
    int bufferLen = width_ * 30 * BYTES_PER_PIXEL;
    dest_map = (uint8_t *)heap_caps_malloc(bufferLen, MALLOC_CAP_DMA);
    assert(dest_map);
    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();

    ESP_LOGI(TAG, "Initialize LVGL port");
    lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    port_cfg.task_priority = 2;
    port_cfg.timer_period_ms = 50;
    lvgl_port_init(&port_cfg);
   
    lvgl_port_lock(0);
    display_ = lv_display_create(width_, height_); 
    lv_display_set_flush_cb(display_, lvgl_port_flush_callback);  
    uint8_t *buffer1 = NULL;
    uint8_t *buffer2 = NULL;
    buffer1 = (uint8_t *)heap_caps_malloc(bufferLen, MALLOC_CAP_DMA);
    buffer2 = (uint8_t *)heap_caps_malloc(bufferLen, MALLOC_CAP_DMA);
    assert(buffer1);
    assert(buffer2);
    lv_display_set_buffers(display_, buffer1, buffer2, bufferLen, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_user_data(display_, panel);    
    lv_display_add_event_cb(display_,lvgl_rounder_cb,LV_EVENT_INVALIDATE_AREA,NULL); 
    lv_disp_set_rotation(display_, LV_DISPLAY_ROTATION_270);
    lvgl_port_unlock();

    if (display_ == nullptr) {
        ESP_LOGE(TAG, "Failed to add display");
        return;
    }

    if (offset_x != 0 || offset_y != 0) {
        lv_display_set_offset(display_, offset_x, offset_y);
    }
    SetupUI();
}
#include "LVGL_Driver.h"

static const char *LVGL_TAG = "LVGL";   
lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
lv_disp_drv_t disp_drv;      // contains callback functions

lv_indev_drv_t indev_drv;
esp_timer_handle_t lvgl_tick_timer = NULL;


static void *buf1 = NULL;
static void *buf2 = NULL;     
#define DRAW_BUF_LINES 60       



void example_lvgl_flush_cb(lv_disp_drv_t *drv,
                           const lv_area_t *area,
                           lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle =
        (esp_lcd_panel_handle_t) drv->user_data;

    esp_lcd_panel_draw_bitmap(
        panel_handle,
        area->x1,
        area->y1,
        area->x2 + 1,
        area->y2 + 1,
        color_map
    );

    lv_disp_flush_ready(drv);
}

void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

/*Read the touchpad*/
void example_touchpad_read( lv_indev_drv_t * drv, lv_indev_data_t * data )
{
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;

    /* Read touch controller data */
    esp_lcd_touch_read_data(drv->user_data);

    /* Get coordinates */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(drv->user_data, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0) {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PR;
        // ESP_LOGI(LVGL_TAG, "X=%u Y=%u", data->point.x, data->point.y);
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

static void lvgl_tick_cb(void* arg)
{
    // Only increment LVGL tick
    lv_tick_inc(1); // 1 ms tick
}

void lvgl_task(void *arg)
{
    while (1) {
        uint32_t delay_ms = lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

void LVGL_Init(void)
{
    ESP_LOGI(LVGL_TAG, "Initialize LVGL library");
    lv_init();
#if CONFIG_EXAMPLE_DOUBLE_FB
    ESP_LOGI(LVGL_TAG, "Use frame buffers as LVGL draw buffers");
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 2, &buf1, &buf2));
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES);
#else
    buf1 = heap_caps_malloc(
        EXAMPLE_LCD_H_RES * DRAW_BUF_LINES * sizeof(lv_color_t),
        MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL
    );

    buf2 = heap_caps_malloc(
        EXAMPLE_LCD_H_RES * DRAW_BUF_LINES * sizeof(lv_color_t),
        MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL
    );

    lv_disp_draw_buf_init(
        &disp_buf,
        buf1,
        buf2,
        EXAMPLE_LCD_H_RES * DRAW_BUF_LINES
    );
//     ESP_LOGI(LVGL_TAG, "Allocate separate LVGL draw buffers from PSRAM");
//    buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * (EXAMPLE_LCD_V_RES / 2) * sizeof(lv_color_t),
//                         MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
//     assert(buf1);

//     // Optional second buffer for double buffering
//     buf2 = heap_caps_malloc(EXAMPLE_LCD_H_RES * (EXAMPLE_LCD_V_RES / 2) * sizeof(lv_color_t),
//                             MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
//     assert(buf2);
//     // initialize LVGL draw buffers
//     lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * (EXAMPLE_LCD_V_RES / 2));
#endif // CONFIG_EXAMPLE_DOUBLE_FB

    ESP_LOGI(LVGL_TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
// #if CONFIG_EXAMPLE_DOUBLE_FB
    // disp_drv.full_refresh = true; // the full_refresh mode can maintain the synchronization between the two frame buffers
// #endif
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    ESP_LOGI(LVGL_TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &lvgl_tick_cb,
        .name = "lvgl_tick"
    };

    /********************* LVGL *********************/
    ESP_LOGI(LVGL_TAG,"Register display indev to LVGL");
    lv_indev_drv_init ( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = example_touchpad_read;
    indev_drv.user_data = tp;
    lv_indev_drv_register( &indev_drv );

    // ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    // ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));


    esp_timer_handle_t lvgl_tick_timer;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, 1000)); 

    xTaskCreatePinnedToCore(lvgl_task, "lvgl_task", 8192, NULL, 5, NULL, 1);

}
#include <stdio.h>
#include "lv_conf.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "TCA9554PWR.h"
#include "ST7701S.h"
#include "GT911.h"
#include "LVGL_Driver.h"
#include "boot/boot_screen.h"
#include "ui/screens/ui_Screen1.h"
#include "ui/ui.h"


float afr = 14.7f;
float boost = -20.0f;       
bool afr_increasing = true; 
bool boost_increasing = true; 

static lv_color_t green_color;
static lv_color_t red_color;
static lv_color_t blue_color;
static lv_color_t orange_color;

static lv_timer_t *blink_timer = NULL;


static inline int map_int(int x,int in_min, int in_max, int out_min, int out_max){
    return (x - in_min) * (out_max - out_min)
           / (in_max - in_min)
           + out_min;
}

static inline float constrain_float(float x, float low, float high) {
    if (x < low) return low;
    if (x > high) return high;
    return x;
}

static void init_label_styles(void){

    blue_color = lv_palette_main(LV_PALETTE_BLUE);
    green_color = lv_palette_main(LV_PALETTE_GREEN);
    red_color = lv_palette_main(LV_PALETTE_RED);
    orange_color = lv_palette_main(LV_PALETTE_DEEP_ORANGE);
}

static void update_label_if_needed(lv_obj_t *label, float new_value, lv_color_t new_color){
    char buf[20];
    sprintf(buf, "%.1f", new_value);

    const char *old_text = lv_label_get_text(label);
    if (strcmp(old_text, buf) != 0) {
        lv_label_set_text(label, buf);
    }
    lv_color_t old_color = lv_obj_get_style_text_color(label, LV_PART_MAIN);
    if (old_color.full != new_color.full) {
        lv_obj_set_style_text_color(label, new_color, LV_PART_MAIN);
    }

}

void simulate_guage_movement(float gauge_value, float max_value, float min_value, float step, lv_obj_t * text, const char * type, bool isIncreasing){
    
    gauge_value += (isIncreasing ? step : -step);
    lv_color_t color = green_color;
    
    if (gauge_value >= max_value || gauge_value <= min_value){
        isIncreasing = !isIncreasing;
        gauge_value = constrain_float(gauge_value, min_value, max_value);
    }
    if(strcmp(type, "afr") == 0) {
        afr = gauge_value;
        afr_increasing = isIncreasing;
        color = (gauge_value > 15.5f || gauge_value < 10.2f) ? red_color : green_color;
    } else if(strcmp(type, "boost") == 0) {
        boost = gauge_value;
        boost_increasing = isIncreasing;
        color = (gauge_value > 22.0f) ? red_color : green_color;

    }
    update_label_if_needed(text, gauge_value, color);
}


void gauge_timer(lv_timer_t * t){
    simulate_guage_movement(afr, 20.0f, 9.0f, 0.2f, ui_Label5, "afr", afr_increasing);
    simulate_guage_movement(boost, 30.0f, -20.0f, 0.2f, ui_Label6, "boost", boost_increasing);
}



void build_ui(){
    lv_init();
    ui_init();
    init_label_styles();

    lv_timer_create(gauge_timer, 100, NULL);
    
    while(1) {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(16));
    }
}

void app_main(void){   
    I2C_Init();
    EXIO_Init();
    LCD_Init();
    Touch_Init();
    LVGL_Init();

    build_ui();
}

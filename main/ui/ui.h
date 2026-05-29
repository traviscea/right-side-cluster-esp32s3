

#ifndef _STIGAUGECIRCLELEFT_UI_H
#define _STIGAUGECIRCLELEFT_UI_H

#ifdef __cplusplus
extern "C" {
#endif
    #include "lvgl.h"

#include "ui_helpers.h"
#include "ui_events.h"

///////////////////// SCREENS ////////////////////
#include "screens/ui_Screen1.h"
#include "screens/ui_Screen2.h"

///////////////////// VARIABLES ////////////////////
typedef enum {

    SCREEN_GAUGES,
    SCREEN_TIRES

} screen_t;

extern screen_t currentScreen;

typedef struct {
    lv_obj_t *container;
    lv_obj_t *zone1;
    lv_obj_t *zone2;
    lv_obj_t *zone3;
    lv_obj_t *zone4;
    lv_obj_t *zone5;
    lv_obj_t *zone6;
    lv_obj_t *zone1_label;
    lv_obj_t *zone2_label;
    lv_obj_t *zone3_label;
    lv_obj_t *zone4_label;
    lv_obj_t *zone5_label;
    lv_obj_t *zone6_label;
    lv_obj_t *titleLabel;
    lv_obj_t *statusDot;
    lv_obj_t *batteryIcon;
    lv_obj_t *batteryFill;
    lv_obj_t *batteryLabel;
} TireWidget;

extern TireWidget tire_FL;
extern TireWidget tire_FR;
extern TireWidget tire_RL;
extern TireWidget tire_RR;


// EVENTS
extern lv_obj_t *ui____initial_actions0;

LV_IMG_DECLARE( ui_black_boot_png);  
// UI INIT
void ui_init(void);
void ui_destroy(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif

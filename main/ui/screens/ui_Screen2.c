#include "../ui.h"
#include "lvgl.h"
#include <stdio.h>

lv_obj_t *ui_Screen2 = NULL;
lv_obj_t *ui_tempLegend = NULL;
lv_obj_t *ui_tempLegendLabelTop = NULL;
lv_obj_t *ui_tempLegendLabelBottom = NULL;

static lv_point_t line_points[] = {
    {0, 315},      // start (left middle)
    {480, 315}     // end (right middle)
};

extern lv_color_t temp_to_color(float temp);

static lv_style_transition_dsc_t color_trans;

static const lv_style_prop_t color_props[] = {
    LV_STYLE_BG_COLOR,
    0
};


void createTireWidget(
    TireWidget *tire,
    lv_obj_t *parent,
    const char *name,
    int x,
    int y
) {

    // =========================================
    // MAIN CONTAINER
    // =========================================

    tire->container =
        lv_obj_create(parent);

    lv_obj_set_size(
        tire->container,
        100,
        150
    );

    lv_obj_set_pos(
        tire->container,
        x,
        y
    );

    lv_obj_set_style_radius(
        tire->container,
        30,
        0
    );

    lv_obj_set_style_border_width(
        tire->container,
        4,
        0
    );

    lv_obj_set_style_border_color(
        tire->container,
        lv_color_white(),
        0
    );

    lv_obj_set_style_bg_color(
        tire->container,
        lv_color_black(),
        0
    );

    lv_obj_set_style_pad_all(
        tire->container,
        0,
        0
    );

    lv_obj_set_style_clip_corner(
        tire->container,
        true,
        0
    );

    lv_obj_clear_flag(
        tire->container,
        LV_OBJ_FLAG_SCROLLABLE
    );

    // =========================================
    // STATUS DOT
    // =========================================

    tire->statusDot =
        lv_obj_create(parent);

    lv_obj_remove_style_all(
        tire->statusDot
    );

    lv_obj_set_size(
        tire->statusDot,
        32,
        32
    );

    lv_obj_set_style_radius(
        tire->statusDot,
        LV_RADIUS_CIRCLE,
        0
    );

    lv_obj_set_style_bg_opa(
        tire->statusDot,
        LV_OPA_COVER,
        0
    );

    lv_obj_set_style_bg_color(
        tire->statusDot,
        lv_palette_main(LV_PALETTE_RED),
        0
    );

    lv_obj_set_pos(
        tire->statusDot,
        x + 14,
        y - 34
    );



    tire->zone1 =
        lv_obj_create(tire->container);

    lv_obj_remove_style_all(
        tire->zone1
    );

    lv_obj_set_style_bg_opa(
        tire->zone1,
        LV_OPA_COVER,
        0
    );

    lv_obj_set_style_bg_color(
        tire->zone1,
        lv_color_black(),
        0
    );

    lv_obj_set_style_border_width(
        tire->zone1,
        0,
        0
    );

    lv_obj_set_size(
        tire->zone1,
        20,
        150
    );

    lv_obj_set_style_transition(
        tire->zone1,
        &color_trans,
        0
    );


    tire->zone2 =
        lv_obj_create(tire->container);

    lv_obj_remove_style_all(
        tire->zone2
    );

    lv_obj_set_style_bg_opa(
        tire->zone2,
        LV_OPA_COVER,
        0
    );

    lv_obj_set_style_bg_color(
        tire->zone2,
        lv_color_black(),
        0
    );

    lv_obj_set_style_border_width(
        tire->zone2,
        0,
        0
    );

    lv_obj_set_size(
        tire->zone2,
        20,
        150
    );

    lv_obj_set_style_transition(
        tire->zone2,
        &color_trans,
        0
    );

    tire->zone3 =
        lv_obj_create(tire->container);

    lv_obj_remove_style_all(
        tire->zone3
    );

    lv_obj_set_style_bg_opa(
        tire->zone3,
        LV_OPA_COVER,
        0
    );

    lv_obj_set_style_bg_color(
        tire->zone3,
        lv_color_black(),
        0
    );

   lv_obj_set_style_border_width(
        tire->zone3,
        0,
        0
    );

    lv_obj_set_size(
        tire->zone3,
        20,
        150
    );

    lv_obj_set_style_transition(
        tire->zone3,
        &color_trans,
        0
    );

    tire->zone4 =
        lv_obj_create(tire->container);

    lv_obj_remove_style_all(
        tire->zone4
    );

    lv_obj_set_style_bg_opa(
        tire->zone4,
        LV_OPA_COVER,
        0
    );

    lv_obj_set_style_bg_color(
        tire->zone4,
        lv_color_black(),
        0
    );

   lv_obj_set_style_border_width(
        tire->zone4,
        0,
        0
    );

    lv_obj_set_size(
        tire->zone4,
        20,
        150
    );

    lv_obj_set_style_transition(
        tire->zone4,
        &color_trans,
        0
    );

    tire->zone5 =
        lv_obj_create(tire->container);

    lv_obj_remove_style_all(
        tire->zone5
    );

    lv_obj_set_style_bg_opa(
        tire->zone5,
        LV_OPA_COVER,
        0
    );

    lv_obj_set_style_bg_color(
        tire->zone5,
        lv_color_black(),
        0
    );

   lv_obj_set_style_border_width(
        tire->zone5,
        0,
        0
    );

    lv_obj_set_size(
        tire->zone5,
        20,
        150
    );
    
    lv_obj_set_style_transition(
        tire->zone5,
        &color_trans,
        0
    );


    lv_obj_set_pos(tire->zone1, 0, 0);
    lv_obj_set_pos(tire->zone2, 20, 0);
    lv_obj_set_pos(tire->zone3, 40, 0);
    lv_obj_set_pos(tire->zone4, 60, 0);
    lv_obj_set_pos(tire->zone5, 80, 0);


    tire->titleLabel =
        lv_label_create(parent);

    lv_label_set_text(
        tire->titleLabel,
        name
    );

    lv_obj_set_style_text_color(
        tire->titleLabel,
        lv_color_white(),
        0
    );

    lv_obj_set_style_text_font(
        tire->titleLabel,
        &lv_font_montserrat_16,
        0
    );

    lv_obj_align_to(
        tire->titleLabel,
        tire->container,
        LV_ALIGN_OUT_TOP_MID,
        -20,
        -10
    );


    // =========================================
    // BATTERY ICON
    // =========================================

    tire->batteryIcon =
        lv_obj_create(parent);

    lv_obj_set_size(
        tire->batteryIcon,
        34,
        16
    );

    lv_obj_set_style_radius(
        tire->batteryIcon,
        3,
        0
    );

    lv_obj_set_style_border_width(
        tire->batteryIcon,
        2,
        0
    );

    lv_obj_set_style_border_color(
        tire->batteryIcon,
        lv_color_white(),
        0
    );

    lv_obj_set_style_bg_opa(
        tire->batteryIcon,
        LV_OPA_TRANSP,
        0
    );

    lv_obj_set_style_pad_all(
        tire->batteryIcon,
        0,
        0
    );

    lv_obj_clear_flag(
        tire->batteryIcon,
        LV_OBJ_FLAG_SCROLLABLE
    );

    lv_obj_align_to(
        tire->batteryIcon,
        tire->titleLabel,
        LV_ALIGN_OUT_RIGHT_MID,
        10,
        0
    );

    // =========================================
    // BATTERY TIP
    // =========================================

    lv_obj_t *tip =
        lv_obj_create(parent);

    lv_obj_remove_style_all(tip);

    lv_obj_set_size(
        tip,
        4,
        8
    );

    lv_obj_set_style_bg_color(
        tip,
        lv_color_white(),
        0
    );

    lv_obj_set_style_bg_opa(
        tip,
        LV_OPA_COVER,
        0
    );

    lv_obj_align_to(
        tip,
        tire->batteryIcon,
        LV_ALIGN_OUT_RIGHT_MID,
        0,
        0
    );

    // =========================================
    // BATTERY FILL
    // =========================================

    tire->batteryFill =
        lv_obj_create(
            tire->batteryIcon
        );

    lv_obj_remove_style_all(
        tire->batteryFill
    );

    lv_obj_set_size(
        tire->batteryFill,
        28,
        10
    );

    lv_obj_set_pos(
        tire->batteryFill,
        1,
        1
    );

    lv_obj_set_style_bg_color(
        tire->batteryFill,
        lv_palette_main(LV_PALETTE_GREEN),
        0
    );

    lv_obj_set_style_bg_opa(
        tire->batteryFill,
        LV_OPA_COVER,
        0
    );
}



void ui_Screen2_screen_init(void) {
    LV_FONT_DECLARE(Doto_Extra_Bold_84);
    LV_FONT_DECLARE(Schoolbell_24);

    lv_style_transition_dsc_init(
        &color_trans,
        color_props,
        lv_anim_path_linear,
        900,
        0,
        NULL
    );

    ui_Screen2 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen2, LV_OBJ_FLAG_SCROLLABLE);

    createTireWidget(
        &tire_FL,
        ui_Screen2,
        "FL",
        100,
        70
    );

    createTireWidget(
        &tire_FR,
        ui_Screen2,
        "FR",
        280,
        70
    );

    createTireWidget(
        &tire_RL,
        ui_Screen2,
        "RL",
        100,
        270
    );

    createTireWidget(
        &tire_RR,
        ui_Screen2,
        "RR",
        280,
        270
    );

    // =========================================
    // TEMP LEGEND CONTAINER
    // =========================================

    ui_tempLegend =
        lv_obj_create(ui_Screen2);

    lv_obj_set_size(
        ui_tempLegend,
        38,
        260
    );

    lv_obj_set_pos(
        ui_tempLegend,
        220,
        110
    );

    lv_obj_set_style_radius(
        ui_tempLegend,
        0,
        0
    );

    lv_obj_set_style_border_width(
        ui_tempLegend,
        2,
        0
    );

    lv_obj_set_style_border_color(
        ui_tempLegend,
        lv_color_white(),
        0
    );

    lv_obj_set_style_pad_all(
        ui_tempLegend,
        0,
        0
    );

    lv_obj_clear_flag(
        ui_tempLegend,
        LV_OBJ_FLAG_SCROLLABLE
    );

    // =========================================
    // SMOOTH THERMAL LEGEND
    // =========================================

    const int SEGMENTS = 96;

    for(int i = 0; i < SEGMENTS; i++)
    {
        lv_obj_t *seg =
            lv_obj_create(ui_tempLegend);

        lv_obj_remove_style_all(seg);

        lv_obj_set_style_border_width(
            seg,
            0,
            0
        );

        lv_obj_set_style_radius(
            seg,
            0,
            0
        );

        float segHeight =
            260.0f / SEGMENTS;

        // -------------------------------------
        // NORMALIZED POSITION
        // -------------------------------------

        float p =
            (float)i
            /
            (SEGMENTS - 1);

        // -------------------------------------
        // MATCH temp_to_color()
        // -------------------------------------

        float temp =
            70.0f
            +
            (
                p
                *
                (220.0f - 70.0f)
            );

        lv_color_t c =
            temp_to_color(temp);

        // -------------------------------------
        // SEGMENT STYLE
        // -------------------------------------

        lv_obj_set_size(
            seg,
            38,
            (int)(segHeight + 1.5f)
        );

        lv_obj_set_pos(
            seg,
            0,
            (int)(260.0f - (i * segHeight))
        );

        lv_obj_set_style_bg_color(
            seg,
            c,
            0
        );

        lv_obj_set_style_bg_opa(
            seg,
            LV_OPA_COVER,
            0
        );
    }

    // =========================================
    // LEGEND TICKS
    // =========================================

    const int tickTemps[5] =
    {
        220,
        180,
        140,
        100,
        70
    };

    for(int i=0; i<5; i++)
    {
        // -------------------------------------
        // NORMALIZED POSITION
        // -------------------------------------

        float p =
            (tickTemps[i] - 70.0f)
            /
            (228.0f - 62.0f);

        int y =
            260
            -
            (p * 260);


        // -------------------------------------
        // LABEL
        // -------------------------------------

        lv_obj_t *label =
            lv_label_create(ui_Screen2);

        char buf[16];

        snprintf(
            buf,
            sizeof(buf),
            "%3d",
            tickTemps[i]
        );

        lv_label_set_text(
            label,
            buf
        );

        lv_obj_set_style_text_color(
            label,
            lv_color_black(),
            0
        );

        lv_obj_set_style_text_font(
            label,
            &lv_font_montserrat_14,
            0
        );

        lv_obj_set_pos(
            label,
            227,
            88 + y
        );
    }

}

void ui_Screen2_screen_destroy(void){
    if (ui_Screen2) lv_obj_del(ui_Screen2);

    ui_Screen2 = NULL;

}

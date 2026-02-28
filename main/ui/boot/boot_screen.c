#include "boot_screen.h"
#include "ui.h"

lv_obj_t *boot_screen= NULL;
static lv_obj_t *boot_arc;
static lv_obj_t *boot_logo;

/* ---------- Callbacks ---------- */
static void arc_anim_cb(void *obj, int32_t v)
{
    lv_arc_set_value(obj, v);
}

static void logo_opa_cb(void *obj, int32_t v)
{
    lv_obj_set_style_opa(obj, v, 0);
}

/* ---------- Create Screen ---------- */
void boot_screen_create(void)
{
    boot_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(boot_screen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(boot_screen, LV_OPA_COVER, 0);

    // boot_arc = lv_arc_create(boot_screen);
    // lv_obj_set_size(boot_arc, 340, 340);
    // lv_obj_center(boot_arc);
    // lv_arc_set_rotation(boot_arc, 135);
    // lv_arc_set_bg_angles(boot_arc, 0, 270);
    // lv_arc_set_value(boot_arc, 0);
    // lv_obj_set_style_arc_color(boot_arc, lv_color_black(), LV_PART_MAIN);

    // lv_obj_remove_style(boot_arc, NULL, LV_PART_KNOB);
    // lv_obj_set_style_arc_width(boot_arc, 15, LV_PART_INDICATOR);
    // lv_obj_set_style_arc_color(
    //     boot_arc, lv_color_hex(0xFF1E1E), LV_PART_INDICATOR
    // );

    boot_logo = lv_img_create(boot_screen);
    lv_img_set_src(boot_logo, &ui_black_boot_png);
    lv_obj_center(boot_logo);
    lv_obj_set_style_opa(boot_logo, LV_OPA_0, 0);

    // // Glow
    // lv_obj_set_style_shadow_color(
    //     boot_logo, lv_color_hex(0xFF1E1E), 0
    // );
    // lv_obj_set_style_shadow_width(boot_logo, 30, 0);
    // lv_obj_set_style_shadow_opa(boot_logo, LV_OPA_60, 0);
}

/* ---------- Start Animation ---------- */
void boot_start(void)
{
    lv_scr_load(boot_screen);

    // lv_anim_t a;
    // // lv_anim_init(&a);
    // // lv_anim_set_var(&a, boot_arc);
    // // lv_anim_set_exec_cb(&a, arc_anim_cb);
    // // lv_anim_set_values(&a, 0, 100);
    // // lv_anim_set_time(&a, 1000);
    // // lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    // // lv_anim_start(&a);

    // lv_anim_init(&a);
    // lv_anim_set_var(&a, boot_logo);
    // lv_anim_set_exec_cb(&a, logo_opa_cb);
    // lv_anim_set_values(&a, 0, 255);
    // lv_anim_set_time(&a, 500);
    // lv_anim_set_delay(&a, 700);
    // lv_anim_start(&a);
}

/* ---------- Exit ---------- */
void boot_finish(lv_obj_t *next_screen)
{
    // Animate fade from boot screen → main UI
    lv_scr_load_anim(next_screen, LV_SCR_LOAD_ANIM_FADE_ON, 2000, 3000, true);

    // Delete boot objects
    if (boot_screen) {
        lv_obj_del(boot_screen);
        boot_screen = NULL;
    }
}
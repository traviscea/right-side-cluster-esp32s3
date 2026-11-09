// Right gauge: boost and AFR needles laid out like the left fuel/coolant
// gauge, with the lap timer and session best lap in the middle. Same style as the
// center tach and left gauge (black face, red-orange backlit marks, Jost).
// LVGL version: 8.4

#ifndef UI_SCREEN1_H
#define UI_SCREEN1_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------- Look & feel (edit these to taste) ----------------
// Boost (left half): bottom of the scale -> top, a tick mark every 5 psi
#define UI_BOOST_MIN_PSI     -15.0f
#define UI_BOOST_MAX_PSI      30.0f
#define UI_BOOST_TICK_PSI      5.0f
#define UI_BOOST_WARN_PSI     25.0f  // dotted overboost zone starts here; hub turns red at/above

// AFR (right half): rich at the bottom -> lean at the top
#define UI_AFR_MIN             9.0f
#define UI_AFR_MAX            17.0f

#define UI_AFR_CHECK_PSI       5.0f  // AFR is only judged at/above this much boost...
#define UI_AFR_RICH_WARN      10.5f  // ...and turns red below this
#define UI_AFR_LEAN_WARN      11.9f  // ...or above this

// Needle smoothing (seconds). Small values keep boost snappy while taking the
// edge off sensor noise; AFR flickers more, so it gets a little more.
#define UI_BOOST_SMOOTH_S     0.06f
#define UI_AFR_SMOOTH_S       0.25f
#define UI_NEEDLE_TICK_MS       20

#define UI_COLOR_NEEDLE    0xFF2A12
#define UI_COLOR_VALUE     0xE6F0FF  // normal numbers (ice white)
#define UI_COLOR_HOT       0xFF2020  // red: warning
#define UI_COLOR_STATUS    0xFFC400  // restart message (yellow)

// ---------------- Face color themes ----------------
// The face (ticks, numerals, labels, icons) follows the current theme. Needles,
// readouts, warnings and the red dotted zones keep their own colors.
// The theme list must be identical on all three screens: the center sends the
// theme number to the side gauges.
typedef struct { const char *name; uint32_t dial; uint32_t dim; } ui_theme_t;
#define UI_THEME_COUNT 6
extern const ui_theme_t ui_themes[UI_THEME_COUNT];

uint32_t ui_theme_dial(void);              // current face color
uint32_t ui_theme_dim(void);               // current secondary color (captions)
uint8_t  ui_get_theme(void);
// Switch theme. Call from the LVGL thread (or with the LVGL lock held). Cheap
// to call every frame: it only redraws when the theme actually changes.
void     ui_set_theme(uint8_t index);

#define UI_COLOR_DIAL      (ui_theme_dial())
#define UI_COLOR_DIAL_DIM  (ui_theme_dim())
#define UI_COLOR_REDLINE   0xFF2A0A  // dotted red zones stay red on every theme

// 1 = pre-render the static dial into one image at boot (smoother needles).
#define UI_DIAL_USE_SNAPSHOT 1

// SCREEN: ui_Screen1
extern void ui_Screen1_screen_init(void);
extern void ui_Screen1_screen_destroy(void);
extern lv_obj_t *ui_Screen1;

// Update the gauge. Each call skips work when nothing visible changed.
void ui_right_set_boost(float psi);            // negative = vacuum
void ui_right_set_afr(float afr);              // colored using the last boost value
void ui_right_set_lap_ms(uint32_t lap_ms);     // 0 shows --:--.--
void ui_right_set_best_lap_ms(uint32_t best);  // session best; 0 shows --:--.--

// Show a short status line at the bottom of the screen, then remove it after
// duration_ms (used to report an unexpected restart).
void ui_show_status(const char *text, uint32_t duration_ms);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif

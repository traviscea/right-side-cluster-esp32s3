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

#define UI_COLOR_DIAL      0xFF3D1F  // backlit red-orange (same as the other gauges)
#define UI_COLOR_DIAL_DIM  0xB82A14  // captions
#define UI_COLOR_NEEDLE    0xFF2A12
#define UI_COLOR_VALUE     0xE6F0FF  // normal numbers (ice white)
#define UI_COLOR_COLD      0x4FB4FF  // blue: vacuum
#define UI_COLOR_HOT       0xFF2020  // red: warning

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

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif

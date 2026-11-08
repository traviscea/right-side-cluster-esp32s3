// Right gauge, laid out like the left fuel/coolant gauge:
//   * boost needle on the left (vacuum at the bottom, boost at the top, dotted
//     overboost zone), AFR needle on the right (rich bottom, lean top)
//   * live boost and AFR numbers inside the needle hubs
//   * current lap time (top) and session best lap (bottom) as digital readouts
// Designed for the 480x480 round display.
//
// Both scales sit on arcs that follow the round screen edge, while each
// needle pivots from its own hub. Every tick is placed where its needle
// direction meets that arc, so the needle always points exactly at it.
//
// The static dial is drawn once and flattened into one image with
// lv_snapshot at boot, so a needle move only re-blits a slice of it.

#include "../ui.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define UI_BASE_RES 480
#define K           ((float)LV_HOR_RES / UI_BASE_RES)
#define UI_SCALE(x) ((lv_coord_t)lroundf((x) * K))

// ---- Geometry (480px space, origin at the screen center, y down) ----
#define PIVOT_X     78.0f    // needle hubs at (-78, 0) and (+78, 0)
#define SCALE_R     228.0f   // outer edge of both scales, concentric with the screen
#define HALF_SPAN   40.0f    // each scale covers 180+-40 deg (boost) / 0+-40 deg (AFR)
#define HUB_D       88

LV_FONT_DECLARE(ui_font_rlabel_32);
LV_FONT_DECLARE(ui_font_rvalue_62);
LV_FONT_DECLARE(ui_font_rcaption_20);
LV_FONT_DECLARE(ui_font_rhub_32);

lv_obj_t *ui_Screen1 = NULL;

typedef struct {
    float pivot_x;
    lv_obj_t *needle, *glow;
    lv_point_t pts[2], glow_pts[2];
    float target, shown, drawn_angle;
    bool seeded;
} gauge_t;

static gauge_t boost_g = { .pivot_x = -PIVOT_X };
static gauge_t afr_g   = { .pivot_x =  PIVOT_X };

static lv_obj_t *boost_hub_value, *afr_hub_value, *lap_value, *best_value;
static lv_img_dsc_t *dial_snapshot = NULL;
static lv_timer_t *needle_timer = NULL;

static int last_boost_tenths = -100000, last_afr_tenths = -100000;
static uint32_t last_lap = 0xFFFFFFFF, last_best = 0xFFFFFFFF;
static float g_last_boost = 0;

// ---------------------------------------------------------------- geometry

static const float D2R = 3.14159265f / 180.0f;

// Screen angle (deg, clockwise from 3 o'clock) of a scale position 0..100
static float left_angle(float v)  { return 180.0f - HALF_SPAN + v * (2 * HALF_SPAN) / 100.0f; } // bottom-left -> top-left
static float right_angle(float p) { return HALF_SPAN - p * (2 * HALF_SPAN) / 100.0f; }          // bottom-right -> top-right

static float boost_pos(float psi)
{
    float p = (psi - UI_BOOST_MIN_PSI) * 100.0f / (UI_BOOST_MAX_PSI - UI_BOOST_MIN_PSI);
    return p < 0 ? 0 : p > 100 ? 100 : p;
}

static float afr_pos(float afr)
{
    float p = (afr - UI_AFR_MIN) * 100.0f / (UI_AFR_MAX - UI_AFR_MIN);
    return p < 0 ? 0 : p > 100 ? 100 : p;
}

static void arc_point(float ang_deg, float r, float *x, float *y)
{
    *x = r * cosf(ang_deg * D2R);
    *y = r * sinf(ang_deg * D2R);
}

static float hub_angle(float pivot_x, float x, float y)
{
    return atan2f(y, x - pivot_x) / D2R;
}

// ---------------------------------------------------------------- helpers

static lv_obj_t *make_label(lv_obj_t *parent, const lv_font_t *font, uint32_t color, const char *txt)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(color), 0);
    return l;
}

static void place(lv_obj_t *o, float x, float y)
{
    lv_obj_align(o, LV_ALIGN_CENTER, UI_SCALE(x), UI_SCALE(y));
}

// ---------------------------------------------------------------- static dial drawing

typedef struct { lv_draw_ctx_t *ctx; float cx, cy; } dctx_t;

static lv_point_t to_px(const dctx_t *d, float x, float y)
{
    lv_point_t p = { (lv_coord_t)lroundf(d->cx + x * K), (lv_coord_t)lroundf(d->cy + y * K) };
    return p;
}

// Tick at a screen angle, pointing from the arc toward that gauge's hub
static void draw_tick(const dctx_t *d, float pivot_x, float ang, float len, float width)
{
    float ox, oy;
    arc_point(ang, SCALE_R, &ox, &oy);
    float dx = pivot_x - ox, dy = -oy;
    float n = sqrtf(dx * dx + dy * dy);
    lv_point_t a = to_px(d, ox, oy);
    lv_point_t b = to_px(d, ox + dx / n * len, oy + dy / n * len);
    lv_draw_line_dsc_t ld;
    lv_draw_line_dsc_init(&ld);
    ld.color = lv_color_hex(UI_COLOR_DIAL);
    ld.width = UI_SCALE(width) < 1 ? 1 : UI_SCALE(width);
    lv_draw_line(d->ctx, &ld, &a, &b);
}

// Thin line along the edge plus ticks. 'divisions' equal steps; ends are long
// and heavy, every 'major_every'-th tick medium, the rest short.
static void draw_scale(const dctx_t *d, float pivot_x, float (*angle_of)(float), int divisions, int major_every)
{
    float a0 = angle_of(0), a1 = angle_of(100);
    float lo = a0 < a1 ? a0 : a1, hi = a0 < a1 ? a1 : a0;
    lv_draw_arc_dsc_t ad;
    lv_draw_arc_dsc_init(&ad);
    ad.color = lv_color_hex(UI_COLOR_DIAL);
    ad.width = UI_SCALE(3);
    lv_point_t c = { (lv_coord_t)lroundf(d->cx), (lv_coord_t)lroundf(d->cy) };
    int16_t s = (int16_t)lroundf(lo), e = (int16_t)lroundf(hi);
    if (s < 0) s += 360;
    if (e < 0) e += 360;
    lv_draw_arc(d->ctx, &ad, &c, UI_SCALE(SCALE_R), (uint16_t)s, (uint16_t)e);

    for (int i = 0; i <= divisions; i++) {
        float ang = angle_of(i * 100.0f / divisions);
        if (i == 0 || i == divisions)   draw_tick(d, pivot_x, ang, 30, 6);
        else if (i % major_every == 0)  draw_tick(d, pivot_x, ang, 20, 4);
        else                            draw_tick(d, pivot_x, ang, 12, 3);
    }
}

// Dotted band inside the arc between two scale angles, laid out in short
// hub-pointing columns so both ends are straight and parallel to the ticks
static void draw_stipple(const dctx_t *d, float pivot_x, float ang_from, float ang_to)
{
    if (ang_from > ang_to) { float t = ang_from; ang_from = ang_to; ang_to = t; }
    float x0, y0, x1, y1;
    arc_point(ang_from, SCALE_R, &x0, &y0);
    arc_point(ang_to, SCALE_R, &x1, &y1);
    float h0 = hub_angle(pivot_x, x0, y0), h1 = hub_angle(pivot_x, x1, y1);
    float hlo = h0 < h1 ? h0 : h1, hhi = h0 < h1 ? h1 : h0;

    lv_draw_rect_dsc_t dot;
    lv_draw_rect_dsc_init(&dot);
    dot.radius = LV_RADIUS_CIRCLE;
    dot.bg_color = lv_color_hex(UI_COLOR_DIAL);
    dot.bg_opa = LV_OPA_COVER;
    lv_coord_t ds = UI_SCALE(3) < 2 ? 2 : UI_SCALE(3);

    const float col_gap = 2.5f, edge_gap = 4.5f;
    static const float depth_even[] = { 8, 18 };
    static const float depth_odd[]  = { 13, 23 };
    float step = col_gap / SCALE_R / D2R;
    int col = 0;
    for (float ang = ang_from - 6; ang <= ang_to + 6; ang += step, col++) {
        float ox, oy;
        arc_point(ang, SCALE_R, &ox, &oy);
        float dx = pivot_x - ox, dy = -oy;
        float n = sqrtf(dx * dx + dy * dy);
        float h = hub_angle(pivot_x, ox, oy);
        float margin = edge_gap / n / D2R;
        if (h < hlo + margin || h > hhi - margin) continue;
        const float *depths = (col & 1) ? depth_odd : depth_even;
        for (int k = 0; k < 2; k++) {
            lv_point_t p = to_px(d, ox + dx / n * depths[k], oy + dy / n * depths[k]);
            lv_area_t a = { p.x - ds / 2, p.y - ds / 2, p.x - ds / 2 + ds - 1, p.y - ds / 2 + ds - 1 };
            lv_draw_rect(d->ctx, &dot, &a);
        }
    }
}

static void dial_draw_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_area_t a;
    lv_obj_get_coords(obj, &a);
    dctx_t d = { lv_event_get_draw_ctx(e), a.x1 + lv_area_get_width(&a) / 2.0f, a.y1 + lv_area_get_height(&a) / 2.0f };

    draw_stipple(&d, -PIVOT_X, left_angle(boost_pos(UI_BOOST_WARN_PSI)), left_angle(100));
    // Boost: a tick every 5 psi, medium ticks at -10/0/10/20 (every 10 psi
    // counted from the bottom). AFR: a tick every 1.0, medium every 2.
    int boost_div = (int)lroundf((UI_BOOST_MAX_PSI - UI_BOOST_MIN_PSI) / UI_BOOST_TICK_PSI);
    draw_scale(&d, -PIVOT_X, left_angle, boost_div, 2);
    draw_scale(&d,  PIVOT_X, right_angle, 8, 2);
}

static void label_at(lv_obj_t *root, float ang, const char *txt)
{
    float x, y;
    arc_point(ang, 188, &x, &y);
    place(make_label(root, &ui_font_rlabel_32, UI_COLOR_DIAL, txt), x, y);
}

static lv_obj_t *build_static_dial(lv_obj_t *parent)
{
    lv_obj_t *root = lv_obj_create(parent);
    lv_obj_remove_style_all(root);
    lv_obj_set_size(root, LV_HOR_RES, LV_VER_RES);
    lv_obj_center(root);
    lv_obj_set_style_bg_color(root, lv_color_hex(0x0B0B0B), 0);
    lv_obj_set_style_bg_opa(root, LV_OPA_COVER, 0);
    lv_obj_clear_flag(root, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(root, dial_draw_cb, LV_EVENT_DRAW_MAIN_END, NULL);

    // Scale end numbers, just inside the ends (like F/E and H/C on the left gauge)
    char buf[8];
    const float inset = 8;
    lv_snprintf(buf, sizeof(buf), "%d", (int)UI_BOOST_MAX_PSI);
    label_at(root, left_angle(100) - inset, buf);
    lv_snprintf(buf, sizeof(buf), "%d", (int)UI_BOOST_MIN_PSI);
    label_at(root, left_angle(0) + inset, buf);
    lv_snprintf(buf, sizeof(buf), "%d", (int)UI_AFR_MAX);
    label_at(root, right_angle(100) + inset, buf);
    lv_snprintf(buf, sizeof(buf), "%d", (int)UI_AFR_MIN);
    label_at(root, right_angle(0) - inset, buf);

    // Gauge names where the left gauge has its icons
    float x, y;
    arc_point(left_angle(50), 172, &x, &y);
    lv_obj_t *l = make_label(root, &ui_font_rcaption_20, UI_COLOR_DIAL, "BOOST\n(psi)");
    lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
    place(l, x, y);
    arc_point(right_angle(50), 178, &x, &y);
    place(make_label(root, &ui_font_rcaption_20, UI_COLOR_DIAL, "AFR"), x, y);

    // Captions on the hub side of the middle readouts
    place(make_label(root, &ui_font_rcaption_20, UI_COLOR_DIAL_DIM, "LAP"), 0, -124);
    place(make_label(root, &ui_font_rcaption_20, UI_COLOR_DIAL_DIM, "BEST"), 0, 124);
    return root;
}

// ---------------------------------------------------------------- live parts

static lv_obj_t *make_line(lv_obj_t *parent, lv_coord_t width, lv_opa_t opa)
{
    lv_obj_t *l = lv_line_create(parent);
    lv_obj_set_style_line_width(l, width, 0);
    lv_obj_set_style_line_color(l, lv_color_hex(UI_COLOR_NEEDLE), 0);
    lv_obj_set_style_line_opa(l, opa, 0);
    lv_obj_clear_flag(l, LV_OBJ_FLAG_CLICKABLE);
    return l;
}

static void set_line(lv_obj_t *l, lv_point_t *pts, lv_coord_t x0, lv_coord_t y0, lv_coord_t x1, lv_coord_t y1)
{
    lv_coord_t minx = LV_MIN(x0, x1), miny = LV_MIN(y0, y1);
    lv_obj_invalidate(l);
    lv_obj_set_pos(l, minx, miny);
    pts[0].x = x0 - minx; pts[0].y = y0 - miny;
    pts[1].x = x1 - minx; pts[1].y = y1 - miny;
    lv_line_set_points(l, pts, 2);
}

static void needle_to(gauge_t *g, float ang)
{
    float ox, oy;
    arc_point(ang, SCALE_R, &ox, &oy);
    float dx = ox - g->pivot_x, dy = oy;
    float n = sqrtf(dx * dx + dy * dy);
    float len = n - 10;
    float cx = LV_HOR_RES / 2.0f, cy = LV_VER_RES / 2.0f;
    lv_coord_t x0 = (lv_coord_t)lroundf(cx + g->pivot_x * K);
    lv_coord_t y0 = (lv_coord_t)lroundf(cy);
    lv_coord_t x1 = (lv_coord_t)lroundf(cx + (g->pivot_x + dx / n * len) * K);
    lv_coord_t y1 = (lv_coord_t)lroundf(cy + (dy / n * len) * K);
    set_line(g->glow, g->glow_pts, x0, y0, x1, y1);
    set_line(g->needle, g->pts, x0, y0, x1, y1);
}

static lv_obj_t *make_hub(lv_obj_t *parent, float x, lv_obj_t **value_out)
{
    lv_obj_t *h = lv_obj_create(parent);
    lv_obj_remove_style_all(h);
    lv_obj_set_size(h, UI_SCALE(HUB_D), UI_SCALE(HUB_D));
    place(h, x, 0);
    lv_obj_set_style_radius(h, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(h, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(h, lv_color_hex(0x202123), 0);
    lv_obj_set_style_bg_grad_color(h, lv_color_hex(0x0A0A0B), 0);
    lv_obj_set_style_bg_grad_dir(h, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_border_width(h, UI_SCALE(3), 0);
    lv_obj_set_style_border_color(h, lv_color_hex(0x3C3E41), 0);
    lv_obj_clear_flag(h, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    *value_out = make_label(h, &ui_font_rhub_32, UI_COLOR_VALUE, "--");
    lv_obj_center(*value_out);
    return h;
}

static void set_color(lv_obj_t *label, uint32_t color)
{
    lv_color_t c = lv_color_hex(color);
    if (lv_obj_get_style_text_color(label, 0).full != c.full) lv_obj_set_style_text_color(label, c, 0);
}

static void set_text(lv_obj_t *label, const char *txt)
{
    if (strcmp(lv_label_get_text(label), txt) != 0) lv_label_set_text(label, txt);
}

// Light smoothing so the needles stay responsive but don't buzz on noise
static void step_gauge(gauge_t *g, float tau, float (*angle_of)(float))
{
    if (!g->seeded) return;
    float dt = UI_NEEDLE_TICK_MS / 1000.0f;
    g->shown += (g->target - g->shown) * (dt / (tau + dt));
    float a = angle_of(g->shown);
    if (fabsf(a - g->drawn_angle) >= 0.25f) { needle_to(g, a); g->drawn_angle = a; }
}

static void needle_timer_cb(lv_timer_t *t)
{
    (void)t;
    step_gauge(&boost_g, UI_BOOST_SMOOTH_S, left_angle);
    step_gauge(&afr_g, UI_AFR_SMOOTH_S, right_angle);
}

// ---------------------------------------------------------------- screen

void ui_Screen1_screen_init(void)
{
    ui_Screen1 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen1, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(ui_Screen1, lv_color_hex(0x0B0B0B), 0);
    lv_obj_set_style_bg_opa(ui_Screen1, LV_OPA_COVER, 0);

    lv_obj_t *dial = build_static_dial(ui_Screen1);
    lv_obj_update_layout(ui_Screen1);
#if LV_USE_SNAPSHOT && UI_DIAL_USE_SNAPSHOT
    dial_snapshot = lv_snapshot_take(dial, LV_IMG_CF_TRUE_COLOR);
    if (dial_snapshot) {
        lv_obj_del(dial);
        lv_obj_t *img = lv_img_create(ui_Screen1);
        lv_img_set_src(img, dial_snapshot);
        lv_obj_center(img);
        lv_obj_clear_flag(img, LV_OBJ_FLAG_CLICKABLE);
    }
#endif

    lap_value = make_label(ui_Screen1, &ui_font_rvalue_62, UI_COLOR_VALUE, "--:--.--");
    place(lap_value, 0, -170);
    best_value = make_label(ui_Screen1, &ui_font_rvalue_62, UI_COLOR_VALUE, "--:--.--");
    place(best_value, 0, 170);

    boost_g.glow = make_line(ui_Screen1, UI_SCALE(14), LV_OPA_30);
    boost_g.needle = make_line(ui_Screen1, UI_SCALE(6), LV_OPA_COVER);
    afr_g.glow = make_line(ui_Screen1, UI_SCALE(14), LV_OPA_30);
    afr_g.needle = make_line(ui_Screen1, UI_SCALE(6), LV_OPA_COVER);
    make_hub(ui_Screen1, -PIVOT_X, &boost_hub_value);
    make_hub(ui_Screen1, PIVOT_X, &afr_hub_value);

    boost_g.seeded = afr_g.seeded = false;
    boost_g.drawn_angle = afr_g.drawn_angle = -1000;
    needle_to(&boost_g, left_angle(boost_pos(0)));
    needle_to(&afr_g, right_angle(0));
    needle_timer = lv_timer_create(needle_timer_cb, UI_NEEDLE_TICK_MS, NULL);

    last_boost_tenths = last_afr_tenths = -100000;
    last_lap = last_best = 0xFFFFFFFF;
}

void ui_Screen1_screen_destroy(void)
{
    if (needle_timer) lv_timer_del(needle_timer);
    needle_timer = NULL;
    if (ui_Screen1) lv_obj_del(ui_Screen1);
#if LV_USE_SNAPSHOT
    if (dial_snapshot) lv_snapshot_free(dial_snapshot);
#endif
    dial_snapshot = NULL;
    ui_Screen1 = NULL;
    boost_g.needle = boost_g.glow = afr_g.needle = afr_g.glow = NULL;
    boost_hub_value = afr_hub_value = lap_value = best_value = NULL;
}

// ---------------------------------------------------------------- setters

static void seed(gauge_t *g, float pos)
{
    g->target = pos;
    if (!g->seeded) { g->shown = pos; g->seeded = true; }
}

static void fmt_tenths(char *buf, size_t n, int tenths)
{
    lv_snprintf(buf, n, "%s%d.%d", tenths < 0 ? "-" : "", LV_ABS(tenths) / 10, LV_ABS(tenths) % 10);
}

void ui_right_set_boost(float psi)
{
    if (!boost_g.needle) return;
    g_last_boost = psi;
    seed(&boost_g, boost_pos(psi));

    int tenths = (int)lroundf(psi * 10);
    if (tenths == last_boost_tenths) return;
    last_boost_tenths = tenths;
    char buf[12];
    fmt_tenths(buf, sizeof(buf), tenths);
    set_text(boost_hub_value, buf);
    set_color(boost_hub_value, psi < 0 ? UI_COLOR_COLD : psi >= UI_BOOST_WARN_PSI ? UI_COLOR_HOT : UI_COLOR_VALUE);
}

void ui_right_set_afr(float afr)
{
    if (!afr_g.needle) return;
    seed(&afr_g, afr_pos(afr));

    bool warn = g_last_boost >= UI_AFR_CHECK_PSI && (afr < UI_AFR_RICH_WARN || afr > UI_AFR_LEAN_WARN);
    set_color(afr_hub_value, warn ? UI_COLOR_HOT : UI_COLOR_VALUE);
    int tenths = (int)lroundf(afr * 10);
    if (tenths == last_afr_tenths) return;
    last_afr_tenths = tenths;
    char buf[12];
    fmt_tenths(buf, sizeof(buf), tenths);
    set_text(afr_hub_value, buf);
}

static void fmt_lap(char *buf, size_t n, uint32_t ms)
{
    if (ms == 0) lv_snprintf(buf, n, "--:--.--");
    else lv_snprintf(buf, n, "%lu:%02lu.%02lu", (unsigned long)(ms / 60000),
                     (unsigned long)((ms % 60000) / 1000), (unsigned long)((ms % 1000) / 10));
}

void ui_right_set_lap_ms(uint32_t ms)
{
    if (!lap_value || ms == last_lap) return;
    last_lap = ms;
    char buf[16];
    fmt_lap(buf, sizeof(buf), ms);
    set_text(lap_value, buf);
}

void ui_right_set_best_lap_ms(uint32_t ms)
{
    if (!best_value || ms == last_best) return;
    last_best = ms;
    char buf[16];
    fmt_lap(buf, sizeof(buf), ms);
    set_text(best_value, buf);
}

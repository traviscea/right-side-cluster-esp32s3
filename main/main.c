#include <stdio.h>
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
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>
#include <stdint.h>
#include <math.h>


#define UART_PORT      UART_NUM_1
#define UART_RX_PIN    44
#define UART_BAUD      2000000

#define SOF_BYTE       0xA5
#define PKT_LEN        26

#define ENABLE_LOGS    false

#define HIGH_BOOST_WARNING   22.5
#define PSI_START_AFR_CHECK  5.0
#define AFR_RICH_WARN        10.5
#define AFR_LEAN_WARN        11.9
static const char *TAG = "RX";

static lv_color_t green_color;
static lv_color_t red_color;
static lv_color_t blue_color;
static lv_color_t orange_color;
static lv_color_t purple_color;
static lv_color_t pink_color;

typedef struct __attribute__((packed)) {
    uint16_t oil_temp;      
    uint16_t water_temp;    
    uint16_t oil_pressure;  
    uint16_t fuel_pressure; 
    uint16_t fuel_level;   
    uint16_t afr;         
    int16_t boost;          
    uint32_t lap_time_ms;    
    int32_t  lap_delta_ms; 
} gauge_payload_t;

static volatile float g_afr = 14.7f;
static volatile float g_boost = -0.0f;
static volatile uint32_t g_lap_timer = 0;


static void format_lap_time(uint64_t ms, char *buf){
    if (ms > 0) {
        uint32_t min = ms / 60000;
        uint32_t sec = (ms % 60000) / 1000;
        uint32_t hundredths = (ms % 1000) / 10;;

        snprintf(buf, 16, "%02lu:%02lu.%02lu", min, sec, hundredths);
    } else {
        snprintf(buf, 16, "--:--:--");
    }

}

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len){
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

static void uart_init(void){
    uart_config_t cfg = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));

    ESP_ERROR_CHECK(uart_set_pin(
        UART_PORT,
        UART_PIN_NO_CHANGE,
        UART_RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));

    ESP_ERROR_CHECK(uart_driver_install(
        UART_PORT,
        4096,
        0,
        0,
        NULL,
        0
    ));
}

static void uart_rx_task(void *arg){
    uint8_t buf[PKT_LEN];
    int idx = 0;

    while (1) {
        uint8_t byte;
        if (uart_read_bytes(UART_PORT, &byte, 1, portMAX_DELAY) != 1)
            continue;

        // sync on SOF
        if (idx == 0 && byte != SOF_BYTE)
            continue;

        buf[idx++] = byte;

        if (idx == PKT_LEN) {
            idx = 0;

            uint16_t rx_crc = buf[PKT_LEN - 2] | (buf[PKT_LEN - 1] << 8);

            uint16_t calc_crc = crc16_ccitt(&buf[1], PKT_LEN - 3);

            if (rx_crc != calc_crc) {
                ESP_LOGW(TAG, "CRC fail");
                continue;
            }

            gauge_payload_t p;
            memcpy(&p, &buf[2], sizeof(p));

            float afr = p.afr * 0.1f;
            float boost = p.boost * 0.1f;


            g_afr = afr;
            g_boost = boost;
            g_lap_timer = p.lap_time_ms;
            
            #if ENABLE_LOGS
            ESP_LOGI(TAG,
                "AFR %.2f Boost %.1fPSI",
                p.afr / 10.0f,
                p.boost / 10.0f
            );
            #endif
        }
    }
}


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

    blue_color = lv_palette_main(LV_PALETTE_CYAN);
    green_color = lv_color_hex(0x28FF00);
    red_color = lv_palette_main(LV_PALETTE_PINK);
    orange_color = lv_palette_main(LV_PALETTE_DEEP_ORANGE);
    purple_color = lv_palette_main(LV_PALETTE_PURPLE);
    pink_color = lv_palette_main(LV_PALETTE_PINK);
}

static void update_afr_label(float new_value, lv_color_t new_color){
    char buf[12];
    snprintf(buf, sizeof(buf), "%4.1f", new_value);

    // Only update text if changed
    const char *old_text = lv_label_get_text(ui_label_afr_value);
    if (strcmp(old_text, buf) != 0) {
        lv_label_set_text(ui_label_afr_value, buf);
    }

    // Only update color if changed
    lv_color_t old_color = lv_obj_get_style_text_color(ui_label_afr_value, LV_PART_MAIN | LV_STATE_DEFAULT);
    if (old_color.full != new_color.full) {
        lv_obj_set_style_text_color(ui_label_afr_value, new_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    }

}

static void update_boost_label(float boost_val, lv_color_t color){
    char buf[16];

    // Always show magnitude only
    float abs_boost = fabsf(boost_val);
    snprintf(buf, sizeof(buf), "%4.1f", abs_boost);

    const char *old_text = lv_label_get_text(ui_label_boost_value);
    if (strcmp(old_text, buf) != 0) {
        lv_label_set_text(ui_label_boost_value, buf);
    }

    // Update color if needed
    lv_color_t old_color = lv_obj_get_style_text_color(ui_label_boost_value, LV_PART_MAIN | LV_STATE_DEFAULT);

    if (old_color.full != color.full) {
        lv_obj_set_style_text_color(ui_label_boost_value, color,
                                    LV_PART_MAIN | LV_STATE_DEFAULT);
    }

    // Toggle negative sign visibility
    if (boost_val < 0.0f) {
        lv_obj_set_style_text_opa(ui_boost_sign, LV_OPA_COVER, 0);
    } else {
        lv_obj_set_style_text_opa(ui_boost_sign, LV_OPA_TRANSP, 0);
    }
}

static void update_lap_timer(){
    char lap_buf[16];
    format_lap_time(g_lap_timer, lap_buf);

    const char *old_text = lv_label_get_text(ui_lap_timer_value);
    if (strcmp(old_text, lap_buf) != 0) {
        lv_label_set_text(ui_lap_timer_value, lap_buf);
    }
}

void update_gauge_values(){
    lv_color_t boost_color = green_color;
    lv_color_t afr_color = green_color;
    if (g_boost >= PSI_START_AFR_CHECK && (g_afr < AFR_RICH_WARN  || g_afr > AFR_LEAN_WARN)){
        afr_color = red_color;
    }
    if(g_boost >= HIGH_BOOST_WARNING){
        boost_color = red_color;
    }
    update_afr_label(g_afr, afr_color);
    update_boost_label(g_boost, boost_color);
    update_lap_timer();

}

void gauge_timer(lv_timer_t * t){
    update_gauge_values();
}


void app_main(void){   
    I2C_Init();
    EXIO_Init();
    LCD_Init();
    Touch_Init();
    LVGL_Init();

    init_label_styles();

    ui_init();

    uart_init();

    xTaskCreate(
        uart_rx_task,
        "uart_rx",
        4096,
        NULL,
        10,
        NULL
    );

    lv_timer_create(gauge_timer, 50, NULL);

    
    Set_Backlight(0); 
    vTaskDelay(pdMS_TO_TICKS(750)); 
    Set_Backlight(100);
}
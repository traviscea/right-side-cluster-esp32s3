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
#include "esp_timer.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>
#include <stdint.h>
#include <math.h>


#define UART_PORT      UART_NUM_1
#define UART_RX_PIN    44
#define UART_BAUD      2000000

#define SOF_BYTE       0xA5
#define PKT_LEN        27   // SOF + seq + 23-byte payload + CRC16

#define ENABLE_LOGS    false

// Boost / AFR warning thresholds and colors now live in ui/screens/ui_Screen1.h
static const char *TAG = "RX";

typedef struct __attribute__((packed)) {
    uint16_t oil_temp;      
    uint16_t water_temp;    
    uint16_t oil_pressure;  
    uint16_t fuel_pressure; 
    uint16_t fuel_level;   
    uint16_t afr;         
    int16_t boost;          
    uint32_t lap_time_ms;    
    uint32_t best_lap_ms;    // session best lap (0 = none yet)
    uint8_t  theme;          // face color theme from the center cluster
} gauge_payload_t;

static volatile uint8_t g_theme = 0;        // face color theme, from the center cluster
static volatile float g_afr = 14.7f;
static volatile float g_boost = -0.0f;
static volatile uint32_t g_lap_timer = 0;
static volatile uint32_t g_best_lap = 0;


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

            g_theme = p.theme;

            float afr = p.afr * 0.1f;
            float boost = p.boost * 0.1f;


            g_afr = afr;
            g_boost = boost;
            g_lap_timer = p.lap_time_ms;
            g_best_lap = p.best_lap_ms;
            
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


// Push the latest UART values into the gauge. Colors and warnings are
// handled by the screen (see ui/screens/ui_Screen1.h).
//---------------------------- FACE COLOR THEME ----------------------------//
// The center cluster sends the theme number in every packet (it owns the
// button). It's remembered here too, so the gauge starts up in the right color
// before the first packet arrives.

static uint8_t load_theme(void)
{
    uint8_t t = 0;
    nvs_handle_t h;
    if (nvs_open("ui", NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u8(h, "theme", &t);
        nvs_close(h);
    }
    return t < UI_THEME_COUNT ? t : 0;
}

static void save_theme(uint8_t t)
{
    nvs_handle_t h;
    if (nvs_open("ui", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "theme", t);
        nvs_commit(h);
        nvs_close(h);
    }
}

// Called from gauge_timer (LVGL thread). Applies a new theme right away and
// saves it once it has stayed the same for 3 s.
static void theme_update(void)
{
    static uint8_t saved = 0xFF;
    static int64_t changed_ms = 0;
    uint8_t t = g_theme;
    if (t >= UI_THEME_COUNT) return;
    if (saved == 0xFF) saved = ui_get_theme();      // first call: what we booted with

    int64_t now_ms = esp_timer_get_time() / 1000;
    if (t != ui_get_theme()) {
        ui_set_theme(t);
        changed_ms = now_ms;
    }
    if (t != saved && now_ms - changed_ms > 3000) {
        save_theme(t);
        saved = t;
    }
}
//--------------------------------------------------------------------------//

void gauge_timer(lv_timer_t * t){
    theme_update();
    ui_right_set_boost(g_boost);
    ui_right_set_afr(g_afr);
    ui_right_set_lap_ms(g_lap_timer);
    ui_right_set_best_lap_ms(g_best_lap);
}


// Short, screen-friendly name for a reset reason
static const char *reset_reason_name(esp_reset_reason_t r)
{
    switch (r) {
        case ESP_RST_POWERON:   return "POWER ON";
        case ESP_RST_EXT:       return "EXT RESET";
        case ESP_RST_SW:        return "SW RESET";
        case ESP_RST_PANIC:     return "CRASH";
        case ESP_RST_INT_WDT:   return "INT WDT";
        case ESP_RST_TASK_WDT:  return "TASK WDT";
        case ESP_RST_WDT:       return "WDT";
        case ESP_RST_DEEPSLEEP: return "DEEP SLEEP";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "UNKNOWN";
    }
}

// Keeps a running count of unexpected restarts (anything but a normal power
// on) in NVS. Returns the count including this boot.
static uint32_t count_unexpected_restart(esp_reset_reason_t r)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) return 0;

    uint32_t count = 0;
    nvs_handle_t h;
    if (nvs_open("diag", NVS_READWRITE, &h) != ESP_OK) return 0;
    nvs_get_u32(h, "unexp_rst", &count);          // stays 0 if not stored yet
    if (r != ESP_RST_POWERON) {
        count++;
        nvs_set_u32(h, "unexp_rst", count);
        nvs_commit(h);
    }
    nvs_close(h);
    return count;
}

void app_main(void){   
    // Why the board (re)started: POWERON is normal, CRASH = code panic,
    // TASK/INT WDT = something hung, BROWNOUT = supply voltage dipped.
    esp_reset_reason_t rr = esp_reset_reason();
    ESP_LOGW("BOOT", "Reset reason: %s (%d)", reset_reason_name(rr), (int)rr);
    uint32_t unexpected_restarts = count_unexpected_restart(rr);   // also starts NVS
    g_theme = load_theme();
    ui_set_theme(g_theme);               // before ui_init, so the first build uses it

    I2C_Init();
    EXIO_Init();
    LCD_Init();
    Touch_Init();
    LVGL_Init();

    // LVGL isn't thread-safe: hold the lock while building the UI so the
    // LVGL task can't render at the same time (that crashes at boot).
    LVGL_Lock(-1);
    ui_init();
    lv_timer_create(gauge_timer, 50, NULL);
    // After an unexpected restart, show what it was at the bottom of the
    // screen for 15 s, with how many unexpected restarts there have been.
    if (rr != ESP_RST_POWERON) {
        char msg[40];
        snprintf(msg, sizeof(msg), "RESET: %s #%lu", reset_reason_name(rr), (unsigned long)unexpected_restarts);
        ui_show_status(msg, 15000);
    }
    LVGL_Unlock();

    uart_init();

    xTaskCreate(
        uart_rx_task,
        "uart_rx",
        4096,
        NULL,
        10,
        NULL
    );


    Set_Backlight(0); 
    vTaskDelay(pdMS_TO_TICKS(750)); 
    Set_Backlight(100);
}
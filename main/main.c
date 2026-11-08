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
} gauge_payload_t;

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
void gauge_timer(lv_timer_t * t){
    ui_right_set_boost(g_boost);
    ui_right_set_afr(g_afr);
    ui_right_set_lap_ms(g_lap_timer);
    ui_right_set_best_lap_ms(g_best_lap);
}


void app_main(void){   
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
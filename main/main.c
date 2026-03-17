#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_task_wdt.h"

#include "driver/gpio.h"
#include "esp_sleep.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_sntp.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "time.h"
#include "math.h"

#include "wifi_manager.h"
#include "webserver.h"
#include "esp_spiffs.h"

// ─────────────────────────────────────────────
// Pin / channel definitions
// ─────────────────────────────────────────────

#define OUT_POWER        GPIO_NUM_19
#define IN_ZERO_CROSSING GPIO_NUM_18
#define IN_BTN           GPIO_NUM_21
#define NTC_PIN          ADC_CHANNEL_6

// ─────────────────────────────────────────────
// Timing
// ─────────────────────────────────────────────

#define TIME_CONTROL_INTERVAL 250   // ms — PID loop period
#define TIME_ADC_INTERVAL      50   // ms — temperature sampling period
#define MAX_TIMER_BTN_ON   300000   // ms — max allowed for the button

// ─────────────────────────────────────────────
// NTC / temperature
// ─────────────────────────────────────────────

#define BETA            3950.0f
#define REF_TEMP        298.15f     // 25 °C in Kelvin
#define REF_RESISTANCE  10000.0f   // NTC resistance at REF_TEMP
#define SERIE_RESISTOR  10000.0f   // fixed resistor in voltage divider
#define FACTORADC       0.05f      // low-pass filter coefficient

// PID gains — initial values only.
// At runtime read from g_ws_config (protected by xTempMutex).
// Defaults mirror g_ws_config initialisation in webserver.c.

// ─────────────────────────────────────────────
// Setpoints
// ─────────────────────────────────────────────

// Setpoints — initial values only, runtime values live in g_ws_config.

// ─────────────────────────────────────────────
// Shared state (protected by xTempMutex)
// ─────────────────────────────────────────────

float setpoint  = 75.0f;
float waterTemp = 25.0f;
SemaphoreHandle_t xTempMutex = NULL;

// ─────────────────────────────────────────────
// Zero-crossing / TRIAC (volatile — written in ISR)
// ─────────────────────────────────────────────

volatile uint32_t crossedTime     = 0;
volatile uint32_t lastCrossedTime = 0;
volatile int16_t  PID             = 0;
volatile bool     isCrossing      = false;

// ─────────────────────────────────────────────
// Button / control state
// ─────────────────────────────────────────────

uint32_t timerBtn    = 0;
uint16_t lastTimeBtn = 0;
bool     commandBtn  = false;
bool     heatError   = false;

// ─────────────────────────────────────────────
// ADC handles
// ─────────────────────────────────────────────

adc_oneshot_unit_handle_t adc_handle;
adc_cali_handle_t         adc_cali_handle = NULL;
bool                      adc_calibrated  = false;

// ─────────────────────────────────────────────
// Task / timer handles
// ─────────────────────────────────────────────

static TaskHandle_t      powerTaskHandle = NULL;
static esp_timer_handle_t triac_timer;

// ─────────────────────────────────────────────
// Log tags
// ─────────────────────────────────────────────

static const char *TAG_TEMP = "temperature";
static const char *TAG_CTRL = "control";
static const char *TAG_PW   = "out_power";
static const char *TAG_BTN  = "button";
static const char *TAG_TEL  = "telemetry";

// ─────────────────────────────────────────────
// ISR — zero crossing
// ─────────────────────────────────────────────

static void IRAM_ATTR zero_cross_isr(void *arg)
{
    isCrossing        = true;
    lastCrossedTime   = crossedTime;
    crossedTime       = esp_timer_get_time() + 125; // Midle of opto windown

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(powerTaskHandle, 0, eNoAction, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

// ─────────────────────────────────────────────
// TRIAC timer callback
// ─────────────────────────────────────────────

static void triac_fire_cb(void *arg)
{
    if (heatError || PID <= 0) return;

    gpio_set_level(OUT_POWER, 1);
    esp_rom_delay_us(200);
    gpio_set_level(OUT_POWER, 0);
}

// Called by power_control_task once powerTaskHandle is assigned.
static void gpio_isr_enable(void)
{
    gpio_intr_enable(IN_ZERO_CROSSING);
}

// ─────────────────────────────────────────────
// ADC read (returns millivolts when calibrated)
// ─────────────────────────────────────────────

static int analogRead_adc1(void)
{
    int raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, NTC_PIN, &raw));

    if (adc_calibrated && adc_cali_handle != NULL) {
        int voltage_mv = 0;
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, raw, &voltage_mv));
        return voltage_mv;
    }

    return raw;
}

// ─────────────────────────────────────────────
// Task: power_control
// Waits for ISR notification, calculates phase-cut delay, schedules TRIAC fire.
// ─────────────────────────────────────────────

void power_control_task(void *arg)
{
    powerTaskHandle = xTaskGetCurrentTaskHandle();
    esp_task_wdt_add(NULL);

    // Safe to enable the ISR now — powerTaskHandle is valid
    gpio_isr_enable();

    uint32_t cycleTime  = 8333;
    int64_t  timeToWait = 0;

    while (!heatError) {
        esp_task_wdt_reset();

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        gpio_set_level(OUT_POWER, 0); // ensure output is off at zero crossing

        cycleTime = crossedTime - lastCrossedTime;
        if (cycleTime < 8000 || cycleTime > 10500) cycleTime = 8333; // fallback 60 Hz

        if (PID <= 0) continue;

        timeToWait = (int64_t)crossedTime
                   + (int64_t)(cycleTime * (255 - PID) / 255);

        int64_t delay_us = timeToWait - esp_timer_get_time();

        // Guard: if the firing moment already passed, skip this cycle
        // instead of scheduling a timer for billions of microseconds
        if (delay_us <= 0 || delay_us > 8330) continue;

        esp_timer_stop(triac_timer);
        esp_timer_start_once(triac_timer, (uint64_t)delay_us);
    }

    gpio_set_level(OUT_POWER, 0);
    ESP_LOGW(TAG_PW, "Power task exited — output off");
    vTaskDelete(NULL);
}

// ─────────────────────────────────────────────
// Task: logic_control
// Button debounce + setpoint selection + PID calculation.
// ─────────────────────────────────────────────

void logic_control_task(void *arg)
{
    esp_task_wdt_add(NULL);

    static bool isBoiling = false;
    static float oldEr = 0.0f;
    static float i     = 0.0f;
    float er = 0.0f, p = 0.0f, d = 0.0f;

    while (1) {
        esp_task_wdt_reset();

        // --- Read all shared state under mutex ---
        float localTemp          = 25.0f;
        float localSetpoint      = 80.0f;
        float localCoffeeSp      = 75.0f;
        float localBoilingSp     = 100.0f;
        float kp = 12.0f, ki = 0.0004f, kd = 40.0f;

        if (xSemaphoreTake(xTempMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            localTemp        = waterTemp;
            localSetpoint    = setpoint;
            localCoffeeSp    = g_ws_config.coffee_setpoint;
            localBoilingSp   = g_ws_config.boiling_setpoint;
            kp               = g_ws_config.kp;
            ki               = g_ws_config.ki;
            kd               = g_ws_config.kd;
            xSemaphoreGive(xTempMutex);
        }

        // --- Button debounce ---
        if (timerBtn < MAX_TIMER_BTN_ON) {
            if (!gpio_get_level(IN_BTN)) {
                vTaskDelay(pdMS_TO_TICKS(1));
                commandBtn = !gpio_get_level(IN_BTN);
            } else {
                commandBtn = false;
                i = 0.0f;
            }

            // Short press → boiling setpoint
            if (lastTimeBtn > 10 && lastTimeBtn < 100 && commandBtn) {
                isBoiling = true;
                lastTimeBtn   = 0;
            }

            if (!commandBtn) isBoiling = false;

            if (isBoiling) {
                localSetpoint = localBoilingSp;
            } else {
                localSetpoint = localCoffeeSp;
            }
        

        } else {
            heatError = true;
            i   = 0.0f;
            PID = 0;
            ESP_LOGW(TAG_BTN, "Overtime — shutting down");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        // --- PID ---
        if (!commandBtn) {
            PID = 0;
            if (xSemaphoreTake(xTempMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                setpoint = localCoffeeSp;
                xSemaphoreGive(xTempMutex);
            }
            vTaskDelay(pdMS_TO_TICKS(TIME_CONTROL_INTERVAL * 2));
            continue;
        }

        er = localSetpoint - localTemp;

        p = er * kp;
        if (p >  255.0f) p =  255.0f;
        else if (p < -255.0f) p = -255.0f;

        i += er * ki;
        if (i >  255.0f) i =  255.0f;
        else if (i < -255.0f) i = -255.0f;

        d = (er - oldEr) * kd;
        if (d >  255.0f) d =  255.0f;
        else if (d < -255.0f) d = -255.0f;

        oldEr = er;

        PID = (int16_t)(p + i + d);
        if      (PID <  25) PID = 0;
        else if (PID > 255) PID = 255;

        // Write setpoint back under mutex so telemetry task sees the active value
        if (xSemaphoreTake(xTempMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            setpoint = localSetpoint;
            xSemaphoreGive(xTempMutex);
        }

        ESP_LOGD(TAG_CTRL, "Temp %.2f  SP %.2f  PID %d", localTemp, localSetpoint, PID);
        vTaskDelay(pdMS_TO_TICKS(TIME_CONTROL_INTERVAL));
    }
}

// ─────────────────────────────────────────────
// Task: temperature
// ADC read → NTC calculation → low-pass filter → shared waterTemp.
// ─────────────────────────────────────────────

void temperature_task(void *arg)
{
    static float temp = 25.0f;
    esp_task_wdt_add(NULL);

    while (1) {
        int   adcVal  = analogRead_adc1();
        float voltage = (float)adcVal / 1000.0f;

        if (voltage <= 0.2f || voltage >= 3.1f) {
            heatError = true;
            if (xSemaphoreTake(xTempMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                waterTemp = 199.0f;
                xSemaphoreGive(xTempMutex);
            }
            ESP_LOGE(TAG_TEMP, "Sensor error — voltage: %.3f V", voltage);
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        float ntcResistance      = (3.3f * SERIE_RESISTOR / voltage) - SERIE_RESISTOR;
        float kTemperature       = 1.0f / (1.0f / REF_TEMP + (1.0f / BETA) * logf(ntcResistance / REF_RESISTANCE));
        float currentTemperature = kTemperature - 273.15f;

        if (temp == 25.0f) temp = currentTemperature; // Skip filter on first read
        else {
            temp = (1.0f - FACTORADC) * temp + FACTORADC * currentTemperature;
        }

        if (xSemaphoreTake(xTempMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            waterTemp = temp;
            xSemaphoreGive(xTempMutex);
        }

        if (temp > 120.0f && !heatError) {
            heatError = true;
            ESP_LOGW(TAG_TEMP, "Overtemp! %.2f °C", temp);
        }

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(TIME_ADC_INTERVAL));
    }
}

// ─────────────────────────────────────────────
// Task: aux
// Button hold timer + configurable auto-hibernate.
// Hibernate timeout is stored in g_ws_config.hibernate_timeout_ms
// and can be updated via the webserver (future).
// ─────────────────────────────────────────────

void aux_task(void *arg)
{
    esp_task_wdt_add(NULL);

    while (1) {
        esp_task_wdt_reset();
        if (commandBtn) {
            timerBtn++;
            // User is active — reset the hibernate countdown
            // (timerBtn is in 10 ms ticks, MAX_TIMER_ON is in ms)
        } else {
            if (timerBtn) {
                lastTimeBtn = timerBtn;
                timerBtn    = 0;
                ESP_LOGI(TAG_BTN, "Button released — held for %d ticks", lastTimeBtn);
            }
        }

        // --- Auto-hibernate after configurable timeout ---
        uint32_t elapsed_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
        uint32_t timeout_ms = g_ws_config.hibernate_timeout_ms;

        if (elapsed_ms >= timeout_ms) {
            ESP_LOGW(TAG_BTN, "Hibernate timeout reached (%lu ms) — shutting down",
                     (unsigned long)timeout_ms);

            // Guarantee output is off before sleeping
            gpio_set_level(OUT_POWER, 0);
            heatError = true;   // stops power_control_task loop

            // Brief delay so webserver can serve one last status update
            vTaskDelay(pdMS_TO_TICKS(1000));

            ESP_LOGI(TAG_BTN, "Entering deep sleep");
            esp_deep_sleep_start();
            // No return
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ─────────────────────────────────────────────
// Task: telemetry
// Samples waterTemp + setpoint + PID every HISTORY_INTERVAL_MS
// and pushes to the webserver history buffer.
// ─────────────────────────────────────────────

static void telemetry_task(void *arg)
{
    ESP_LOGI(TAG_TEL, "Telemetry task started");

    while (1) {
        float   localTemp     = 0.0f;
        float   localSetpoint = 0.0f;
        int16_t localPid      = 0;

        if (xSemaphoreTake(xTempMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            localTemp     = waterTemp;
            localSetpoint = setpoint;
            xSemaphoreGive(xTempMutex);
        }

        localPid = PID;  // volatile int16_t — atomic read on ESP32, no mutex needed

        webserver_history_push(localTemp, localSetpoint, localPid,
                               localPid > 0 && !heatError);

        vTaskDelay(pdMS_TO_TICKS(HISTORY_INTERVAL_MS));
    }
}

static void gpio_setup(void)
{
    gpio_set_direction(OUT_POWER,        GPIO_MODE_OUTPUT);
    gpio_set_direction(IN_ZERO_CROSSING, GPIO_MODE_INPUT);
    gpio_set_direction(IN_BTN,           GPIO_MODE_INPUT);
    gpio_set_pull_mode(IN_ZERO_CROSSING, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(IN_BTN,           GPIO_PULLUP_ONLY);
    gpio_set_level(OUT_POWER, 0);

    // ISR service installed here but interrupt NOT yet enabled.
    // gpio_isr_enable() is called after powerTaskHandle is valid.
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_set_intr_type(IN_ZERO_CROSSING, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(IN_ZERO_CROSSING, zero_cross_isr, NULL);
    gpio_intr_disable(IN_ZERO_CROSSING);  // disabled until task is ready
}

// ─────────────────────────────────────────────
// ADC init
// ─────────────────────────────────────────────

static void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten    = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_6, &chan_cfg));

    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_1,
        .atten    = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_cali_create_scheme_line_fitting(&cali_cfg, &adc_cali_handle) == ESP_OK) {
        adc_calibrated = true;
        ESP_LOGI("ADC", "Calibration enabled");
    } else {
        ESP_LOGW("ADC", "Calibration not available on this chip");
    }
}

// ─────────────────────────────────────────────
// TRIAC timer init
// ─────────────────────────────────────────────

static void init_triac_timer(void)
{
    const esp_timer_create_args_t timer_args = {
        .callback = &triac_fire_cb,
        .name     = "triac_timer",
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &triac_timer));
}

// ─────────────────────────────────────────────
// NTP time sync
// ─────────────────────────────────────────────

static void sync_time(void)
{
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();

    time_t    now      = 0;
    struct tm timeinfo = {0};
    int       retry    = 0;

    while (timeinfo.tm_year < (2016 - 1900) && ++retry < 10) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    if (timeinfo.tm_year >= (2016 - 1900)) {
        ESP_LOGI("NTP", "Time synced: %s", asctime(&timeinfo));
    } else {
        ESP_LOGW("NTP", "NTP sync failed");
    }

    // Brazil (Brasília time) = UTC-3
    setenv("TZ", "BRT+3", 1);
    tzset();
    localtime_r(&now, &timeinfo);
    ESP_LOGI("NTP", "Local time (BRT): %s", asctime(&timeinfo));
}

// ─────────────────────────────────────────────
// app_main
// ─────────────────────────────────────────────

void app_main(void)
{
    // 1. Hardware init — ISR disabled until power_control_task is ready
    gpio_setup();
    adc_init();
    init_triac_timer();

    // 2. Synchronisation primitives
    xTempMutex = xSemaphoreCreateMutex();
    configASSERT(xTempMutex);

    // 3. NVS — before config load and before any task that reads g_ws_config
    ESP_ERROR_CHECK(nvs_flash_init());

    // 4. Load persisted config (setpoints, PID gains, hibernate timeout)
    //    Must happen before tasks start so aux_task reads the correct timeout
    webserver_config_load_nvs();

    // 5. Mount SPIFFS — before webserver_start() inside wifi_manager_init()
    esp_vfs_spiffs_conf_t spiffs_cfg = {
        .base_path              = "/spiffs",
        .partition_label        = NULL,
        .max_files              = 4,
        .format_if_mount_failed = false,
    };
    esp_err_t spiffs_err = esp_vfs_spiffs_register(&spiffs_cfg);
    if (spiffs_err != ESP_OK) {
        ESP_LOGE("SPIFFS", "Mount failed (%s) — webserver will return 500 for static files",
                 esp_err_to_name(spiffs_err));
    } else {
        size_t total = 0, used = 0;
        esp_spiffs_info(NULL, &total, &used);
        ESP_LOGI("SPIFFS", "Mounted — %d KB used / %d KB total",
                 (int)(used / 1024), (int)(total / 1024));
    }

    // 6. Control tasks — heating operational before WiFi/webserver init begins
    //    power_control_task enables the zero-crossing ISR internally once ready
    xTaskCreate(power_control_task, "power_control", 4096, NULL, 5, NULL);
    xTaskCreate(temperature_task,   "temperature",   4096, NULL, 5, NULL);
    xTaskCreate(logic_control_task, "logic_control", 4096, NULL, 3, NULL);
    xTaskCreatePinnedToCore(aux_task, "aux", 4096, NULL, 1, NULL, 1);

    // 7. Mark OTA partition valid here — before any blocking operation (NTP, WiFi)
    //    This ensures the rollback timer is cancelled even if WiFi or NTP hangs
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err != ESP_OK) {
        ESP_LOGE("OTA", "Failed to mark app valid: %s", esp_err_to_name(err));
    }

    // 8. Network infrastructure
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 9. WiFi: STA (from NVS) or AP fallback + mDNS + webserver
    wifi_manager_init();

    // 10. Telemetry — started after webserver so history buffer is ready
    xTaskCreate(telemetry_task, "telemetry", 3072, NULL, 2, NULL);

    // 11. NTP — only in STA mode, non-blocking for the rest of the system
    if (wifi_manager_get_mode() == WIFI_MODE_STA_CONNECTED) {
        sync_time();
    }
}
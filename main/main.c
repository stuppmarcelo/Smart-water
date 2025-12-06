#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_sleep.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h" 

#include "esp_log.h"
#include "esp_event.h"

#include "esp_wifi.h"
#include "esp_netif.h"

#include "esp_https_ota.h"
#include "esp_http_client.h"
#include "esp_ota_ops.h"
#include "esp_system.h"

#include "math.h"


#define OUT_POWER GPIO_NUM_19
#define IN_ZERO_CROSSING GPIO_NUM_18
#define IN_BTN GPIO_NUM_21
#define NTC_PIN ADC_CHANNEL_6

#define TIME_CONTROL_INTERVAL 50
#define TIME_ADC_INTERVAL 50
#define MAX_TIMER_ON 300000

#define BETA 3950.0
#define REF_TEMP 298.15
#define REF_RESISTANCE 10000.0
#define SERIE_RESISTOR 10000.0
#define FACTORADC 0.01

#define KP 20.00
#define KI 0.01
#define KD 100.00

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group;

#define OTA_URL "https://github.com/stuppmarcelo/smart-water/releases/latest/download/firmware.bin"

extern const unsigned char _binary_fullchain_pem_start[] asm("_binary_fullchain_pem_start");
extern const unsigned char _binary_fullchain_pem_end[]   asm("_binary_fullchain_pem_end");

static int s_retry_num = 0;

static const char *TAGWIFI = "wifi";
static const char *TAG_OTA = "OTA";
static const char *TAG_TEMP = "temperature_control";
static const char *TAG_CTRL = "control";
static const char *TAG_BTN = "button_process";


float setpoint = 80.0f;
float waterTemp = 25.0f;

uint32_t timerOperate = 0;
uint32_t timerBtn = 0;
uint16_t lastTimeBtn = 0;

uint16_t PID = 0;

uint8_t ErrorCounter = 0;

bool commandBtn = false;
bool isCrossing = false;
bool heatError = false;

// ADC
adc_oneshot_unit_handle_t adc_handle;
adc_cali_handle_t adc_cali_handle = NULL;
bool adc_calibrated = false;

// WiFi
#define SSID "M&M"
#define PASSWORD "39402100"

// ********* Function prototypes *********
int analogRead_adc1(void);
void logic_control(void);
void power_control(void);
void gpio_setup(void);
void wifi_init_sta(void);
void adc_init(void);

esp_err_t perform_ota_update(void)
{
    ESP_LOGI(TAG_OTA, "Starting OTA update...");

    esp_http_client_config_t config = {
        .url = OTA_URL,
        .timeout_ms = 5000,
        .keep_alive_enable = true,
        .cert_pem = (char *)_binary_fullchain_pem_start,
    };

    esp_https_ota_config_t ota_config = {
        .http_config = &config,
    };

    esp_https_ota_handle_t https_ota_handle = NULL;
    esp_err_t err = esp_https_ota_begin(&ota_config, &https_ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_OTA, "esp_https_ota_begin failed: %s", esp_err_to_name(err));
        return err;
    }

    esp_app_desc_t new_app_info;
    err = esp_https_ota_get_img_desc(https_ota_handle, &new_app_info);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_OTA, "Failed to read new image description: %s", esp_err_to_name(err));
        esp_https_ota_abort(https_ota_handle);
        return err;
    }

    const esp_app_desc_t *running_desc = esp_app_get_description();
    ESP_LOGI(TAG_OTA, "Running version: %s", running_desc->version);
    ESP_LOGI(TAG_OTA, "New image  version: %s", new_app_info.version);

    /* If versions match, abort and return ESP_OK (no update needed) */
    if (memcmp(new_app_info.version, running_desc->version, sizeof(new_app_info.version)) == 0) {
        ESP_LOGI(TAG_OTA, "No new firmware. Versions match.");
        esp_https_ota_abort(https_ota_handle);
        return ESP_OK;
    }

    ESP_LOGI(TAG_OTA, "New firmware available, starting download/flash...");

    /* Perform download/flash loop */
    while ((err = esp_https_ota_perform(https_ota_handle)) == ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
        int bytes_downloaded = esp_https_ota_get_image_len_read(https_ota_handle);
        ESP_LOGI(TAG_OTA, "Downloaded %d bytes...", bytes_downloaded);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG_OTA, "esp_https_ota_perform failed: %s", esp_err_to_name(err));
        esp_https_ota_abort(https_ota_handle);
        return err;
    }

    /* finish and validate the image */
    err = esp_https_ota_finish(https_ota_handle);
    if (err == ESP_OK) {
        ESP_LOGI(TAG_OTA, "OTA finished successfully, rebooting...");
        esp_restart();
    } else {
        ESP_LOGE(TAG_OTA, "esp_https_ota_finish failed: %s", esp_err_to_name(err));
        esp_https_ota_abort(https_ota_handle);
        return err;
    }

    /* should not reach here, but return error if we do */
    return ESP_FAIL;
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 10) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAGWIFI, "Retrying connection...");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGW(TAGWIFI, "Connection failed");
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAGWIFI, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

int analogRead_adc1(void) {
    int raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, NTC_PIN, &raw));

    if (adc_calibrated) {
        int voltage_mv = 0;
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, raw, &voltage_mv));
        return voltage_mv;  // you can decide if you want raw or mV
    }

    return raw; // fallback when no calibration is possible
}

void logic_control(void) {
    static float oldEr = 0.0f;
    float er = setpoint - waterTemp;
    float p = er * KP;
    static float i = 0.0f;
    float d = 0.0f;

    i += p * KI;
    if (i > 255.00) i = 255.00f;
    else if (i < 0.00) i = 0.00f;

    d = (er - oldEr) * KD;

    oldEr = er;

    PID = p + i + d;
    if (PID > 255) PID = 255;

    // Button logic
    if (timerBtn < MAX_TIMER_ON) {
        if (!gpio_get_level(IN_BTN)) {
            vTaskDelay(pdMS_TO_TICKS(1));
            commandBtn = !gpio_get_level(IN_BTN);
        } else {
            commandBtn = false;
        }

        if (lastTimeBtn > 100 && lastTimeBtn < 1000 && commandBtn) {
            setpoint = 120.0f;
            lastTimeBtn = 0;
        }

        if (!commandBtn && lastTimeBtn == 0) setpoint = 80.0f;

    } else {
        heatError = true;
        ESP_LOGW(TAG_BTN, "Overtime On!");
    }

    ESP_LOGI(TAG_CTRL, "Temp %.2f  PID %d", waterTemp, PID);
}

void power_control(void) {
    if (commandBtn && !heatError) {
        vTaskDelay(pdMS_TO_TICKS(1));
        if (commandBtn && PID > 127)
            gpio_set_level(OUT_POWER, 1);
        else
            gpio_set_level(OUT_POWER, 0);
    } else {
        gpio_set_level(OUT_POWER, 0);
    }
}

void control_task(void *arg) {

    while (1) {
        logic_control();
        power_control();
        
        vTaskDelay(pdMS_TO_TICKS(TIME_CONTROL_INTERVAL));
    }
}

void temperature_task(void *arg) {
    static float temp = 25.00f;

    while(1) {

        int adcVal = analogRead_adc1();
        float voltage = (float)adcVal / 1000.0f;

        if (voltage <= 0.2 || voltage >= 3.1) {
            heatError = true;
            waterTemp = 199.00f;
            ESP_LOGE(TAG_TEMP, "Temp Sensor Error! Voltage:%f", voltage);
            continue;
        }

        float ntcResistance = (3.3f * SERIE_RESISTOR / voltage) - SERIE_RESISTOR;
        float kTemperature = 1.0 / (1.0 / REF_TEMP + (1.0 / BETA) * log(ntcResistance / REF_RESISTANCE));
        float currentTemperature = kTemperature - 273.15f;

        temp = (1.0 - FACTORADC) * temp + FACTORADC * currentTemperature;

        waterTemp = temp;

        if (waterTemp > 120.0f && !heatError) {
            heatError = true;
            ESP_LOGW(TAG_TEMP, "Overtemp! %f", waterTemp);
        }
        vTaskDelay(pdMS_TO_TICKS(TIME_ADC_INTERVAL));
    }

}

void aux_task(void *arg) {

    while(1) {
       if (commandBtn) {
            timerBtn++;         // Not precise timer but enghout for it
        }
        else {
            if (timerBtn) {
                lastTimeBtn = timerBtn;
                timerBtn = 0;
            }
        }
        

        if ((esp_timer_get_time() / 1000) > MAX_TIMER_ON) {
            adc_cali_delete_scheme_line_fitting(adc_cali_handle);
            gpio_set_level(OUT_POWER, 0);
            perform_ota_update();
            esp_deep_sleep_start();         // Falls on deep sleep with no return 
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void gpio_setup(void) {
    gpio_set_direction(OUT_POWER, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN_ZERO_CROSSING, GPIO_MODE_INPUT);
    gpio_set_direction(IN_BTN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(IN_ZERO_CROSSING, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(IN_BTN, GPIO_PULLUP_ONLY);
    gpio_set_level(OUT_POWER, 0);
}
 
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_init();                   // Initialize TCP/IP
    esp_event_loop_create_default();    // Event loop
    esp_netif_create_default_wifi_sta();// Create WiFi interface

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    // Register handlers
    esp_event_handler_instance_register(WIFI_EVENT,
                                        ESP_EVENT_ANY_ID,
                                        &wifi_event_handler,
                                        NULL,
                                        NULL);
    esp_event_handler_instance_register(IP_EVENT,
                                        IP_EVENT_STA_GOT_IP,
                                        &wifi_event_handler,
                                        NULL,
                                        NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = SSID,
            .password = PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK
        }
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAGWIFI, "WiFi init finished.");

    // Wait until connected or failed
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAGWIFI, "Connected to AP: %s", wifi_config.sta.ssid);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGW(TAGWIFI, "Failed to connect to SSID: %s", wifi_config.sta.ssid);
    } else {
        ESP_LOGE(TAGWIFI, "Unexpected event");
    }
}

void adc_init(void)
{
    // ------------ Create ADC unit ------------
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    // ------------ Configure channel ------------
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_6, &config));
        // GPIO34 = ADC1_CH6

    // ------------ Try to enable calibration ------------
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12,
    };

    if (adc_cali_create_scheme_line_fitting(&cali_config, &adc_cali_handle) == ESP_OK) {
        adc_calibrated = true;
        ESP_LOGI("ADC", "Calibration enabled");
    } else {
        ESP_LOGW("ADC", "Calibration not supported on this device/regulator");
    }
}

void app_main(void) {

    // GPIO Setup
    gpio_setup();

    // ADC Setup
    adc_init();

    // Start loop task
    xTaskCreate(control_task, "control", 4096, NULL, 2, NULL);
    xTaskCreate(temperature_task, "temperature", 4096, NULL, 1, NULL);
    xTaskCreate(aux_task, "auxiliar", 4096, NULL, 5, NULL);

    wifi_init_sta();

    ESP_LOGI(TAGWIFI, "Wifi Ready!");
}

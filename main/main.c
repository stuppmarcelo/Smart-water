#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_task_wdt.h"

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
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_sntp.h"
#include "time.h"

#include "esp_https_ota.h"
#include "esp_http_client.h"
#include "esp_ota_ops.h"
#include "esp_system.h"

#include "math.h"

#include "esp_crt_bundle.h"

#define OUT_POWER GPIO_NUM_19
#define IN_ZERO_CROSSING GPIO_NUM_18
#define IN_BTN GPIO_NUM_21
#define NTC_PIN ADC_CHANNEL_6

#define TIME_CONTROL_INTERVAL 250
#define TIME_ADC_INTERVAL 50
#define MAX_TIMER_ON 300000

#define BETA 3950.0f
#define REF_TEMP 298.15f
#define REF_RESISTANCE 10000.0f
#define SERIE_RESISTOR 10000.0f
#define FACTORADC 0.01f

#define KP 12.00f
#define KI 0.0004f
#define KD 40.00f

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group;

#define OTA_URL "http://192.168.68.52:8000/Smart-water.bin"    



static int s_retry_num = 0;

static const char *TAGWIFI = "wifi";
static const char *TAG_OTA = "OTA";
static const char *TAG_TEMP = "temperature_control";
static const char *TAG_CTRL = "control";
static const char *TAG_PW = "out_power";
static const char *TAG_BTN = "button_process";

const float coffeeSetpoint = 75.0f;
const float boilingSetpoint = 120.0f;
float setpoint = 80.0f;
float waterTemp = 25.0f;

volatile uint32_t crossedTime = 0;
volatile uint32_t lastCrossedTime = 0;
uint32_t timerOperate = 0;
uint32_t timerBtn = 0;
uint16_t lastTimeBtn = 0;

volatile int16_t PID = 0;

uint8_t ErrorCounter = 0;

volatile bool isCrossing = false;
bool commandBtn = false;
bool heatError = false;

// ADC
adc_oneshot_unit_handle_t adc_handle;
adc_cali_handle_t adc_cali_handle = NULL;
bool adc_calibrated = false;

// WiFi
#define SSID "M&M"
#define PASSWORD "39402100"

static TaskHandle_t powerTaskHandle = NULL;

// Triac timer
static esp_timer_handle_t triac_timer;

// ********* Function prototypes *********
int analogRead_adc1(void);
void gpio_setup(void);
void wifi_init_sta(void);
void adc_init(void);

/**
 * How to run OTA
 * 
 * 1 - Build your code with the correct IP server
 * 2 - Save the project.bin file from build to a different path
 * 3 - Open the terminal and run cd /your/path/here/
 * 4 - Run on terminal python3 -m http.server 8000
 * 5 - Run ESP OTA program 
 */

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
        ESP_LOGI(TAG_OTA, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGI(TAG_OTA, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGI(TAG_OTA, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGI(TAG_OTA, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGI(TAG_OTA, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGI(TAG_OTA, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGI(TAG_OTA, "HTTP_EVENT_DISCONNECTED");
        break;
    case HTTP_EVENT_REDIRECT:
        ESP_LOGI(TAG_OTA, "HTTP_EVENT_REDIRECT");
        break;
    default:
        break;
    }
    return ESP_OK;
}

esp_err_t perform_ota_update(void) {

    esp_netif_ip_info_t ip;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip);

    ESP_LOGI(TAG_OTA, "Starting OTA update...");

    esp_https_ota_config_t ota_config = {
        .http_config = &(esp_http_client_config_t) {
            .url = OTA_URL,
            .crt_bundle_attach = esp_crt_bundle_attach,
            .skip_cert_common_name_check = true,
            .keep_alive_enable = true,
            .timeout_ms = 60000,
        },
        .partial_http_download = true,
    };

    esp_err_t ret = esp_https_ota(&ota_config);

    if (ret == ESP_OK) {
        const esp_partition_t *ota = esp_ota_get_next_update_partition(NULL);
        esp_ota_set_boot_partition(ota);

        ESP_LOGI("OTA", "OTA OK, restarting...");

        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
        return ESP_OK; // Never will be returned
    } 
    else {
        ESP_LOGE("OTA", "OTA failed");
    }

    return ESP_FAIL;
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 4) {
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
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");

        esp_netif_dns_info_t dns;
        dns.ip.u_addr.ip4.addr = esp_ip4addr_aton("8.8.8.8");  // Google DNS
        esp_netif_set_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns);

        ESP_LOGI(TAGWIFI, "DNS 8.8.8.8 defined");
    }
}

static void IRAM_ATTR zero_cross_isr(void *arg) {
    isCrossing = true;
    lastCrossedTime = crossedTime;
    crossedTime = esp_timer_get_time() + 125; // global time + half pulse opto windown time

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(powerTaskHandle, 0, eNoAction, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

int analogRead_adc1(void) {
    int raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, NTC_PIN, &raw));

    if (adc_calibrated && adc_cali_handle != NULL) {
        int voltage_mv = 0;
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, raw, &voltage_mv));
        return voltage_mv;  // you can decide if you want raw or mV
    }

    return raw; // fallback when no calibration is possible
}

static void triac_fire_cb(void *arg) {
    if (heatError || PID <= 0) {
        return;
    }

    gpio_set_level(OUT_POWER, 1);   // Gate ON
    esp_rom_delay_us(200);          // Gate time
    gpio_set_level(OUT_POWER, 0);   // Gate OFF
}

static void init_triac_timer(void) {
    const esp_timer_create_args_t timer_args = {
        .callback = &triac_fire_cb,
        .name = "triac_timer"
    };

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &triac_timer));
}

void power_control_task(void *arg) {

    powerTaskHandle = xTaskGetCurrentTaskHandle();
    esp_task_wdt_add(NULL);

    uint32_t cycleTime = 0;
    uint32_t timeToWait = 0;

    while (!heatError){
        esp_task_wdt_reset();

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait notify from ISR

        gpio_set_level(OUT_POWER, 0); // Turn off the output when it's reach the zero point

        cycleTime = crossedTime - lastCrossedTime; // Calculates the cycle time

        if (cycleTime < 8000 || cycleTime > 10500) cycleTime = 8333; // fallback 60Hz

        if (PID <= 0) continue; // Return loop until have a valid PID val

        timeToWait = (cycleTime / 255) * (255 - PID); // Calculate how many time wait to turn the output on
        timeToWait += crossedTime; // Sum this to a global time

        uint32_t delay_us = timeToWait - esp_timer_get_time(); // Calculate correct delay based on global time

        // Cancel any pending timer
        esp_timer_stop(triac_timer);

        // Schedule TRIAC fire
        esp_timer_start_once(triac_timer, delay_us);
    }

    gpio_set_level(OUT_POWER, 0);

    ESP_LOGW(TAG_PW, "Power task was deleted and output turned off");

    vTaskDelete(NULL);
}

void logic_control_task(void *arg) {

    esp_task_wdt_add(NULL);

    static float oldEr = 0.0f;
    float er = 0.0f;
    float p = 0.0f;
    static float i = 0.0f;
    float d = 0.0f;

    while (1) {

        esp_task_wdt_reset();

        // *********** Btn logic  ***********
        if (timerBtn < MAX_TIMER_ON) {
            if (!gpio_get_level(IN_BTN)) {
                vTaskDelay(pdMS_TO_TICKS(1));
                commandBtn = !gpio_get_level(IN_BTN);
            } else {
                commandBtn = false;
                i = 0.0f;
            }

            if (lastTimeBtn > 10 && lastTimeBtn < 100 && commandBtn) {
                setpoint = boilingSetpoint; // Update setpoint with the new value
                lastTimeBtn = 0; // Reset timer count to enter here once
                oldEr = setpoint - waterTemp; // Update oldError to avoid wrong oscilation
            }

            if (!commandBtn) setpoint = coffeeSetpoint;

        } else {
            heatError = true;
            i = 0.0f;
            PID = 0;
            ESP_LOGW(TAG_BTN, "Overtime On!");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        // *********** PID Control  *********** 
        if (!commandBtn) {
            PID = 0;
            vTaskDelay(pdMS_TO_TICKS(TIME_CONTROL_INTERVAL * 2)); // Double time because nothing needs to be calculated
            ESP_LOGD(TAG_CTRL, "Btn off, skiping PID logic");
            continue;
        }
        er = setpoint - waterTemp;
        p = er * KP;
        if (p > 255.00f) p = 255.00f;
        else if (p < -255.00f) p = -255.00f;

        i += er * KI;
        if (i > 255.00f) i = 255.00f;
        else if (i < -255.00f) i = -255.00f;

        d = (er - oldEr) * KD;
        if (d > 255.00f) d = 255.00f;
        else if (d < -255.00f) d = -255.00f;

        oldEr = er;

        PID = (int16_t)(p + i + d);
        if (PID < 20) PID = 0;
        else if (PID > 255) PID = 255;

        ESP_LOGD(TAG_CTRL, "Temp %.2f  PID %d", waterTemp, PID);
        vTaskDelay(pdMS_TO_TICKS(TIME_CONTROL_INTERVAL));
    }
}

void temperature_task(void *arg) {
    static float temp = 25.00f;

    esp_task_wdt_add(NULL);

    while(1) {

        int adcVal = analogRead_adc1();
        float voltage = (float)adcVal / 1000.0f;

        if (voltage <= 0.2f || voltage >= 3.1f) {
            heatError = true;
            waterTemp = 199.00f;
            ESP_LOGE(TAG_TEMP, "Temp Sensor Error! Voltage:%f", voltage);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        float ntcResistance = (3.3f * SERIE_RESISTOR / voltage) - SERIE_RESISTOR;
        float kTemperature = 1.0f / (1.0f / REF_TEMP + (1.0f / BETA) * log(ntcResistance / REF_RESISTANCE));
        float currentTemperature = kTemperature - 273.15f;

        temp = (1.0f - FACTORADC) * temp + FACTORADC * currentTemperature;

        waterTemp = temp;

        if (waterTemp > 120.0f && !heatError) {
            heatError = true;
            ESP_LOGW(TAG_TEMP, "Overtemp! %f", waterTemp);
        }

        esp_task_wdt_reset();
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
                ESP_LOGI(TAG_BTN, "Button was released");
                ESP_LOGI(TAG_BTN, "Last time btn: %d", lastTimeBtn);
            }
        }
        

        if ((esp_timer_get_time() / 1000) > MAX_TIMER_ON) {
            gpio_set_level(OUT_POWER, 0);
            perform_ota_update();
            esp_deep_sleep_start();         // Falls on deep sleep with no return 
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void gpio_setup(void) {
    gpio_set_direction(OUT_POWER, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN_ZERO_CROSSING, GPIO_MODE_INPUT);
    gpio_set_direction(IN_BTN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(IN_ZERO_CROSSING, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(IN_BTN, GPIO_PULLUP_ONLY);
    gpio_set_level(OUT_POWER, 0);

    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_set_intr_type(IN_ZERO_CROSSING, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(IN_ZERO_CROSSING, zero_cross_isr, NULL);
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
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                        ESP_EVENT_ANY_ID,
                                        &wifi_event_handler,
                                        NULL,
                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                        IP_EVENT_STA_GOT_IP,
                                        &wifi_event_handler,
                                        NULL,
                                        NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = SSID,
            .password = PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
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

void sync_time(void) {
    // Configure SNTP
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");    
    sntp_init();

    time_t now = 0;
    struct tm timeinfo = {0};

    // Wait until time is set
    int retry = 0;
    const int retry_count = 10;
    while (timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count)
    {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    if (timeinfo.tm_year >= (2016 - 1900))
    {
        ESP_LOGI("Time", "Time synced: %s", asctime(&timeinfo));
    }
    else
    {
        ESP_LOGW("Time", "Failed to sync NTP time\n");
    }

    setenv("TZ", "GMT+3", 1);
    tzset();

    localtime_r(&now, &timeinfo);
    ESP_LOGI("Time", "Local time (Brazil): %s", asctime(&timeinfo));
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

    // Triac timer init
    init_triac_timer();

    // Start loop task
    xTaskCreate(logic_control_task, "logic_control", 4096, NULL, 3, NULL);
    xTaskCreate(power_control_task, "power_control", 4096, NULL, 5, NULL);
    xTaskCreate(temperature_task, "temperature", 4096, NULL, 5, NULL);
    xTaskCreatePinnedToCore(aux_task, "auxiliar", 4096, NULL, 1, NULL, 1);  


    // NVS init
    ESP_ERROR_CHECK(nvs_flash_init());

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_sta();
    ESP_LOGI(TAGWIFI, "Wifi Ready!");

    sync_time();

    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err != ESP_OK) {
        ESP_LOGE("OTA", "Failed to mark app valid (%s)", esp_err_to_name(err));
    }
}

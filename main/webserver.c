#include "webserver.h"

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_http_server.h"
#include "esp_vfs.h"

#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "webserver";

// ─────────────────────────────────────────────
//  Global definitions
// ─────────────────────────────────────────────

history_buf_t      g_history    = { .head = -1, .count = 0 };
webserver_config_t g_ws_config  = {
    .coffee_setpoint      = 75.0f,
    .boiling_setpoint     = 100.0f,
    .kp                   = 12.0f,
    .ki                   = 0.0004f,
    .kd                   = 40.0f,
    .hibernate_timeout_ms = 300000,   // 5 minutes default
};
webserver_status_t g_ws_status  = { 0 };
wifi_pending_t     g_wifi_pending = { .pending = false };

// Mutex protecting g_history and g_ws_status from concurrent access
// (push from telemetry task, reads from HTTP handlers)
static SemaphoreHandle_t s_history_mutex = NULL;

// Buffer for streaming the /api/history JSON — allocated once at start
// Worst case: 300 entries × ~50 chars each + envelope ≈ 16KB
#define JSON_HISTORY_BUF_SIZE (300 * 52 + 128)
static char *s_history_json_buf = NULL;

// Scratch buffer for small JSON responses (status, config ACK, errors)
#define JSON_SCRATCH_SIZE 512
static char s_scratch[JSON_SCRATCH_SIZE];

// OTA write handle — kept across chunked POST /api/ota
static esp_ota_handle_t s_ota_handle    = 0;
static const esp_partition_t *s_ota_part = NULL;
static bool s_ota_in_progress           = false;

// ─────────────────────────────────────────────
//  Internal helpers
// ─────────────────────────────────────────────

static void wifi_get_info(char *mode_out, char *ip_out, int8_t *rssi_out)
{
    wifi_mode_t mode = WIFI_MODE_NULL;
    esp_wifi_get_mode(&mode);

    if (mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA) {
        strncpy(mode_out, "STA", 4);
        wifi_ap_record_t ap = {0};
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            *rssi_out = ap.rssi;
        }
        esp_netif_ip_info_t ip_info = {0};
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
            snprintf(ip_out, 16, IPSTR, IP2STR(&ip_info.ip));
        }
    } else {
        strncpy(mode_out, "AP", 4);
        *rssi_out = 0;
        strncpy(ip_out, "192.168.4.1", 16);
    }
}

static void fw_version_get(char *out, size_t len)
{
    const esp_app_desc_t *desc = esp_app_get_description();
    if (desc) {
        snprintf(out, len, "%s", desc->version);
    } else {
        snprintf(out, len, "unknown");
    }
}

// ─────────────────────────────────────────────
//  History push  (called from telemetry task)
// ─────────────────────────────────────────────

void webserver_history_push(float temp, float setpoint, int16_t pid, bool heating)
{
    if (!s_history_mutex) return;

    xSemaphoreTake(s_history_mutex, portMAX_DELAY);

    // Advance head
    g_history.head = (g_history.head + 1) % HISTORY_SIZE;
    if (g_history.count < HISTORY_SIZE) g_history.count++;

    history_entry_t *e = &g_history.entries[g_history.head];
    e->temp        = temp;
    e->setpoint    = setpoint;
    e->timestamp_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);

    // Update live status snapshot while we hold the mutex
    g_ws_status.temp     = temp;
    g_ws_status.setpoint = setpoint;
    g_ws_status.pid      = pid;
    g_ws_status.heating  = heating;
    g_ws_status.uptime_s = e->timestamp_s;
    g_ws_status.free_heap = (uint32_t)esp_get_free_heap_size();

    // Hibernate countdown
    uint32_t timeout_s  = g_ws_config.hibernate_timeout_ms / 1000;
    g_ws_status.hibernate_remaining_s =
        (e->timestamp_s < timeout_s) ? (timeout_s - e->timestamp_s) : 0;

    // Setpoint name
    if (setpoint >= 90.0f) {
        strncpy(g_ws_status.sp_name, "Fervura", sizeof(g_ws_status.sp_name));
    } else {
        strncpy(g_ws_status.sp_name, "Café / Chá", sizeof(g_ws_status.sp_name));
    }

    // WiFi info (cheap enough to update every second)
    wifi_get_info(g_ws_status.wifi_mode, g_ws_status.ip, &g_ws_status.rssi);
    fw_version_get(g_ws_status.fw_version, sizeof(g_ws_status.fw_version));

    xSemaphoreGive(s_history_mutex);
}

// ─────────────────────────────────────────────
//  Handler: GET /
//  Serves index.html from SPIFFS
// ─────────────────────────────────────────────

static esp_err_t handler_root(httpd_req_t *req)
{
    FILE *f = fopen("/spiffs/index.html", "r");
    if (!f) {
        ESP_LOGE(TAG, "index.html not found in SPIFFS");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "index.html not found — reflash SPIFFS partition");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "text/html");
    char chunk[512];
    size_t n;
    while ((n = fread(chunk, 1, sizeof(chunk), f)) > 0) {
        httpd_resp_send_chunk(req, chunk, n);
    }
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

// ─────────────────────────────────────────────
//  Handler: GET /chart.umd.min.js
//  Serves Chart.js from SPIFFS
// ─────────────────────────────────────────────

static esp_err_t handler_chartjs(httpd_req_t *req)
{
    FILE *f = fopen("/spiffs/chart.umd.min.js", "r");
    if (!f) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/javascript");
    // Cache for 1 day — the file never changes between OTA updates
    httpd_resp_set_hdr(req, "Cache-Control", "max-age=86400");

    char chunk[1024];
    size_t n;
    while ((n = fread(chunk, 1, sizeof(chunk), f)) > 0) {
        httpd_resp_send_chunk(req, chunk, n);
    }
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

// ─────────────────────────────────────────────
//  Handler: GET /api/status
// ─────────────────────────────────────────────

static esp_err_t handler_api_status(httpd_req_t *req)
{
    xSemaphoreTake(s_history_mutex, portMAX_DELAY);
    webserver_status_t snap = g_ws_status;   // copy under mutex
    xSemaphoreGive(s_history_mutex);

    snprintf(s_scratch, sizeof(s_scratch),
        "{"
        "\"temp\":%.1f,"
        "\"setpoint\":%.1f,"
        "\"pid\":%d,"
        "\"heating\":%s,"
        "\"uptime\":%lu,"
        "\"hibernate_remaining\":%lu,"
        "\"wifi_mode\":\"%s\","
        "\"ip\":\"%s\","
        "\"rssi\":%d,"
        "\"heap\":\"%lu KB\","
        "\"fw\":\"%s\","
        "\"sp_name\":\"%s\""
        "}",
        snap.temp,
        snap.setpoint,
        snap.pid,
        snap.heating ? "true" : "false",
        (unsigned long)snap.uptime_s,
        (unsigned long)snap.hibernate_remaining_s,
        snap.wifi_mode,
        snap.ip,
        snap.rssi,
        (unsigned long)(snap.free_heap / 1024),
        snap.fw_version,
        snap.sp_name
    );

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_sendstr(req, s_scratch);
    return ESP_OK;
}

// ─────────────────────────────────────────────
//  Handler: GET /api/history
//  Returns ordered JSON from oldest to newest
// ─────────────────────────────────────────────

static esp_err_t handler_api_history(httpd_req_t *req)
{
    if (!s_history_json_buf) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    xSemaphoreTake(s_history_mutex, portMAX_DELAY);

    int count = g_history.count;
    int head  = g_history.head;

    // oldest entry index
    int oldest = (count < HISTORY_SIZE)
                 ? 0
                 : (head + 1) % HISTORY_SIZE;

    // Build JSON: { "labels":[...], "temps":[...], "setpoints":[...] }
    int pos = 0;
    pos += snprintf(s_history_json_buf + pos, JSON_HISTORY_BUF_SIZE - pos,
                    "{\"labels\":[");

    for (int i = 0; i < count; i++) {
        int idx = (oldest + i) % HISTORY_SIZE;
        uint32_t ts = g_history.entries[idx].timestamp_s;
        pos += snprintf(s_history_json_buf + pos, JSON_HISTORY_BUF_SIZE - pos,
                        "%s%lu", i ? "," : "", (unsigned long)ts);
    }

    pos += snprintf(s_history_json_buf + pos, JSON_HISTORY_BUF_SIZE - pos,
                    "],\"temps\":[");

    for (int i = 0; i < count; i++) {
        int idx = (oldest + i) % HISTORY_SIZE;
        pos += snprintf(s_history_json_buf + pos, JSON_HISTORY_BUF_SIZE - pos,
                        "%s%.1f", i ? "," : "", g_history.entries[idx].temp);
    }

    pos += snprintf(s_history_json_buf + pos, JSON_HISTORY_BUF_SIZE - pos,
                    "],\"setpoints\":[");

    for (int i = 0; i < count; i++) {
        int idx = (oldest + i) % HISTORY_SIZE;
        pos += snprintf(s_history_json_buf + pos, JSON_HISTORY_BUF_SIZE - pos,
                        "%s%.1f", i ? "," : "", g_history.entries[idx].setpoint);
    }

    pos += snprintf(s_history_json_buf + pos, JSON_HISTORY_BUF_SIZE - pos, "]}");

    xSemaphoreGive(s_history_mutex);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_send(req, s_history_json_buf, pos);
    return ESP_OK;
}

// ─────────────────────────────────────────────
//  Handler: GET /api/config
//  Returns current g_ws_config so the UI can
//  populate sliders with the persisted values.
// ─────────────────────────────────────────────

static esp_err_t handler_api_config_get(httpd_req_t *req)
{
    snprintf(s_scratch, sizeof(s_scratch),
        "{"
        "\"coffee_setpoint\":%.1f,"
        "\"boiling_setpoint\":%.1f,"
        "\"kp\":%.4f,"
        "\"ki\":%.7f,"
        "\"kd\":%.2f,"
        "\"hibernate_timeout_min\":%lu"
        "}",
        g_ws_config.coffee_setpoint,
        g_ws_config.boiling_setpoint,
        g_ws_config.kp,
        g_ws_config.ki,
        g_ws_config.kd,
        (unsigned long)(g_ws_config.hibernate_timeout_ms / 60000)
    );

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_sendstr(req, s_scratch);
    return ESP_OK;
}

// ─────────────────────────────────────────────
//  Minimal JSON parser helpers
//  (avoids pulling in cJSON to save ~30KB flash)
// ─────────────────────────────────────────────

static bool json_get_float(const char *json, const char *key, float *out)
{
    char search[48];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return false;
    p += strlen(search);
    while (*p == ' ') p++;
    *out = strtof(p, NULL);
    return true;
}

static bool json_get_string(const char *json, const char *key, char *out, size_t out_len)
{
    char search[48];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return false;
    p += strlen(search);
    while (*p == ' ') p++;
    if (*p != '"') return false;
    p++;
    size_t i = 0;
    while (*p && *p != '"' && i < out_len - 1) out[i++] = *p++;
    out[i] = '\0';
    return true;
}

// ─────────────────────────────────────────────
//  Handler: POST /api/config
//  Accepts: { "coffee_setpoint":75, "boiling_setpoint":100, "kp":12, "ki":0.0004, "kd":40 }
//  All fields are optional — only present keys are applied.
// ─────────────────────────────────────────────

static esp_err_t handler_api_config(httpd_req_t *req)
{
    int total = req->content_len;
    if (total <= 0 || total >= (int)sizeof(s_scratch)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Body too large");
        return ESP_FAIL;
    }

    int received = httpd_req_recv(req, s_scratch, total);
    if (received <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Recv error");
        return ESP_FAIL;
    }
    s_scratch[received] = '\0';

    float val;

    if (json_get_float(s_scratch, "coffee_setpoint", &val) && val >= 50.0f && val <= 90.0f)
        g_ws_config.coffee_setpoint = val;

    if (json_get_float(s_scratch, "boiling_setpoint", &val) && val >= 90.0f && val <= 115.0f)
        g_ws_config.boiling_setpoint = val;

    if (json_get_float(s_scratch, "kp", &val) && val >= 0.0f && val <= 50.0f)
        g_ws_config.kp = val;

    if (json_get_float(s_scratch, "ki", &val) && val >= 0.0f && val <= 0.01f)
        g_ws_config.ki = val;

    if (json_get_float(s_scratch, "kd", &val) && val >= 0.0f && val <= 100.0f)
        g_ws_config.kd = val;

    // hibernate_timeout_ms — accept minutes from UI, store as ms internally
    // Range: 1 min (60000) to 120 min (7200000)
    if (json_get_float(s_scratch, "hibernate_timeout_min", &val)
        && val >= 1.0f && val <= 120.0f) {
        g_ws_config.hibernate_timeout_ms = (uint32_t)(val * 60000.0f);
    }

    // Persist to NVS
    nvs_handle_t nvs;
    if (nvs_open("sw_config", NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_u32(nvs, "coffee_sp",    (uint32_t)(g_ws_config.coffee_setpoint  * 100));
        nvs_set_u32(nvs, "boiling_sp",   (uint32_t)(g_ws_config.boiling_setpoint * 100));
        nvs_set_u32(nvs, "kp",           (uint32_t)(g_ws_config.kp               * 1000));
        nvs_set_u32(nvs, "ki",           (uint32_t)(g_ws_config.ki               * 10000000));
        nvs_set_u32(nvs, "kd",           (uint32_t)(g_ws_config.kd               * 1000));
        nvs_set_u32(nvs, "hibernate_ms", g_ws_config.hibernate_timeout_ms);
        nvs_commit(nvs);
        nvs_close(nvs);
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

// ─────────────────────────────────────────────
//  Handler: POST /api/wifi
//  Accepts: { "ssid":"...", "password":"..." }
//  Saves to NVS and schedules a reboot.
// ─────────────────────────────────────────────

static esp_err_t handler_api_wifi(httpd_req_t *req)
{
    int total = req->content_len;
    if (total <= 0 || total >= (int)sizeof(s_scratch)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Body too large");
        return ESP_FAIL;
    }

    int received = httpd_req_recv(req, s_scratch, total);
    if (received <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Recv error");
        return ESP_FAIL;
    }
    s_scratch[received] = '\0';

    char ssid[64] = {0};
    char pass[64] = {0};

    if (!json_get_string(s_scratch, "ssid", ssid, sizeof(ssid)) || strlen(ssid) == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing ssid");
        return ESP_FAIL;
    }

    json_get_string(s_scratch, "password", pass, sizeof(pass));

    if (strlen(pass) > 0 && strlen(pass) < 8) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Password too short");
        return ESP_FAIL;
    }

    // Save to NVS
    nvs_handle_t nvs;
    if (nvs_open("wifi_cfg", NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_str(nvs, "ssid",     ssid);
        nvs_set_str(nvs, "password", pass);
        nvs_commit(nvs);
        nvs_close(nvs);
    }

    // Signal wifi_manager to reconnect after response is sent
    strncpy(g_wifi_pending.ssid,     ssid, sizeof(g_wifi_pending.ssid));
    strncpy(g_wifi_pending.password, pass, sizeof(g_wifi_pending.password));
    g_wifi_pending.pending = true;

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");

    // Small delay so the response reaches the browser before reboot
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();

    return ESP_OK;
}

// ─────────────────────────────────────────────
//  Handler: POST /api/ota
//  Receives raw .bin over chunked HTTP body.
//  esp_http_server calls this handler repeatedly
//  with successive chunks until content_len is consumed.
// ─────────────────────────────────────────────

#define OTA_CHUNK_SIZE 1024

static esp_err_t handler_api_ota(httpd_req_t *req)
{
    ESP_LOGI(TAG, "OTA upload started, size=%d", req->content_len);

    if (req->content_len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No content");
        return ESP_FAIL;
    }

    s_ota_part = esp_ota_get_next_update_partition(NULL);
    if (!s_ota_part) {
        ESP_LOGE(TAG, "No OTA partition found");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition");
        return ESP_FAIL;
    }

    esp_err_t err = esp_ota_begin(s_ota_part, OTA_SIZE_UNKNOWN, &s_ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return ESP_FAIL;
    }

    s_ota_in_progress = true;

    static char ota_chunk[OTA_CHUNK_SIZE];
    int remaining = req->content_len;
    int received  = 0;

    while (remaining > 0) {
        int to_read = (remaining < OTA_CHUNK_SIZE) ? remaining : OTA_CHUNK_SIZE;
        int n = httpd_req_recv(req, ota_chunk, to_read);

        if (n <= 0) {
            ESP_LOGE(TAG, "OTA recv error or timeout (n=%d)", n);
            esp_ota_abort(s_ota_handle);
            s_ota_in_progress = false;
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive failed");
            return ESP_FAIL;
        }

        err = esp_ota_write(s_ota_handle, ota_chunk, n);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            esp_ota_abort(s_ota_handle);
            s_ota_in_progress = false;
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA write failed");
            return ESP_FAIL;
        }

        remaining -= n;
        received  += n;
        ESP_LOGD(TAG, "OTA progress: %d / %d bytes", received, req->content_len);
    }

    err = esp_ota_end(s_ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        s_ota_in_progress = false;
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA end failed");
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(s_ota_part);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        s_ota_in_progress = false;
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Boot partition failed");
        return ESP_FAIL;
    }

    s_ota_in_progress = false;
    ESP_LOGI(TAG, "OTA complete (%d bytes), rebooting...", received);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");

    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();

    return ESP_OK;
}

// ─────────────────────────────────────────────
//  URI table
// ─────────────────────────────────────────────

static const httpd_uri_t s_uris[] = {
    { .uri = "/",                .method = HTTP_GET,  .handler = handler_root              },
    { .uri = "/chart.umd.min.js",.method = HTTP_GET,  .handler = handler_chartjs           },
    { .uri = "/api/status",      .method = HTTP_GET,  .handler = handler_api_status        },
    { .uri = "/api/history",     .method = HTTP_GET,  .handler = handler_api_history       },
    { .uri = "/api/config",      .method = HTTP_GET,  .handler = handler_api_config_get    },
    { .uri = "/api/config",      .method = HTTP_POST, .handler = handler_api_config        },
    { .uri = "/api/wifi",        .method = HTTP_POST, .handler = handler_api_wifi          },
    { .uri = "/api/ota",         .method = HTTP_POST, .handler = handler_api_ota           },
};

#define URI_COUNT (sizeof(s_uris) / sizeof(s_uris[0]))

// ─────────────────────────────────────────────
//  Public: load config from NVS
//  Falls back to compiled-in defaults for missing keys.
// ─────────────────────────────────────────────

void webserver_config_load_nvs(void)
{
    nvs_handle_t nvs;
    if (nvs_open("sw_config", NVS_READONLY, &nvs) != ESP_OK) {
        ESP_LOGW(TAG, "No saved config in NVS — using defaults");
        return;
    }

    uint32_t u32 = 0;

    if (nvs_get_u32(nvs, "coffee_sp",    &u32) == ESP_OK)
        g_ws_config.coffee_setpoint  = (float)u32 / 100.0f;

    if (nvs_get_u32(nvs, "boiling_sp",   &u32) == ESP_OK)
        g_ws_config.boiling_setpoint = (float)u32 / 100.0f;

    if (nvs_get_u32(nvs, "kp",           &u32) == ESP_OK)
        g_ws_config.kp = (float)u32 / 1000.0f;

    if (nvs_get_u32(nvs, "ki",           &u32) == ESP_OK)
        g_ws_config.ki = (float)u32 / 10000000.0f;

    if (nvs_get_u32(nvs, "kd",           &u32) == ESP_OK)
        g_ws_config.kd = (float)u32 / 1000.0f;

    if (nvs_get_u32(nvs, "hibernate_ms", &u32) == ESP_OK)
        g_ws_config.hibernate_timeout_ms = u32;

    nvs_close(nvs);

    ESP_LOGI(TAG, "Config loaded — coffee:%.1f boil:%.1f kp:%.2f ki:%.4f kd:%.2f hibernate:%lus",
             g_ws_config.coffee_setpoint,
             g_ws_config.boiling_setpoint,
             g_ws_config.kp,
             g_ws_config.ki,
             g_ws_config.kd,
             (unsigned long)(g_ws_config.hibernate_timeout_ms / 1000));
}

// ─────────────────────────────────────────────
//  Public: start / stop
// ─────────────────────────────────────────────

httpd_handle_t webserver_start(void)
{
    // Create mutex and allocate history JSON buffer once
    if (!s_history_mutex) {
        s_history_mutex = xSemaphoreCreateMutex();
        if (!s_history_mutex) {
            ESP_LOGE(TAG, "Failed to create history mutex");
            return NULL;
        }
    }

    if (!s_history_json_buf) {
        s_history_json_buf = malloc(JSON_HISTORY_BUF_SIZE);
        if (!s_history_json_buf) {
            ESP_LOGE(TAG, "Failed to allocate history JSON buffer (%d bytes)",
                     JSON_HISTORY_BUF_SIZE);
            return NULL;
        }
    }

    httpd_config_t config     = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers   = URI_COUNT + 1; // +1: same URI /api/config has GET and POST
    config.stack_size         = 8192;   // larger stack for OTA write + JSON build
    config.recv_wait_timeout  = 30;     // seconds — important for large OTA uploads
    config.send_wait_timeout  = 30;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return NULL;
    }

    for (size_t i = 0; i < URI_COUNT; i++) {
        httpd_register_uri_handler(server, &s_uris[i]);
    }

    ESP_LOGI(TAG, "HTTP server started — %d routes registered", (int)URI_COUNT);
    return server;
}

void webserver_stop(httpd_handle_t server)
{
    if (server) {
        httpd_stop(server);
        ESP_LOGI(TAG, "HTTP server stopped");
    }
}
#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

// ─────────────────────────────────────────────
//  History buffer
// ─────────────────────────────────────────────

#define HISTORY_SIZE      300          // 5 minutes @ 1 sample/s
#define HISTORY_INTERVAL_MS 1000       // sampling interval

typedef struct {
    float    temp;
    float    setpoint;
    uint32_t timestamp_s;              // seconds since boot
} history_entry_t;

// Circular buffer — written by webserver_history_push(), read by /api/history
typedef struct {
    history_entry_t entries[HISTORY_SIZE];
    int     head;                      // index of the MOST RECENT valid entry (-1 = empty)
    int     count;                     // how many valid entries (0..HISTORY_SIZE)
} history_buf_t;

extern history_buf_t g_history;

// ─────────────────────────────────────────────
//  Runtime config (mirrors NVS + live PID)
//  Written by POST /api/config, read by control tasks
// ─────────────────────────────────────────────

typedef struct {
    float    coffee_setpoint;             // °C
    float    boiling_setpoint;            // °C
    float    kp;
    float    ki;
    float    kd;
    uint32_t hibernate_timeout_ms;        // ms — auto deep-sleep after this uptime
} webserver_config_t;

extern webserver_config_t g_ws_config;

// ─────────────────────────────────────────────
//  Live status snapshot
//  Written every HISTORY_INTERVAL_MS by webserver_history_push()
//  Read by GET /api/status — single source of truth for the UI
// ─────────────────────────────────────────────

typedef struct {
    float    temp;
    float    setpoint;
    int16_t  pid;
    bool     heating;
    uint32_t uptime_s;
    uint32_t hibernate_remaining_s;    // seconds until auto deep-sleep (for header countdown)
    char     wifi_mode[4];             // "STA" or "AP"
    char     ip[16];
    int8_t   rssi;
    uint32_t free_heap;
    char     fw_version[12];
    char     sp_name[16];              // "Café / Chá" or "Fervura"
} webserver_status_t;

extern webserver_status_t g_ws_status;

// ─────────────────────────────────────────────
//  Heat override (virtual button)
//  Set by POST /api/heat, consumed by logic_control_task in main.c.
//  The task is responsible for clearing it once target_temp is reached,
//  so behaviour is correct even if the browser tab is closed.
// ─────────────────────────────────────────────

typedef enum {
    HEAT_MODE_COFFEE  = 0,
    HEAT_MODE_BOILING = 1,
} heat_mode_t;

typedef struct {
    bool        active;       // true = override in effect
    heat_mode_t mode;         // which setpoint to use
    float       target_temp;  // °C — task clears override when temp >= this value
} heat_override_t;

extern heat_override_t g_heat_override;

// ─────────────────────────────────────────────
//  Pending WiFi credentials
//  Set by POST /api/wifi, consumed by wifi_manager after reboot
// ─────────────────────────────────────────────

typedef struct {
    char ssid[64];
    char password[64];
    bool pending;                      // true = reboot requested with new creds
} wifi_pending_t;

extern wifi_pending_t g_wifi_pending;

// ─────────────────────────────────────────────
//  Public API
// ─────────────────────────────────────────────

/**
 * Load persisted config from NVS into g_ws_config.
 * Call once in app_main after nvs_flash_init().
 * Falls back to compiled-in defaults for any key not found.
 */
void webserver_config_load_nvs(void);

/**
 * Start the HTTP server and register all URI handlers.
 * Call after WiFi/AP is up.
 * Returns the server handle or NULL on failure.
 */
httpd_handle_t webserver_start(void);

/**
 * Stop the HTTP server.
 */
void webserver_stop(httpd_handle_t server);

/**
 * Push one sample into the circular history buffer and update g_ws_status.
 * Call from a FreeRTOS task every HISTORY_INTERVAL_MS.
 *
 * @param temp        current water temperature
 * @param setpoint    active setpoint
 * @param pid         current PID output (0-255)
 * @param heating     true if TRIAC is firing
 */
void webserver_history_push(float temp, float setpoint, int16_t pid, bool heating);
#pragma once

#include "esp_err.h"
#include <stdbool.h>

// AP config — shown when no credentials are found or STA fails
#define WIFI_AP_SSID     "SmartWater-Setup"
#define WIFI_AP_PASS     "smartwater"
#define WIFI_AP_CHANNEL  1
#define WIFI_AP_MAX_CONN 2

// STA config
#define WIFI_MAX_RETRY   5

// NVS keys
#define NVS_NAMESPACE    "wifi_cfg"
#define NVS_KEY_SSID     "ssid"
#define NVS_KEY_PASS     "pass"

// mDNS
#define MDNS_HOSTNAME    "smartwater"

// WiFi mode reported to the rest of the system
typedef enum {
    WIFI_MODE_DISCONNECTED = 0,
    WIFI_MODE_STA_CONNECTED,
    WIFI_MODE_AP_ON,
} wifi_manager_mode_t;

/**
 * @brief  Initialise WiFi driver (call once, after nvs_flash_init,
 *         esp_netif_init and esp_event_loop_create_default).
 *
 *         Flow:
 *           1. Try to load credentials from NVS.
 *           2. If found  → start STA, wait up to WIFI_MAX_RETRY attempts.
 *           3. If STA OK → start mDNS as "smartwater.local".
 *           4. If no creds or STA failed → start AP "SmartWater-Setup".
 *
 *         The webserver must be started by the caller after this returns,
 *         regardless of the resulting mode.
 */
void wifi_manager_init(void);

/**
 * @brief  Save new WiFi credentials to NVS and reboot.
 *         Called by the webserver when the user submits the network form.
 */
esp_err_t wifi_manager_save_credentials(const char *ssid, const char *password);

/**
 * @brief  Return the current WiFi operating mode.
 */
wifi_manager_mode_t wifi_manager_get_mode(void);
# Smart Water

ESP32-based firmware for precise electric kettle temperature control. Uses PID with TRIAC phase-cut power regulation, NTC thermistor sensing, and an embedded web interface for real-time monitoring and configuration.

---

## Features

- PID temperature control with TRIAC phase-cut (0–100% power resolution)
- Exponential low-pass filter on ADC readings for stable thermal sensing
- Embedded web interface (SPA) served from SPIFFS — no cloud dependency
- Real-time temperature chart (5-minute rolling window, 300 data points)
- Configurable setpoints: Coffee/Tea (50–90 °C) and Boiling (90–115 °C)
- Auto-hibernate via deep sleep after configurable idle timeout
- Over-the-air (OTA) firmware updates via drag-and-drop in the web UI
- Wi-Fi provisioning with AP fallback and mDNS (`smartwater.local`)
- All settings persisted in NVS (setpoints, PID constants, Wi-Fi credentials, timeout)

---

## Hardware

| Component | Details |
|---|---|
| Microcontroller | ESP32 (4 MB flash) |
| Temperature sensor | NTC 10 kΩ — resistor divider, GPIO34 / ADC1_CH6 |
| Power control | TRIAC with phase-cut firing |
| Zero-crossing detection | Optocoupler → GPIO18 (POSEDGE interrupt) |
| TRIAC gate output | GPIO19 (via optocoupler) |
| Physical button | GPIO21 (internal pull-up) |
| Safety | External mechanical thermostat + software watchdog |

The power circuit is fully isolated from the microcontroller via optocouplers on both the zero-crossing detection and TRIAC gate paths.

---

## How It Works

### Temperature Control

The ADC is sampled every 50 ms with linear curve calibration. Readings pass through an exponential low-pass filter (`α = 0.01`) before feeding the PID controller. The low coefficient ensures stability at the cost of slow response to abrupt changes — appropriate for the thermal profile of a kettle.

The PID runs every 250 ms and produces an output in the range 0–255. This value maps to a firing delay within the 60 Hz half-cycle (~8333 µs):

```c
delay_us = cycleTime * (255 - PID_output) / 255
```

A guard discards the cycle if the firing moment has already passed (`delay_us ≤ 0`) or exceeds the half-cycle duration (`delay_us > 8500 µs`), preventing undefined behavior.

### Physical Button

| Action | Behavior |
|---|---|
| Press and hold | Activates heating at Coffee/Tea setpoint |
| Short press + release | Switches to Boiling setpoint |
| Release | Deactivates heating |

Press duration is counted in 10 ms ticks by `aux_task`.

### Auto-Hibernate

After a configurable idle timeout (default: 5 minutes), the firmware cuts power output, signals a heating error to stop phase control, waits 1 second for the web server to serve a final status response, then enters deep sleep. The timeout is configurable via the web interface and persisted in NVS.

---

## Firmware Architecture

Built with **ESP-IDF v5.1**, compiled with CMake.

### FreeRTOS Tasks

| Task | Core | Priority | Stack | Role |
|---|---|---|---|---|
| `power_control` | 0 | 5 | 4096 B | Waits for ISR notification, calculates phase delay, schedules TRIAC gate via `esp_timer` |
| `temperature` | 0 | 5 | 4096 B | ADC read → NTC calculation → filter → updates `waterTemp` |
| `logic_control` | 0 | 3 | 4096 B | Button debounce, setpoint selection, PID calculation |
| `aux` | 1 | 1 | 4096 B | Button press counter, hibernate watchdog |
| `telemetry` | 0 | 2 | 3072 B | Samples system state every 1 s and feeds the web server history buffer |

> The zero-crossing ISR is enabled only after `power_control_task` assigns its own handle to `powerTaskHandle`, preventing notifications to a null handle during boot.

### Module Layout

```
main/
├── main.c              — Control tasks, ISR, GPIO, ADC, app_main
├── wifi_manager.c/h    — STA with AP fallback, mDNS (smartwater.local)
├── webserver.c/h       — HTTP server, REST routes, history ring buffer, OTA
└── CMakeLists.txt

spiffs/
├── index.html          — Web interface (SPA)
└── chart.umd.min.js    — Chart.js 4.4.1 (local, no CDN dependency)

partitions.csv          — Custom partition table: dual OTA + SPIFFS
```

### Partition Table (4 MB)

| Name | Type | Offset | Size |
|---|---|---|---|
| nvs | data/nvs | 0x9000 | 24 KB |
| otadata | data/ota | 0xF000 | 8 KB |
| app0 (ota_0) | app/ota_0 | 0x20000 | 1.5 MB |
| app1 (ota_1) | app/ota_1 | 0x1A0000 | 1.5 MB |
| spiffs | data/spiffs | 0x320000 | 896 KB |

---

## Web Interface

Accessible at `http://smartwater.local` in STA mode or `http://192.168.4.1` in AP mode.

### Capabilities

- Real-time temperature chart (5-minute window, 300 points)
- History restored on page reload via `GET /api/history`
- Adjustable setpoints: Coffee/Tea (50–90 °C) and Boiling (90–115 °C)
- Live PID output bar
- Hibernate countdown in the header with visual alert under 2 minutes
- Wi-Fi configuration with automatic reboot
- Collapsible maintenance panel:
  - PID constants (Kp, Ki, Kd) with modal confirmation before applying
  - Hibernate timeout (1–120 minutes)
  - OTA firmware upload (drag-and-drop `.bin`)
  - System information (IP address, free heap, RSSI, firmware version)

### REST API

| Method | Route | Description |
|---|---|---|
| GET | `/` | Serves `index.html` from SPIFFS |
| GET | `/chart.umd.min.js` | Serves Chart.js from SPIFFS (1-day cache) |
| GET | `/api/status` | JSON with current system state |
| GET | `/api/history` | JSON with temperature ring buffer |
| GET | `/api/config` | JSON with current configuration |
| POST | `/api/config` | Updates setpoints, PID constants, and/or hibernate timeout |
| POST | `/api/wifi` | Saves Wi-Fi credentials to NVS and reboots |
| POST | `/api/ota` | Receives `.bin` as `application/octet-stream` and writes via OTA |

---

## Wi-Fi

On boot, `wifi_manager` attempts to connect using credentials stored in NVS. If no credentials exist or the connection fails after 5 attempts (15 s total timeout), it starts an open Access Point with SSID `SmartWater-Setup`. The web server is available in both modes with the same interface.

In STA mode, mDNS announces `smartwater.local` on the local network, eliminating the need to know the dynamic IP.

---

## OTA Updates

Firmware can be updated over-the-air by uploading a `.bin` file through the web interface. The binary is sent as raw bytes (`Content-Type: application/octet-stream`) and written in 1 KB chunks to the inactive OTA partition. The device reboots into the new partition after a successful write.

ESP-IDF automatic rollback is active. `esp_ota_mark_app_valid_cancel_rollback()` is called in `app_main` after the control tasks are running but before Wi-Fi initializes. If the firmware hangs before that call, ESP-IDF automatically reverts to the previous partition on the next boot.

> **Note:** The SPIFFS partition (web interface) is **not** updated via OTA. To update the HTML:
> ```bash
> idf.py spiffs-flash
> ```

---

## Build & Flash

### Prerequisites

- ESP-IDF v5.1
- Python 3.9+

### First-time flash

```bash
idf.py set-target esp32
idf.py menuconfig   # confirm: custom partition table, offset 0x8000
idf.py erase-flash  # required when changing partition table — clears NVS
idf.py flash monitor
```

> After `erase-flash`, Wi-Fi credentials are cleared and must be reconfigured via the web interface.

### Subsequent updates

```bash
idf.py flash          # firmware only
idf.py spiffs-flash   # web interface only
```

Or use the OTA upload in the web UI for firmware updates.

### Setting the firmware version

In the root `CMakeLists.txt`:

```cmake
set(PROJECT_VER "1.0.0")
```

The version is displayed in the web interface under System Information.

---

## NVS Persistent Storage

| Namespace | Key | Type | Description |
|---|---|---|---|
| `sw_wifi` | `ssid` | string | Wi-Fi SSID |
| `sw_wifi` | `password` | string | Wi-Fi password |
| `sw_config` | `coffee_sp` | u32 | Coffee/Tea setpoint × 100 |
| `sw_config` | `boiling_sp` | u32 | Boiling setpoint × 100 |
| `sw_config` | `kp` | u32 | Kp × 1000 |
| `sw_config` | `ki` | u32 | Ki × 10,000,000 |
| `sw_config` | `kd` | u32 | Kd × 1000 |
| `sw_config` | `hibernate_ms` | u32 | Hibernate timeout in ms |

---

## Safety

- Power circuit isolated from MCU via optocouplers (zero-crossing and TRIAC gate)
- External mechanical thermostat prevents operation in case of overheating
- Software task watchdog (`esp_task_wdt`) on all critical tasks
- Software temperature limit: output cut above 120 °C
- Configurable operation timeout with automatic deep sleep
- Range validation on all `POST /api/config` fields before applying
- Modal confirmation in the web UI before applying new PID constants

---

## License

Licensed under the MIT License. See LICENSE for details.
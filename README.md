# Smart Water

Firmware para controle de temperatura de uma chaleira elétrica baseado em ESP32. Utiliza controle PID com disparo por fase (phase-cut) via TRIAC, sensor NTC para leitura de temperatura e interface web embarcada para monitoramento e configuração em tempo real.

---

## Hardware

| Componente | Detalhe |
|---|---|
| Microcontrolador | ESP32 (4MB flash) |
| Sensor de temperatura | NTC 10kΩ (divisor resistivo, GPIO34 / ADC1_CH6) |
| Controle de potência | TRIAC com disparo por fase (phase-cut) |
| Detecção de zero-crossing | Optoacoplador → GPIO18 (interrupção POSEDGE) |
| Saída de disparo TRIAC | GPIO19 (gate via optoacoplador) |
| Botão físico | GPIO21 (pull-up interno) |
| Proteção de segurança | Termostato mecânico externo + watchdog de software |

O circuito de potência é completamente isolado do microcontrolador por optoacopladores tanto no zero-crossing quanto no disparo do gate do TRIAC.

---

## Funcionamento

### Controle de temperatura

A temperatura é lida a cada 50ms pelo ADC com calibração de curva de linha. O valor passa por um filtro passa-baixa exponencial (`α = 0.01`) antes de alimentar o PID. O coeficiente baixo do filtro garante estabilidade mas implica resposta lenta a mudanças bruscas — adequado para o perfil térmico de uma chaleira.

O controle PID calcula a saída (0–255) a cada 250ms. Esse valor determina o ângulo de fase no qual o TRIAC é disparado dentro do semiciclo de 60Hz (~8333µs). O cálculo de delay é feito com aritmética inteira com multiplicação antes da divisão para evitar perda de precisão:

```
delay_us = cycleTime * (255 - PID) / 255
```

Um guard descarta o ciclo se o momento de disparo já passou (`delay_us ≤ 0`) ou se o valor calculado excede um semiciclo (`delay_us > 8500µs`), prevenindo comportamento indefinido.

### Botão físico

- **Pressionar e segurar**: ativa o aquecimento com o setpoint de Café/Chá
- **Pressão curta + soltar**: troca para o setpoint de Fervura
- **Soltar**: desativa o aquecimento

O tempo de pressão é contado em ticks de 10ms pela `aux_task`.

### Hibernação automática

Após um tempo configurável (padrão: 5 minutos), o sistema desliga a saída de potência, sinaliza erro de aquecimento para parar o controle de fase, aguarda 1 segundo para o webserver servir um último status e entra em deep sleep. O timeout é configurável pela interface web e persistido na NVS.

---

## Arquitetura do firmware

Desenvolvido com ESP-IDF v5.1. Compilado com CMake.

### Tarefas FreeRTOS

| Task | Core | Prioridade | Stack | Função |
|---|---|---|---|---|
| `power_control` | 0 | 5 | 4096 | Aguarda notificação do ISR, calcula delay de fase, agenda disparo do TRIAC via `esp_timer` |
| `temperature` | 0 | 5 | 4096 | Leitura ADC → cálculo NTC → filtro → atualiza `waterTemp` |
| `logic_control` | 0 | 3 | 4096 | Debounce do botão, seleção de setpoint, cálculo PID |
| `aux` | 1 | 1 | 4096 | Contador do botão, watchdog de hibernação |
| `telemetry` | 0 | 2 | 3072 | Amostra estado a cada 1s e alimenta buffer de histórico do webserver |

A ISR de zero-crossing só é habilitada após `power_control_task` atribuir seu próprio handle (`powerTaskHandle`), evitando notificação para handle nulo durante o boot.

### Módulos

```
main/
├── main.c              — Tasks de controle, ISR, GPIO, ADC, app_main
├── wifi_manager.c/h    — STA com fallback para AP, mDNS (smartwater.local)
├── webserver.c/h       — HTTP server, rotas REST, buffer de histórico, OTA
└── CMakeLists.txt

spiffs_data/
├── index.html          — Interface web (SPA)
└── chart.umd.min.js    — Chart.js 4.4.1 (local, sem dependência de CDN)

partitions.csv          — Tabela customizada com duas partições OTA + SPIFFS
```

### Tabela de partições (4MB)

| Nome | Tipo | Offset | Tamanho |
|---|---|---|---|
| nvs | data/nvs | 0x9000 | 24KB |
| otadata | data/ota | 0xF000 | 8KB |
| app0 (ota_0) | app/ota_0 | 0x20000 | 1.5MB |
| app1 (ota_1) | app/ota_1 | 0x1A0000 | 1.5MB |
| spiffs | data/spiffs | 0x320000 | 896KB |

---

## Interface web

Acessível por `http://smartwater.local` no modo STA ou `http://192.168.4.1` no modo AP.

### Funcionalidades

- Gráfico de temperatura em tempo real (janela de 5 minutos, 300 pontos)
- Histórico restaurado ao recarregar a página via `GET /api/history`
- Setpoints configuráveis: Café/Chá (50–90°C) e Fervura (90–115°C)
- Barra de saída PID em tempo real
- Contador de hibernação no header com alerta visual abaixo de 2 minutos
- Configuração de rede WiFi com reboot automático
- Painel de manutenção recolhível com:
  - Constantes PID (Kp, Ki, Kd) com confirmação modal antes de aplicar
  - Timeout de hibernação (1–120 minutos)
  - Upload de firmware OTA (arrastar e soltar `.bin`)
  - Informações do sistema (IP, heap livre, RSSI, versão do firmware)

### API REST

| Método | Rota | Descrição |
|---|---|---|
| GET | `/` | Serve `index.html` do SPIFFS |
| GET | `/chart.umd.min.js` | Serve Chart.js do SPIFFS (cache 1 dia) |
| GET | `/api/status` | JSON com estado atual do sistema |
| GET | `/api/history` | JSON com buffer circular de temperatura |
| GET | `/api/config` | JSON com configurações atuais |
| POST | `/api/config` | Atualiza setpoints, PID e/ou timeout de hibernação |
| POST | `/api/wifi` | Salva credenciais WiFi na NVS e reinicia |
| POST | `/api/ota` | Recebe `.bin` como `application/octet-stream` e grava via OTA |

---

## WiFi

O `wifi_manager` tenta conectar às credenciais salvas na NVS ao boot. Se não houver credenciais ou a conexão falhar após 5 tentativas (timeout total de 15s), sobe um Access Point aberto com SSID `SmartWater-Setup`. Em ambos os modos o webserver está disponível com a mesma interface.

No modo STA, o mDNS anuncia `smartwater.local` na rede local, eliminando a necessidade de conhecer o IP dinâmico.

---

## OTA

O firmware suporta atualização over-the-air via upload de `.bin` pela interface web. O arquivo é enviado como binário puro (`Content-Type: application/octet-stream`) e gravado em chunks de 1KB na partição OTA inativa. Após gravação bem-sucedida, o ESP reinicia na nova partição.

O rollback automático do ESP-IDF está ativo. `esp_ota_mark_app_valid_cancel_rollback()` é chamado no `app_main` logo após as tasks de controle subirem, antes do WiFi. Se o firmware travar antes dessa chamada, o ESP-IDF reverte automaticamente para a partição anterior no próximo boot.

A partição SPIFFS (interface web) **não** é atualizada via OTA — apenas o firmware. Para atualizar o HTML:

```bash
idf.py spiffs-flash
```

---

## Build e flash

### Pré-requisitos

- ESP-IDF v5.1
- Python 3.9+

### Compilar e gravar (primeira vez)

```bash
idf.py set-target esp32
idf.py menuconfig   # confirmar: tabela customizada, offset 0x8000
idf.py erase-flash
idf.py flash monitor
```

O `erase-flash` é obrigatório ao trocar a tabela de partições. Apaga NVS — as credenciais WiFi precisarão ser reconfiguradas pela interface web.

### Atualização posterior

Via OTA pela interface web (firmware) ou:

```bash
idf.py flash          # firmware
idf.py spiffs-flash   # interface web
```

### Definir versão do firmware

No `CMakeLists.txt` raiz:

```cmake
set(PROJECT_VER "1.0.0")
```

A versão aparece na interface web em Informações do Sistema.

---

## Configurações persistidas na NVS

| Namespace | Chave | Tipo | Descrição |
|---|---|---|---|
| `sw_wifi` | `ssid` | string | SSID da rede WiFi |
| `sw_wifi` | `password` | string | Senha da rede WiFi |
| `sw_config` | `coffee_sp` | u32 | Setpoint café × 100 |
| `sw_config` | `boiling_sp` | u32 | Setpoint fervura × 100 |
| `sw_config` | `kp` | u32 | Kp × 1000 |
| `sw_config` | `ki` | u32 | Ki × 10000000 |
| `sw_config` | `kd` | u32 | Kd × 1000 |
| `sw_config` | `hibernate_ms` | u32 | Timeout de hibernação em ms |

---

## Segurança

- Circuito de potência isolado por optoacopladores
- Termostato mecânico externo inibe operação em caso de superaquecimento
- Watchdog de software por task (`esp_task_wdt`) em todas as tasks críticas
- Limite de temperatura por software: desliga em > 120°C
- Timeout de operação configurável com hibernação automática por deep sleep
- Validação de range em todos os campos do `POST /api/config` antes de aplicar
- Confirmação modal na interface antes de aplicar novas constantes PID

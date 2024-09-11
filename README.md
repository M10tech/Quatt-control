the README

## version history

### 0.1.4 ota_string, ota-api functions and MQTT return
- parse ota_string for MQTT config
- ota-api functions now ESP ported
- MQTT return value now works
- names of accessories updated
- included remaining timer call items

### 0.1.3 moved ota-api into lcm-api
- lcm-api is a component (fork)
- added TEMP2HK macro

### 0.1.2 added time from SNTP
- and fixed unterminated string in esp32-homekit upstream

### 0.1.1 remove time dependency
- while time reference not solved

### 0.1.0 added homekit outline
- seems to deliver working code
- a lot of fine-tuning pending
- TODO time, RTC, ota_config, ota_api

### 0.0.10 finally fix stuck ds18b20 sensor
- just telling the 1-wire-pin to ground does not actually work
- using a second pin as open-drain to become ground terminal for 1-wire
- when CRC error found, make this float for two seconds and sensor unstuck
- start reporting to domoticz for real

### 0.0.9 introduced MQTT and SINKBUS
- skeleton MQTT code, needs refinement
- SINKBUS puts one-wire bus down for 9 seconds
- to battle the stuck ds18b20 issue

### 0.0.8 get rid of ESP_ERROR_CHECK
- revert previous attempts as unneeded
- This is bad programming as admitted by espressif
- "Many ESP-IDF examples use ESP_ERROR_CHECK to handle errors from various APIs.
  This is not the best practice for applications, and is done to make example code more concise"
- reset bus on bad readings
- remove console colors

### 0.0.7 further stability improvement attempt
- pin temperature task to core 1 instead of 0
- v0.0.5 was pinning to available core

### 0.0.6 further stability improvement attempt
- pin temperature task to core 0 instead of 1

### 0.0.5 stability improvement attempt
- version 0.0.4 crashes usually within 1000s
- version 0.0.3 was rock solid
- fixed the sensor_count not reset at retry

### 0.0.4 Added ds18b20 sensor reading
- run it every second but cycle through one sensor at a time
- driver does not support ds18b20_any broadcast conversion trigger

### 0.0.3 Not published
- fixed OT receiver being unreliable
- essential to have GPIO interrupt pinned to core 1
- skeleton timer loop

### 0.0.2 Added OT receiver
- works for two channels
- changed pin numbers

### 0.0.1 Send_OT_frame
- takes one uint32_t as payload
- works for two TX channels
- blocks for 50ms


the README

## version history

### 0.4.7 cleanup temperature measurement timing
- original was not well aligned in time
- grouped temperature activities together in code
- initialization values must be multiple of 8/256
- better S3 initialization

### 0.4.6 align float operations with 1/256 granularity
- prevents rounding in values sent to opentherm
- factor = zero now means no restriction
- default factor now 0.3
- max factor 1 degree

### 0.4.5 gradual setpoint tracking
- repurpose factor to define tracking delta
- update OT room_temp only once per minute instead of every 10s
- OT CH setpoint now zero when off and 35 deg when on

### 0.4.4 reset some GPIO ports to input in lcm-api
- todo: reset all relevant ports
- change UDPLGP to UDPLUS

### 0.4.3 changed maximum thermostat setpoint
- makes it easier to select a temperature
- range now 10 - 25 degrees celcius
- changed factor to become the setpoint temperature of the heatpump

### 0.4.2 added ping-guard
- will reset if no ping for 5 minutes
- added hysteresis values to heater outcome logic

### 0.4.1 need harder reset of DS18B20 sensors
- 2 seconds is not enough, make it 7s
- needs different approach to collect temperatures

### 0.4.0 fixed storage of pairing in esp32-homekit
- storage_add_pairing was overwriting new data with stored data
- since pairing_t has a string, we need to add closing zero
- when remove_pairing, scan all slots for multiple copies due to issues above
- use better logging of pairing records
- when unpair, reset homekit storage and reset device

### 0.3.5 publish heatpump temperatures to domoticz
- suppress MQTT publish reports
- report line cleanup

### 0.3.4 non double negative logic doesn't confuse you
- delta is negative when heating is required
- clean up reporting line

### 0.3.3 basic heater algorithm
- algorithm uses setpoint based on up and down roomtemp
- changed default temp to 20 degrees

### 0.3.2 one more glitch in heat_on value
- when trigger missed, retrigger part needed update also
- normalised some OpenTherm commands

### 0.3.1 fixed bad heat_on value
- also activated when heater=off and PumpOffTimer confirmed

### 0.3.0 change PumpOffTime rules
- POT when heat_result==off or upstairs
- so pump only works when heat_result==downstairs

### 0.2.7 restore timeout
- error rate for CH1_NO_RSP was not improved @0.7%
- crashes went from zero in 200000 to 8 in 80000 seconds

### 0.2.6 adjust timeout more
- still CH1_NO_RSP to be fixed
- average temperature initialization

### 0.2.5 adjust timeout and report more
- there is like 1% CH1_NO_RSP to be fixed
- allow less time for boiler and more for Quatt
- report 4 items for heatpump

### 0.2.4 swap channels again

### 0.2.3 brackets my friend
- in a macro, ALWAYS put the input between brackets!

### 0.2.2 OT float conversion and mode correction
- conversion of float to OT f8.8 format fixed
- made that a macro
- tgt_heat2 start value should be HEAT

### 0.2.1 heatpump OT commands and test 2nd channel
- based on interaction between tado and Quatt
- assigned boiler to other hardware channel to test stability
- initialize temperature reading buffers more consistently

### 0.2.0 conversion of dual-heater-opentherm complete
- ported this from ESP8266 code to my ESP32 code
- ready for new adventures
- pending: RTC memory, current code seems rock solid as it is
- pending: pinger, add later, if at all

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


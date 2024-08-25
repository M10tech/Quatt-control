the README

## version history

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


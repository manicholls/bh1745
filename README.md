# bh1745
esphome module for bh1745 light sensor

Tested on ESPHome 10.3 using a Pimoroni Enviro Indoor.

Add the following to your YAML:

```yaml
external_components:
  - source: github://manicholls/bh1745@main
    components: [bh1745]

i2c:
  - id: bus_a   # RTC + QWST + BME688 + BH1745
    sda: 4
    scl: 5
    scan: true

bh1745:
  id: bh1745_base               # <-- Unique ID for the component instance
  address: 0x38                 # I2C Address (0x38 or 0x39)
  update_interval: 15s           # Polling interval
  integration_time: 320ms       # 160ms, 320ms, 640ms, 1280ms
  gain: 32x                     # 1x or 32x

sensor:
  - platform: bh1745
    id: bh1745_base
    illuminance:
      name: "Ambient Lux"
      unit_of_measurement: "lx"
      accuracy_decimals: 2
    red:
      name: "Ambient Red Channel"
    green:
      name: "Ambient Green Channel"
    blue:
      name: "Ambient Blue Channel"
```


Full configuration for an Enviro Indoor can be found in my esphome repo.

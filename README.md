# qmc5883p — ESPHome external component

ESPHome sensor for **QMC5883P/QMC5883L** 3-axis magnetometers (the common “HMC5883L” clones).  
Reads XYZ, applies optional **axes_map / rotation / offsets / matrix**, publishes in **µT**.

## Features
- Continuous-mode reads with stable defaults (OSR=512, RNG=2G, ODR=50 Hz).
- Axis remap/invert, 0/90/180/270° rotation, hard-iron offsets, soft-iron 3×3 matrix.
- Exposes `field_strength_x/y/z` sensors for downstream heading math.

## Wiring
- **VCC 3.3 V**, **GND**, **SDA/SCL** → ESP32.  
- Use safe I²C pins (e.g. 16/17, 32/33). Tie grounds together. Keep >10 cm from motors/ESC wiring.

## Install
Project layout:
esphome/
├─ your.yaml
└─ my_components/
└─ qmc5883p/
├─ init.py
├─ sensor.py
├─ qmc5883p.h
└─ qmc5883p.cpp


Enable in YAML:
```yaml
external_components:
  - source:
      type: local
      path: ./my_components
    components: [qmc5883p]

i2c:
  id: bus_a
  sda: 16
  scl: 17
  scan: true

sensor:
  - platform: qmc5883p
    i2c_id: bus_a
    address: 0x2C        # or 0x0D, scan first
    id: magnetometer
    update_interval: 1000ms
    axes_map: [x, y, z]  # x|y|z with optional '-' to invert
    rotation: 0          # 0|90|180|270
    # offsets: [x_off, y_off, z_off]
    # matrix:  [1,0,0, 0,1,0, 0,0,1]

    field_strength_x: { id: mag_x, internal: true }
    field_strength_y: { id: mag_y, internal: true }
    field_strength_z: { id: mag_z, internal: true }

sensor:
  - platform: template
    name: "Heading (mag)"
    unit_of_measurement: "°"
    accuracy_decimals: 1
    update_interval: 1s
    lambda: |-
      float x = id(mag_x).state, y = id(mag_y).state;
      if (isnan(x) || isnan(y)) return NAN;
      float h = atan2f(y, x) * 180.0f / M_PI;
      if (h < 0) h += 360.0f;
      // optional: h += <declination_deg>; wrap to 0..360
      return h;

Calibration (hard-iron offsets)

Temporarily enable web UI for easy OTA/values:
web_server:
  port: 80
  ota: true
  auth: { username: admin, password: change-me }

Rotate the sensor slowly through 360° and record min/max of mag_x and mag_y.
Compute: x_off = -(x_max + x_min)/2, y_off = -(y_max + y_min)/2.

Add to the sensor block:

offsets: [<x_off>, <y_off>, 0]

Tip: If N/E/S/W don’t read ~0/90/180/270 after offsets, fix orientation via axes_map first: try [y, -x, z] or [-y, x, z] (keep rotation: 0). Use update_interval: 1000–1500ms.

Soft-iron matrix (optional)

If your plot of (X,Y) is an ellipse, solve a 2D ellipsoid fit and place the 3×3 row-major matrix in matrix:.

Troubleshooting

Skewed 90°/270° → wrong axes_map/rotation.

Random jumps → uncalibrated offsets or magnetic noise; slow update_interval.

I²C errors/reboots → check wiring/pull-ups, move away from noisy power, reduce template rates.

License

MIT (same as ESPHome components).


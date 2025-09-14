#pragma once
#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace qmc5883p {

// QMC5883P variant register map:
//  CHIPID:    0x00 (expect 0x80)
//  DATA:      0x01..0x06  (X_L, X_H, Y_L, Y_H, Z_L, Z_H)  LSB-first
//  STATUS:    0x09 (bit0=DRDY)
//  CONTROL_1: 0x0A (MODE/ODR/OSR)
//  CONTROL_2: 0x0B (SOFT_RST/RNG/SR)
//  AXIS_SIGN: 0x29 (optional axis/sign config)
class QMC5883PSensor : public PollingComponent, public i2c::I2CDevice {
 public:
  QMC5883PSensor() = default;

  // YAML wiring
  void set_output_sensors(sensor::Sensor *xs, sensor::Sensor *ys, sensor::Sensor *zs) { x_=xs; y_=ys; z_=zs; }
  void set_field_sensor(sensor::Sensor *fs) { field_ = fs; }

  // Calibration
  void set_offsets(float ox, float oy, float oz) { off_x_=ox; off_y_=oy; off_z_=oz; }
  void set_matrix(float a11,float a12,float a13, float a21,float a22,float a23, float a31,float a32,float a33) {
    m11_=a11; m12_=a12; m13_=a13; m21_=a21; m22_=a22; m23_=a23; m31_=a31; m32_=a32; m33_=a33;
  }

  // Orientation
  void set_rotation(int r) { rotation_ = r; }              // 0/90/180/270 (deg) about +Z after mapping
  void set_axes_map(int8_t a, int8_t b, int8_t c) {        // codes: ±1=X, ±2=Y, ±3=Z
    axes_map_[0]=a; axes_map_[1]=b; axes_map_[2]=c;
  }

  // Component
  void setup() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  bool init_();
  bool read_xyz_raw_(int16_t &x, int16_t &y, int16_t &z);       // raw from registers (no calibration)
  static bool wait_drdy_(i2c::I2CDevice *dev, uint32_t ms_timeout);

  bool ok_{false};
  uint8_t bad_reads_{0};

  // Outputs
  sensor::Sensor *x_{nullptr};
  sensor::Sensor *y_{nullptr};
  sensor::Sensor *z_{nullptr};
  sensor::Sensor *field_{nullptr};

  // Calibration
  float off_x_{0}, off_y_{0}, off_z_{0};
  float m11_{1}, m12_{0}, m13_{0};
  float m21_{0}, m22_{1}, m23_{0};
  float m31_{0}, m32_{0}, m33_{1};

  // Orientation
  int rotation_{0};                  // 0 | 90 | 180 | 270 (deg)
  int8_t axes_map_[3] { +1, +2, +3 }; // +X,+Y,+Z  (codes: ±1=X, ±2=Y, ±3=Z)

  // QMC5883P registers
  static constexpr uint8_t REG_CHIPID    = 0x00;
  static constexpr uint8_t REG_X_L       = 0x01; // burst 6 bytes to Z_H
  static constexpr uint8_t REG_STATUS    = 0x09; // DRDY bit0
  static constexpr uint8_t REG_CONTROL_1 = 0x0A; // MODE/ODR/OSR
  static constexpr uint8_t REG_CONTROL_2 = 0x0B; // SOFT_RST/RNG/SR
  static constexpr uint8_t REG_AXIS_SIGN = 0x29; // optional
};

}  // namespace qmc5883p
}  // namespace esphome

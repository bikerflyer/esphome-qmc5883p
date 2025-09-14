#include "qmc5883p.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <cmath>

namespace esphome {
namespace qmc5883p {

static const char *const TAG = "qmc5883p";

bool QMC5883PSensor::wait_drdy_(i2c::I2CDevice *dev, uint32_t ms_timeout) {
  uint8_t s = 0; uint32_t t0 = millis();
  while (millis() - t0 < ms_timeout) {
    if (!dev->read_bytes(REG_STATUS, &s, 1)) return false;
    if (s & 0x01) return true;
    delay(4);
  }
  return false;
}

void QMC5883PSensor::setup() {
  ok_ = init_();
  if (!ok_) ESP_LOGW(TAG, "Init failed (addr=0x%02X)", this->get_i2c_address());
}

bool QMC5883PSensor::init_() {
  // CONTROL_2: SOFT_RST
  this->write_byte(REG_CONTROL_2, 0x80);
  delay(10);

  // Optional: board reference example sets 0x29=0x06 (axis/sign)
  this->write_byte(REG_AXIS_SIGN, 0x06);
  delay(1);

  // CONTROL_2: Set/Reset ON + 8G range (some boards read back 0x00; OK)
  this->write_byte(REG_CONTROL_2, 0x08);
  delay(2);

  // CONTROL_1: continuous mode, modest ODR/OSR for stability
  // 0xC3 = OSR2=3, OSR1=0, ODR=10Hz, MODE=continuous
  if (!this->write_byte(REG_CONTROL_1, 0xC3)) return false;
  delay(10);

  // Prefer DRDY, but don't fail the init if it doesn't assert
  (void) wait_drdy_(this, 60);

  int16_t x,y,z;
  if (this->read_xyz_raw_(x,y,z)) {
    ESP_LOGI(TAG, "Init OK: first XYZ=(%d,%d,%d)", x,y,z);
    return true;
  } else {
    ESP_LOGW(TAG, "First read failed; will retry in update()");
    // still consider init OK so update() can keep trying
    return true;
  }
}

bool QMC5883PSensor::read_xyz_raw_(int16_t &x, int16_t &y, int16_t &z) {
  uint8_t buf[6] = {0};
  if (!this->read_bytes(REG_X_L, buf, 6)) return false;  // 0x01..0x06

  auto u16 = [&](int lo, int hi)->int16_t {
    return (int16_t)((uint16_t(buf[hi]) << 8) | uint16_t(buf[lo]));  // LSB first
  };
  x = u16(0,1);
  y = u16(2,3);
  z = u16(4,5);

  // Sanity: reject all-zeros or saturated extremes
  if ((x==0 && y==0 && z==0) ||
      (x==0x7FFF || y==0x7FFF || z==0x7FFF) ||
      (x==-32768 || y==-32768 || z==-32768))
    return false;

  return true;
}

void QMC5883PSensor::update() {
  if (!ok_) {
    static uint8_t retry=0;
    if (++retry % 10 == 0) { ESP_LOGW(TAG,"Re-attempting init..."); ok_=init_(); }
    return;
  }

  int16_t rx,ry,rz;
  if (!read_xyz_raw_(rx,ry,rz)) {
    if (++bad_reads_ >= 5) { bad_reads_=0; ok_=false; ESP_LOGW(TAG,"Repeated bad reads; will reinit"); }
    return;
  }
  bad_reads_ = 0;

  // Hard-iron offsets
  float fx = (float)rx - off_x_;
  float fy = (float)ry - off_y_;
  float fz = (float)rz - off_z_;

  // Soft-iron 3x3
  float cx = m11_*fx + m12_*fy + m13_*fz;
  float cy = m21_*fx + m22_*fy + m23_*fz;
  float cz = m31_*fx + m32_*fy + m33_*fz;

  // Axes map (swap/flip before yaw rotation)
  auto pick = [&](int8_t code)->float {
    float v = 0.0f; int a = std::abs(code);
    switch (a) {
      case 1: v = cx; break;  // X
      case 2: v = cy; break;  // Y
      case 3: v = cz; break;  // Z
      default: v = 0.0f; break;
    }
    return (code < 0) ? -v : v;
  };
  float mx = pick(axes_map_[0]);  // mapped X
  float my = pick(axes_map_[1]);  // mapped Y
  float mz = pick(axes_map_[2]);  // mapped Z

  // Optional rotation around +Z (post-mapping)
  float tx = mx, ty = my, tz = mz;
  switch (rotation_) {
    case 90:  tx =  my; ty = -mx; tz = mz; break;
    case 180: tx = -mx; ty = -my; tz = mz; break;
    case 270: tx = -my; ty =  mx; tz = mz; break;
    default: break;
  }

  if (x_) x_->publish_state(tx);
  if (y_) y_->publish_state(ty);
  if (z_) z_->publish_state(tz);

  if (field_) {
    float mag = sqrtf(tx*tx + ty*ty + tz*tz);
    field_->publish_state(mag);
  }
}

}  // namespace qmc5883p
}  // namespace esphome

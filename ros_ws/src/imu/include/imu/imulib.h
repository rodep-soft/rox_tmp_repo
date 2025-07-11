/*!
 * @file BNO055.h
 *
 *  @mainpage BNO055 Orientation Sensor Library
 *
 *  @section intro_sec Introduction
 *
 *  This is a C++ library for the BNO055 orientation sensor using boost::asio
 *
 *  Originally based on Adafruit BNO055 library, converted to standard C++
 *
 *  @section license License
 *
 *  MIT license, all text above must be included in any redistribution
 */

#ifndef BNO055_H
#define BNO055_H

#include <boost/asio.hpp>
#include <cstdint>
#include <vector>
#include <memory>
#include <chrono>
#include <thread>

// BNO055 I2C addresses
#define BNO055_ADDRESS_A (0x28)
#define BNO055_ADDRESS_B (0x29)
#define BNO055_ID (0xA0)

// BNO055 Register addresses
typedef enum {
  // Page id register definition
  BNO055_PAGE_ID_ADDR = 0X07,

  // PAGE0 REGISTER DEFINITION START
  BNO055_CHIP_ID_ADDR = 0x00,
  BNO055_ACCEL_REV_ID_ADDR = 0x01,
  BNO055_MAG_REV_ID_ADDR = 0x02,
  BNO055_GYRO_REV_ID_ADDR = 0x03,
  BNO055_SW_REV_ID_LSB_ADDR = 0x04,
  BNO055_SW_REV_ID_MSB_ADDR = 0x05,
  BNO055_BL_REV_ID_ADDR = 0X06,

  // Accel data register
  BNO055_ACCEL_DATA_X_LSB_ADDR = 0X08,
  BNO055_ACCEL_DATA_X_MSB_ADDR = 0X09,
  BNO055_ACCEL_DATA_Y_LSB_ADDR = 0X0A,
  BNO055_ACCEL_DATA_Y_MSB_ADDR = 0X0B,
  BNO055_ACCEL_DATA_Z_LSB_ADDR = 0X0C,
  BNO055_ACCEL_DATA_Z_MSB_ADDR = 0X0D,

  // Mag data register
  BNO055_MAG_DATA_X_LSB_ADDR = 0X0E,
  BNO055_MAG_DATA_X_MSB_ADDR = 0X0F,
  BNO055_MAG_DATA_Y_LSB_ADDR = 0X10,
  BNO055_MAG_DATA_Y_MSB_ADDR = 0X11,
  BNO055_MAG_DATA_Z_LSB_ADDR = 0X12,
  BNO055_MAG_DATA_Z_MSB_ADDR = 0X13,

  // Gyro data registers
  BNO055_GYRO_DATA_X_LSB_ADDR = 0X14,
  BNO055_GYRO_DATA_X_MSB_ADDR = 0X15,
  BNO055_GYRO_DATA_Y_LSB_ADDR = 0X16,
  BNO055_GYRO_DATA_Y_MSB_ADDR = 0X17,
  BNO055_GYRO_DATA_Z_LSB_ADDR = 0X18,
  BNO055_GYRO_DATA_Z_MSB_ADDR = 0X19,

  // Euler data registers
  BNO055_EULER_H_LSB_ADDR = 0X1A,
  BNO055_EULER_H_MSB_ADDR = 0X1B,
  BNO055_EULER_R_LSB_ADDR = 0X1C,
  BNO055_EULER_R_MSB_ADDR = 0X1D,
  BNO055_EULER_P_LSB_ADDR = 0X1E,
  BNO055_EULER_P_MSB_ADDR = 0X1F,

  // Quaternion data registers
  BNO055_QUATERNION_DATA_W_LSB_ADDR = 0X20,
  BNO055_QUATERNION_DATA_W_MSB_ADDR = 0X21,
  BNO055_QUATERNION_DATA_X_LSB_ADDR = 0X22,
  BNO055_QUATERNION_DATA_X_MSB_ADDR = 0X23,
  BNO055_QUATERNION_DATA_Y_LSB_ADDR = 0X24,
  BNO055_QUATERNION_DATA_Y_MSB_ADDR = 0X25,
  BNO055_QUATERNION_DATA_Z_LSB_ADDR = 0X26,
  BNO055_QUATERNION_DATA_Z_MSB_ADDR = 0X27,

  // Linear acceleration data registers
  BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR = 0X28,
  BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR = 0X29,
  BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR = 0X2A,
  BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR = 0X2B,
  BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR = 0X2C,
  BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR = 0X2D,

  // Gravity data registers
  BNO055_GRAVITY_DATA_X_LSB_ADDR = 0X2E,
  BNO055_GRAVITY_DATA_X_MSB_ADDR = 0X2F,
  BNO055_GRAVITY_DATA_Y_LSB_ADDR = 0X30,
  BNO055_GRAVITY_DATA_Y_MSB_ADDR = 0X31,
  BNO055_GRAVITY_DATA_Z_LSB_ADDR = 0X32,
  BNO055_GRAVITY_DATA_Z_MSB_ADDR = 0X33,

  // Temperature data register
  BNO055_TEMP_ADDR = 0X34,

  // Status registers
  BNO055_CALIB_STAT_ADDR = 0X35,
  BNO055_SELFTEST_RESULT_ADDR = 0X36,
  BNO055_INTR_STAT_ADDR = 0X37,

  BNO055_SYS_CLK_STAT_ADDR = 0X38,
  BNO055_SYS_STAT_ADDR = 0X39,
  BNO055_SYS_ERR_ADDR = 0X3A,

  // Unit selection register
  BNO055_UNIT_SEL_ADDR = 0X3B,
  BNO055_DATA_SELECT_ADDR = 0X3C,

  // Mode registers
  BNO055_OPR_MODE_ADDR = 0X3D,
  BNO055_PWR_MODE_ADDR = 0X3E,

  BNO055_SYS_TRIGGER_ADDR = 0X3F,
  BNO055_TEMP_SOURCE_ADDR = 0X40,

  // Axis remap registers
  BNO055_AXIS_MAP_CONFIG_ADDR = 0X41,
  BNO055_AXIS_MAP_SIGN_ADDR = 0X42,

  // Offset registers
  ACCEL_OFFSET_X_LSB_ADDR = 0X55,
  ACCEL_OFFSET_X_MSB_ADDR = 0X56,
  ACCEL_OFFSET_Y_LSB_ADDR = 0X57,
  ACCEL_OFFSET_Y_MSB_ADDR = 0X58,
  ACCEL_OFFSET_Z_LSB_ADDR = 0X59,
  ACCEL_OFFSET_Z_MSB_ADDR = 0X5A,

  MAG_OFFSET_X_LSB_ADDR = 0X5B,
  MAG_OFFSET_X_MSB_ADDR = 0X5C,
  MAG_OFFSET_Y_LSB_ADDR = 0X5D,
  MAG_OFFSET_Y_MSB_ADDR = 0X5E,
  MAG_OFFSET_Z_LSB_ADDR = 0X5F,
  MAG_OFFSET_Z_MSB_ADDR = 0X60,

  GYRO_OFFSET_X_LSB_ADDR = 0X61,
  GYRO_OFFSET_X_MSB_ADDR = 0X62,
  GYRO_OFFSET_Y_LSB_ADDR = 0X63,
  GYRO_OFFSET_Y_MSB_ADDR = 0X64,
  GYRO_OFFSET_Z_LSB_ADDR = 0X65,
  GYRO_OFFSET_Z_MSB_ADDR = 0X66,

  ACCEL_RADIUS_LSB_ADDR = 0X67,
  ACCEL_RADIUS_MSB_ADDR = 0X68,
  MAG_RADIUS_LSB_ADDR = 0X69,
  MAG_RADIUS_MSB_ADDR = 0X6A
} bno055_reg_t;

// Power modes
typedef enum {
  POWER_MODE_NORMAL = 0X00,
  POWER_MODE_LOWPOWER = 0X01,
  POWER_MODE_SUSPEND = 0X02
} power_mode_t;

// Operation modes
typedef enum {
  OPERATION_MODE_CONFIG = 0X00,
  OPERATION_MODE_ACCONLY = 0X01,
  OPERATION_MODE_MAGONLY = 0X02,
  OPERATION_MODE_GYRONLY = 0X03,
  OPERATION_MODE_ACCMAG = 0X04,
  OPERATION_MODE_ACCGYRO = 0X05,
  OPERATION_MODE_MAGGYRO = 0X06,
  OPERATION_MODE_AMG = 0X07,
  OPERATION_MODE_IMUPLUS = 0X08,
  OPERATION_MODE_COMPASS = 0X09,
  OPERATION_MODE_M4G = 0X0A,
  OPERATION_MODE_NDOF_FMC_OFF = 0X0B,
  OPERATION_MODE_NDOF = 0X0C
} opmode_t;

// Axis remap configurations
typedef enum {
  REMAP_CONFIG_P0 = 0x21,
  REMAP_CONFIG_P1 = 0x24, // default
  REMAP_CONFIG_P2 = 0x24,
  REMAP_CONFIG_P3 = 0x21,
  REMAP_CONFIG_P4 = 0x24,
  REMAP_CONFIG_P5 = 0x21,
  REMAP_CONFIG_P6 = 0x21,
  REMAP_CONFIG_P7 = 0x24
} axis_remap_config_t;

// Axis remap signs
typedef enum {
  REMAP_SIGN_P0 = 0x04,
  REMAP_SIGN_P1 = 0x00, // default
  REMAP_SIGN_P2 = 0x06,
  REMAP_SIGN_P3 = 0x02,
  REMAP_SIGN_P4 = 0x03,
  REMAP_SIGN_P5 = 0x01,
  REMAP_SIGN_P6 = 0x07,
  REMAP_SIGN_P7 = 0x05
} axis_remap_sign_t;

// Vector types
typedef enum {
  VECTOR_ACCELEROMETER = BNO055_ACCEL_DATA_X_LSB_ADDR,
  VECTOR_MAGNETOMETER = BNO055_MAG_DATA_X_LSB_ADDR,
  VECTOR_GYROSCOPE = BNO055_GYRO_DATA_X_LSB_ADDR,
  VECTOR_EULER = BNO055_EULER_H_LSB_ADDR,
  VECTOR_LINEARACCEL = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR,
  VECTOR_GRAVITY = BNO055_GRAVITY_DATA_X_LSB_ADDR
} vector_type_t;

#define NUM_BNO055_OFFSET_REGISTERS (22)

// Vector class for 3D data
template <int N>
class Vector {
public:
  Vector() {
    for (int i = 0; i < N; i++) {
      data[i] = 0.0;
    }
  }
  
  Vector(double x, double y, double z) {
    if (N >= 3) {
      data[0] = x;
      data[1] = y;
      data[2] = z;
    }
  }
  
  double& operator[](int index) { return data[index]; }
  const double& operator[](int index) const { return data[index]; }
  
  double x() const { return data[0]; }
  double y() const { return data[1]; }
  double z() const { return data[2]; }
  
private:
  double data[N];
};

// Quaternion class
class Quaternion {
public:
  Quaternion() : w_(0), x_(0), y_(0), z_(0) {}
  Quaternion(double w, double x, double y, double z) : w_(w), x_(x), y_(y), z_(z) {}
  
  double w() const { return w_; }
  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }
  
private:
  double w_, x_, y_, z_;
};

// Revision info structure
struct rev_info_t {
  uint8_t accel_rev;
  uint8_t mag_rev;
  uint8_t gyro_rev;
  uint16_t sw_rev;
  uint8_t bl_rev;
};

// Offset structure
struct offsets_t {
  int16_t accel_offset_x;
  int16_t accel_offset_y;
  int16_t accel_offset_z;
  
  int16_t mag_offset_x;
  int16_t mag_offset_y;
  int16_t mag_offset_z;
  
  int16_t gyro_offset_x;
  int16_t gyro_offset_y;
  int16_t gyro_offset_z;
  
  int16_t accel_radius;
  int16_t mag_radius;
};

// Main BNO055 class
class BNO055 {
public:
  BNO055(int32_t sensorID = -1, uint8_t address = BNO055_ADDRESS_A, 
         const std::string& i2c_device = "/dev/i2c-1");
  ~BNO055();
  
  bool begin(opmode_t mode = OPERATION_MODE_NDOF);
  void setMode(opmode_t mode);
  opmode_t getMode();
  
  void setAxisRemap(axis_remap_config_t remapcode);
  void setAxisSign(axis_remap_sign_t remapsign);
  void setExtCrystalUse(bool usextal);
  
  void getSystemStatus(uint8_t* system_status, uint8_t* self_test_result, uint8_t* system_error);
  void getRevInfo(rev_info_t* info);
  void getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag);
  
  int8_t getTemp();
  Vector<3> getVector(vector_type_t vector_type);
  Quaternion getQuat();
  
  bool getSensorOffsets(uint8_t* calibData);
  bool getSensorOffsets(offsets_t& offsets_type);
  void setSensorOffsets(const uint8_t* calibData);
  void setSensorOffsets(const offsets_t& offsets_type);
  
  bool isFullyCalibrated();
  void enterSuspendMode();
  void enterNormalMode();

private:
  bool write8(bno055_reg_t reg, uint8_t value);
  uint8_t read8(bno055_reg_t reg);
  bool readLen(bno055_reg_t reg, uint8_t* buffer, uint8_t len);
  
  void delay(int ms);
  
  int32_t _sensorID;
  opmode_t _mode;
  uint8_t _address;
  std::string _i2c_device;
  int _i2c_fd;
};

#endif // BNO055_H

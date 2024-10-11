/* MPU6050_light library for Arduino
 *
 * Authors: Romain JL. Fétick (github.com/rfetick)
 *              simplifications and corrections
 *          Tockn (github.com/tockn)
 *              initial author (v1.5.2)
 */

#include "MPU6050_light.h"
#include "Arduino.h"

int chip_type = CHIP_MPU6050;

/* Wrap an angle in the range [-limit,+limit] (special thanks to Edgar Bonet!) */
static float wrap(float angle, float limit)
{
  while (angle > limit)
    angle -= 2 * limit;
  while (angle < -limit)
    angle += 2 * limit;
  return angle;
}

/* INIT and BASIC FUNCTIONS */

MPU6050::MPU6050(TwoWire &w)
{
  wire = &w;
  setFilterGyroCoef(DEFAULT_GYRO_COEFF);
  setGyroOffsets(0, 0, 0);
  setAccOffsets(0, 0, 0);
}

byte MPU6050::begin(int gyro_config_num, int acc_config_num)
{
  // changed calling register sequence [https://github.com/rfetick/MPU6050_light/issues/1] -> thanks to augustosc
  address = MPU6050_ADDR;
  byte status_mpu6050 = writeData(MPU6050_PWR_MGMT_1_REGISTER, 0x01); // check only the first connection with status
  address = LSM6DS3_ADDR;
  byte status_lsm6ds3 = readData(LSM6DS3_WHO_AM_I);

  byte status = status_mpu6050;
  if (status_mpu6050 == 0)
  {
    address = MPU6050_ADDR;
    chip_type = CHIP_MPU6050;
  }
  else if (status_lsm6ds3 == LSM6DS3_WHO_AM_I_VALUE)
  {
    status = 0;
    address = LSM6DS3_ADDR;
    chip_type = CHIP_LSM6DS3;
  }

  if (chip_type == CHIP_MPU6050)
  {
    writeData(MPU6050_SMPLRT_DIV_REGISTER, 0x00);
    writeData(MPU6050_CONFIG_REGISTER, 0x00);
    setGyroConfig(gyro_config_num);
    setAccConfig(acc_config_num);

    this->update();
    angleX = this->getAccAngleX();
    angleY = this->getAccAngleY();
    preInterval = millis(); // may cause lack of angular accuracy if begin() is much before the first update()
  }

  if (chip_type == CHIP_LSM6DS3)
  {
    writeData(0x10, 0b01110001); // gyro_config_num  range = +- 500 deg/s
    gyro_lsb_to_degsec = 65.5;   //  +- 500 deg/s
    writeData(0x11, 0b01110100); // acc_config_num  range = +- 2 g
    acc_lsb_to_g = 16384.0;      // +- 2g
    // Serial1.println(readData(0x10), 2);
    // Serial1.println(readData(0x11), 2);
  }
  return status;
}

byte MPU6050::writeData(byte reg, byte data)
{
  wire->beginTransmission(address);
  wire->write(reg);
  wire->write(data);
  byte status = wire->endTransmission();
  return status; // 0 if success
}

// This method is not used internaly, maybe by user...
byte MPU6050::readData(byte reg)
{
  wire->beginTransmission(address);
  wire->write(reg);
  wire->endTransmission(true);
  wire->requestFrom(address, (uint8_t)1);
  byte data = wire->read();
  return data;
}

/* SETTER */

byte MPU6050::setGyroConfig(int config_num)
{
  byte status;
  switch (config_num)
  {
  case 0: // range = +- 250 deg/s
    gyro_lsb_to_degsec = 131.0;
    status = writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x00);
    break;
  case 1: // range = +- 500 deg/s
    gyro_lsb_to_degsec = 65.5;
    status = writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x08);
    break;
  case 2: // range = +- 1000 deg/s
    gyro_lsb_to_degsec = 32.8;
    status = writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x10);
    break;
  case 3: // range = +- 2000 deg/s
    gyro_lsb_to_degsec = 16.4;
    status = writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x18);
    break;
  default: // error
    status = 1;
    break;
  }
  return status;
}

byte MPU6050::setAccConfig(int config_num)
{
  byte status;
  switch (config_num)
  {
  case 0: // range = +- 2 g
    acc_lsb_to_g = 16384.0;
    status = writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x00);
    break;
  case 1: // range = +- 4 g
    acc_lsb_to_g = 8192.0;
    status = writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x08);
    break;
  case 2: // range = +- 8 g
    acc_lsb_to_g = 4096.0;
    status = writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x10);
    break;
  case 3: // range = +- 16 g
    acc_lsb_to_g = 2048.0;
    status = writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x18);
    break;
  default: // error
    status = 1;
    break;
  }
  return status;
}

void MPU6050::setGyroOffsets(float x, float y, float z)
{
  gyroXoffset = x;
  gyroYoffset = y;
  gyroZoffset = z;
}

void MPU6050::setAccOffsets(float x, float y, float z)
{
  accXoffset = x;
  accYoffset = y;
  accZoffset = z;
}

void MPU6050::setFilterGyroCoef(float gyro_coeff)
{
  if ((gyro_coeff < 0) or (gyro_coeff > 1))
  {
    gyro_coeff = DEFAULT_GYRO_COEFF;
  } // prevent bad gyro coeff, should throw an error...
  filterGyroCoef = gyro_coeff;
}

void MPU6050::setFilterAccCoef(float acc_coeff)
{
  setFilterGyroCoef(1.0 - acc_coeff);
}

/* CALC OFFSET */

void MPU6050::calcOffsets(bool is_calc_gyro, bool is_calc_acc)
{
  if (is_calc_gyro)
  {
    setGyroOffsets(0, 0, 0);
  }
  if (is_calc_acc)
  {
    setAccOffsets(0, 0, 0);
  }
  float ag[6] = {0, 0, 0, 0, 0, 0}; // 3*acc, 3*gyro

  for (int i = 0; i < CALIB_OFFSET_NB_MES; i++)
  {
    this->fetchData();
    ag[0] += accX;
    ag[1] += accY;
    ag[2] += (accZ - 1.0);
    ag[3] += gyroX;
    ag[4] += gyroY;
    ag[5] += gyroZ;
    delay(1); // wait a little bit between 2 measurements
  }

  if (is_calc_acc)
  {
    accXoffset = ag[0] / CALIB_OFFSET_NB_MES;
    accYoffset = ag[1] / CALIB_OFFSET_NB_MES;
    accZoffset = ag[2] / CALIB_OFFSET_NB_MES;
  }

  if (is_calc_gyro)
  {
    gyroXoffset = ag[3] / CALIB_OFFSET_NB_MES;
    gyroYoffset = ag[4] / CALIB_OFFSET_NB_MES;
    gyroZoffset = ag[5] / CALIB_OFFSET_NB_MES;
  }
}

/* UPDATE */

void MPU6050::fetchData()
{
  if (chip_type == CHIP_MPU6050)
  {
    wire->beginTransmission(address);
    wire->write(MPU6050_ACCEL_OUT_REGISTER);
    wire->endTransmission(false);
    wire->requestFrom(address, (uint8_t)14);

    int16_t rawData[7]; // [ax,ay,az,temp,gx,gy,gz]

    for (int i = 0; i < 7; i++)
    {
      rawData[i] = wire->read() << 8;
      rawData[i] |= wire->read();
    }

    accX = ((float)rawData[0]) / acc_lsb_to_g - accXoffset;
    accY = ((float)rawData[1]) / acc_lsb_to_g - accYoffset;
    accZ = (!upsideDownMounting - upsideDownMounting) * ((float)rawData[2]) / acc_lsb_to_g - accZoffset;
    temp = (rawData[3] + TEMP_LSB_OFFSET) / TEMP_LSB_2_DEGREE;
    gyroX = ((float)rawData[4]) / gyro_lsb_to_degsec - gyroXoffset;
    gyroY = ((float)rawData[5]) / gyro_lsb_to_degsec - gyroYoffset;
    gyroZ = ((float)rawData[6]) / gyro_lsb_to_degsec - gyroZoffset;
  }
  else if (chip_type == CHIP_LSM6DS3)
  {
    byte status = readData(0x1E);
    // 有温度了
    if (status & LSM6DS3_OUT_STATUS_MASK_TEMP)
    {
      wire->beginTransmission(address);
      wire->write(LSM6DS3_OUT_TEMP_REGISTER);
      wire->endTransmission(false);
      wire->requestFrom(address, (uint8_t)2);
      int16_t rawData;
      byte low = wire->read();
      byte high = wire->read();
      rawData = low | high << 8;
      temp = rawData / LSM6DS3_TEMP_LSB_PER_DEG + LSM6DS3_TEMP_OFFSET_DEG;
    }
    if (status & LSM6DS3_OUT_STATUS_MASK_G)
    {
      wire->beginTransmission(address);
      wire->write(LSM6DS3_OUT_G_REGISTER);
      wire->endTransmission(false);
      wire->requestFrom(address, (uint8_t)6);
      int16_t rawData[3];
      for (int i = 0; i < 3; i++)
      {
        byte low = wire->read();
        byte high = wire->read();
        rawData[i] = low | high << 8;
      }
      gyroX = ((float)rawData[0]) / gyro_lsb_to_degsec - gyroXoffset;
      gyroY = ((float)rawData[1]) / gyro_lsb_to_degsec - gyroYoffset;
      gyroZ = ((float)rawData[2]) / gyro_lsb_to_degsec - gyroZoffset;
    }
    if (status & LSM6DS3_OUT_STATUS_MASK_XL)
    {
      wire->beginTransmission(address);
      wire->write(LSM6DS3_OUT_XL_REGISTER);
      wire->endTransmission(false);
      wire->requestFrom(address, (uint8_t)6);
      int16_t rawData[3];
      for (int i = 0; i < 3; i++)
      {
        byte low = wire->read();
        byte high = wire->read();
        rawData[i] = low | high << 8;
      }
      accX = ((float)rawData[0]) / acc_lsb_to_g - accXoffset;
      accY = ((float)rawData[1]) / acc_lsb_to_g - accYoffset;
      accZ = (!upsideDownMounting - upsideDownMounting) * ((float)rawData[2]) / acc_lsb_to_g - accZoffset;
    }
  }
}

void MPU6050::update()
{
  // retrieve raw data
  this->fetchData();

  // estimate tilt angles: this is an approximation for small angles!
  float sgZ = accZ < 0 ? -1 : 1;                                              // allow one angle to go from -180 to +180 degrees
  angleAccX = atan2(accY, sgZ * sqrt(accZ * accZ + accX * accX)) * RAD_2_DEG; // [-180,+180] deg
  angleAccY = -atan2(accX, sqrt(accZ * accZ + accY * accY)) * RAD_2_DEG;      // [- 90,+ 90] deg

  unsigned long Tnew = millis();
  float dt = (Tnew - preInterval) * 1e-3;
  preInterval = Tnew;

  // Correctly wrap X and Y angles (special thanks to Edgar Bonet!)
  // https://github.com/gabriel-milan/TinyMPU6050/issues/6
  angleX = wrap(filterGyroCoef * (angleAccX + wrap(angleX + gyroX * dt - angleAccX, 180)) + (1.0 - filterGyroCoef) * angleAccX, 180);
  angleY = wrap(filterGyroCoef * (angleAccY + wrap(angleY + sgZ * gyroY * dt - angleAccY, 90)) + (1.0 - filterGyroCoef) * angleAccY, 90);
  angleZ += gyroZ * dt; // not wrapped
}

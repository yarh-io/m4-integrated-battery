/*!
 * @file adafruit_lc709203f_linux.cpp
 *
 * Linux I2C Driver for the Adafruit LC709203F Battery Monitor library
 *
 * This is a library for the Adafruit LC709203F breakout:
 * https://www.adafruit.com/products/4712
 *
 * This library depends on the gpio library - https://abyz.me.uk/rpi/pigpio/
 *
 * BSD license (see license.txt)
 */

/*
 * Based on:
 * https://github.com/adafruit/adafruit_lc709203f_linux
 */

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>    //Needed for I2C port
#include <fcntl.h>     //Needed for I2C port
#include <sys/ioctl.h> //Needed for I2C port
#include <linux/i2c.h>
#include <linux/i2c-dev.h> //Needed for I2C port
#include "adafruit_lc709203f_linux.h"

// #define DEBUG_PRINT

/**
 * Performs a CRC8 calculation on the supplied values.
 *
 * @param data  Pointer to the data to use when calculating the CRC8.
 * @param len   The number of bytes in 'data'.
 * @return The computed CRC8 value.
 */
static uint8_t lc709_crc8(uint8_t *data, int len)
{
  const uint8_t POLYNOMIAL(0x07);
  uint8_t crc(0x00);

  for (int j = len; j; --j)
  {
    crc ^= *data++;

    for (int i = 8; i; --i)
    {
      crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
    }
  }
  return crc;
}
/*!
 *    @brief  Instantiates a new LC709203F class
 */
adafruit_lc709203f_linux::adafruit_lc709203f_linux(void) {}

adafruit_lc709203f_linux::~adafruit_lc709203f_linux(void)
{
  close(i2c_handle);
}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @return True if initialization was successful, otherwise false.
 */
bool adafruit_lc709203f_linux::initialize()
{
  /* Open i2c device, use i2c bus 1, SDA:GPIO2(pin3), SCL:GPIO3(pin5) */
  if ((i2c_handle = open("/dev/i2c-1", O_RDWR)) < 0)
  {
#ifdef DEBUG_PRINT
    printf("--->Err: Failed to open the i2c bus\n");
#endif
    return false;
  }

  if (ioctl(i2c_handle, I2C_SLAVE, LC709203F_I2CADDR_DEFAULT) < 0)
  {
    printf("--->Err: Failed to acquire bus access and/or talk to slave.\n");
    // ERROR HANDLING; you can check errno to see what went wrong
    return false;
  }

  if (!setPowerMode(LC709203F_POWER_OPERATE))
  {
#ifdef DEBUG_PRINT
    printf("--->Err: setPowerMode\n");
#endif
    return false;
  }

  if (!setPackSize(LC709203F_APA_3000MAH))
  {
#ifdef DEBUG_PRINT
    printf("--->Err: setPackSize\n");
#endif
    return false;
  }

  // use 4.2V profile
  if (!setBattProfile(0x1))
  {
#ifdef DEBUG_PRINT
    printf("--->Err: setBattProfile\n");
#endif
    return false;
  }

  if (!setTemperatureMode(LC709203F_TEMPERATURE_I2C))
  {
#ifdef DEBUG_PRINT
    printf("--->Err: setTemperatureMode\n");
#endif
    return false;
  }

  return true;
}

/*!
 *    @brief  Get IC LSI version
 *    @return 16-bit value read from LC709203F_CMD_ICVERSION register
 */
uint16_t adafruit_lc709203f_linux::getICversion(void)
{
  uint16_t vers = 0;
  readWord(LC709203F_CMD_ICVERSION, &vers);
  return vers;
}

/*!
 *    @brief  Initialize the RSOC algorithm
 *    @return True on I2C command success
 */
bool adafruit_lc709203f_linux::initRSOC(void)
{
  return writeWord(LC709203F_CMD_INITRSOC, 0xAA55);
}

/*!
 *    @brief  Get battery voltage
 *    @return Floating point value read in Volts
 */
float adafruit_lc709203f_linux::cellVoltage(void)
{
  uint16_t voltage = 0;
  readWord(LC709203F_CMD_CELLVOLTAGE, &voltage);
  return voltage / 1000.0;
}

/*!
 *    @brief  Get battery state in percent (0-100%)
 *    @return Floating point value from 0 to 100.0
 */
float adafruit_lc709203f_linux::cellPercent(void)
{
  uint16_t percent = 0;
  readWord(LC709203F_CMD_CELLITE, &percent);
  return percent / 10.0;
}

/*!
 *    @brief  Get battery thermistor temperature
 *    @return Floating point value from -20 to 60 *C
 */
float adafruit_lc709203f_linux::getCellTemperature(void)
{
  uint16_t temp = 0;
  readWord(LC709203F_CMD_CELLTEMPERATURE, &temp);
  return (float)temp / 10.0;
}

/*!
 *    @brief  Set the temperature mode (external or internal)
 *    @param t The desired mode: LC709203F_TEMPERATURE_I2C or
 * LC709203F_TEMPERATURE_THERMISTOR
 *    @return True on successful I2C write
 */
bool adafruit_lc709203f_linux::setTemperatureMode(lc709203_tempmode_t t)
{
  return writeWord(LC709203F_CMD_STATUSBIT, (uint16_t)t);
}

/*!
 *    @brief  Set the approximate pack size, helps RSOC calculation
 *    @param apa The lc709203_adjustment_t enumerated approximate cell size
 *    @return True on successful I2C write
 */
bool adafruit_lc709203f_linux::setPackSize(lc709203_adjustment_t apa)
{
  return writeWord(LC709203F_CMD_APA, (uint16_t)apa);
}

/*!
 *    @brief  Set battery APA value, per LC709203F datasheet
 *    @param apa_value 8-bit APA value to use for the attached battery
 *    @return True on successful I2C write
 */
bool adafruit_lc709203f_linux::setPackAPA(uint8_t apa_value)
{
  return writeWord(LC709203F_CMD_APA, (uint16_t)apa_value);
}

/*!
 *    @brief  Set the alarm pin to respond to an RSOC percentage level
 *    @param percent The threshold value, set to 0 to disable alarm
 *    @return True on successful I2C write
 */
bool adafruit_lc709203f_linux::setAlarmRSOC(uint8_t percent)
{
  return writeWord(LC709203F_CMD_ALARMRSOC, percent);
}

/*!
 *    @brief  Set the alarm pin to respond to a battery voltage level
 *    @param voltage The threshold value, set to 0 to disable alarm
 *    @return True on successful I2C write
 */
bool adafruit_lc709203f_linux::setAlarmVoltage(float voltage)
{
  return writeWord(LC709203F_CMD_ALARMVOLT, voltage * 1000);
}

/*!
 *    @brief  Set the power mode, LC709203F_POWER_OPERATE or
 * LC709203F_POWER_SLEEP
 *    @param t The power mode desired
 *    @return True on successful I2C write
 */
bool adafruit_lc709203f_linux::setPowerMode(lc709203_powermode_t t)
{
  return writeWord(LC709203F_CMD_POWERMODE, (uint16_t)t);
}

/*!
 *    @brief  Get the thermistor B value (e.g. 3950)
 *    @return The uint16_t B value
 */
uint16_t adafruit_lc709203f_linux::getThermistorB(void)
{
  uint16_t val = 0;
  readWord(LC709203F_CMD_THERMISTORB, &val);
  return val;
}

/*!
 *    @brief  Set the thermistor B value (e.g. 3950)
 *    @param b The value to set it to
 *    @return True on successful I2C write
 */
bool adafruit_lc709203f_linux::setThermistorB(uint16_t b)
{
  return writeWord(LC709203F_CMD_THERMISTORB, b);
}

/*!
 *    @brief  Get the battery profile parameter
 *    @return The uint16_t profile value (0 or 1)
 */
uint16_t adafruit_lc709203f_linux::getBattProfile(void)
{
  uint16_t val = 0;
  readWord(LC709203F_CMD_BATTPROF, &val);
  return val;
}

/*!
 *    @brief  Set the battery profile parameter
 *    @param b The value to set it to (0 or 1)
 *    @return True on successful I2C write
 */
bool adafruit_lc709203f_linux::setBattProfile(uint16_t b)
{
  return writeWord(LC709203F_CMD_BATTPROF, b);
}

int adafruit_lc709203f_linux::writeWord(uint8_t command, uint16_t data)
{
  uint8_t send[5];
  send[0] = LC709203F_I2CADDR_DEFAULT * 2; // write byte
  send[1] = command;                       // command / register
  send[2] = data & 0xFF;
  send[3] = data >> 8;
  send[4] = lc709_crc8(send, 4);

  return (writeI2CBlockData(command, (char *)(send + 2), 3) < 0) ? false : true;
}

int adafruit_lc709203f_linux::readWord(uint8_t command, uint16_t *data)
{
  uint8_t reply[6];
  reply[0] = LC709203F_I2CADDR_DEFAULT * 2; // write byte
  reply[1] = command;                       // command / register
  reply[2] = reply[0] | 0x1;                // read byte
  int result;

  if ((result = readI2CBlockData(command, (char *)(reply + 3), 3)) < 0)
  {
#ifdef DEBUG_PRINT
    printf("--->Err: readWord : i2cReadI2CBlockData : %u\n", result);
#endif
    return false;
  }

  uint8_t crc = lc709_crc8(reply, 5);
  // CRC failure?
  if (crc != reply[5])
  {
// #ifdef DEBUG_PRINT
    printf("--->Err: readWord : CRC failure : %u != %u, data=%u\n", crc, reply[5], *data);
// #endif
    return false;
  }

  *data = reply[4];
  *data <<= 8;
  *data |= reply[3];

  return true;
}

int adafruit_lc709203f_linux::readI2CBlockData(unsigned reg, char *buf, unsigned count)
{
  struct i2c_smbus_ioctl_data ioctl_data;
  union i2c_smbus_data smbus_data;

  int status;

  smbus_data.block[0] = count;

  ioctl_data.read_write = I2C_SMBUS_READ;
  ioctl_data.command = reg;
  ioctl_data.size = I2C_SMBUS_I2C_BLOCK_DATA;
  ioctl_data.data = &smbus_data;

  status = ioctl(i2c_handle, I2C_SMBUS, &ioctl_data);

  if (status < 0)
  {
// #ifdef DEBUG_PRINT
    printf("--->Err: Failed readI2CBlockData, status=%d\n", status);
// #endif
    return false;
  }

  for (int i = 0; i < smbus_data.block[0]; i++)
  {
    buf[i] = smbus_data.block[i + 1];
  }

  return smbus_data.block[0];
}

int adafruit_lc709203f_linux::writeI2CBlockData(unsigned reg, char *buf, unsigned count)
{
  struct i2c_smbus_ioctl_data ioctl_data;
  union i2c_smbus_data smbus_data;

  int status;

  for (unsigned i = 1; i <= count; i++)
  {
    smbus_data.block[i] = buf[i - 1];
  }

  smbus_data.block[0] = count;

  ioctl_data.read_write = I2C_SMBUS_WRITE;
  ioctl_data.command = reg;
  ioctl_data.size = I2C_SMBUS_I2C_BLOCK_DATA;
  ioctl_data.data = &smbus_data;

  status = ioctl(i2c_handle, I2C_SMBUS, &ioctl_data);

  if (status < 0)
  {
// #ifdef DEBUG_PRINT
    printf("--->Err: Failed writeI2CBlockData, status=%d\n", status);
// #endif
    return false;
  }

  return status;
}
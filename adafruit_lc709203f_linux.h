/*!
 * @file adafruit_lc709203f.h
 *
 * Linux I2C Driver for the Adafruit LC709203F Battery Monitor library
 *
 * This is a library for the Adafruit LC709203F breakout:
 * https://www.adafruit.com/products/4712
 *
 * BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_LC709203F_H
#define _ADAFRUIT_LC709203F_H

#include <stdint.h>

#define LC709203F_I2CADDR_DEFAULT 0x0B     ///< LC709203F default i2c address
#define LC709203F_CMD_THERMISTORB 0x06     ///< Read/write thermistor B
#define LC709203F_CMD_INITRSOC 0x07        ///< Initialize RSOC calculation
#define LC709203F_CMD_CELLTEMPERATURE 0x08 ///< Read/write batt temperature
#define LC709203F_CMD_CELLVOLTAGE 0x09     ///< Read batt voltage
#define LC709203F_CMD_APA 0x0B             ///< Adjustment Pack Application
#define LC709203F_CMD_RSOC 0x0D            ///< Read state of charge
#define LC709203F_CMD_CELLITE 0x0F         ///< Read batt indicator to empty
#define LC709203F_CMD_ICVERSION 0x11       ///< Read IC version
#define LC709203F_CMD_BATTPROF 0x12        ///< Set the battery profile
#define LC709203F_CMD_ALARMRSOC 0x13       ///< Alarm on percent threshold
#define LC709203F_CMD_ALARMVOLT 0x14       ///< Alarm on voltage threshold
#define LC709203F_CMD_POWERMODE 0x15       ///< Sets sleep/power mode
#define LC709203F_CMD_STATUSBIT 0x16       ///< Temperature obtaining method
#define LC709203F_CMD_PARAMETER 0x1A       ///< Batt profile code

/*!  Battery temperature source */
typedef enum {
  LC709203F_TEMPERATURE_I2C = 0x0000,
  LC709203F_TEMPERATURE_THERMISTOR = 0x0001,
} lc709203_tempmode_t;

/*!  Chip power state */
typedef enum {
  LC709203F_POWER_OPERATE = 0x0001,
  LC709203F_POWER_SLEEP = 0x0002,
} lc709203_powermode_t;

/*!  Approx battery pack size */
typedef enum {
  LC709203F_APA_100MAH = 0x08,
  LC709203F_APA_200MAH = 0x0B,
  LC709203F_APA_500MAH = 0x10,
  LC709203F_APA_1000MAH = 0x19,
  LC709203F_APA_2000MAH = 0x2D,
  LC709203F_APA_3000MAH = 0x36,
} lc709203_adjustment_t;

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the LC709203F I2C battery monitor
 */
class adafruit_lc709203f_linux {
public:
  adafruit_lc709203f_linux();
  ~adafruit_lc709203f_linux();

  bool initialize();
  bool initRSOC(void);

  bool setPowerMode(lc709203_powermode_t t);
  bool setPackSize(lc709203_adjustment_t apa);
  bool setPackAPA(uint8_t apa_value);

  uint16_t getICversion(void);
  float cellVoltage(void);
  float cellPercent(void);

  uint16_t getThermistorB(void);
  bool setThermistorB(uint16_t b);

  uint16_t getBattProfile(void);
  bool setBattProfile(uint16_t b);

  bool setTemperatureMode(lc709203_tempmode_t t);
  float getCellTemperature(void);

  bool setAlarmRSOC(uint8_t percent);
  bool setAlarmVoltage(float voltage);

protected:
  int i2c_handle = -1; // Initialize as invalid handle
  int readWord(uint8_t command, uint16_t *data);
  int writeWord(uint8_t command, uint16_t data);
  int readI2CBlockData(unsigned reg, char *buf, unsigned count);
  int writeI2CBlockData(unsigned reg, char *buf, unsigned count);
};

#endif

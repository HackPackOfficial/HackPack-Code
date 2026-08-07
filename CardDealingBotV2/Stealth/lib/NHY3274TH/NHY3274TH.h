#ifndef _NHY3274TH_H_
#define _NHY3274TH_H_

#include <Arduino.h>
#include <Wire.h>

#define NHY3274TH_ADDRESS (0x38) // I2C address for the NHY3274TH

// Register map for the NHY3274TH sensor
#define NHY3274TH_ENABLE 0x00   // CLS operation mode control, waiting mode control, SW reset. Default value = 0
#define NHY3274TH_ATIME 0x01    // Interrupt pin control, interrupt persist control. Default value = 0x03
#define NHY3274TH_INT_FLAG 0x02 // Interrupt pin control, interrupt persist control, defult 0x03
#define NHY3274TH_WTIME 0x03    // Wait time. Default value = 0x00.
#define NHY3274TH_CLS_GAIN 0x04 // CLS analog gain setting. Default value 0x81
#define NHY3274TH_AILTH 0x05    // CLS integrated time setting. Default value 0x03
//  #define NHY3274TH_PERS 0x0B // CLS persistence setting
//  #define NHY3274TH_THRES_LL 0x0C // CLS lower interrupt threshold - LSB
//  #define NHY3274TH_THRES_LH 0x0D // CLS lower interrupt threshold - MSB
//  #define NHY3274TH_THRES_HL 0x0E // CLS higher interrupt threshold - LSB
//  #define NHY3274TH_THRES_HH 0x0F // CLS higher interrupt threshold - MSB
#define NHY3274TH_RDATA_L 0x1C  // Red data low byte
#define NHY3274TH_RDATA_H 0x1D  // Red data high byte
#define NHY3274TH_GDATA_L 0x1E  // Green data low byte
#define NHY3274TH_GDATA_H 0x1F  // Green data high byte
#define NHY3274TH_BDATA_L 0x20  // Blue data low byte
#define NHY3274TH_BDATA_H 0x21  // Blue data high byte
#define NHY3274TH_WDATA_L 0x22  // White data low byte
#define NHY3274TH_WDATA_H 0x23  // White data high byte
#define NHY3274TH_IRDATA_L 0x24 // IR data low byte
#define NHY3274TH_IRDATA_H 0x25 // IR data high byte

class NHY3274TH
{
public:
  NHY3274TH(uint8_t address = NHY3274TH_ADDRESS);
  boolean begin(TwoWire *theWire = &Wire);
  void enable();
  void disable();
  void getRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *w);
  uint16_t read16(uint8_t reg);
  void setIntegrationTime(uint8_t integrationTime);
  void setGain(uint8_t gain);

private:
  uint8_t _address;
  TwoWire *_wire;
  void write8(uint8_t reg, uint8_t value);
  uint8_t read8(uint8_t reg);
};

#endif
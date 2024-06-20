#pragma once

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"

#define LORA_DEFAULT_SPI_FREQUENCY 8E6
#define PA_OUTPUT_RFO_PIN 0
#define PA_OUTPUT_PA_BOOST_PIN 1

class LoRaClass
{
public:
  LoRaClass();
  ~LoRaClass();

  int begin(long frequency);
  void end();

  int beginPacket(int implicitHeader = false);
  int endPacket(bool async = false);

  uint8_t parsePacket(int size = 0);
  uint8_t packetRssi();
  float packetSnr();
  long packetFrequencyError();

  int rssi();

  void onReceive(void (*callback)(int));
  void onTxDone(void (*callback)());
  void receive(int size = 0);

  void idle();
  void sleep();

  void setTxPower(int level, int outputPin = PA_OUTPUT_PA_BOOST_PIN);
  void setFrequency(long frequency);
  void setSpreadingFactor(int sf);
  void setSignalBandwidth(long sbw);
  void setCodingRate4(int denominator);
  void setPreambleLength(long length);
  void setSyncWord(int sw);
  void enableCrc();
  void disableCrc();
  void enableInvertIQ();
  void disableInvertIQ();

  void setOCP(uint8_t mA); // Over Current Protection control

  void setGain(uint8_t gain); // Set LNA gain

  void print(uint8_t data);
  // void print(char data);

  uint8_t getRegValue(uint8_t reg);
  void setFifoAddrPtr(uint8_t value);

private:
  void explicitHeaderMode();
  void implicitHeaderMode();

  void handleDio0Rise();
  bool isTransmitting();

  int getSpreadingFactor();
  long getSignalBandwidth();

  void setLdoFlag();

  static inline void read_register(uint8_t reg, uint8_t *buf, uint16_t len);
  static inline void write_register(uint8_t reg, uint8_t data);

  static void onDio0Rise();

  int _ss;
  int _reset;
  int _dio0;
  long _frequency;
  int _packetIndex;
  int _implicitHeaderMode;
  void (*_onReceive)(int);
  void (*_onTxDone)();

  static void cs_select();
  static void cs_deselect();
};

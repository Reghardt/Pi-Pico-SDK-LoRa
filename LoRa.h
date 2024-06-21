#pragma once

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"

#define LORA_DEFAULT_SPI_FREQUENCY 8E6
#define PA_OUTPUT_RFO_PIN 0
#define PA_OUTPUT_PA_BOOST_PIN 1

class LoRa
{
public:
  LoRa(spi_inst *spi_bus, uint16_t cs_pin, uint8_t rst_pin, uint8_t dio0_pin);
  ~LoRa();

  int begin(long frequency);
  void end();

  int beginPacket(int implicitHeader = false);
  int endPacket(bool async = false);

  uint8_t parsePacket(int size = 0);
  uint8_t packetRssi();
  float packetSnr();
  long packetFrequencyError();

  int rssi();

  virtual uint8_t available();
  virtual uint8_t read();

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

  void handleDio0Rise();

private:
  void explicitHeaderMode();
  void implicitHeaderMode();

  // void handleDio0Rise();
  bool isTransmitting();

  int getSpreadingFactor();
  long getSignalBandwidth();

  void setLdoFlag();

  inline void read_register(uint8_t reg, uint8_t *buf, uint16_t len);
  inline void write_register(uint8_t reg, uint8_t data);

  static void onDio0Rise();

  spi_inst *m_spi_bus;
  uint8_t m_cs_pin;
  uint8_t m_rst_pin;
  uint8_t m_dio0_pin;
  long m_frequency;
  int m_packetIndex;
  int m_implicitHeaderMode;
  void (*m_onReceive)(int);
  void (*m_onTxDone)();

  void cs_select();
  void cs_deselect();
};

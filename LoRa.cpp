#include "LoRa.h"

// SPI Defines
#define READ_BIT 0x80
#define SPI_PORT spi1
#define SPI_RX_PIN 12
#define SPI_TX_PIN 15
#define SPI_SCK_PIN 14

#define LORA_CS_PIN 13
#define LORA_RST_PIN 11
#define LORA_DIO0_PIN 10

// registers
#define REG_FIFO 0x00                 // same
#define REG_OP_MODE 0x01              // same
#define REG_FRF_MSB 0x06              // same
#define REG_FRF_MID 0x07              // same
#define REG_FRF_LSB 0x08              // same
#define REG_PA_CONFIG 0x09            // same
#define REG_OCP 0x0b                  // same
#define REG_LNA 0x0c                  // same
#define REG_FIFO_ADDR_PTR 0x0D        // same
#define REG_FIFO_TX_BASE_ADDR 0x0E    // same
#define REG_FIFO_RX_BASE_ADDR 0x0F    // same
#define REG_FIFO_RX_CURRENT_ADDR 0x10 // same
#define REG_IRQ_FLAGS 0x12            // same
#define REG_RX_NB_BYTES 0x13          // same
#define REG_PKT_SNR_VALUE 0x19        // same
#define REG_PKT_RSSI_VALUE 0x1a       // same
#define REG_RSSI_VALUE 0x1b           // same
#define REG_MODEM_CONFIG_1 0x1d       // same
#define REG_MODEM_CONFIG_2 0x1e       // same
#define REG_PREAMBLE_MSB 0x20         // same
#define REG_PREAMBLE_LSB 0x21         // same
#define REG_PAYLOAD_LENGTH 0x22       // same
#define REG_MODEM_CONFIG_3 0x26       // same
#define REG_FREQ_ERROR_MSB 0x28
#define REG_FREQ_ERROR_MID 0x29
#define REG_FREQ_ERROR_LSB 0x2a
#define REG_RSSI_WIDEBAND 0x2c
#define REG_DETECTION_OPTIMIZE 0x31
#define REG_INVERTIQ 0x33
#define REG_DETECTION_THRESHOLD 0x37
#define REG_SYNC_WORD 0x39
#define REG_INVERTIQ2 0x3b
#define REG_DIO_MAPPING_1 0x40
#define REG_VERSION 0x42
#define REG_PA_DAC 0x4d

// modes
#define MODE_LONG_RANGE_MODE 0x80
#define MODE_SLEEP 0x00
#define MODE_STDBY 0x01
#define MODE_TX 0x03
#define MODE_RX_CONTINUOUS 0x05
#define MODE_RX_SINGLE 0x06

// PA config
#define PA_BOOST 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK 0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK 0x40

#define RF_MID_BAND_THRESHOLD 525E6
#define RSSI_OFFSET_HF_PORT 157
#define RSSI_OFFSET_LF_PORT 164

#define MAX_PKT_LENGTH 255

#define ISR_PREFIX

#define FIFO_RX_START_LOC 0x00
#define FIFO_TX_START_LOC 0x80

#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

LoRaClass::LoRaClass(/* args */) : _frequency(0),
                                   _packetIndex(0),
                                   _implicitHeaderMode(0),
                                   //  _onReceive(NULL),
                                   _onTxDone(NULL)
{
}

LoRaClass::~LoRaClass()
{
}

int LoRaClass::begin(long frequency)
{
  spi_init(SPI_PORT, 500 * 1000);
  gpio_set_function(SPI_RX_PIN, GPIO_FUNC_SPI);
  gpio_set_function(SPI_SCK_PIN, GPIO_FUNC_SPI);
  gpio_set_function(SPI_TX_PIN, GPIO_FUNC_SPI);
  bi_decl(bi_3pins_with_func(SPI_RX_PIN, SPI_TX_PIN, SPI_SCK_PIN, GPIO_FUNC_SPI)); // Make the SPI pins available to picotool

  gpio_set_function(LORA_CS_PIN, GPIO_FUNC_SIO);
  gpio_set_dir(LORA_CS_PIN, GPIO_OUT); // Chip select is active-low, so we'll initialise it to a driven-high state
  gpio_put(LORA_CS_PIN, 1);
  bi_decl(bi_1pin_with_name(LORA_CS_PIN, "LORA_CS_PIN"));

  gpio_set_function(LORA_RST_PIN, GPIO_FUNC_SIO);
  gpio_set_dir(LORA_RST_PIN, GPIO_OUT);
  bi_decl(bi_1pin_with_name(LORA_RST_PIN, "LORA_RST_PIN"));

  gpio_put(LORA_RST_PIN, 0);
  sleep_ms(10);
  gpio_put(LORA_RST_PIN, 1);
  sleep_ms(10);

  uint8_t version;
  read_register(0x42, &version, 1);

  if (version != 0x12)
  {
    printf("Error: LoRa could not start. Reading: 0x%x\n", version);
    return 0;
  }
  else
  {
    printf("Version: 0x%x\n", version);
    // put in sleep mode
    sleep();

    // set frequency
    setFrequency(frequency);

    // set base addresses
    write_register(REG_FIFO_TX_BASE_ADDR, 0);
    write_register(REG_FIFO_RX_BASE_ADDR, 0);

    // set LNA boost
    uint8_t RegLna;
    read_register(REG_LNA, &RegLna, 1);
    write_register(REG_LNA, RegLna | 0x03);

    // set auto AGC
    write_register(REG_MODEM_CONFIG_3, 0x04);

    // set output power to 17 dBm
    setTxPower(17);

    // put in standby mode
    idle();

    printf("Lora started successfully!");
    return 1;
  }
}

void LoRaClass::end()
{
  // put in sleep mode
  sleep();

  // stop SPI
  // _spi->end();
}

int LoRaClass::beginPacket(int implicitHeader)
{
  if (isTransmitting())
  {
    return 0;
  }

  // put in standby mode
  idle();

  if (implicitHeader)
  {
    implicitHeaderMode();
  }
  else
  {
    explicitHeaderMode();
  }

  // reset FIFO address and payload length
  write_register(REG_FIFO_ADDR_PTR, 0);
  write_register(REG_PAYLOAD_LENGTH, 0);

  return 1;
}

int LoRaClass::endPacket(bool async)
{
  if ((async) && (_onTxDone))
  {
    write_register(REG_DIO_MAPPING_1, 0x40); // DIO0 => TXDONE
  }

  // put in TX mode
  // When activated the RFM95/96/97/98(W) powers all remaining blocks required for transmit, ramps the PA,
  // transmits the packet and returns to Stand-by mode.
  write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

  if (!async)
  {
    // wait for TX done
    while (true)
    {
      uint8_t RegIrqFlags;
      read_register(REG_IRQ_FLAGS, &RegIrqFlags, 1);
      if ((RegIrqFlags & IRQ_TX_DONE_MASK) == 0)
      {
        // yield();
        // sleep_ms(1);
      }
      else
      {
        break;
      }
    }
    // clear IRQ's
    write_register(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
  }
  return 1;
}

bool LoRaClass::isTransmitting()
{
  uint8_t RegOpMode;
  read_register(REG_OP_MODE, &RegOpMode, 1);
  if ((RegOpMode & MODE_TX) == MODE_TX)
  {
    return true;
  }

  uint8_t RegIrqFlags;
  read_register(REG_IRQ_FLAGS, &RegIrqFlags, 1);
  if (RegIrqFlags & IRQ_TX_DONE_MASK)
  {
    // clear IRQ's
    write_register(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
  }

  return false;
}

uint8_t LoRaClass::parsePacket(int size)
{
  uint8_t packetLength = 0;
  uint8_t irqFlags;
  read_register(REG_IRQ_FLAGS, &irqFlags, 1);

  if (size > 0)
  {
    implicitHeaderMode();

    write_register(REG_PAYLOAD_LENGTH, size & 0xff);
  }
  else
  {
    explicitHeaderMode();
  }

  // clear IRQ's
  write_register(REG_IRQ_FLAGS, irqFlags);

  if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0)
  {
    // received a packet
    _packetIndex = 0;

    // read packet length
    if (_implicitHeaderMode)
    {
      read_register(REG_PAYLOAD_LENGTH, &packetLength, 1);
    }
    else
    {
      read_register(REG_RX_NB_BYTES, &packetLength, 1);
    }

    // set FIFO address to current RX address
    uint8_t RegFifoRxCurrentAddr;
    read_register(REG_FIFO_RX_CURRENT_ADDR, &RegFifoRxCurrentAddr, 1);
    write_register(REG_FIFO_ADDR_PTR, RegFifoRxCurrentAddr);

    // put in standby mode
    idle();
  }
  else
  {
    uint8_t RegOpMode;
    read_register(REG_OP_MODE, &RegOpMode, 1);
    if (RegOpMode != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE))
    {
      // not currently in RX mode

      // reset FIFO address
      write_register(REG_FIFO_ADDR_PTR, 0);

      // put in single RX mode
      write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
    }
  }

  return packetLength;
}

uint8_t LoRaClass::packetRssi()
{
  uint8_t RegPktRssiValue;
  read_register(REG_PKT_RSSI_VALUE, &RegPktRssiValue, 1);
  return (RegPktRssiValue - (_frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

float LoRaClass::packetSnr()
{
  uint8_t RegPktSnrValue;
  read_register(REG_PKT_SNR_VALUE, &RegPktSnrValue, 1);
  return (RegPktSnrValue * 0.25);
}

long LoRaClass::packetFrequencyError()
{
  uint8_t RegFreqErrorMsb;
  uint8_t RegFreqErrorMid;
  uint8_t RegFreqErrorLsb;

  read_register(REG_FREQ_ERROR_MSB, &RegFreqErrorMsb, 1);
  read_register(REG_FREQ_ERROR_MID, &RegFreqErrorMid, 1);
  read_register(REG_FREQ_ERROR_LSB, &RegFreqErrorLsb, 1);

  int32_t freqError = 0;
  freqError = static_cast<int32_t>(RegFreqErrorMsb & 0x7);
  freqError <<= 8L;
  freqError += static_cast<int32_t>(RegFreqErrorMid);
  freqError <<= 8L;
  freqError += static_cast<int32_t>(RegFreqErrorLsb);

  if (RegFreqErrorMsb & 0x8)
  {                      // Sign bit is on
    freqError -= 524288; // B1000'0000'0000'0000'0000
  }

  const float fXtal = 32E6;                                                                                         // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
  const float fError = ((static_cast<float>(freqError) * (1L << 24)) / fXtal) * (getSignalBandwidth() / 500000.0f); // p. 37

  return static_cast<long>(fError);
}

int LoRaClass::rssi()
{
  uint8_t RegRssiValue;
  read_register(REG_RSSI_VALUE, &RegRssiValue, 1);
  return (RegRssiValue - (_frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

void LoRaClass::cs_select()
{
  asm volatile("nop \n nop \n nop");
  gpio_put(LORA_CS_PIN, 0); // Active low
  asm volatile("nop \n nop \n nop");
}

void LoRaClass::cs_deselect()
{
  asm volatile("nop \n nop \n nop");
  gpio_put(LORA_CS_PIN, 1);
  asm volatile("nop \n nop \n nop");
}

void LoRaClass::read_register(uint8_t reg, uint8_t *buf, uint16_t len)
{
  // For this particular device, we send the device the register we want to read
  // first, then subsequently read from the device. The register is auto incrementing
  // so we don't need to keep sending the register we want, just the first.
  reg = ((reg << 1) >> 1); // make msb bit 0 with l,r bit shifting, as to read. "reg |= 0x7f" is equivalent to this line
  cs_select();
  spi_write_blocking(SPI_PORT, &reg, 1);
  // sleep_ms(10);
  asm volatile("nop \n nop \n nop");
  spi_read_blocking(SPI_PORT, 0, buf, len);
  cs_deselect();
  // sleep_ms(10);
  asm volatile("nop \n nop \n nop");
}

void LoRaClass::write_register(uint8_t reg, uint8_t data)
{
  uint8_t buf[2];
  buf[0] = reg | 0x80; // make msb 1, as to write
  buf[1] = data;
  cs_select();
  spi_write_blocking(SPI_PORT, buf, 2);
  cs_deselect();
  // sleep_ms(1);
  asm volatile("nop \n nop \n nop");
}

/*
  Callback must be set before enabling interrupt
*/
void LoRaClass::enableInterruptOnReceive()
{
  // _onReceive = callback;
  interruptOnReceive = true;
  // if (callback)
  // {
  gpio_set_function(LORA_DIO0_PIN, GPIO_FUNC_SIO);
  gpio_set_dir(LORA_DIO0_PIN, GPIO_IN);
  gpio_set_irq_enabled(LORA_DIO0_PIN, GPIO_IRQ_EDGE_RISE, true);

  // gpio_set_irq_enabled_with_callback(LORA_DIO0_PIN, GPIO_IRQ_EDGE_RISE, true, LoRaClass::onDio0Rise);

  // attachInterrupt(digitalPinToInterrupt(_dio0), LoRaClass::onDio0Rise, RISING);
  // }
  // else
  // {
  //   // detachInterrupt(digitalPinToInterrupt(_dio0));
  // }
}

// void LoRaClass::onTxDone(void (*callback)())
// {
//   _onTxDone = callback;

//   if (callback)
//   {
//     pinMode(_dio0, INPUT);
//     attachInterrupt(digitalPinToInterrupt(_dio0), LoRaClass::onDio0Rise, RISING);
//   }
//   else
//   {
//     detachInterrupt(digitalPinToInterrupt(_dio0));
//   }
// }

void LoRaClass::receive(int size)
{

  write_register(REG_DIO_MAPPING_1, 0x00); // DIO0 => RXDONE

  if (size > 0)
  {
    implicitHeaderMode();

    write_register(REG_PAYLOAD_LENGTH, size & 0xff);
  }
  else
  {
    explicitHeaderMode();
  }

  write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void LoRaClass::idle()
{
  write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY); // result in 1000 0001. Datasheet p87
}
void LoRaClass::sleep()
{
  write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void LoRaClass::setTxPower(int level, int outputPin)
{
  if (PA_OUTPUT_RFO_PIN == outputPin)
  {
    // RFO
    if (level < 0)
    {
      level = 0;
    }
    else if (level > 14)
    {
      level = 14;
    }

    write_register(REG_PA_CONFIG, 0x70 | level);
  }
  else
  {
    // PA BOOST
    if (level > 17)
    {
      if (level > 20)
      {
        level = 20;
      }

      // subtract 3 from level, so 18 - 20 maps to 15 - 17
      level -= 3;

      // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
      write_register(REG_PA_DAC, 0x87);
      setOCP(140);
    }
    else
    {
      if (level < 2)
      {
        level = 2;
      }
      // Default value PA_HF/LF or +17dBm
      write_register(REG_PA_DAC, 0x84);
      setOCP(100);
    }

    write_register(REG_PA_CONFIG, PA_BOOST | (level - 2));
  }
}

void LoRaClass::setFrequency(long frequency)
{
  _frequency = frequency;

  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

  write_register(REG_FRF_MSB, (uint8_t)(frf >> 16));
  write_register(REG_FRF_MID, (uint8_t)(frf >> 8));
  write_register(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

int LoRaClass::getSpreadingFactor()
{
  uint8_t RegModemConfig2;
  read_register(REG_MODEM_CONFIG_2, &RegModemConfig2, 1);
  return RegModemConfig2 >> 4;
}

void LoRaClass::setSpreadingFactor(int sf)
{
  if (sf < 6)
  {
    sf = 6;
  }
  else if (sf > 12)
  {
    sf = 12;
  }

  if (sf == 6)
  {
    write_register(REG_DETECTION_OPTIMIZE, 0xc5);
    write_register(REG_DETECTION_THRESHOLD, 0x0c);
  }
  else
  {
    write_register(REG_DETECTION_OPTIMIZE, 0xc3);
    write_register(REG_DETECTION_THRESHOLD, 0x0a);
  }

  uint8_t RegModemConfig2;
  read_register(REG_MODEM_CONFIG_2, &RegModemConfig2, 1);
  write_register(REG_MODEM_CONFIG_2, (RegModemConfig2 & 0x0f) | ((sf << 4) & 0xf0));
  setLdoFlag();
}

long LoRaClass::getSignalBandwidth()
{
  uint8_t RegModemConfig1;
  read_register(REG_MODEM_CONFIG_1, &RegModemConfig1, 1);
  uint8_t bw = (RegModemConfig1 >> 4);

  switch (bw)
  {
  case 0:
    return 7.8E3;
  case 1:
    return 10.4E3;
  case 2:
    return 15.6E3;
  case 3:
    return 20.8E3;
  case 4:
    return 31.25E3;
  case 5:
    return 41.7E3;
  case 6:
    return 62.5E3;
  case 7:
    return 125E3;
  case 8:
    return 250E3;
  case 9:
    return 500E3;
  }

  return -1;
}

void LoRaClass::setSignalBandwidth(long sbw)
{
  int bw;

  if (sbw <= 7.8E3)
  {
    bw = 0;
  }
  else if (sbw <= 10.4E3)
  {
    bw = 1;
  }
  else if (sbw <= 15.6E3)
  {
    bw = 2;
  }
  else if (sbw <= 20.8E3)
  {
    bw = 3;
  }
  else if (sbw <= 31.25E3)
  {
    bw = 4;
  }
  else if (sbw <= 41.7E3)
  {
    bw = 5;
  }
  else if (sbw <= 62.5E3)
  {
    bw = 6;
  }
  else if (sbw <= 125E3)
  {
    bw = 7;
  }
  else if (sbw <= 250E3)
  {
    bw = 8;
  }
  else /*if (sbw <= 250E3)*/
  {
    bw = 9;
  }

  uint8_t RegModemConfig1;
  read_register(REG_MODEM_CONFIG_1, &RegModemConfig1, 1);
  write_register(REG_MODEM_CONFIG_1, (RegModemConfig1 & 0x0f) | (bw << 4));
  setLdoFlag();
}

void LoRaClass::setLdoFlag()
{
  // Section 4.1.1.5
  long symbolDuration = 1000 / (getSignalBandwidth() / (1L << getSpreadingFactor()));

  // Section 4.1.1.6
  bool ldoOn = symbolDuration > 16;

  uint8_t RegModemConfig3;
  read_register(REG_MODEM_CONFIG_3, &RegModemConfig3, 1);
  bitWrite(RegModemConfig3, 3, ldoOn);
  write_register(REG_MODEM_CONFIG_3, RegModemConfig3);
}

void LoRaClass::setCodingRate4(int denominator)
{
  if (denominator < 5)
  {
    denominator = 5;
  }
  else if (denominator > 8)
  {
    denominator = 8;
  }

  int cr = denominator - 4;

  uint8_t RegModemConfig1;
  read_register(REG_MODEM_CONFIG_1, &RegModemConfig1, 1);
  write_register(REG_MODEM_CONFIG_1, (RegModemConfig1 & 0xf1) | (cr << 1));
}

void LoRaClass::setPreambleLength(long length)
{
  write_register(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  write_register(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void LoRaClass::setSyncWord(int sw)
{
  write_register(REG_SYNC_WORD, sw);
}

void LoRaClass::enableCrc()
{
  uint8_t RegModemConfig2;
  read_register(REG_MODEM_CONFIG_2, &RegModemConfig2, 1);
  write_register(REG_MODEM_CONFIG_2, RegModemConfig2 | 0x04);
}

void LoRaClass::disableCrc()
{
  uint8_t RegModemConfig2;
  read_register(REG_MODEM_CONFIG_2, &RegModemConfig2, 1);
  write_register(REG_MODEM_CONFIG_2, RegModemConfig2 & 0xfb);
}

void LoRaClass::enableInvertIQ()
{
  write_register(REG_INVERTIQ, 0x66);
  write_register(REG_INVERTIQ2, 0x19);
}

void LoRaClass::disableInvertIQ()
{
  write_register(REG_INVERTIQ, 0x27);
  write_register(REG_INVERTIQ2, 0x1d);
}

void LoRaClass::setOCP(uint8_t mA)
{
  uint8_t ocpTrim = 27;

  if (mA <= 120)
  {
    ocpTrim = (mA - 45) / 5;
  }
  else if (mA <= 240)
  {
    ocpTrim = (mA + 30) / 10;
  }

  write_register(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

void LoRaClass::setGain(uint8_t gain)
{
  // check allowed range
  if (gain > 6)
  {
    gain = 6;
  }

  // set to standby
  idle();

  // set gain
  if (gain == 0)
  {
    // if gain = 0, enable AGC
    write_register(REG_MODEM_CONFIG_3, 0x04);
  }
  else
  {
    // disable AGC
    write_register(REG_MODEM_CONFIG_3, 0x00);

    // clear Gain and set LNA boost
    write_register(REG_LNA, 0x03);

    // set gain
    uint8_t RegLna;
    read_register(REG_LNA, &RegLna, 1);
    write_register(REG_LNA, RegLna | (gain << 5));
  }
}

void LoRaClass::explicitHeaderMode()
{
  _implicitHeaderMode = 0;
  uint8_t RegModemConfig1;
  read_register(REG_MODEM_CONFIG_1, &RegModemConfig1, 1);
  write_register(REG_MODEM_CONFIG_1, RegModemConfig1 & 0xfe);
}

void LoRaClass::implicitHeaderMode()
{
  _implicitHeaderMode = 1;

  uint8_t RegModemConfig1;
  read_register(REG_MODEM_CONFIG_1, &RegModemConfig1, 1);
  write_register(REG_MODEM_CONFIG_1, RegModemConfig1 | 0x01);
}

void LoRaClass::handleDio0Rise()
{
  uint8_t irqFlags;

  read_register(REG_IRQ_FLAGS, &irqFlags, 1);

  // clear IRQ's
  write_register(REG_IRQ_FLAGS, irqFlags);

  if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0)
  {

    if ((irqFlags & IRQ_RX_DONE_MASK) != 0)
    {
      // received a packet
      _packetIndex = 0;

      // read packet length
      int packetLength;
      if (_implicitHeaderMode)
      {
        uint8_t RegPayloadLength;
        read_register(REG_PAYLOAD_LENGTH, &RegPayloadLength, 1);
        packetLength = RegPayloadLength;
      }
      else
      {
        uint8_t RegRxNbBytes;
        read_register(REG_RX_NB_BYTES, &RegRxNbBytes, 1);
        packetLength = RegRxNbBytes;
      }

      // set FIFO address to current RX address
      uint8_t RegFifoRxCurrentAddr;
      read_register(REG_FIFO_RX_CURRENT_ADDR, &RegFifoRxCurrentAddr, 1);
      write_register(REG_FIFO_ADDR_PTR, RegFifoRxCurrentAddr);

      // if (_onReceive)
      // {
      //   _onReceive(packetLength);
      // }
    }
    else if ((irqFlags & IRQ_TX_DONE_MASK) != 0)
    {
      if (_onTxDone)
      {
        _onTxDone();
      }
    }
  }
}

void LoRaClass::print(uint8_t data)
{
  uint8_t RegPayloadLength;
  read_register(REG_PAYLOAD_LENGTH, &RegPayloadLength, 1);

  write_register(0x00, data);
  write_register(REG_PAYLOAD_LENGTH, RegPayloadLength + 1);
}

// void LoRaClass::print(char data)
// {
//   uint8_t RegPayloadLength;
//   read_register(REG_PAYLOAD_LENGTH, &RegPayloadLength, 1);

//   write_register(0x00, data);
//   write_register(REG_PAYLOAD_LENGTH, RegPayloadLength + 1);
// }

uint8_t LoRaClass::getRegValue(uint8_t reg)
{
  uint8_t val;
  read_register(reg, &val, 1);
  return val;
}

void LoRaClass::setFifoAddrPtr(uint8_t value)
{
  write_register(REG_FIFO_ADDR_PTR, value);
}

void LoRaClass::onDio0Rise()
{
  // LoRaClass.handleDio0Rise();
}
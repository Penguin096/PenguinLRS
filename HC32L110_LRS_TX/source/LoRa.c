// Copyright (c) Penguin096. All rights reserved.

#include <D:\Users\Pavel\Documents\IAR Embedded Workbench\HC32L110_LRS\source\LoRa.h>

// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_RSSI_VALUE           0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define RF_MID_BAND_THRESHOLD    525E6
#define RSSI_OFFSET_HF_PORT      157
#define RSSI_OFFSET_LF_PORT      164

#define MAX_PKT_LENGTH           255

#define ISR_PREFIX

    void LoRaexplicitHeaderMode();
    void LoRaimplicitHeaderMode();

    boolean_t LoRaisTransmitting();

    long LoRagetSignalBandwidth();

    void LoRasetLdoFlag();

    uint8_t LoRareadRegister(uint8_t address);
    void LoRawriteRegister(uint8_t address, uint8_t value);

    long _frequency;
    int _packetIndex;
    int _implicitHeaderMode;
    void (*_onReceive)(int);
    void (*_onTxDone)();

uint8_t LoRabegin(long frequency)
{
    Gpio_InitIOExt(LORA_CTRL_PORT, LORA_NCTRL_PIN, GpioDirOut, TRUE, FALSE, FALSE, FALSE); //NCTRL
    Gpio_InitIOExt(LORA_CTRL_PORT, LORA_CTRL_PIN, GpioDirOut, TRUE, FALSE, FALSE, FALSE); //CTRL
    
    // setup pins
    Gpio_SetFunc_SPICS_P14();
    // set SS high
    Spi_SetCS(TRUE);

    if (LORA_RESET_PIN != -1) {
      
    }
    
    if (LORA_RESET_PIN != -1) {
      Gpio_InitIOExt(LORA_RESET_PORT, LORA_RESET_PIN, GpioDirOut, TRUE, FALSE, TRUE, FALSE);
      Gpio_SetIO(LORA_RESET_PORT, LORA_RESET_PIN, FALSE);

      // perform reset
      Gpio_SetIO(LORA_RESET_PORT, LORA_RESET_PIN, FALSE);
      delay1ms(10);
      Gpio_SetIO(LORA_RESET_PORT, LORA_RESET_PIN, TRUE);
      delay1ms(10);
    }

    // start SPI
    Clk_SetPeripheralGate(ClkPeripheralSpi,TRUE);
    
    //Configure the MISO, MOSI, CLK.
    Gpio_SetFunc_SPIMISO_P23();
    Gpio_SetFunc_SPIMOSI_P24();
    Gpio_SetFunc_SPICLK_P15();
    
    stc_spi_config_t  SPIConfig;
    
    SPIConfig.bCPHA = Spicphafirst; //Set Most significant bit first
    SPIConfig.bCPOL = Spicpollow;
    SPIConfig.bIrqEn = FALSE;
    SPIConfig.bMasterMode = SpiMaster;
    SPIConfig.u8BaudRate = SpiClkDiv2;  // 2Mhz
    SPIConfig.pfnIrqCb = NULL;
    
    Spi_Init(&SPIConfig);

    // check version
    uint8_t version = LoRareadRegister(REG_VERSION);
    if (version != 0x12) {
      return 0;
    }

    // put in sleep mode
    LoRasleep();

    // set frequency
    LoRasetFrequency(frequency);

    // set base addresses
    LoRawriteRegister(REG_FIFO_TX_BASE_ADDR, 0);
    LoRawriteRegister(REG_FIFO_RX_BASE_ADDR, 0);

    // set LNA boost
    LoRawriteRegister(REG_LNA, LoRareadRegister(REG_LNA) | 0x03);

    // set auto AGC
    LoRawriteRegister(REG_MODEM_CONFIG_3, 0x04);

    // set output power to 17 dBm
    LoRasetTxPower(17, PA_OUTPUT_PA_BOOST_PIN);

    // put in standby mode
    LoRaidle();

  return 1;
}

void LoRaend()
{
    // put in sleep mode
    LoRasleep();

    // stop SPI
    Spi_DeInit();
}

boolean_t LoRabeginPacket(boolean_t implicitHeader)
{
    if (LoRaisTransmitting()) {
      return 0;
    }

    // put in standby mode
    LoRaidle();

    if (implicitHeader) {
      LoRaimplicitHeaderMode();
    } else {
      LoRaexplicitHeaderMode();
    }

    // reset FIFO address and paload length
    LoRawriteRegister(REG_FIFO_ADDR_PTR, 0);
    LoRawriteRegister(REG_PAYLOAD_LENGTH, 0);

    return 1;
}

int LoRaendPacket(boolean_t async)
{
    LoRaSetOpMode( MODE_TX );
    
//    if ((async) && (_onTxDone))
//        LoRawriteRegister(REG_DIO_MAPPING_1, 0x40); // DIO0 => TXDONE

    // put in TX mode
    LoRawriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

    if (!async) {
      // wait for TX done
      while ((LoRareadRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
      }
      // clear IRQ's
      LoRawriteRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    }

    return 1;
}

boolean_t LoRaisTransmitting()
{
    if ((LoRareadRegister(REG_OP_MODE) & MODE_TX) == MODE_TX) {
      return TRUE;
    }

    if (LoRareadRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) {
      // clear IRQ's
      LoRawriteRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    }

    return FALSE;
}

int LoRaparsePacket(int size)
{
    int packetLength = 0;
    int irqFlags = LoRareadRegister(REG_IRQ_FLAGS);

    if (size > 0) {
      LoRaimplicitHeaderMode();

      LoRawriteRegister(REG_PAYLOAD_LENGTH, size & 0xff);
    } else {
      LoRaexplicitHeaderMode();
    }

    // clear IRQ's
    LoRawriteRegister(REG_IRQ_FLAGS, irqFlags);

    if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
      // received a packet
      _packetIndex = 0;

      // read packet length
      if (_implicitHeaderMode) {
        packetLength = LoRareadRegister(REG_PAYLOAD_LENGTH);
      } else {
        packetLength = LoRareadRegister(REG_RX_NB_BYTES);
      }

      // set FIFO address to current RX address
      LoRawriteRegister(REG_FIFO_ADDR_PTR, LoRareadRegister(REG_FIFO_RX_CURRENT_ADDR));

      // put in standby mode
      LoRaidle();
    } else if (LoRareadRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
      // not currently in RX mode

      // reset FIFO address
      LoRawriteRegister(REG_FIFO_ADDR_PTR, 0);
      
      LoRaSetOpMode( MODE_RX_SINGLE );

      // put in single RX mode
      LoRawriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
    }

    return packetLength;
}

int16_t LoRapacketRssi()//dBm
{
    return (LoRareadRegister(REG_PKT_RSSI_VALUE) - (_frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

float LoRapacketSnr()
{
    return ((int8_t)LoRareadRegister(REG_PKT_SNR_VALUE)) * 0.25;
}

long LoRapacketFrequencyError()
{
    int32_t freqError = 0;
    freqError = (int32_t)(LoRareadRegister(REG_FREQ_ERROR_MSB) & 7);
    freqError <<= 8L;
    freqError += (int32_t)(LoRareadRegister(REG_FREQ_ERROR_MID));
    freqError <<= 8L;
    freqError += (int32_t)(LoRareadRegister(REG_FREQ_ERROR_LSB));

    if (LoRareadRegister(REG_FREQ_ERROR_MSB) & 8) { // Sign bit is on
       freqError -= 524288; // B1000'0000'0000'0000'0000
    }

    const float fXtal = 32E6; // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
    const float fError = (((float)(freqError) * (1L << 24)) / fXtal) * (LoRagetSignalBandwidth() / 500000.0f); // p. 37

    return (long)(fError);
}

int LoRarssi()
{
    return (LoRareadRegister(REG_RSSI_VALUE) - (_frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

size_t LoRawrite(const uint8_t *buffer, size_t size)
{
    int currentLength = LoRareadRegister(REG_PAYLOAD_LENGTH);

    // check size
    if ((currentLength + size) > MAX_PKT_LENGTH) {
      size = MAX_PKT_LENGTH - currentLength;
    }

    // write data
    for (size_t i = 0; i < size; i++) {
      LoRawriteRegister(REG_FIFO, buffer[i]);
    }

    // update length
    LoRawriteRegister(REG_PAYLOAD_LENGTH, currentLength + size);
    
    return size;
}

int LoRaavailable()
{
    return (LoRareadRegister(REG_RX_NB_BYTES) - _packetIndex);
}

int LoRaread()
{
    if (!LoRaavailable()) {
      return -1;
    }

    _packetIndex++;

    return LoRareadRegister(REG_FIFO);
}

int LoRapeek()
{
    if (!LoRaavailable()) {
      return -1;
    }

    // store current FIFO address
    int currentAddress = LoRareadRegister(REG_FIFO_ADDR_PTR);

    // read
    uint8_t b = LoRareadRegister(REG_FIFO);

    // restore FIFO address
    LoRawriteRegister(REG_FIFO_ADDR_PTR, currentAddress);

    return b;
}

void LoRareceive(int size)
{
    LoRaSetOpMode( MODE_RX_CONTINUOUS );

  //  LoRawriteRegister(REG_DIO_MAPPING_1, 0x00); // DIO0 => RXDONE

    if (size > 0) {
      LoRaimplicitHeaderMode();

      LoRawriteRegister(REG_PAYLOAD_LENGTH, size & 0xff);
    } else {
      LoRaexplicitHeaderMode();
    }

    LoRawriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void LoRaidle()
{
    LoRaSetOpMode( MODE_STDBY );
    LoRawriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);    
}

void LoRasleep()
{
    LoRaSetOpMode( MODE_SLEEP );
    LoRawriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}       

void LoRasetTxPower(int level, int outputPin)
{
    if (PA_OUTPUT_RFO_PIN == outputPin) {
      // RFO
      if (level < 0) {
        level = 0;
      } else if (level > 14) {
        level = 14;
      }
      
      LoRawriteRegister(REG_PA_CONFIG, 0x70 | level);
    } else {
      // PA BOOST
      if (level > 17) {
        if (level > 20) {
          level = 20;
        }
        
        // subtract 3 from level, so 18 - 20 maps to 15 - 17
        level -= 3;
        
        // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
        LoRawriteRegister(REG_PA_DAC, 0x87);
        LoRasetOCP(140);
      } else {
        if (level < 2) {
          level = 2;
        }
        //Default value PA_HF/LF or +17dBm
        LoRawriteRegister(REG_PA_DAC, 0x84);
        LoRasetOCP(100);
      }
      
      LoRawriteRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
    }
}

void LoRasetFrequency(long frequency)
{
    _frequency = frequency;

    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

    LoRawriteRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
    LoRawriteRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
    LoRawriteRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

int LoRagetSpreadingFactor()
{
    return LoRareadRegister(REG_MODEM_CONFIG_2) >> 4;
}

void LoRasetSpreadingFactor(int sf)
{
    if (sf < 6) {
      sf = 6;
    } else if (sf > 12) {
      sf = 12;
    }

    if (sf == 6) {
      LoRawriteRegister(REG_DETECTION_OPTIMIZE, 0xc5);
      LoRawriteRegister(REG_DETECTION_THRESHOLD, 0x0c);
    } else {
      LoRawriteRegister(REG_DETECTION_OPTIMIZE, 0xc3);
      LoRawriteRegister(REG_DETECTION_THRESHOLD, 0x0a);
    }

    LoRawriteRegister(REG_MODEM_CONFIG_2, (LoRareadRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
    LoRasetLdoFlag();
}

long LoRagetSignalBandwidth()
{
  int bw = (LoRareadRegister(REG_MODEM_CONFIG_1) >> 4);

  switch (bw) {
    case 0: return 7.8E3;
    case 1: return 10.4E3;
    case 2: return 15.6E3;
    case 3: return 20.8E3;
    case 4: return 31.25E3;
    case 5: return 41.7E3;
    case 6: return 62.5E3;
    case 7: return 125E3;
    case 8: return 250E3;
    case 9: return 500E3;
  }

  return -1;
}

void LoRasetSignalBandwidth(long sbw)
{
  int bw;

  if (sbw <= 7.8E3) {
    bw = 0;
  } else if (sbw <= 10.4E3) {
    bw = 1;
  } else if (sbw <= 15.6E3) {
    bw = 2;
  } else if (sbw <= 20.8E3) {
    bw = 3;
  } else if (sbw <= 31.25E3) {
    bw = 4;
  } else if (sbw <= 41.7E3) {
    bw = 5;
  } else if (sbw <= 62.5E3) {
    bw = 6;
  } else if (sbw <= 125E3) {
    bw = 7;
  } else if (sbw <= 250E3) {
    bw = 8;
  } else /*if (sbw <= 250E3)*/ {
    bw = 9;
  }

  LoRawriteRegister(REG_MODEM_CONFIG_1, (LoRareadRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
  LoRasetLdoFlag();
}

void LoRasetLdoFlag()
{
    // Section 4.1.1.5
    long symbolDuration = 1000 / ( LoRagetSignalBandwidth() / (1L << LoRagetSpreadingFactor()) ) ;

    // Section 4.1.1.6
    boolean_t ldoOn = symbolDuration > 16;

    uint8_t config3 = LoRareadRegister(REG_MODEM_CONFIG_3);
    if(ldoOn) config3 |= (1 << 3);
    else config3 &= ~(1 << 3);
    LoRawriteRegister(REG_MODEM_CONFIG_3, config3);
}

void LoRasetCodingRate4(int denominator)
{
    if (denominator < 5) {
      denominator = 5;
    } else if (denominator > 8) {
      denominator = 8;
    }

    int cr = denominator - 4;

    LoRawriteRegister(REG_MODEM_CONFIG_1, (LoRareadRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void LoRasetPreambleLength(long length)
{
    LoRawriteRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
    LoRawriteRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void LoRasetSyncWord(int sw)
{
  LoRawriteRegister(REG_SYNC_WORD, sw);
}

void LoRaenableCrc()
{
    LoRawriteRegister(REG_MODEM_CONFIG_2, LoRareadRegister(REG_MODEM_CONFIG_2) | 0x04);
}

void LoRadisableCrc()
{
    LoRawriteRegister(REG_MODEM_CONFIG_2, LoRareadRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

void LoRaenableInvertIQ()
{
    LoRawriteRegister(REG_INVERTIQ,  0x66);
    LoRawriteRegister(REG_INVERTIQ2, 0x19);
}

void LoRadisableInvertIQ()
{
    LoRawriteRegister(REG_INVERTIQ,  0x27);
    LoRawriteRegister(REG_INVERTIQ2, 0x1d);
}

void LoRasetOCP(uint8_t mA)
{
    uint8_t ocpTrim = 27;

    if (mA <= 120) {
      ocpTrim = (mA - 45) / 5;
    } else if (mA <=240) {
      ocpTrim = (mA + 30) / 10;
    }

    LoRawriteRegister(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

void LoRasetGain(uint8_t gain)
{
    // check allowed range
    if (gain > 6) {
      gain = 6;
    }
    
    // set to standby
    LoRaidle();
    
    // set gain
    if (gain == 0) {
      // if gain = 0, enable AGC
      LoRawriteRegister(REG_MODEM_CONFIG_3, 0x04);
    } else {
      // disable AGC
      LoRawriteRegister(REG_MODEM_CONFIG_3, 0x00);
          
      // clear Gain and set LNA boost
      LoRawriteRegister(REG_LNA, 0x03);
          
      // set gain
      LoRawriteRegister(REG_LNA, LoRareadRegister(REG_LNA) | (gain << 5));
    }
}

int LoRarandom()
{
    return LoRareadRegister(REG_RSSI_WIDEBAND);
}

void LoRaexplicitHeaderMode()
{
    _implicitHeaderMode = 0;

    LoRawriteRegister(REG_MODEM_CONFIG_1, LoRareadRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

void LoRaimplicitHeaderMode()
{
    _implicitHeaderMode = 1;

    LoRawriteRegister(REG_MODEM_CONFIG_1, LoRareadRegister(REG_MODEM_CONFIG_1) | 0x01);
}

void LoRahandleDio0Rise()
{
    int irqFlags = LoRareadRegister(REG_IRQ_FLAGS);

    // clear IRQ's
    LoRawriteRegister(REG_IRQ_FLAGS, irqFlags);

    if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {

      if ((irqFlags & IRQ_RX_DONE_MASK) != 0) {
        // received a packet
        _packetIndex = 0;

        // read packet length
        int packetLength = _implicitHeaderMode ? LoRareadRegister(REG_PAYLOAD_LENGTH) : LoRareadRegister(REG_RX_NB_BYTES);

        // set FIFO address to current RX address
        LoRawriteRegister(REG_FIFO_ADDR_PTR, LoRareadRegister(REG_FIFO_RX_CURRENT_ADDR));

        if (_onReceive) {
          _onReceive(packetLength);
        }
      }
      else if ((irqFlags & IRQ_TX_DONE_MASK) != 0) {
        if (_onTxDone) {
          _onTxDone();
        }
      }
    }
}

uint8_t LoRareadRegister(uint8_t address)
{
    uint8_t response;
    
    Spi_SetCS(FALSE);

    Spi_SendData(address & 0x7f);
    response = Spi_ReceiveData();

    Spi_SetCS(TRUE);
    
    return response;
}

void LoRawriteRegister(uint8_t address, uint8_t value)
{
    Spi_SetCS(FALSE);

    Spi_SendData(address | 0x80);
    Spi_SendData(value);

    Spi_SetCS(TRUE);
}

void LoRaSetOpMode( uint8_t opMode )
{
    if( opMode == MODE_SLEEP )
    {
        Gpio_SetIO(3, 3, FALSE);
        Gpio_SetIO(3, 4, FALSE);
    }
    else
    {
        if( opMode == MODE_TX )
        {
            Gpio_SetIO(3, 3, TRUE);
            Gpio_SetIO(3, 4, FALSE);
        }
        else
        {
            Gpio_SetIO(3, 3, FALSE);
            Gpio_SetIO(3, 4, TRUE);
        }
    }
}

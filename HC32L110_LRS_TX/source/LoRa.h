// Copyright (c) Penguin096. All rights reserved.

#ifndef LORA_H
#define LORA_H
/*****************************************************************************
 * Include files
 *****************************************************************************/
#include "spi.h"
     
#ifdef __cplusplus
extern "C"
{
#endif
 
#define LORA_CTRL_PORT                  3
#define LORA_NCTRL_PIN                  3
#define LORA_CTRL_PIN                   4
  
#define LORA_RESET_PORT                 3
#define LORA_RESET_PIN                  2

#define PA_OUTPUT_RFO_PIN               0
#define PA_OUTPUT_PA_BOOST_PIN          1

#ifdef __cplusplus
#endif

    uint8_t LoRabegin(long frequency);
    void LoRaend();

    boolean_t LoRabeginPacket(boolean_t implicitHeader);
    int LoRaendPacket(boolean_t async);

    int LoRaparsePacket(int size);
    int16_t LoRapacketRssi();
    float LoRapacketSnr();
    long LoRapacketFrequencyError();

    int LoRarssi();

    // from Print
    size_t LoRawrite(const uint8_t *buffer, size_t size);

    // from Stream
    int LoRaavailable();
    int LoRaread();
    int LoRapeek();

    void LoRareceive(int size);

    void LoRaidle();
    void LoRasleep();

    void LoRasetTxPower(int level, int outputPin);
    void LoRasetFrequency(long frequency);
    void LoRasetSpreadingFactor(int sf);
    void LoRasetSignalBandwidth(long sbw);
    void LoRasetCodingRate4(int denominator);
    void LoRasetPreambleLength(long length);
    void LoRasetSyncWord(int sw);
    void LoRasetOCP(uint8_t mA); // Over Current Protection control
    void LoRasetGain(uint8_t gain); // Set LNA gain   
    void LoRaSetOpMode( uint8_t opMode );
    
    void LoRaenableCrc();
    void LoRadisableCrc();
    void LoRaenableInvertIQ();
    void LoRadisableInvertIQ();

    int LoRarandom();
  
#endif /* __UART_H__ */
/******************************************************************************
 * EOF (not truncated)
 *****************************************************************************/

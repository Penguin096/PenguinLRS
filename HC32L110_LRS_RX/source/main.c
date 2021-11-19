#include "clk.h"
#include "millis.h"
#include "gpio.h"
#include "ddl.h"
#include "wdt.h"
#include "uart.h"
#include "bt.h"
#include "Serial.h"
#include "LoRa.h"
#include "crc.h"
#include "flash.h"

//############################################
#define Red_LED_ON Gpio_SetIO(0, 3, TRUE)
#define Red_LED_OFF Gpio_SetIO(0, 3, FALSE)
//#######################################
#define SBUS_MIN_VALUE 173   // 
#define SBUS_MAX_VALUE 1811  // 
#define SBUS_UPDATE_RATE 14  //ms NORM SPEED:14   HIGH SPEED:7
//#######################################

//Addr
#define TX_OUTPUT_POWER         0x3ff0  // dBm  
#define LORA_BANDWIDTH          0x3ff2  // [ 125 kHz, 
                                        //   250 kHz,
                                        //   500 kHz,
#define LORA_SPREADING_FACTOR   0x3ff4  // [SF7..SF12] 
#define LORA_CODINGRATE         0x3ff6  // [4/5,
                                        //  4/6,
                                        //  4/7,
                                        //  4/8]
#define SyncWord                0x3ff8  // SyncWord 

#define FAILSAFE

static uint16_t failsafeCnt = 0;
static uint16_t rssi;

static uint32_t sbusTime = 0;
static uint8_t sbusPacket[25];
uint8_t SerialBuff[4];
boolean_t isFailsafe;

uint8_t RFch;

uint8_t packet[20];

boolean_t Read_Packet(void);
void sbusPreparePacket(boolean_t isSignalLoss, boolean_t isFailsafe);

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void FlashInt(void)
 {
    if (TRUE == Flash_GetIntFlag(flash_int0))
    {
        Flash_ClearIntFlag(flash_int0);
//        u32FlashTestFlag |= 0x01;
        Flash_DisableIrq(flash_int0);
    }
    if (TRUE == Flash_GetIntFlag(flash_int1))
    {
        Flash_ClearIntFlag(flash_int1);
//        u32FlashTestFlag |= 0x02;
        Flash_DisableIrq(flash_int1);
    }
      
 }

en_result_t BtTimer2Init(void)
{
    stc_bt_config_t   stcConfig;
    en_result_t       enResult = Ok;
    uint16_t u16ArrData = 0x10000 - 4000;
    uint16_t u16InitCntData = 0x10000 - 4000;
    
    DDL_ZERO_STRUCT(stcConfig);
    
    stcConfig.pfnTim2Cb = Bt2Int;
       
    stcConfig.enGateP = BtPositive;
    stcConfig.enGate  = BtGateDisable;
    stcConfig.enPRS   = BtPCLKDiv4;
    stcConfig.enTog   = BtTogDisable;
    stcConfig.enCT    = BtTimer;
    stcConfig.enMD    = BtMode2;

    if (Ok != Bt_Init(TIM2, &stcConfig))
    {
        enResult = Error;
    }
    
    //TIM2
    Bt_ClearIntFlag(TIM2);
    Bt_EnableIrq(TIM2);
    EnableNvic(TIM2_IRQn, 3, TRUE);
    
    //
    Bt_ARRSet(TIM2, u16ArrData);
    Bt_Cnt16Set(TIM2, u16InitCntData);
    Bt_Run(TIM2); 
    
    return enResult;
}

int32_t main(void)
{
  ////////////////////////WDT Init////////////////////////  
  stc_wdt_config_t  stcWdt_Config;
  
  DDL_ZERO_STRUCT(stcWdt_Config); 
   
  stcWdt_Config.u8LoadValue = 0x06;//102ms
  stcWdt_Config.enResetEnable = WRESET_EN;//
  stcWdt_Config.pfnWdtIrqCb = NULL;
  
  Clk_SetPeripheralGate(ClkPeripheralWdt,TRUE);//
  Wdt_Init(&stcWdt_Config);
  //////////////////////////////////////////////////////// 
  
  stc_clk_config_t stcClkCfg;
  
  // <Before the clock initialization, you can set the clock source to use: set the RCH 24MHz here (the default is 4MHz)
  Clk_SetRCHFreq(ClkFreq16Mhz);
  
  stcClkCfg.enClkSrc  = ClkRCH; // <Select internal RCH as an HCLK clock source;
  stcClkCfg.enHClkDiv = ClkDiv1;
  stcClkCfg.enPClkDiv = ClkDiv1;
  
  Clk_Init(&stcClkCfg);
  
  /* open peripheral clk */
  Clk_SetPeripheralGate(ClkPeripheralGpio, TRUE);
  Clk_SetPeripheralGate(ClkPeripheralBt, TRUE);
  Clk_SetPeripheralGate(ClkPeripheralWdt,TRUE);
  Clk_SetPeripheralGate(ClkPeripheralCrc, TRUE);
  ////////////////////////TIM Init////////////////////////    
  BtTimer2Init();
  //////////////////////////////////////////////////////// 
   
  ////////////////////////////////////////////////////////
  Gpio_InitIOExt(0, 3, GpioDirOut, FALSE, FALSE, FALSE, FALSE); //AUX  
  Gpio_InitIOExt(3, 5, GpioDirIn, TRUE, FALSE, FALSE, FALSE); //M0
  Gpio_InitIO(3, 6, GpioDirIn); //M1
  ////////////////////////////////////////////////////////
  
  if(Gpio_GetIO(3, 5) == 0) {//M0
    Serial_begin(UARTCH0, 4800); 
    uint32_t previousMillis = 0;
    boolean_t ledState = FALSE;
    while(1) {
      if(Uart_GetStatus(UARTCH0,UartRxFull)) {
        uint8_t q = 0;
        uint8_t u8RxData [10];
        while (q < 10) {
          u8RxData[q] = Uart_ReceiveData(UARTCH0);
          Uart_ClrStatus(UARTCH0,UartRxFull);
          if (u8RxData[q] == 0x0A) {
            break;
          }
          if (u8RxData[0] == 0x55) {
            q++;
            while (Uart_GetStatus(UARTCH0,UartRxFull) == 0);
          }
        }

        if (u8RxData[1] == 0xDD) {//Read All
          Uart_SendData(UARTCH0, 0x55);
          Uart_SendData(UARTCH0, 0xDC);
          uint8_t u8TxData[6] = {*((volatile uint8_t*)TX_OUTPUT_POWER), (*((volatile uint16_t*)LORA_BANDWIDTH) >> 8), *((volatile uint16_t*)LORA_BANDWIDTH), *((volatile uint8_t*)LORA_SPREADING_FACTOR), *((volatile uint8_t*)LORA_CODINGRATE), *((volatile uint8_t*)SyncWord)};
          for (uint8_t i = 0; i < 6; i++) {
            Uart_SendData(UARTCH0, u8TxData[i]);              
          }
          Uart_SendData(UARTCH0, 0x0A);
        }else if (u8RxData[1] == 0xDB) {//Save
                   
          Flash_Init(FlashInt, 2);
          
          Flash_SectorErase(TX_OUTPUT_POWER);
          
          Flash_WriteByte(TX_OUTPUT_POWER, u8RxData[2]);          
          Flash_WriteHalfWord(LORA_BANDWIDTH, (u8RxData[3]<<8) | u8RxData[4]);
          Flash_WriteByte(LORA_SPREADING_FACTOR, u8RxData[5]);   
          Flash_WriteByte(LORA_CODINGRATE, u8RxData[6]);   
          Flash_WriteByte(SyncWord, u8RxData[7]); 
        }
        Uart_ClrStatus(UARTCH0,UartRxFull);
      }
      
      if(millis() - previousMillis > 200) {
        previousMillis = millis();  
        if (ledState == FALSE)
          ledState = TRUE;
        else
          ledState = FALSE; 
        Gpio_SetIO(0, 3, ledState);
      }
    }
  }
  
  Serial_begin(UARTCH0, 100000); 
  Wdt_Start();
  
  if (!LoRabegin(868E6)) {
#ifdef __DEBUG
    Serial_println_Char(UARTCH0, "Starting LoRa failed!");
#endif
    while (1);
  }    
#ifdef __DEBUG
    else {
      uint16_t i=0;    
      char s [] = "Starting LoRa done!";
      while(i<strlen(s))
      {
        Uart_SendData(UARTCH0,s[i]);
        i++;
      }
      Uart_SendData(UARTCH0,0x0D);
      Uart_SendData(UARTCH0,0x0A);
    }
#endif
  
    //read Flash
    // put in sleep mode
    LoRasleep();
    LoRasetTxPower(*((volatile uint8_t*)TX_OUTPUT_POWER), PA_OUTPUT_PA_BOOST_PIN);
    LoRasetSpreadingFactor(*((volatile uint8_t*)LORA_SPREADING_FACTOR));
    LoRasetSignalBandwidth(*((volatile uint16_t*)LORA_BANDWIDTH));
    LoRasetCodingRate4(*((volatile uint8_t*)LORA_CODINGRATE));
    // put in standby mode
    LoRaidle();
    
    LoRasetSyncWord(SyncWord);           // ranges from 0-0xFF, default 0x34, see API docs
        
    LoRareceive(0);

    while (1)
    {
      if ((millis() - sbusTime) > SBUS_UPDATE_RATE) {
        sbusTime = millis();
        sbusPreparePacket(FALSE, isFailsafe);
        
        // Reset Watchdog Timer
        Wdt_Feed();
        
        for (uint8_t i = 0; i < 25; i++) {
          Uart_SendData(UARTCH0, sbusPacket[i]);              
        }
      }
      
      unsigned long pause = millis();
      while (1) {
        
#if defined(FAILSAFE)
        if (failsafeCnt > 25) {  //fs delay 1000ms
          failsafeCnt = 0;
          isFailsafe = TRUE;
        }
#endif
        
        if ((millis() - pause) > 40) {
          Red_LED_OFF;
          failsafeCnt++;
          break;
        }

        // try to parse packet
        if (LoRaparsePacket(FALSE) == 0) {          
          continue;
        }
      
        if (Read_Packet() == 0) {          
          continue;
        }         
        
        RFch = packet[12] & 0x0F;
        
        for (uint8_t i = 0; i < 4; i++) {
          SerialBuff [i] = packet[13+i];
        } 
  
        Red_LED_ON;
        failsafeCnt = 0;
        isFailsafe = FALSE;
        break;
      }
    }
}

boolean_t Read_Packet(void) {
  uint8_t i = 0;
  while (LoRaavailable()) {
    packet[i] = LoRaread();  
    i++;
    if(i == 20) {
      rssi = map(LoRapacketRssi()+157, 0, 157, SBUS_MIN_VALUE, SBUS_MAX_VALUE);
      uint16_t Crc16 = 0;
      Crc16 = packet[18]<<8;
      Crc16 |= packet[19];
      if (CRC16_Get8(packet, 18) == Crc16) {
        return 1;
      } else {
        return 0;
      }
    }
  }
  return 0;
}

void sbusPreparePacket(boolean_t isSignalLoss, boolean_t isFailsafe) {
  memset(sbusPacket, 0x00, 25);  //Zero out packet data
  sbusPacket[0] = 0x0F;          //Header
  sbusPacket[24] = 0x00;         //Footer 0x00 for SBUS
  
  uint8_t SBUS_Current_Packet_Bit = 0;
  uint8_t SBUS_Packet_Position = 1;
  
  uint16_t sbusval;
  
  if (!isFailsafe) {
    for (uint8_t i = 0; i<11; i++) {
      sbusPacket[1+i] = packet[i]; 
    }
    
    sbusPacket[10] &= ~0xE0;
    sbusPacket[10] |= rssi << 5;
    sbusPacket[11] = rssi >> 3;
    
    for (uint8_t i = 0; i<4; i++) {
      if (packet[11] & (1 << i)) {
        sbusval = SBUS_MAX_VALUE;
      } else {
        sbusval = SBUS_MIN_VALUE;      
      }
    }
  } else {
    
    for (uint8_t i = 0; i<8; i++) {
      if (i == 7 || i == 2) {
        sbusval = SBUS_MIN_VALUE;
      } else sbusval = 992;
      
      for (uint8_t SBUS_Current_Channel_Bit = 0; SBUS_Current_Channel_Bit < 11; SBUS_Current_Channel_Bit++) {
        if (SBUS_Current_Packet_Bit > 7) {
          SBUS_Current_Packet_Bit = 0;  //If we just set bit 7 in a previous step, reset the packet bit to 0 and
          SBUS_Packet_Position++;       //Move to the next packet uint8_t
        }
        sbusPacket[SBUS_Packet_Position] |= (((sbusval >> SBUS_Current_Channel_Bit) & 0x1) << SBUS_Current_Packet_Bit);  //Downshift the channel data bit, then upshift it to set the packet data uint8_t
        SBUS_Current_Packet_Bit++;
      } 
    }
  }
  if (packet[12] & (1 << 0)) sbusPacket[23] |= (1 << 0);
  if (packet[12] & (1 << 1)) sbusPacket[23] |= (1 << 1);
  if (isSignalLoss) sbusPacket[23] |= (1 << 2);
  if (isFailsafe) sbusPacket[23] |= (1 << 3);
}
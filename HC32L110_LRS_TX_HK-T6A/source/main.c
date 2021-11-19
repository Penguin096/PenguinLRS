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

//#######################################
#define RC_CHANNEL_MIN 1155   // Servo rx minimum position 
#define RC_CHANNEL_MAX 1929  // Servo rx maximum position 
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

//////////////////////CONFIGURATION///////////////////////////////
#define chanel_number 8  //set the number of chanels
//////////////////////////////////////////////////////////////////

uint8_t packet[18];
uint8_t SerialBuff[4] = "HELO";
int ledState = FALSE;
//uint32_t previousMillis2 = 0;
volatile boolean_t RxFlg;
uint8_t Packet[20];
volatile uint8_t IDX;

void PreparePacket(boolean_t digitalCH1, boolean_t digitalCH2, boolean_t isSignalLoss, boolean_t isFailsafe);

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
  uint16_t u16ArrData = 0x10000 - 5530;
  uint16_t u16InitCntData = 0x10000 - 5530;
  
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

void RxIntCallback(void)
{     
  packet[IDX] = Uart_ReceiveData(UARTCH0);
  Uart_ClrStatus(UARTCH0,UartRxFull);
  
  if(IDX == 0 && packet[0] != 0x55) return;
  if(IDX == 1 && packet[1] != 0xFC) 
  {
    IDX = 0;
    return;
  }
  
  IDX++;
  
  if (IDX >= 18)
  {
    IDX = 0;
    RxFlg = TRUE;
  }
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
  Clk_SetRCHFreq(ClkFreq22_12Mhz);
  
  stcClkCfg.enClkSrc  = ClkRCH; // <Select internal RCH as an HCLK clock source;
  stcClkCfg.enHClkDiv = ClkDiv1;
  stcClkCfg.enPClkDiv = ClkDiv1;
  
  Clk_Init(&stcClkCfg);
  
  /* open peripheral clk */
  Clk_SetPeripheralGate(ClkPeripheralGpio, TRUE);
  Clk_SetPeripheralGate(ClkPeripheralBt, TRUE);
  Clk_SetPeripheralGate(ClkPeripheralCrc, TRUE);
  ////////////////////////TIM Init////////////////////////    
  BtTimer2Init();
  //////////////////////////////////////////////////////// 
  
  ////////////////////////////////////////////////////////
  Gpio_InitIOExt(0, 3, GpioDirOut, TRUE, FALSE, FALSE, FALSE); //AUX  
  Gpio_InitIO(3, 5, GpioDirIn); //M0
  Gpio_InitIO(3, 6, GpioDirIn); //M1
  ////////////////////////////////////////////////////////
  
  if(Gpio_GetIO(3, 5) == 0) {//M0
    Serial_begin(UARTCH0, 4800); 
    Uart_DisableIrq(UARTCH0, UartRxIrq);
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
          
          Flash_Init(FlashInt, 3);
          
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
  
  Serial_begin(UARTCH0, 115200); 
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
  
  while (1)
  {
    //    if(RxFlg && (millis() - previousMillis2 > 10)) {
    if(RxFlg) {      
      
      Uart_DisableIrq(UARTCH0,UartRxIrq);
      
      PreparePacket(FALSE, FALSE, FALSE, FALSE);
      
      //      previousMillis2 = millis(); 
      
      LoRabeginPacket(FALSE);
      LoRawrite(Packet, 20);      
      
      RxFlg = FALSE;
      
      LoRaendPacket(FALSE);
      LoRaidle();
      
      if (ledState == FALSE)
        ledState = TRUE;
      else
        ledState = FALSE; 
      Gpio_SetIO(0, 3, ledState);
      
      Uart_ClrStatus(UARTCH0,UartRxFull);
      Uart_EnableIrq(UARTCH0,UartRxIrq);
    }
    
    Wdt_Feed();
  }
}

void PreparePacket(boolean_t digitalCH1, boolean_t digitalCH2, boolean_t isSignalLoss, boolean_t isFailsafe) {
  memset(Packet, 0x00, 20);  //Zero out packet data
  
  uint8_t Current_Packet_Bit = 0;
  uint8_t Packet_Position = 0;
  
  for (uint8_t Current_Channel = 0; Current_Channel < 8; Current_Channel++) {
    
    uint16_t val;
    if (isFailsafe) {
      if (Current_Channel == 2) {
        val = RC_CHANNEL_MIN;
      } else val = 1543;
    } else {
      if(Current_Channel < 6){/////////////////for HK
        val = packet[2 + (2 * Current_Channel)] << 8 | packet[3 + (2 * Current_Channel)];
      }
      val = min(val, RC_CHANNEL_MAX);
      val = max(val, RC_CHANNEL_MIN);
    }
    
    val = map(val, RC_CHANNEL_MIN, RC_CHANNEL_MAX, SBUS_MIN_VALUE, SBUS_MAX_VALUE);
    
    for (uint8_t Current_Channel_Bit = 0; Current_Channel_Bit < 11; Current_Channel_Bit++) {
      if (Current_Packet_Bit > 7) {
        Current_Packet_Bit = 0;  //If we just set bit 7 in a previous step, reset the packet bit to 0 and
        Packet_Position++;       //Move to the next packet uint8_t
      }
      Packet[Packet_Position] |= (((val >> Current_Channel_Bit) & 0x1) << Current_Packet_Bit);  //Downshift the channel data bit, then upshift it to set the packet data uint8_t
      Current_Packet_Bit++;
    }
  }
  
  //  if (digitalCH9 > (RC_CHANNEL_MAX/2)) Packet[11] |= (1 << 0);
  //  if (digitalCH10 > (RC_CHANNEL_MAX/2)) Packet[11] |= (1 << 1);
  //  if (digitalCH11 > (RC_CHANNEL_MAX/2)) Packet[11] |= (1 << 2);
  //  if (digitalCH12 > (RC_CHANNEL_MAX/2)) Packet[11] |= (1 << 3);
  //  if (digitalCH13 > (RC_CHANNEL_MAX/2)) Packet[11] |= (1 << 4);
  //  if (digitalCH14 > (RC_CHANNEL_MAX/2)) Packet[11] |= (1 << 5);
  //  if (digitalCH15 > (RC_CHANNEL_MAX/2)) Packet[11] |= (1 << 6);
  //  if (digitalCH16 > (RC_CHANNEL_MAX/2)) Packet[11] |= (1 << 7);
  
  //  if (digitalCH1) Packet[12] |= (1 << 0);
  //  if (digitalCH2) Packet[12] |= (1 << 1);
  //  if (isSignalLoss) Packet[12] |= (1 << 2);
  //  if (isFailsafe) Packet[12] |= (1 << 3);
  
  //  if (RFCH1) Packet[12] |= (1 << 4);
  //  if (RFCH2) Packet[12] |= (1 << 5);
  //  if (RFCH3) Packet[12] |= (1 << 6);
  //  if (RFCH4) Packet[12] |= (1 << 7);
  
  for (uint8_t i = 0; i < 4; i++) {
    Packet[13+i] = SerialBuff [i];
  } 
  
  uint16_t Crc16 = 0;
  Crc16 =  CRC16_Get8(Packet, 18);
  Packet[18] = Crc16 >> 8;
  Packet[19] = Crc16;
}
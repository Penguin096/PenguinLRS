#include "clk.h"
#include "millis.h"
#include "gpio.h"
#include "ddl.h"
#include "wdt.h"
#include "uart.h"
#include "bt.h"
//#include "lpm.h"
#include "Serial.h"
#include "LoRa.h"

#define TX_OUTPUT_POWER                             20        // dBm
#define LORA_BANDWIDTH                              250E3     // [ 125 kHz,
                                                              //   250 kHz,
                                                              //   500 kHz,

#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             8         // [5: 4/5,
                                                              //  6: 4/6,
                                                              //  7: 4/7,
                                                              //  8: 4/8]

#define BufferSize                                  18

//############################################
#define Red_LED_ON Gpio_SetIO(0, 3, TRUE)
#define Red_LED_OFF Gpio_SetIO(0, 3, FALSE)
//#######################################
#define RC_CHANNEL_MIN 1155   // Servo rx minimum position 
#define RC_CHANNEL_MAX 1929  // Servo rx maximum position 
#define SBUS_MIN_VALUE 173   // 
#define SBUS_MAX_VALUE 1811  // 
#define SBUS_UPDATE_RATE 14  //ms NORM SPEED:14   HIGH SPEED:7
//#######################################

#define FAILSAFE
//////////////////////CONFIGURATION///////////////////////////////
#define chanel_number 6  //set the number of chanels
//////////////////////////////////////////////////////////////////

static uint16_t failsafeCnt = 0;
static uint16_t rssi;

static uint32_t sbusTime = 0;
static uint8_t sbusPacket[25];
boolean_t isFailsafe;

//char u8TxData [] = "HUADA MCU!";
//uint8_t Buffer[BufferSize] = "1234567890";

//volatile boolean_t RxFlg;
//volatile unsigned char IDX;
uint8_t packet[BufferSize];

boolean_t Read_Packet(void);
void sbusPreparePacket(boolean_t digitalCH1, boolean_t digitalCH2, boolean_t isSignalLoss, boolean_t isFailsafe);


long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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

//void RxIntCallback(void)
//{     
//    packet[IDX] = Uart_ReceiveData(UARTCH0);
//    Uart_ClrStatus(UARTCH0,UartRxFull);
//    
//    if(IDX == 0 && packet[0] != 0x55) return;
//    if(IDX == 1 && packet[1] != 0xFC) 
//    {
//      IDX = 0;
//      return;
//    }
//    
//    IDX++;
//    
//    if (IDX >= 18)
//    {
//      IDX = 0;
//      RxFlg = TRUE;
//    }
//}

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
  
  Wdt_Start();
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
  
  ////////////////////////TIM Init////////////////////////    
  BtTimer2Init();
  //////////////////////////////////////////////////////// 
  
  Serial_begin(UARTCH0, 100000); 
  
  ////////////////////////////////////////////////////////
  Gpio_InitIOExt(0, 3, GpioDirOut, TRUE, FALSE, FALSE, FALSE); //AUX  
  Gpio_InitIO(3, 5, GpioDirIn); //M0
  Gpio_InitIO(3, 6, GpioDirIn); //M1
  ////////////////////////////////////////////////////////
  
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
    // put in sleep mode
    LoRasleep();
    LoRasetTxPower(TX_OUTPUT_POWER, PA_OUTPUT_PA_BOOST_PIN);
    LoRasetSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRasetSignalBandwidth(LORA_BANDWIDTH);
    LoRasetCodingRate4(LORA_CODINGRATE);
    // put in standby mode
    LoRaidle();
    
    LoRasetSyncWord(0xF3);           // ranges from 0-0xFF, default 0x34, see API docs
    
    LoRareceive(0);

    while (1)
    {
      if ((millis() - sbusTime) > SBUS_UPDATE_RATE) {
        sbusTime = millis();
        sbusPreparePacket(FALSE, FALSE, FALSE, isFailsafe);
        
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
    if(i == 0 && packet[0] != 0x55) continue;
    if(i == 1 && packet[1] != 0xFC) 
    {
      i = 0;
      break;
    }
    i++;
    if(i == 18) {
      rssi = map(LoRapacketRssi()+157, 0, 157, RC_CHANNEL_MIN, RC_CHANNEL_MAX);
      return 1;
    }
  }
  return 0;
}

void sbusPreparePacket(boolean_t digitalCH1, boolean_t digitalCH2, boolean_t isSignalLoss, boolean_t isFailsafe) {
  // Serial.write(packet, 18);
  memset(sbusPacket, 0x00, 25);  //Zero out packet data
  sbusPacket[0] = 0x0F;          //Header
  sbusPacket[24] = 0x00;         //Footer 0x00 for SBUS

  uint8_t SBUS_Current_Packet_Bit = 0;
  uint8_t SBUS_Packet_Position = 1;

  for (uint8_t SBUS_Current_Channel = 0; SBUS_Current_Channel < chanel_number + 1; SBUS_Current_Channel++) {

    uint16_t sbusval;
    if (isFailsafe) {
      if (SBUS_Current_Channel == chanel_number || SBUS_Current_Channel == 2) {
        sbusval = RC_CHANNEL_MIN;
      } else sbusval = 1543;
    } else {
      if (SBUS_Current_Channel != chanel_number) {
        sbusval = packet[2 + (2 * SBUS_Current_Channel)] << 8 | packet[3 + (2 * SBUS_Current_Channel)];
      } else {
        sbusval = rssi;
      }
      sbusval = min(sbusval, RC_CHANNEL_MAX);
      sbusval = max(sbusval, RC_CHANNEL_MIN);
    }

    sbusval = map(sbusval, RC_CHANNEL_MIN, RC_CHANNEL_MAX, SBUS_MIN_VALUE, SBUS_MAX_VALUE);

    for (uint8_t SBUS_Current_Channel_Bit = 0; SBUS_Current_Channel_Bit < 11; SBUS_Current_Channel_Bit++) {
      if (SBUS_Current_Packet_Bit > 7) {
        SBUS_Current_Packet_Bit = 0;  //If we just set bit 7 in a previous step, reset the packet bit to 0 and
        SBUS_Packet_Position++;       //Move to the next packet uint8_t
      }
      sbusPacket[SBUS_Packet_Position] |= (((sbusval >> SBUS_Current_Channel_Bit) & 0x1) << SBUS_Current_Packet_Bit);  //Downshift the channel data bit, then upshift it to set the packet data uint8_t
      SBUS_Current_Packet_Bit++;
    }
  }

  if (digitalCH1) sbusPacket[23] |= (1 << 0);
  if (digitalCH2) sbusPacket[23] |= (1 << 1);
  if (isSignalLoss) sbusPacket[23] |= (1 << 2);
  if (isFailsafe) sbusPacket[23] |= (1 << 3);
}

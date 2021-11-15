#include "clk.h"
#include "millis.h"
#include "gpio.h"
#include "ddl.h"
#include "wdt.h"
#include "uart.h"
#include "bt.h"
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

#define BufferSize                                  32

//char u8TxData [] = "HUADA MCU!";
//uint8_t Buffer[10] = "1234567890";
int ledState = FALSE;
uint32_t previousMillis = 0;
uint32_t previousMillis2 = 0;
volatile boolean_t RxFlg;
volatile unsigned char IDX;
uint8_t packet[BufferSize];

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
uint8_t Da=0xff;
//static void WdtCallback(void)
//{
//    // comment following to demonstrate the hardware watchdog reset
//    Da = ~Da;
//    Gpio_SetIO(0,3,Data);
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
  Clk_SetRCHFreq(ClkFreq22_12Mhz);
  
  stcClkCfg.enClkSrc  = ClkRCH; // <Select internal RCH as an HCLK clock source;
  stcClkCfg.enHClkDiv = ClkDiv1;
  stcClkCfg.enPClkDiv = ClkDiv1;
  
  Clk_Init(&stcClkCfg);
  
  /* open peripheral clk */
  Clk_SetPeripheralGate(ClkPeripheralGpio, TRUE);
  Clk_SetPeripheralGate(ClkPeripheralBt, TRUE);
  
  ////////////////////////TIM Init////////////////////////    
  BtTimer2Init();
  //////////////////////////////////////////////////////// 
  
  Serial_begin(UARTCH0, 115200); 
  
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

  while (1)
  {
//    if(RxFlg && (millis() - previousMillis2 > 10)) {
    if(RxFlg) {      
      
      Uart_DisableIrq(UARTCH0,UartRxIrq);
      
//      previousMillis2 = millis(); 
      
      LoRabeginPacket(FALSE);
      LoRawrite(packet, 18);      
      
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
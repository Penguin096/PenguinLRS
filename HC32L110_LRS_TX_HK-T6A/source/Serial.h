#ifndef __Serial_H__
#define __Serial_H__

#include "uart.h"
#include "bt.h"

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
  
extern void RxIntCallback(void);
extern void ErrIntCallback(void);

void Serial_begin(uint8_t UARTCH, uint32_t baud)
{
    ASSERT(IS_VALID_CH(u8Idx));
    
    uint16_t timer=0;
    uint32_t pclk=0;
    
    stc_uart_config_t  stcConfig;
    stc_uart_irq_cb_t stcUartIrqCb;
    stc_uart_multimode_t stcMulti;
    stc_uart_baud_config_t stcBaud;
    stc_bt_config_t stcBtConfig;
    
    DDL_ZERO_STRUCT(stcConfig);
    DDL_ZERO_STRUCT(stcUartIrqCb);
    DDL_ZERO_STRUCT(stcMulti);
    DDL_ZERO_STRUCT(stcBaud);
    DDL_ZERO_STRUCT(stcBtConfig);
    
    Gpio_InitIOExt(0,2,GpioDirOut,TRUE,FALSE,FALSE,FALSE); //  UART_TX
    Gpio_InitIOExt(0,1,GpioDirOut,TRUE,FALSE,FALSE,FALSE); //  UART_RX 
     
    Gpio_SetFunc_UART0_TXD_P02();
    Gpio_SetFunc_UART0_RXD_P01();
       
    if(UARTCH == UARTCH0)
    {
      Clk_SetPeripheralGate(ClkPeripheralUart0,TRUE);
    }
    else
    {
      Clk_SetPeripheralGate(ClkPeripheralUart1,TRUE);
    }
    
//    stcUartIrqCb.pfnRxIrqCb = NULL;
    stcUartIrqCb.pfnRxIrqCb = RxIntCallback;
    stcUartIrqCb.pfnTxIrqCb = NULL;
    stcUartIrqCb.pfnRxErrIrqCb = NULL;
    stcConfig.pstcIrqCb = &stcUartIrqCb;
//    stcConfig.bTouchNvic = FALSE;
    stcConfig.bTouchNvic = TRUE;
        
    stcConfig.enRunMode = UartMode1;
    
    stcMulti.enMulti_mode = UartNormal;
    
    stcConfig.pstcMultiMode = &stcMulti;
    
    stcBaud.bDbaud = 0u;
    stcBaud.u32Baud = baud;
    stcBaud.u8Mode = UartMode1; 
    pclk = Clk_GetPClkFreq();
    timer=Uart_SetBaudRate(UARTCH,pclk,&stcBaud);
    
    stcBtConfig.enMD = BtMode2;
    stcBtConfig.enCT = BtTimer;
    
    if(UARTCH == UARTCH0)
    {
      Bt_Init(TIM0, &stcBtConfig);
      Bt_ARRSet(TIM0,timer);
      Bt_Cnt16Set(TIM0,timer);
      Bt_Run(TIM0);
    }
    else
    {
      Bt_Init(TIM1, &stcBtConfig);
      Bt_ARRSet(TIM1,timer);
      Bt_Cnt16Set(TIM1,timer);
      Bt_Run(TIM1);
    }
    
    Uart_Init(UARTCH, &stcConfig);
    Uart_EnableIrq(UARTCH,UartRxIrq);
    Uart_ClrStatus(UARTCH,UartRxFull);
    Uart_EnableFunc(UARTCH,UartRx); 
}

void Serial_println_Char(uint8_t UARTCH, char *s)
{
    uint16_t i=0;     
    while(i<strlen(s))
    {
      Uart_SendData(UARTCH,s[i]);
      i++;
    }
    Uart_SendData(UARTCH,0x0D);
    Uart_SendData(UARTCH,0x0A);
}

#ifdef __cplusplus
}
#endif

#endif
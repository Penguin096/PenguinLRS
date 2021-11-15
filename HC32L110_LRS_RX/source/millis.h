#ifndef __millis_H__
#define __millis_H__

#include "bt.h"

/* C #include "bt.h"binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

volatile uint32_t millis_t;

void Bt2Int(void)
{   
    if (Bt_GetIntFlag(TIM2))
    {
      Bt_ClearIntFlag(TIM2);
      millis_t++;
    }
}

unsigned long millis()
{
    return millis_t;
}

#ifdef __cplusplus
}
#endif
  
#endif
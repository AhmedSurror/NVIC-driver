#ifndef SYSTICK_H_
#define SYSTICK_H_

extern volatile void (*g_callBackPtr_systick)(void) = NULL_PTR;

#define SYSTICK_COUNT_FLAG_MASK     0x00010000

void SysTick_init(uint16 a_TimeInMilliSeconds);

void SysTick_startBusyWait(uint16 a_TimeInMilliSeconds);

void SysTick_handler(void);

void SysTick_setCallBack(volatile void (*Ptr2Func) (void));

void SysTick_stop(void);

void SysTick_start(void);

void SysTick_deInit(void);

#endif /* SYSTICK_H_ */

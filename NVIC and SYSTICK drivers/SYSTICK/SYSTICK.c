#include "SYSTICK.h"
#include "tm4c123gh6pm_registers.h"

void SysTick_init(uint16 a_TimeInMilliSeconds)
{
    if(a_TimeInMilliSeconds > 1000)
    {
        a_TimeInMilliSeconds = 1000;
    }

    SYSTICK_CTRL_REG = 0;

    SYSTICK_RELOAD_REG = (a_TimeInMilliSeconds * 16000) - 1;

    SYSTICK_CURRENT_REG = 0;

    SYSTICK_CTRL_REG |= 0x07;
}

void SysTick_startBusyWait(uint16 a_TimeInMilliSeconds)
{
    if(a_TimeInMilliSeconds > 1000)
    {
        a_TimeInMilliSeconds = 1000;
    }

    SYSTICK_CTRL_REG = 0;

    SYSTICK_RELOAD_REG = (a_TimeInMilliSeconds * 16000) - 1;

    SYSTICK_CURRENT_REG = 0;

    SYSTICK_CTRL_REG |= 0x05;
}

void SysTick_handler(void)
{
    if(g_callBackPtr_systick != NULL_PTR)
    {
        (*g_callBackPtr_systick)();
    }
}

void SysTick_setCallBack(volatile void (*Ptr2Func) (void))
{
    g_callBackPtr_systick = Ptr2Func;
}

void SysTick_stop(void)
{
    SYSTICK_CTRL_REG &= ~(1 << 0);
}

void SysTick_start(void)
{
    SYSTICK_CTRL_REG |= (1 << 0);
}

void SysTick_deInit(void)
{
    SYSTICK_RELOAD_REG = 0;

    SYSTICK_CURRENT_REG = 0;

    SYSTICK_CTRL_REG = 0;
}

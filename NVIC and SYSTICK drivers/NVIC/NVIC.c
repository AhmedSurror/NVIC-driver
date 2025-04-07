#include "tm4c123gh6pm_registers.h"
#include "NVIC.h"
#include "NVIC_Masks.h"

void NVIC_enableIRQ(NVIC_IRQType IRQ_Num)
{
    uint8 IRQ_newNum;

    if(IRQ_Num > 0 && IRQ_Num <=31)
    {
        NVIC_EN0_REG = (1 << IRQ_Num);
    }
    else if(IRQ_Num > 32 && IRQ_Num <= 63)
    {
        IRQ_newNum = IRQ_Num - 32;

        NVIC_EN1_REG = (1 << IRQ_newNum);
    }
    else if(IRQ_Num > 63 && IRQ_Num <= 95)
    {
        IRQ_newNum = IRQ_Num - 64;

        NVIC_EN2_REG = (1 << IRQ_newNum);
    }
    else if(IRQ_Num > 95 && IRQ_Num <= 127)
    {
        IRQ_newNum = IRQ_Num - 96;

        NVIC_EN3_REG = (1 << IRQ_newNum);
    }
    else
    {
        IRQ_newNum = IRQ_Num - 128;

        NVIC_EN4_REG = (1 << IRQ_newNum);
    }
}

void NVIC_disableIRQ(NVIC_IRQType IRQ_Num)
{
    uint8 IRQ_newNum;

    if(IRQ_Num > 0 && IRQ_Num <=31)
    {
        NVIC_DIS0_REG = (1 << IRQ_Num);
    }
    else if(IRQ_Num > 32 && IRQ_Num <= 63)
    {
        IRQ_newNum = IRQ_Num - 32;

        NVIC_DIS1_REG = (1 << IRQ_newNum);
    }
    else if(IRQ_Num > 63 && IRQ_Num <= 95)
    {
        IRQ_newNum = IRQ_Num - 64;

        NVIC_DIS2_REG = (1 << IRQ_newNum);
    }
    else if(IRQ_Num > 95 && IRQ_Num <= 127)
    {
        IRQ_newNum = IRQ_Num - 96;

        NVIC_DIS3_REG = (1 << IRQ_newNum);
    }
    else
    {
        IRQ_newNum = IRQ_Num - 128;

        NVIC_DIS4_REG = (1 << IRQ_newNum);
    }
}

void NVIC_setPriorityIRQ(NVIC_IRQType IRQ_Num, NVIC_IRQPriorityType IRQ_Priority)
{
    switch(IRQ_Num)
    {
    case GPIO_Port_A:
        NVIC_PRI0_REG = (NVIC_PRI0_REG & GPIO_PORTA_PRI0_EN_MASK) | (IRQ_Priority << GPIO_PORTA_PRI0_POS);
        break;

    case GPIO_Port_B:
        NVIC_PRI0_REG = (NVIC_PRI0_REG & GPIO_PORTB_PRI0_EN_MASK) | (IRQ_Priority << GPIO_PORTB_PRI0_POS);
        break;

    case GPIO_Port_C:
        NVIC_PRI0_REG = (NVIC_PRI0_REG & GPIO_PORTC_PRI0_EN_MASK) | (IRQ_Priority << GPIO_PORTC_PRI0_POS);
        break;

    case GPIO_Port_D:
        NVIC_PRI0_REG = (NVIC_PRI0_REG & GPIO_PORTD_PRI0_EN_MASK) | (IRQ_Priority << GPIO_PORTD_PRI0_POS);
        break;

    case GPIO_Port_E:
        NVIC_PRI1_REG = (NVIC_PRI1_REG & GPIO_PORTE_PRI1_EN_MASK) | (IRQ_Priority << GPIO_PORTE_PRI1_POS);
        break;

    case UART0:
        NVIC_PRI1_REG = (NVIC_PRI1_REG & UART0_PRI1_EN_MASK) | (IRQ_Priority << UART0_PRI1_POS);
        break;

    case UART1:
        NVIC_PRI1_REG = (NVIC_PRI1_REG & UART1_PRI1_EN_MASK) | (IRQ_Priority << UART1_PRI1_POS);
        break;

    case SSI0:
        NVIC_PRI1_REG = (NVIC_PRI1_REG & SSI0_PRI1_EN_MASK) | (IRQ_Priority << SSI0_PRI1_POS);
        break;

    case TWI0:
        NVIC_PRI2_REG = (NVIC_PRI2_REG & TWI0_PRI2_EN_MASK) | (IRQ_Priority << TWI0_PRI2_POS);
        break;

    case PWM0_FAULT:
        NVIC_PRI2_REG = (NVIC_PRI2_REG & PWM0_FAULT_PRI2_EN_MASK) | (IRQ_Priority << PWM0_FAULT_PRI2_POS);
        break;

    case PWM0_Generator_0:
        NVIC_PRI2_REG = (NVIC_PRI2_REG & PWM0_Generator_0_PRI2_EN_MASK) | (IRQ_Priority << PWM0_Generator_0_PRI2_POS);
        break;

    case PWM0_Generator_1:
        NVIC_PRI2_REG = (NVIC_PRI2_REG & PWM0_Generator_1_PRI2_EN_MASK) | (IRQ_Priority << PWM0_Generator_1_PRI2_POS);
        break;

    case PWM0_Generator_2:
        NVIC_PRI3_REG = (NVIC_PRI3_REG & PWM0_Generator_2_PRI3_EN_MASK) | (IRQ_Priority << PWM0_Generator_2_PRI3_POS);
        break;

    case QEI0:
        NVIC_PRI3_REG = (NVIC_PRI3_REG & QEI0_PRI3_EN_MASK) | (IRQ_Priority << QEI0_PRI3_POS);
        break;

    case ADC0_SEQUENCE_0:
        NVIC_PRI3_REG = (NVIC_PRI3_REG & ADC0_SEQUENCE_0_PRI3_EN_MASK) | (IRQ_Priority << ADC0_SEQUENCE_0_PRI3_POS);
        break;

    case ADC0_SEQUENCE_1:
        NVIC_PRI3_REG = (NVIC_PRI3_REG & ADC0_SEQUENCE_1_PRI3_EN_MASK) | (IRQ_Priority << ADC0_SEQUENCE_1_PRI3_POS);
        break;

    case ADC0_SEQUENCE_2:
        NVIC_PRI4_REG = (NVIC_PRI4_REG & ADC0_SEQUENCE_2_PRI4_EN_MASK) | (IRQ_Priority << ADC0_SEQUENCE_2_PRI4_POS);
        break;

    case ADC0_SEQUENCE_3:
        NVIC_PRI4_REG = (NVIC_PRI4_REG & ADC0_SEQUENCE_3_PRI4_EN_MASK) | (IRQ_Priority << ADC0_SEQUENCE_3_PRI4_POS);
        break;

    case Watchdog:
        NVIC_PRI4_REG = (NVIC_PRI4_REG & Watchdog_PRI4_EN_MASK) | (IRQ_Priority << Watchdog_PRI4_POS);
        break;

    case Bit_16_32_Timer_0A:
        NVIC_PRI4_REG = (NVIC_PRI4_REG & Timer_0A_PRI4_EN_MASK) | (IRQ_Priority << Timer_0A_PRI4_POS);
        break;

    case Bit_16_32_Timer_0B:
        NVIC_PRI5_REG = (NVIC_PRI5_REG & Timer_0B_PRI5_EN_MASK) | (IRQ_Priority << Timer_0B_PRI5_POS);
        break;

    case Bit_16_32_Timer_1A:
        NVIC_PRI5_REG = (NVIC_PRI5_REG & Timer_1A_PRI5_EN_MASK) | (IRQ_Priority << Timer_1A_PRI5_POS);
        break;

    case Bit_16_32_Timer_1B:
        NVIC_PRI5_REG = (NVIC_PRI5_REG & Timer_1B_PRI5_EN_MASK) | (IRQ_Priority << Timer_1B_PRI5_POS);
        break;

    case Bit_16_32_Timer_2A:
        NVIC_PRI5_REG = (NVIC_PRI5_REG & Timer_2A_PRI5_EN_MASK) | (IRQ_Priority << Timer_2A_PRI5_POS);
        break;

    case Bit_16_32_Timer_2B:
        NVIC_PRI6_REG = (NVIC_PRI6_REG & Timer_2B_PRI6_EN_MASK) | (IRQ_Priority << Timer_2B_PRI6_POS);
        break;

    case Analog_Comparator_0:
        NVIC_PRI6_REG = (NVIC_PRI6_REG & COMP_0_PRI6_EN_MASK) | (IRQ_Priority << COMP_0_PRI6_POS);
        break;

    case Analog_Comparator_1:
        NVIC_PRI6_REG = (NVIC_PRI6_REG & COMP_1_PRI6_EN_MASK) | (IRQ_Priority << COMP_1_PRI6_POS);
        break;

    case System_Control:
        NVIC_PRI7_REG = (NVIC_PRI7_REG & System_Control_PRI7_EN_MASK) | (IRQ_Priority << System_Control_PRI7_POS);
        break;

    case Flash_Memory_Control_and_EEPROM:
        NVIC_PRI7_REG = (NVIC_PRI7_REG & FLASH_CTRL_PRI7_EN_MASK) | (IRQ_Priority << FLASH_CTRL_PRI7_POS);
        break;

    case GPIO_Port_F:
        NVIC_PRI7_REG = (NVIC_PRI7_REG & GPIO_Port_F_PRI7_EN_MASK) | (IRQ_Priority << GPIO_Port_F_PRI7_POS);
        break;

    case UART2:
    case SSI1:
    }
}

void NVIC_enableException(NVIC_ExceptionType Exception_Num)
{
    switch(Exception_Num)
    {
    case MEM_MANAGE:
        NVIC_SYSTEM_SYSHNDCTRL_REG |=(1 << MPU_SYSHND_POS);
        break;

    case BUS_FAULT:
        NVIC_SYSTEM_SYSHNDCTRL |= (1 << BUSFAULT_SYSHND_POS);
        break;

    case USAGE_FAULT:
        NVIC_SYSTEM_SYSHNDCTRL |= (1 << USAGEFAULT_SYSHND_POS);
        break;

    case SVC:
        NVIC_SYSTEM_SYSHNDCTRL |= (1 << SVC_SYSHND_POS);
        break;

    case DEBUG_MONITOR:

    case PEND_SV:
        NVIC_SYSTEM_SYSHNDCTRL |= (1 << PENDSV_SYSHND_POS);
        break;

    case SYSTICK:
        SYSTICK_CTRL_REG |= 0x07;
    }
}

void NVIC_disableException(NVIC_ExceptionType Exception_Num)
{
    switch(Exception_Num)
    {
    case MEM_MANAGE:
        NVIC_SYSTEM_SYSHNDCTRL_REG &= ~(1 << MPU_SYSHND_POS);
        break;

    case BUS_FAULT:
        NVIC_SYSTEM_SYSHNDCTRL &= ~(1 << BUSFAULT_SYSHND_POS);
        break;

    case USAGE_FAULT:
        NVIC_SYSTEM_SYSHNDCTRL &= ~(1 << USAGEFAULT_SYSHND_POS);
        break;

    case SVC:
        NVIC_SYSTEM_SYSHNDCTRL &= ~(1 << SVC_SYSHND_POS);
        break;

    case DEBUG_MONITOR:

    case PEND_SV:
        NVIC_SYSTEM_SYSHNDCTRL &= ~(1 << PENDSV_SYSHND_POS);
        break;

    case SYSTICK:
        SYSTICK_CTRL_REG |= 0x05;
    }
}

void NVIC_setPriorityException(NVIC_ExceptionType Exception_Num, NVIC_ExceptionPriorityType Exception_Priority)
{
    switch(Exception_Num)
    {
    case MEM_MANAGE:
        NVIC_SYSTEM_PRI1_REG = (NVIC_SYSTEM_PRI1_REG & MPU_PRI_MASK) | (Exception_Priority << MPU_BITS_POS);
        break;

    case BUS_FAULT:
        NVIC_SYSTEM_PRI_REG = (NVIC_SYSTEM_PRI1_REG & BUSFAULT_PRI_MASK) | (Exception_Priority << BUSFAULT_BITS_POS);
        break;

    case USAGE_FAULT:
        NVIC_SYSTEM_PRI1_REG = (NVIC_SYSTEM_PRI1_REG & USAGEFAULT_PRI_MASK) | (Exception_Priority << USAGEFAULT_BITS_POS);
        break;

    case SVC:
        NVIC_SYSTEM_PRI2_REG = (NVIC_SYSTEM_PRI2_REG & SVC_PRIORITY_MASK) | (Exception_Priority << SVC_PRIORITY_BITS_POS);
        break;

    case DEBUG_MONITOR:

    case PEND_SV:
        NVIC_SYSTEM_PRI3_REG = (NVIC_SYSTEM_PRI3_REG & PENDSV_PRI_MASK) | (Exception_Priority << PENDSV_PRI_BITS_POS);
        break;

    case SYSTICK:
        NVIC_SYSTEM_PRI3_REG = (NVIC_SYSTEM_PRI3_REG & SYSTICK_PRIORITY_MASK) | (Exception_Priority << SYSTICK_PRIORITY_BITS_POS);
    }
}

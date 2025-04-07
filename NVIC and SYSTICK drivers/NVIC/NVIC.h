#ifndef NVIC_H_
#define NVIC_H_

#include "std_types.h"

typedef enum
{
    GPIO_Port_A = 0, GPIO_Port_B, GPIO_Port_C, GPIO_Port_D, GPIO_Port_D, GPIO_Port_E, UART0, UART1, SSI0, TWI0,
    PWM0_FAULT, PWM0_Generator_0, PWM0_Generator_1, PWM0_Generator_2, QEI0, ADC0_SEQUENCE_0, ADC0_SEQUENCE_1, ADC0_SEQUENCE_2,
    ADC0_SEQUENCE_3, Watchdog, Bit_16_32_Timer_0A, Bit_16_32_Timer_0B, Bit_16_32_Timer_1A, Bit_16_32_Timer_1B,
    Bit_16_32_Timer_2A, Bit_16_32_Timer_2B, Analog_Comparator_0, Analog_Comparator_1, System_Control = 28, Flash_Memory_Control_and_EEPROM,
    GPIO_Port_F, UART2 = 33, SSI1, Bit_16_32_Timer_3A, Bit_16_32_Timer_3B, TWI1, QEI1, CAN0, CAN1, Hibernation_Module = 43, USB,
    PWM_Generator_3, μDMA_Software, ADC1_SEQUENCE_0, ADC1_SEQUENCE_1, ADC1_SEQUENCE_2, ADC1_SEQUENCE_3, SSI2 = 73, SSI3, UART3, UART4,
    UART5, UART6, UART7, TWI2 = 68, TWI3, Bit_16_32_Timer_4A, Bit_16_32_Timer_4B, Bit_16_32_Timer_5A = 92, Bit_16_32_Timer_5B, Bit_32_64_Timer_0A,
    Bit_32_64_Timer_0B, Bit_32_64_Timer_1A, Bit_32_64_Timer_1B, Bit_32_64_Timer_2A, Bit_32_64_Timer_2B, Bit_32_64_Timer_3A, Bit_32_64_Timer_3B,
    Bit_32_64_Timer_4A, Bit_32_64_Timer_4B, Bit_32_64_Timer_5A, Bit_32_64_Timer_5B, PWM1_Generator_0 = 134, PWM1_Generator_1, PWM1_Generator_2, PWM1_Generator_3
}IRQ_Num;

typedef enum
{
    NMI = 2, HARD_FAULT, MEM_MANAGE, BUS_FAULT, USAGE_FAULT, SVC = 11, DEBUG_MONITOR, PEND_SV = 14, SYSTICK, μDMA_Error = 63,
   System_Exception = 122, PWM1_FAULT = 154
}Exception_Num;

typedef uint8 NVIC_IRQType;
typedef uint8 NVIC_IRQPriorityType;
typedef uint8 NVIC_ExceptionType;
typedef uint8 NVIC_ExceptionPriorityType;

void NVIC_enableIRQ(NVIC_IRQType IRQ_Num);

void NVIC_disableIRQ(NVIC_IRQType IRQ_Num);

void NVIC_setPriorityIRQ(NVIC_IRQType IRQ_Num, NVIC_IRQPriorityType IRQ_Priority);

void NVIC_enableException(NVIC_ExceptionType Exception_Num);

void NVIC_disableException(NVIC_ExceptionType Exception_Num);

void NVIC_setPriorityException(NVIC_ExceptionType Exception_Num, NVIC_ExceptionPriorityType Exception_Priority);

#endif /* NVIC_H_ */

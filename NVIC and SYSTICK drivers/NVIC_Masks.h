#ifndef NVIC_MASKS_H_
#define NVIC_MASKS_H_

/* System IRQs */

/* MPU */
#define MPU_PRI_MASK        0xFFFFFF1F
#define MPU_BITS_POS        5
#define MPU_PRI             1
#define MPU_SYSHND_POS     16

/* BUS FAULT */
#define BUSFAULT_PRI_MASK   0xFFFF1FFF
#define BUSFAULT_BITS_POS   13
#define BUSFAULT_PRI        1
#define BUSFAULT_SYSHND_POS 17

/* USAGE FAULT */
#define USAGEFAULT_PRI_MASK     0xFF1FFFFF
#define USAGEFAULT_BITS_POS     21
#define USAGEFAULT_PRI          2
#define USAGEFAULT_SYSHND_POS   18

/* SVC */
#define SVC_PRIORITY_MASK         0x1FFFFFFF
#define SVC_PRIORITY_DIS          0x0FFFFFFF
#define SVC_PRIORITY              0
#define SVC_PRIORITY_BITS_POS     29
#define SVC_SYSHND_POS            15

/* PEND SV */
#define PENDSV_PRI_MASK         0xFF1FFFFF
#define PENDSV_PRI_DIS          0xFF0FFFFF
#define PENDSV_PRI_BITS_POS     21
#define PENDSV_PRI              7
#define PENDSV_SYSHND_POS       10

/* SYSTICK */
#define SYSTICK_PRIORITY_MASK        0x1FFFFFFF
#define SYSTICK_PRIORITY_DIS         0x0FFFFFFF
#define SYSTICK_INTERRUPT_PRIORITY   3
#define SYSTICK_PRIORITY_BITS_POS    29

/*************************************************************************************************/

/* GPIO PORT A */
#define GPIO_PORTA_PRI0_EN_MASK      0xFFFFFF1F
#define GPIO_PORTA_PRI0_DIS_MASK     0xFFFFFF0F
#define GPIO_PORTA_PRI0_POS          5
#define GPIO_PORTA_EN0_POS           0
#define GPIO_PORTA_PRI               2

/* GPIO PORT B */
#define GPIO_PORTB_PRI0_EN_MASK      0xFFFF1FFF
#define GPIO_PORTB_PRI0_DIS_MASK     0xFFFF0FFF
#define GPIO_PORTB_PRI0_POS          13
#define GPIO_PORTB_EN0_POS           1
#define GPIO_PORTB_PRI               2

/* GPIO PORT C */
#define GPIO_PORTC_PRI0_EN_MASK      0xFF1FFFFF
#define GPIO_PORTC_PRI0_DIS_MASK     0xFF0FFFFF
#define GPIO_PORTC_PRI0_POS          21
#define GPIO_PORTC_EN0_POS           2
#define GPIO_PORTB_PRI               2

/* GPIO PORT D */
#define GPIO_PORTD_PRI0_EN_MASK      0x1FFFFFFF
#define GPIO_PORTD_PRI0_DIS_MASK     0x1FFFFFFF
#define GPIO_PORTD_PRI0_POS          29
#define GPIO_PORTD_EN0_POS           3
#define GPIO_PORTD_PRI               2

/* GPIO PORT E */
#define GPIO_PORTE_PRI1_EN_MASK      0xFFFFFF1F
#define GPIO_PORTE_PRI1_DIS_MASK     0xFFFFFF0F
#define GPIO_PORTE_PRI1_POS          5
#define GPIO_PORTE_EN0_POS           4
#define GPIO_PORTE_PRI               2

/* UART0 */
#define UART0_PRI1_EN_MASK      0xFFFF1FFF
#define UART0_PRI1_DIS_MASK     0xFFFF0FFF
#define UART0_PRI1_POS          13
#define UART0_EN0_POS           5
#define UART0_PRI               2

/* UART1 */
#define UART1_PRI1_EN_MASK      0xFF1FFFFF
#define UART1_PRI1_DIS_MASK     0xFF0FFFFF
#define UART1_PRI1_POS          21
#define UART1_EN0_POS           6
#define UART1_PRI               2

/* SSI0 */
#define SSI0_PRI1_EN_MASK       0x1FFFFFFF
#define SSI0_PRI1_DIS_MASK      0x1FFFFFFF
#define SSI0_PRI1_POS           29
#define SSI0_EN0_POS            7
#define SSI0_PRI                2

/* TWI0 */
#define TWI0_PRI2_EN_MASK       0xFFFFFF1F
#define TWI0_PRI2_DIS_MASK      0xFFFFFF0F
#define TWI0_PRI2_POS           5
#define TWI0_EN0_POS            8
#define TWI0_PRI                2

/* PWM0 FAULT*/
#define PWM0_FAULT_PRI2_EN_MASK             0xFFFF1FFF
#define PWM0_FAULT_PRI2_DIS_MASK            0xFFFF0FFF
#define PWM0_FAULT_PRI2_POS                 13
#define PWM0_FAULT_EN0_POS                  9
#define PWM0_FAULT_PRI                      2

/* PWM0 Generator 0 */
#define PWM0_Generator_0_PRI2_EN_MASK       0xFF1FFFFF
#define PWM0_Generator_0_PRI2_DIS_MASK      0xFF0FFFFF
#define PWM0_Generator_0_PRI2_POS           21
#define PWM0_Generator_0_EN0_POS            10
#define PWM0_Generator_0_PRI                2

/* PWM0 Generator 1 */
#define PWM0_Generator_1_PRI2_EN_MASK       0x1FFFFFFF
#define PWM0_Generator_1_PRI2_DIS_MASK      0x1FFFFFFF
#define PWM0_Generator_1_PRI2_POS           29
#define PWM0_Generator_1_EN0_POS            11
#define PWM0_Generator_1_PRI                2

/* PWM0 Generator 2 */
#define PWM0_Generator_2_PRI3_EN_MASK       0xFFFFFF1F
#define PWM0_Generator_2_PRI3_DIS_MASK      0xFFFFFF0F
#define PWM0_Generator_2_PRI3_POS           5
#define PWM0_Generator_2_EN0_POS            12
#define PWM0_Generator_2_PRI                2

/* QEI0 */
#define QEI0_PRI3_EN_MASK                   0xFFFF1FFF
#define QEI0_PRI3_DIS_MASK                  0xFFFF0FFF
#define QEI0_PRI3_POS                       13
#define QEI0_EN0_POS                        13
#define QEI0_PRI                            2

/* ADC0 SEQUENCE 0 */
#define ADC0_SEQUENCE_0_PRI3_EN_MASK        0xFF1FFFFF
#define ADC0_SEQUENCE_0_PRI3_DIS_MASK       0xFF0FFFFF
#define ADC0_SEQUENCE_0_PRI3_POS            21
#define ADC0_SEQUENCE_0_EN0_POS             14
#define ADC0_SEQUENCE_0_PRI                 2

/* ADC0 SEQUENCE 1 */
#define ADC0_SEQUENCE_1_PRI3_EN_MASK        0x1FFFFFFF
#define ADC0_SEQUENCE_1_PRI3_DIS_MASK       0x1FFFFFFF
#define ADC0_SEQUENCE_1_PRI3_POS            29
#define ADC0_SEQUENCE_1_EN0_POS             15
#define ADC0_SEQUENCE_1_PRI                 2

/* ADC0 SEQUENCE 2 */
#define ADC0_SEQUENCE_2_PRI4_EN_MASK        0xFFFFFF1F
#define ADC0_SEQUENCE_2_PRI4_DIS_MASK       0xFFFFFF0F
#define ADC0_SEQUENCE_2_PRI4_POS            5
#define ADC0_SEQUENCE_2_EN0_POS             16
#define ADC0_SEQUENCE_2_PRI                 2

/* ADC0 SEQUENCE 3 */
#define ADC0_SEQUENCE_3_PRI4_EN_MASK        0xFFFF1FFF
#define ADC0_SEQUENCE_3_PRI4_DIS_MASK       0xFFFF0FFF
#define ADC0_SEQUENCE_3_PRI4_POS            13
#define ADC0_SEQUENCE_3_EN0_POS             17
#define ADC0_SEQUENCE_3_PRI                 2

/* Watchdog Timers 0 and 1 */
#define Watchdog_PRI4_EN_MASK               0xFF1FFFFF
#define Watchdog_PRI4_DIS_MASK              0xFF0FFFFF
#define Watchdog_PRI4_POS                   21
#define Watchdog_EN0_POS                    18
#define Watchdog_PRI                        2

/* Timer 0A 16/32 */
#define Timer_0A_PRI4_EN_MASK               0x1FFFFFFF
#define Timer_0A_PRI4_DIS_MASK              0x1FFFFFFF
#define Timer_0A_PRI4_POS                   29
#define Timer_0A_EN0_POS                    19
#define Timer_0A_PRI                        2

/* Timer 0B 16/32 */
#define Timer_0B_PRI5_EN_MASK               0xFFFFFF1F
#define Timer_0B_PRI5_DIS_MASK              0xFFFFFF0F
#define Timer_0B_PRI5_POS                   5
#define Timer_0B_EN0_POS                    20
#define Timer_0B_PRI                        2

/* Timer 1A 16/32 */
#define Timer_1A_PRI5_EN_MASK               0xFFFF1FFF
#define Timer_1A_PRI5_DIS_MASK              0xFFFF0FFF
#define Timer_1A_PRI5_POS                   13
#define Timer_1A_EN0_POS                    21
#define Timer_1A_PRI                        2

/* Timer 1B 16/32 */
#define Timer_1B_PRI5_EN_MASK               0xFF1FFFFF
#define Timer_1B_PRI5_DIS_MASK              0xFF0FFFFF
#define Timer_1B_PRI5_POS                   21
#define Timer_1B_EN0_POS                    22
#define Timer_1B_PRI                        2

/* Timer 2A 16/32 */
#define Timer_2A_PRI5_EN_MASK               0x1FFFFFFF
#define Timer_2A_PRI5_DIS_MASK              0x1FFFFFFF
#define Timer_2A_PRI5_POS                   29
#define Timer_2A_EN0_POS                    23
#define Timer_2A_PRI                        2

/* Timer 2B 16/32 */
#define Timer_2B_PRI6_EN_MASK               0xFFFFFF1F
#define Timer_2B_PRI6_DIS_MASK              0xFFFFFF0F
#define Timer_2B_PRI6_POS                   5
#define Timer_2B_EN0_POS                    24
#define Timer_2B_PRI                        2

/* Analog Comparator 0 */
#define COMP_0_PRI6_EN_MASK                 0xFFFF1FFF
#define COMP_0_PRI6_DIS_MASK                0xFFFF0FFF
#define COMP_0_PRI6_POS                     13
#define COMP_0_EN0_POS                      25
#define COMP_0_PRI                          2

/* Analog Comparator 1 */
#define COMP_1_PRI6_EN_MASK                 0xFF1FFFFF
#define COMP_1_PRI6_DIS_MASK                0xFF0FFFFF
#define COMP_1_PRI6_POS                     21
#define COMP_1_EN0_POS                      26
#define COMP_1_PRI                          2

/* System Control */
#define System_Control_PRI7_EN_MASK         0xFFFFFF1F
#define System_Control_PRI7_DIS_MASK        0xFFFFFF0F
#define System_Control_PRI7_POS             5
#define System_Control_EN0_POS              28
#define System_Control_PRI                  2

/* Flash Memory Control and EEPROM */
#define FLASH_CTRL_PRI7_EN_MASK             0xFFFF1FFF
#define FLASH_CTRL_PRI7_DIS_MASK            0xFFFF0FFF
#define FLASH_CTRL_PRI7_POS                 13
#define FLASH_CTRL_EN0_POS                  29
#define FLASH_CTRL_PRI                      2

/* GPIO Port F */
#define GPIO_Port_F_PRI7_EN_MASK            0xFF1FFFFF
#define GPIO_Port_F_PRI7_DIS_MASK           0xFF0FFFFF
#define GPIO_Port_F_PRI7_POS                21
#define GPIO_Port_F_EN0_POS                 30
#define GPIO_Port_F_PRI                     2

#endif /* NVIC_MASKS_H_ */

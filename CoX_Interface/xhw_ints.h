//*****************************************************************************
//
//
//*****************************************************************************

#ifndef __XHW_INTS_H__
#define __XHW_INTS_H__

//*****************************************************************************
//
//
//*****************************************************************************

#define xFAULT_NMI              2           // NMI
#define xFAULT_HARD             3           // Hard fault
#define xFAULT_MPU              4           // MPU fault
#define xFAULT_BUS              5           // Bus fault
#define xFAULT_USAGE            6           // Usage fault
#define xFAULT_SVCALL           11          // SVCall
#define xFAULT_DEBUG            12          // Debug monitor
#define xFAULT_PENDSV           14          // PendSV
#define xFAULT_SYSTICK          15          // System Tick
#define xINT_SYSCTL             0           // System Control
#define xINT_WDT                0           // WDT
#define xINT_GPIOA              0           // GPIOA
#define xINT_GPIOB              0           // GPIOB
#define xINT_GPIOC              0           // GPIOC
#define xINT_GPIOD              0           // GPIOD
#define xINT_PWMA               0           // PWM Generator A
#define xINT_PWMB               0           // PWM Generator B
#define xINT_TIMER0             0           // Timer 0
#define xINT_TIMER1             0           // Timer 1
#define xINT_UART0              0           // UART0 Rx and Tx
#define xINT_UART1              0           // UART1 Rx and Tx
#define xINT_SPI0               0           // SPI0 Rx and Tx
#define xINT_SPI1               0           // SPI1 Rx and Tx
#define xINT_I2C0               0           // I2C0 Master and Slave
#define xINT_I2C1               0           // I2C1 Master and Slave
#define xINT_ACMP0              0           // ACMP0
#define xINT_DMA                0           // DMA
#define xINT_ADC0               0           // ADC0
#define xINT_RTC                0           // RTC

//
//
#define xNUM_INTERRUPTS         16

//
//
#define xNUM_PRIORITY           4

//
//
#define xNUM_PRIORITY_BITS      2


//*****************************************************************************
//
//
//*****************************************************************************

#endif // __XHW_INTS_H__

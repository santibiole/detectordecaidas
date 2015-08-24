//*****************************************************************************
//
//! \file xhw_memmap.h
//! \brief Macros defining the memory map of the MCU.
//! \version V2.2.1.0
//! \date 11/20/2011
//! \author CooCox
//! \copy
//!
//! Copyright (c)  2011, CooCox
//! All rights reserved.
//!
//! Redistribution and use in source and binary forms, with or without
//! modification, are permitted provided that the following conditions
//! are met:
//!
//!     * Redistributions of source code must retain the above copyright
//! notice, this list of conditions and the following disclaimer.
//!     * Redistributions in binary form must reproduce the above copyright
//! notice, this list of conditions and the following disclaimer in the
//! documentation and/or other materials provided with the distribution.
//!     * Neither the name of the <ORGANIZATION> nor the names of its
//! contributors may be used to endorse or promote products derived
//! from this software without specific prior written permission.
//!
//! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//! AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//! IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//! ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
//! LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//! CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//! SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//! INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//! CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//! ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
//! THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************
 
#ifndef __XHW_MEMMAP_H__
#define __XHW_MEMMAP_H__
 
//*****************************************************************************
//
//! \addtogroup CoX_Peripheral_Lib
//! @{
 
//
//*****************************************************************************
 
//*****************************************************************************
//
//! \addtogroup LowLayer
//! @{
 
//
//*****************************************************************************
 
//*****************************************************************************
//
//! \addtogroup xLowLayer
//! @{
 
//
//*****************************************************************************
 
//*****************************************************************************
//
//! \addtogroup xLowLayer_Peripheral_Memmap xLowLayer Peripheral Memmap
//! \brief The following are definitions for the base addresses of the memories
//! and peripherals.
//!
//! They are always used as ulBase parameters in the peripheral library.
//! The name of a macro for the base address of a peripheral is in  general
//! format as $Namen$_BASE, e.g. UART0_BASE.
//!
//! @{
 
//
//*****************************************************************************
 
#define xFLASH_BASE             FLASH_BASE  // Flash memory
#define xSRAM_BASE              SRAM_BASE   // SRAM memory
#define xWDT_BASE               WWDG_BASE   // WatchDog
#define xGPIO_PORTA_BASE        GPIOA_BASE  // GPIOA
#define xGPIO_PORTB_BASE        GPIOB_BASE  // GPIOB
#define xGPIO_PORTC_BASE        GPIOC_BASE  // GPIOC
#define xGPIO_PORTD_BASE        GPIOD_BASE  // GPIOD 
#define xGPIO_PORTE_BASE        GPIOE_BASE  // GPIOE
#define xGPIO_PORTF_BASE        GPIOF_BASE  // GPIOF
#define xGPIO_PORTG_BASE        GPIOG_BASE  // GPIOG
#define xGPIO_PORTH_BASE        GPIOH_BASE  // GPIOH
 
#define xUART1_BASE             USART1_BASE // UART0
#define xUART2_BASE             USART2_BASE // UART1
#define xUART3_BASE             USART3_BASE // UART2
#define xUART4_BASE             UART4_BASE  // UART3
#define xUART5_BASE             UART5_BASE  // UART4
 
#define xTIMER2_BASE            TIM2_BASE   // Timer2
#define xTIMER3_BASE            TIM3_BASE   // Timer3
#define xTIMER4_BASE            TIM4_BASE   // Timer4
#define xTIMER5_BASE            TIM5_BASE   // Timer5
#define xTIMER6_BASE            TIM6_BASE   // Timer6
#define xTIMER7_BASE            TIM7_BASE   // Timer7
#define xTIMER9_BASE            TIM9_BASE   // Timer9
#define xTIMER10_BASE           TIM10_BASE  // Timer10
#define xTIMER11_BASE           TIM11_BASE  // Timer11

#define xSPI1_BASE              SPI1_BASE   // SPI0
#define xSPI2_BASE              SPI2_BASE   // SPI1
#define xSPI3_BASE              SPI3_BASE   // SPI2
 
#define xI2C1_BASE              I2C1_BASE   // I2C1
#define xI2C2_BASE              I2C2_BASE   // I2C2
 
#define xADC1_BASE              ADC1_BASE   // ADC1
 
#define xDMA1_BASE              DMA1_BASE   // DMA
 
//*****************************************************************************
//
//! @
//
//*****************************************************************************
 
//*****************************************************************************
//
//! @
//
//*****************************************************************************
 
//*****************************************************************************
//
//! \addtogroup STM32F1xx_LowLayer
//! @{
 
//
//*****************************************************************************
 
//*****************************************************************************
//
//! \addtogroup STM32F1xx_owLayer_Peripheral_Memmap STM32F1xx Peripheral Memmap
//! \brief The following are definitions for the base addresses of the memories
//! and peripherals.
//!
//! They are always used as ulBase parameters in the peripheral library.
//! The name of a macro for the base address of a peripheral is in  general
//! format as $Namen$_BASE, e.g. UART0_BASE.
//!
//! @{
 
//
//*****************************************************************************
 
/** @addtogroup Peripheral_memory_map
  * @{
  */

#define FLASH_BASE            ((uint32_t)0x08000000) /*!< FLASH base address in the alias region */
#define SRAM_BASE             ((uint32_t)0x20000000) /*!< SRAM base address in the alias region */
#define PERIPH_BASE           ((uint32_t)0x40000000) /*!< Peripheral base address in the alias region */

#define SRAM_BB_BASE          ((uint32_t)0x22000000) /*!< SRAM base address in the bit-band region */
#define PERIPH_BB_BASE        ((uint32_t)0x42000000) /*!< Peripheral base address in the bit-band region */

#define FSMC_R_BASE           ((uint32_t)0xA0000000) /*!< FSMC registers base address */

/*!< Peripheral memory map */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x20000)

#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800)
#define TIM5_BASE             (APB1PERIPH_BASE + 0x0C00)
#define TIM6_BASE             (APB1PERIPH_BASE + 0x1000)
#define TIM7_BASE             (APB1PERIPH_BASE + 0x1400)
#define LCD_BASE              (APB1PERIPH_BASE + 0x2400)
#define RTC_BASE              (APB1PERIPH_BASE + 0x2800)
#define WWDG_BASE             (APB1PERIPH_BASE + 0x2C00)
#define IWDG_BASE             (APB1PERIPH_BASE + 0x3000)
#define SPI2_BASE             (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASE             (APB1PERIPH_BASE + 0x3C00)
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400)
#define USART3_BASE           (APB1PERIPH_BASE + 0x4800)
#define UART4_BASE            (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASE            (APB1PERIPH_BASE + 0x5000)
#define I2C1_BASE             (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x5800)
#define PWR_BASE              (APB1PERIPH_BASE + 0x7000)
#define DAC_BASE              (APB1PERIPH_BASE + 0x7400)
#define COMP_BASE             (APB1PERIPH_BASE + 0x7C00)
#define RI_BASE               (APB1PERIPH_BASE + 0x7C04)
#define OPAMP_BASE            (APB1PERIPH_BASE + 0x7C5C)

#define SYSCFG_BASE           (APB2PERIPH_BASE + 0x0000)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x0400)
#define TIM9_BASE             (APB2PERIPH_BASE + 0x0800)
#define TIM10_BASE            (APB2PERIPH_BASE + 0x0C00)
#define TIM11_BASE            (APB2PERIPH_BASE + 0x1000)
#define ADC1_BASE             (APB2PERIPH_BASE + 0x2400)
#define SDIO_BASE             (APB2PERIPH_BASE + 0x2C00)
#define SPI1_BASE             (APB2PERIPH_BASE + 0x3000)
#define USART1_BASE           (APB2PERIPH_BASE + 0x3800)

#define GPIOA_BASE            (AHBPERIPH_BASE + 0x0000)
#define GPIOB_BASE            (AHBPERIPH_BASE + 0x0400)
#define GPIOC_BASE            (AHBPERIPH_BASE + 0x0800)
#define GPIOD_BASE            (AHBPERIPH_BASE + 0x0C00)
#define GPIOE_BASE            (AHBPERIPH_BASE + 0x1000)
#define GPIOH_BASE            (AHBPERIPH_BASE + 0x1400)
#define GPIOF_BASE            (AHBPERIPH_BASE + 0x1800)
#define GPIOG_BASE            (AHBPERIPH_BASE + 0x1C00)
#define CRC_BASE              (AHBPERIPH_BASE + 0x3000)
#define RCC_BASE              (AHBPERIPH_BASE + 0x3800)


#define FLASH_R_BASE          (AHBPERIPH_BASE + 0x3C00) /*!< FLASH registers base address */
#define OB_BASE               ((uint32_t)0x1FF80000)    /*!< FLASH Option Bytes base address */

#define DMA1_BASE             (AHBPERIPH_BASE + 0x6000)
#define DMA1_Channel1_BASE    (DMA1_BASE + 0x0008)
#define DMA1_Channel2_BASE    (DMA1_BASE + 0x001C)
#define DMA1_Channel3_BASE    (DMA1_BASE + 0x0030)
#define DMA1_Channel4_BASE    (DMA1_BASE + 0x0044)
#define DMA1_Channel5_BASE    (DMA1_BASE + 0x0058)
#define DMA1_Channel6_BASE    (DMA1_BASE + 0x006C)
#define DMA1_Channel7_BASE    (DMA1_BASE + 0x0080)

#define DMA2_BASE             (AHBPERIPH_BASE + 0x6400)
#define DMA2_Channel1_BASE    (DMA2_BASE + 0x0008)
#define DMA2_Channel2_BASE    (DMA2_BASE + 0x001C)
#define DMA2_Channel3_BASE    (DMA2_BASE + 0x0030)
#define DMA2_Channel4_BASE    (DMA2_BASE + 0x0044)
#define DMA2_Channel5_BASE    (DMA2_BASE + 0x0058)

#define AES_BASE              ((uint32_t)0x50060000)

#define FSMC_Bank1_R_BASE     (FSMC_R_BASE + 0x0000) /*!< FSMC Bank1 registers base address */
#define FSMC_Bank1E_R_BASE    (FSMC_R_BASE + 0x0104) /*!< FSMC Bank1E registers base address */

#define DBGMCU_BASE           ((uint32_t)0xE0042000) /*!< Debug MCU registers base address */
 
//*****************************************************************************
//
//! @
//
//*****************************************************************************
 
//*****************************************************************************
//
//! @
//
//*****************************************************************************
 
//*****************************************************************************
//
//! @
//
//*****************************************************************************
 
//*****************************************************************************
//
//! @
//
//*****************************************************************************
 
#endif // __XHW_MEMMAP_H__

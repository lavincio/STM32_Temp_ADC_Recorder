/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : platform_config.h
* Author             : MCD Application Team
* Version            : V2.0.1
* Date               : 06/13/2008
* Description        : Evaluation board specific configuration file.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PLATFORM_CONFIG_H
#define __PLATFORM_CONFIG_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Uncomment the line corresponding to the STMicroelectronics evaluation board
   used to run the example */

#if !defined (USE_STM3210B_EVAL)
 #define USE_STM3210B_EVAL
#endif

#define USE_USART1

#ifdef USE_USART1
  #define  USARTx                     USART1
  #define  GPIOx                      GPIOA
  #define  RCC_APB2Periph_GPIOx       RCC_APB2Periph_GPIOA
  #define  GPIO_RxPin                 GPIO_Pin_10
  #define  GPIO_TxPin                 GPIO_Pin_9
#endif /* USE_STM3210B_EVAL */

/* Define For ET-STM32F103 Board */
#ifdef USE_STM3210B_EVAL

  // Define LED PinIO Interface Mask Bit 
  #define  LED1_PIN		               GPIO_Pin_8			// LED1 = PB[8]
  #define  LED2_PIN		               GPIO_Pin_9			// LED2 = PB[9]
  #define  LED3_PIN		               GPIO_Pin_10			// LED3 = PB[10]
  #define  LED4_PIN		               GPIO_Pin_11			// LED4 = PB[11]
  #define  LED5_PIN		               GPIO_Pin_12			// LED5 = PB[12]
  #define  LED6_PIN		               GPIO_Pin_13			// LED6 = PB[13]
  #define  LED7_PIN		               GPIO_Pin_14			// LED7 = PB[14]
  #define  LED8_PIN		               GPIO_Pin_15			// LED8 = PB[15]
  #define  LED_PORT		               GPIOB
  #define  RCC_APB2Periph_GPIO_LED     RCC_APB2Periph_GPIOB

  #define  LED1_HI()    	           GPIO_WriteBit(LED_PORT,LED1_PIN,Bit_SET)
  #define  LED1_LO()		           GPIO_WriteBit(LED_PORT,LED1_PIN,Bit_RESET)

  #define  LED2_HI()    	           GPIO_WriteBit(LED_PORT,LED2_PIN,Bit_SET)
  #define  LED2_LO()		           GPIO_WriteBit(LED_PORT,LED2_PIN,Bit_RESET)

  #define  LED3_HI()    	           GPIO_WriteBit(LED_PORT,LED3_PIN,Bit_SET)
  #define  LED3_LO()		           GPIO_WriteBit(LED_PORT,LED3_PIN,Bit_RESET)

  #define  LED4_HI()    	           GPIO_WriteBit(LED_PORT,LED4_PIN,Bit_SET)
  #define  LED4_LO()		           GPIO_WriteBit(LED_PORT,LED4_PIN,Bit_RESET)

  #define  LED5_HI()    	           GPIO_WriteBit(LED_PORT,LED5_PIN,Bit_SET)
  #define  LED5_LO()		           GPIO_WriteBit(LED_PORT,LED5_PIN,Bit_RESET)

  #define  LED6_HI()    	           GPIO_WriteBit(LED_PORT,LED6_PIN,Bit_SET)
  #define  LED6_LO()		           GPIO_WriteBit(LED_PORT,LED6_PIN,Bit_RESET)

  #define  LED7_HI()    	           GPIO_WriteBit(LED_PORT,LED7_PIN,Bit_SET)
  #define  LED7_LO()		           GPIO_WriteBit(LED_PORT,LED7_PIN,Bit_RESET)

  #define  LED8_HI()    	           GPIO_WriteBit(LED_PORT,LED8_PIN,Bit_SET)
  #define  LED8_LO()		           GPIO_WriteBit(LED_PORT,LED8_PIN,Bit_RESET)
    
#endif /* USE_STM3210B_EVAL */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __PLATFORM_CONFIG_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/

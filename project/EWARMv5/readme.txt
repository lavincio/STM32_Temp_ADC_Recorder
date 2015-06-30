/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : readme.txt
* Author             : MCD Application Team
* Version            : V2.0.1
* Date               : 06/13/2008
* Description        : This sub-directory contains all the user-modifiable files 
*                      needed to create a new project linked with the STM32F10x  
*                      Firmware Library and working with IAR Embedded Workbench 
*                      for ARM (EWARM) software toolchain (version 5.11A and later)
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

Directory contents
===================
- project .ewd/.eww/.ewp: A pre-configured project file with the provided library 
                          structure that produces an executable image with IAR 
                          Embedded Workbench.
                
- stm32f10x_vector.c: This file contains the vector table for STM32F10x.
                      User can also enable the use of external SRAM mounted on 
                      ET-STM32F103 board as data memory.

- stm32f10x_flash.icf : This file is the IAR specific linking and loading file used 
                        to load in Flash and execute code and variables to target 
                        memories and regions. You can customize this file to your 
                        need.

- stm32f10x_ram.icf:  This file is the IAR specific linking and loading file used 
                      to load in RAM and execute code and variables to target 
                      memories and regions. You can customize this file to your 
                      need.
                       
- cortexm3_macro.s: Instruction wrappers for special Cortex-M3 instructions. 

      
How to use it
=============
- Open the Project.eww workspace.
- In the workspace toolbar select the project config:
    - ET-STM32F103: to configure the project 
- Rebuild all files: Project->Rebuild all
- Load project image: Project->Debug
- Run program: Debug->Go(F5)

******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE******

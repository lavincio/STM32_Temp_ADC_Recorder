/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : main.c
* Author             : MCD Application Team
* Version            : V2.0.1
* Date               : 06/13/2008
* Description        : Main program body
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "platform_config.h"
#include "stdio.h"

#define CLK_MAX_SEC 60
#define CLK_MAX_MIN 60
#define CLK_MAX_HOURS 24

#define WREN 0x06
#define READ 0x03
#define WRITE 0x02
//#define RDID 0x9f

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
SPI_InitTypeDef SPI_InitStructure;
ErrorStatus HSEStartUpStatus;
static vu32 TimingDelay;
volatile unsigned char RTC_Update = 0;
unsigned int  ClkHour=0, ClkMin=0, ClkSec=0, SetFlags=0, ClkHourStart=0, ClkMinStart=0, RecordInterval=0, NumberRecords=0, RecordNumber=0;
/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void NVIC_Configuration(void);
void RTCIntrHandler(void);
void RtcSetup (void); 
void Init_USART1(void);
int SendCharUart1(int);
int GetKeyUart1(void);
void InitADC1(void);
unsigned int GetADC1Channel(unsigned char chanel);
void DelayuS(vu32 nCount);	 			// 1uS Delay
void DelaymS(vu32 nTime);				// 1mS Delay
void TimingDelay_Decrement(void);
void Setup_Menu(void);
void Init_SPI(void);
void NvSRAM_WriteEnable(void);
u8 NvSRAM_SendByte(u8 byte);
void NvSRAM_ReadByte(u8, u8, u8);
void NvSRAM_WriteByte(u8, u8, u8, unsigned int, unsigned int, unsigned int, unsigned int, unsigned int);
void UploadData(void);

/* Retarget For USART Printf */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
{
  unsigned int AD1_Ch0, AD1_Ch1, i, IntCounter=0, StartDAS=1, Acquire=0;
  float t0,t1; //Reduce floating to reduce MCU cycles
  u8 addr_3=0, addr_2=0, addr_1=0;
#ifdef DEBUG
  debug();
#endif
  
  /* System Clocks Configuration **********************************************/
  RCC_Configuration();   
  
  /* NVIC Configuration *******************************************************/
  NVIC_Configuration();
  
  /* SysTick end of count event each 1ms with input clock equal to 9MHz (HCLK/8, default) */
  SysTick_SetReload(9000);
  
  /* Enable SysTick interrupt */
  SysTick_ITConfig(ENABLE);
  
  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  /* Enable the RTC Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  RtcSetup();      // Start RTC
  
  Init_SPI();	   // SPI Init
  
  InitADC1();      // ADC1 Init 
  
  Init_USART1();   // UART1
  
Menu1:     
  printf("<S>etup <D>isplay settings <U>pload data <Enter>Capture data\n\r");
  i = GetKeyUart1();
  SendCharUart1(i);   //Echo UART1      
  switch (i)
  {
  case 's':         // Entering Setup Mode
  case 'S':
    printf("\n\rSetup Mode\n\r");
    SendCharUart1(0x0D);
    SendCharUart1(0x0A); 
    Setup_Menu();
    //printf("\n\rTime      ADC1    ADC2\n\r");
    break;
    
  case 'd':
  case 'D':
    printf("\n\rDAS Settings:\n\r");
    printf("\n\rReal Time Clock:");
    printf("    %02d", ClkHour);   // display RTC hours
    printf(":%02d\n\r", ClkMin);   // display RTC minutes
    printf("Start Time:");
    printf("         %02d", ClkHourStart); // display Start hours
    printf(":%02d\n\r", ClkMinStart);      // display Start minutes
    printf("Recording Interval:");
    printf(" %dsec", RecordInterval);      // save # of Interval
    printf("\n\rNumber of Records:");
    printf("  %d\n\r", NumberRecords);     // save # of records
    break;
    
  case 'u':
  case 'U':                                          //  U ?Upload the requested # of records
    printf("\n\rUpload Records (115200, 8N1)\n\r");  // Remind the settings for RS-232
    SendCharUart1(0x0D);
    SendCharUart1(0x0A);
    UploadData();    // Upload all collected data setup by NumberRecords
    break;
    
  case 0x0D:                                     // 0x0D ?Carriage Return
    SendCharUart1(0x0D);
    SendCharUart1(0x0A);               
    SendCharUart1('>');
    printf("\n\r 0x%02X", StartDAS); //Displays the flag StartDAS
    break;         
  }  // end of switch   
  if(i!=0x0D)
    goto Menu1;
  
  printf("\n\r Waiting to Start\n\r");
  printf("\n\rTime      ADC1    ADC2    Remote    Onboard\n\r");
  
  while(RecordNumber<NumberRecords)	  											
  {
    if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE))// Monitor USART interrupt
      goto Menu1;
    
    // RTC Triggers all actions below 
    if (RTC_Update == 1) {
      RTC_Update = 0;                        // Clear the flag Update RTC:
      ClkSec++;
      IntCounter++;        // Update Interval Counter if DAS has started
      if (ClkSec >= CLK_MAX_SEC) { // Update the seconds
        ClkSec = 0;
        ClkMin++;
      }
      if (ClkMin >= CLK_MAX_MIN) { // Update the minutes
        ClkMin = 0;
        ClkHour++;
      }
      if (ClkHour >= CLK_MAX_HOURS) { // Update the hours
        ClkHour = 0;
      }
      if(ClkMinStart == ClkMin && ClkHourStart == ClkHour)//Check to see if time to start
        Acquire = 1;
      if(Acquire == 1){
        //if (StartDAS) IntCounter++;        // Update Interval Counter if DAS has started
        if (IntCounter >= RecordInterval) { 
          IntCounter = 0;
          AD1_Ch0 = GetADC1Channel(ADC_Channel_0);//From port A1, pin 14
          AD1_Ch1 = GetADC1Channel(ADC_Channel_1);//From port A1, pin 15
          //3300mV/4096 2^12 resolution*AD1_Ch1/(1-((1.5kohm/(1.5k+5.6kohm))*0.1 (100F/1000mV)  
          t0=0.102*AD1_Ch0;     
          t1=0.102*AD1_Ch1;
          printf("\nDAS Readout #%d:\n\r", RecordNumber+1);
          printf("\n%02d:%02d:%02d %04d %04d %2.1f %2.1f\n\r", ClkHour, ClkMin, ClkSec, AD1_Ch0, AD1_Ch1, t0, t1);//print ADC1 CH0 CH1 value
          NvSRAM_WriteByte(addr_3, addr_2, addr_1, ClkHour, ClkMin, ClkSec, AD1_Ch0, AD1_Ch1);
          NvSRAM_ReadByte(addr_3, addr_2, addr_1);
          //Add address to next pointer
          if(addr_2==0xFF)// If address is over FF
            addr_3++;
          if(addr_1>=0xFC)
            addr_2++;        
          addr_1+=7;
          RecordNumber++;
        }
      }  //end of starttime check
    } // end of RTC Update
  }  
}

/*******************************************************************************
* Function Name  : RTCIntrHandler
* Description    : RTC Interrupt Handler
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RTCIntrHandler(void)
{
  if (RTC_GetITStatus(RTC_IT_SEC) != RESET)
  {
    // Clear the RTC Second interrupt 
    RTC_ClearITPendingBit(RTC_IT_SEC);
    RTC_Update = 1;    
  }
}

/*----------------------------------------------------------------------------
STM32 Real Time Clock setup.
initializes the RTC Prescaler and RTC counter register
*----------------------------------------------------------------------------*/
void RtcSetup (void) 
{
  /* Enable PWR and BKP clocks */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
  
  /* Allow access to BKP Domain */
  PWR_BackupAccessCmd(ENABLE);
  
  /* Reset Backup Domain */
  BKP_DeInit();
  
  /* Enable LSE */
  RCC_LSEConfig(RCC_LSE_ON);
  /* Wait till LSE is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
  {}
  
  /* Select LSE as RTC Clock Source */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
  
  /* Enable RTC Clock */
  RCC_RTCCLKCmd(ENABLE);
  
  /* Wait for RTC registers synchronization */
  RTC_WaitForSynchro();
  
  /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask();
  
  /* Enable the RTC Second */
  RTC_ITConfig(RTC_IT_SEC, ENABLE);
  
  /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask();
  
  /* Set RTC prescaler: set RTC period to 1sec */
  RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */
  
  /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask();  
  
} // end of stm32_RtcSetup

/*******************************************************************************
* Function Name  : Setup_Menu
* Description    : Setup the DAS parameters
* Input          : u8 indicates what parameter to setup
* Output         : None 
* Return         : None
*******************************************************************************/
void Setup_Menu()
{
  int i;
Menu:
  // if(SetFlags<0x0f){
  printf("\n\rPress C for Clock, T for Start Time, I for Interval, N for # of Records\n\r");
  i = GetKeyUart1();
  switch (i) {
  case 'c':
  case 'C':
    //      SetClock();
    printf("\n\rClock Setup: Enter military hours (00-23):\n\r");
    i = GetKeyUart1();
    SendCharUart1(i);   //Echo UART1
    ClkHour = 10*(i - 0x30); // Extract decimal value
    i = GetKeyUart1();
    SendCharUart1(i);   //Echo UART1
    ClkHour = ClkHour + i - 0x30;
    printf("\n\rNow enter minutes (00-59):\n\r");
    i = GetKeyUart1();
    SendCharUart1(i);   //Echo UART1
    ClkMin = 10*(i - 0x30); // Extract decimal value
    i = GetKeyUart1();
    SendCharUart1(i);   //Echo UART1
    ClkMin = ClkMin + i - 0x30;
    if ((ClkHour > 23) | (ClkMin >59)) {
      printf("\n\rWrong Entry. Press C to re-enter.\n\r");        // print message
      goto Menu;
    }
    else {
      ClkSec = 0;          
      printf("\n\rRTC Updated:\n\r");
      printf("%02d:%02d \n\r", ClkHour, ClkMin);
      SetFlags = SetFlags | 0x01; //RTC Setup Done Flag    
    }
    break;
  case 't':
  case 'T':
    //         SetStartTime();
    printf("\n\rStart Time Setup: Enter hours (00-23):\n\r");
    i = GetKeyUart1();
    SendCharUart1(i);   //Echo UART1
    ClkHourStart = 10*(i - 0x30); // Extract decimal value
    i = GetKeyUart1();
    SendCharUart1(i);   //Echo UART1
    ClkHourStart = ClkHourStart + i - 0x30;
    printf("\n\rNow enter minutes (00-59):\n\r");
    i = GetKeyUart1();
    SendCharUart1(i);   //Echo UART1
    ClkMinStart = 10*(i - 0x30); // Extract decimal value
    i = GetKeyUart1();
    SendCharUart1(i);   //Echo UART1
    ClkMinStart = ClkMinStart + i - 0x30;
    printf("\n\rStart time: %02d:%02d \n\r", ClkHourStart, ClkMinStart);
    SetFlags = SetFlags | 0x02;  //StartTime Interval setup done flag
    break;
  case 'i':
  case 'I':
    //            SetInterval();
    printf("\n\rEnter Recording Interval in sec. (00 - 99)\n\r");
    i = GetKeyUart1();
    SendCharUart1(i);   //Echo UART1
    RecordInterval = 10*(i - 0x30); // Extract decimal value
    i = GetKeyUart1();
    SendCharUart1(i);   //Echo UART1
    RecordInterval = RecordInterval + i - 0x30; // Extract decimal value
    printf("\n\rRecording Interval: %02d sec\n\r", RecordInterval);
    SetFlags = SetFlags | 0x04;  //Recording Interval setup done flag
    break;
  case 'n':
  case 'N':
    //            SetNumber();
    printf("\n\rEnter Number of Records (00001 - 26000), example: 00064\n\r");
    i = GetKeyUart1();
    SendCharUart1(i);   //Echo UART1
    NumberRecords = 10000 * (i - 0x30); // Extract decimal value
    i = GetKeyUart1();
    SendCharUart1(i);   //Echo UART1
    NumberRecords =  NumberRecords + 1000 * (i - 0x30); // Extract decimal value
    i = GetKeyUart1();
    SendCharUart1(i);   //Echo UART1
    NumberRecords =  NumberRecords + 100 * (i - 0x30); // Extract decimal value
    i = GetKeyUart1();
    SendCharUart1(i);   //Echo UART1
    NumberRecords =  NumberRecords + 10 * (i - 0x30); // Extract decimal value
    i = GetKeyUart1();
    SendCharUart1(i);   //Echo UART1
    NumberRecords =  NumberRecords + i - 0x30; // Extract decimal value
    printf("\n\rNumber of Records: %d\n\r", NumberRecords);
    SetFlags = SetFlags | 0x08;  //Recording Interval setup done flag	
    break;
    //  } 
    // goto Menu;
  }
}

/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{   
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();
  
  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);
  
  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();
  
  if(HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    
    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
    
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
    
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 
    
    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);
    
    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
    
    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);
    
    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }
    
    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    
    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
}

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
#ifdef  VECT_TAB_RAM  
  /* Set the Vector Table base location at 0x20000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
#else  /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x08000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
#endif
}

/*************************************************************************
* Function Name: InitADC1
* Parameters: none
* Return: none
*
* Description: ADC Init subroutine
*
*************************************************************************/
void InitADC1(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  ADC_InitTypeDef   ADC_InitStructure;
  
  // ADC init
  // ADC Deinit
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  ADC_DeInit(ADC1);
  
  // PA1 - analog input
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = (GPIOSpeed_TypeDef)0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init (GPIOA, &GPIO_InitStructure);
  
  // ADC Structure Initialization
  ADC_StructInit(&ADC_InitStructure);
  
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);
  
  // Enable the ADC
  ADC_Cmd(ADC1, ENABLE);
  
  // ADC calibration
  // Enable ADC1 reset calibaration register
  ADC_ResetCalibration(ADC1);
  
  // Check the end of ADC1 reset calibration register
  while(ADC_GetResetCalibrationStatus(ADC1) == SET);
  
  // Start ADC1 calibaration
  ADC_StartCalibration(ADC1);
  
  // Check the end of ADC1 calibration
  while(ADC_GetCalibrationStatus(ADC1) == SET);
  
}

/*************************************************************************
* Function Name: GetADC1Channel
* Parameters: Int8U channel
* Return: Int16U
*
* Description: ADC Convert
*
*************************************************************************/
unsigned int GetADC1Channel(unsigned char chanel)
{
  // Configure channel
  ADC_RegularChannelConfig(ADC1, chanel, 1, ADC_SampleTime_55Cycles5);
  
  // Start the conversion
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  
  // Wait until conversion completion
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
  
  // Get the conversion value
  return ADC_GetConversionValue(ADC1);
}

/*******************************************************************************
* Function Name  : Delay
* Description    : Inserts a delay time.
* Input          : nCount: specifies the delay time length.
* Output         : None
* Return         : None
*******************************************************************************/
void DelayuS(vu32 nCount)
{  
  while (nCount--);
}

/*******************************************************************************
* Function Name  : Delay
* Description    : Inserts a delay time.
* Input          : nTime: specifies the delay time length, in milliseconds.
* Output         : None
* Return         : None
*******************************************************************************/
void DelaymS(u32 nTime)
{
  /* Enable the SysTick Counter */
  SysTick_CounterCmd(SysTick_Counter_Enable);
  
  TimingDelay = nTime;
  
  while(TimingDelay != 0);
  
  /* Disable SysTick Counter */
  SysTick_CounterCmd(SysTick_Counter_Disable);
  /* Clear SysTick Counter */
  SysTick_CounterCmd(SysTick_Counter_Clear);
}

/*******************************************************************************
* Function Name  : TimingDelay_Decrement
* Description    : Decrements the TimingDelay variable.
* Input          : None
* Output         : TimingDelay
* Return         : None
*******************************************************************************/
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

/*******************************************************************************
* Function Name  : Init_USART1
* Description    : Initialization of USART1
* Input          : None
* Output         : none
* Return         : None
*******************************************************************************/
void Init_USART1(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  
  /* Enable GPIOA and USART1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
                         RCC_APB2Periph_USART1, ENABLE);
  /* Configure USART1 Rx (PA10) as input floating */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Configure USART1 Tx (PA9) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  USART_InitStructure.USART_BaudRate            = 115200;
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity              = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;  
  USART_Init(USART1, &USART_InitStructure);
  
  USART_Cmd(USART1, ENABLE); // Enable USART1                     
}

/*******************************************************************************
* Function Name  : GetKeyUart1
* Description    : Read character to Serial Port.
* Input          : None
* Output         : none
* Return         : None
*******************************************************************************/
int GetKeyUart1 (void)  
{
  while (!(USART1->SR & USART_FLAG_RXNE));
  return ((int)(USART1->DR & 0x1FF));
}

/*******************************************************************************
* Function Name  : SendCharUart1
* Description    : Write character to Serial Port.
* Input          : None
* Output         : none
* Return         : None
*******************************************************************************/
int SendCharUart1 (int ch)  
{
  while (!(USART1->SR & USART_FLAG_TXE));
  USART1->DR = (ch & 0x1FF);
  return (ch);
}

/*******************************************************************************
* Function Name  : Init_SPI
* Description    : Initializes SPI
* Input          : None
* Output         : none
* Return         : None
*******************************************************************************/
void Init_SPI(void)
{
  /* SPI1 Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE);
  /* Configure SPI1 pins: SCK(PA5) and MOSI(PA7) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  /* Configure SPI1 pins: CS(PA4)  */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  /* Config SPI[1] = Master */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  /*  Sample on the first edge */
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);
  
  /* Enable SPI1 */
  SPI_Cmd(SPI1, ENABLE);
}

/*******************************************************************************
* Function Name  : NvSRAM_WriteEnable
* Description    : Enable the write access to NvSRAM
* Input          : None
* Output         : none
* Return         : None
*******************************************************************************/
void NvSRAM_WriteEnable(void)
{
  /* Select the NvSRAM: Chip Select low*/
  GPIO_ResetBits(GPIOA,GPIO_Pin_4);
  /* Send "Write Enable" instruction*/
  NvSRAM_SendByte(WREN);
  /*Deselect the NvSRAM: Chip Select high*/
  GPIO_SetBits(GPIOA,GPIO_Pin_4);
}

/*******************************************************************************
* Function Name  : NvSRAM_SendByte
* Description    : Sends a byte through the SPI interface and returns the byte
received from the SPI bus.
* Input          : byte: byte to send
* Output         : none
* Return         : The value of the received byte.
*******************************************************************************/
u8 NvSRAM_SendByte(u8 byte)
{
  /*Loop while DR register is not empty*/
  while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE)==RESET);
  /*Send byte through the SPI1 peripheral*/
  SPI_I2S_SendData(SPI1,byte);
  /*Wait to receive a byte*/
  while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)==RESET);
  /*Return the byte read from the SPI bus*/
  return SPI_I2S_ReceiveData(SPI1);
}

/*******************************************************************************
* Function Name  : NvSRAM_ReadByte
* Description    : Read bytes through the SPI interface.
* Input          : FRAM address, byte_size: number of bytes
* Output         : none
* Return         : none
*******************************************************************************/
//ReadTemp format:Name(#ofBytes)
//Hour(1)MInute(1)Second(1)Temp1(2)Temp2(2)
void NvSRAM_ReadByte(u8 addr_3, u8 addr_2, u8 addr_1)
{
  u8 buf;
  int idx,buf_int[4], hour, minute, second;
  float buf_f1, sum1, buf_f2, sum2;
  
  GPIO_ResetBits(GPIOA, GPIO_Pin_4);//CS_Low 
  NvSRAM_SendByte(READ);
  NvSRAM_SendByte(addr_3);
  NvSRAM_SendByte(addr_2);
  NvSRAM_SendByte(addr_1);
  hour= NvSRAM_SendByte(0);
  minute= NvSRAM_SendByte(0);
  second= NvSRAM_SendByte(0);
  for(idx=1;idx>=0;idx--){
    buf= NvSRAM_SendByte(0);
    buf_int[idx]=buf<<idx*8;
    sum1+=buf_int[idx];
    //printf("\n%0.2x %d %2.1f\n\r",buf, buf_int[idx], sum1);
  }
  buf_f1=(float)(sum1/100);
  
  for(idx=1;idx>=0;idx--){
    buf= NvSRAM_SendByte(0);
    buf_int[idx]=buf<<idx*8;
    sum2+=buf_int[idx];
    //printf("\n%0.2x %d %2.1f\n\r",buf, buf_int[idx], sum2);
  }
  buf_f2=(float)(sum2/100);
  
  //printf("\nFRAM Read #%d:\n\r%02d:%02d:%02d %2.1f %2.1f\n\r", RecordNumber+1, hour, minute, second, buf_f1, buf_f2);//Read back T1 & T2 value
  GPIO_SetBits(GPIOA, GPIO_Pin_4);//CS_High 
}

/*******************************************************************************
* Function Name  : NvSRAM_WriteByte
* Description    : Read temperature through the SPI interface.
* Input          : FRAM address, ADC_Ch0, ADC_Ch1
* Output         : none
* Return         : none
*******************************************************************************/
//WriteTemp format:Name(#ofBytes)
//Hour(1)MInute(1)Second(1)Temp1(2)Temp2(2)
void NvSRAM_WriteByte(u8 addr_3, u8 addr_2, u8 addr_1, unsigned int hour, unsigned int minute, unsigned int second, unsigned int adc_num1,  unsigned int adc_num2)
{
  unsigned int Temp_MSB, Temp_LSB, adc_buf;
  
  NvSRAM_WriteEnable();
  GPIO_ResetBits(GPIOA, GPIO_Pin_4);//CS_Low 
  NvSRAM_SendByte(WRITE);
  NvSRAM_SendByte(addr_3);
  NvSRAM_SendByte(addr_2);
  NvSRAM_SendByte(addr_1);
  //printf("\nAddr: %0.2x %0.2x %0.2x\n\r",addr_3, addr_2, addr_1);
  NvSRAM_SendByte(hour);
  NvSRAM_SendByte(minute);
  NvSRAM_SendByte(second);
  adc_buf = (int) (adc_num1*10.2);//Temp1
  Temp_MSB = adc_buf/256;
  Temp_LSB = adc_buf%256;
  NvSRAM_SendByte(Temp_MSB);
  NvSRAM_SendByte(Temp_LSB);
  //printf("\nSend Temp: %d %d %d\n\r",adc_buf, Temp_MSB, Temp_LSB);
  adc_buf = (int) (adc_num2*10.2);//Temp2
  Temp_MSB = adc_buf/256;
  Temp_LSB = adc_buf%256;
  NvSRAM_SendByte(Temp_MSB);
  NvSRAM_SendByte(Temp_LSB);
  //printf("\nSend Temp: %d %d %d\n\r",adc_buf, Temp_MSB, Temp_LSB);
  
  GPIO_SetBits(GPIOA, GPIO_Pin_4);//CS_High 
}

/*******************************************************************************
* Function Name  : UploadData
* Description    : Upload FRAM data begin with addr0 to USART.
* Input          : RecordNumber: number of entries
* Output         : none
* Return         : none
*******************************************************************************/
void UploadData(void)
{
  u8 buf;
  int idx,buf_int[4], hour, minute, second, i;
  float buf_f1, sum1, buf_f2, sum2;
  
  printf("\n\n\nUploading %d record.\n\n\r",RecordNumber);
  
  GPIO_SetBits(GPIOA, GPIO_Pin_4);//CS_High to clear SPI data 
  GPIO_ResetBits(GPIOA, GPIO_Pin_4);//CS_Low 
  NvSRAM_SendByte(READ);
  NvSRAM_SendByte(0x00);
  NvSRAM_SendByte(0x00);
  NvSRAM_SendByte(0x00);
  //printf("\nAddr: 0x00 %0.2x %0.2x\n\r", r_addr2, r_addr2);
  
  for(i=0;i<RecordNumber;i++){    
    sum1=0;
    sum2=0;
    hour= NvSRAM_SendByte(0);
    minute= NvSRAM_SendByte(0);
    second= NvSRAM_SendByte(0);
    for(idx=1;idx>=0;idx--){
      buf= NvSRAM_SendByte(0);
      buf_int[idx]=buf<<idx*8;
      sum1+=buf_int[idx];
      //printf("\n%0.2x %d %2.1f\n\r",buf, buf_int[idx], sum1);
    }
    buf_f1=(float)(sum1/100);
    
    for(idx=1;idx>=0;idx--){
      buf= NvSRAM_SendByte(0);
      buf_int[idx]=buf<<idx*8;
      sum2+=buf_int[idx];
      //printf("\n%0.2x %d %2.1f\n\r",buf, buf_int[idx], sum2);
    }
    buf_f2=(float)(sum2/100);
    
    printf("\n#%d:\n\r%02d:%02d:%02d %2.1f %2.1f\n\r\n\r", i+1, hour, minute, second, buf_f1, buf_f2);//Read back T1 & T2 value
  }
  GPIO_SetBits(GPIOA, GPIO_Pin_4);//CS_High 
}

/*******************************************************************************
* Function Name  : PUTCHAR_PROTOTYPE
* Description    : Retargets the C library printf function to the USART.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
PUTCHAR_PROTOTYPE
{
  /* Write a character to the USART */
  USART_SendData(USARTx, (u8) ch);
  
  /* Loop until the end of transmission */
  while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
  {
  }
  
  return ch;
}

#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/

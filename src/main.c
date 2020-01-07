/******************************************************************************
  * @file    GPIO/GPIO_EXTI/Src/main.c
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    21-April-2017
  * @brief   This example describes how to configure and use GPIOs through
  *          the STM32L4xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/** @addtogroup STM32L4xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_EXTI
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
//STATES FOR PROGRAM
typedef enum
{
	S_FLASHLED=0,
	S_LEDOFF=1,
	S_LEDTIMER=2,
	S_DISPTIME=3,
	S_LEDDISPTIME=4
}prog_state;

/* Private define ------------------------------------------------------------*/
#define MAX_REACTION_TIME         10000000000     // Maximum reaction time in ms
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//variable to trigger mode for LEDs (ON, OFF, Flashing)
int L_FLASH=1;

//triggers that check if the select button is allowed to be pressed
int B_ALLOW=1;
//trigger that checks if the select button was pressed
int B_PRESS=0;

//variables to keep record of times
uint32_t T_START = 0;
uint32_t T_CURR=0;
uint32_t T_END=0;
uint32_t T_SCORE=0;
uint32_t T_RAND=0;
uint16_t T_BEST=20000;

prog_state state=S_FLASHLED;


char lcd_buffer[6];    // LCD display buffer



__IO HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY 
uint16_t EE_status=0;
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777}; // the emulated EEPROM can save 3 varibles, at these three addresses.
uint16_t EEREAD;  //to practice reading the BESTRESULT save in the EE, for EE read/write, require uint16_t type



TIM_HandleTypeDef Tim3_Handle, Tim4_Handle;
TIM_OC_InitTypeDef Tim4_OCInitStructure;
uint16_t Tim3_PrescalerValue, Tim4_PrescalerValue;
__IO uint16_t Tim4_CCR; // the pulse of the TIM4
__O uint8_t factor = 0;


//for using LCD
Point_Typedef singlePoint;
DoublePoint_Typedef doublePoint;
DigitPosition_Typedef charPosition;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void EXTI0_Config(void);
static void Error_Handler(void);
void TIM3_Config(void);
void TIM4_Config(void);
void TIM4_OC_Config(void);
void gen_rand (void);
static uint32_t getTime ( void );  
static void showTime ( uint16_t );                // Draw time value to LCD
void dispCurrTime ( void );               // Update LCD while in game
void checkbest (void);
RNG_HandleTypeDef Rng_Handle;






/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);




/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
	
	HAL_Init();

	uint8_t aChar='a';
	
	singlePoint=POINT_ON ;
	doublePoint=DOUBLEPOINT_OFF;
	charPosition=LCD_DIGIT_POSITION_1;   //OR : 0---6 OR _POSITION_1 (=0), _2...._6, _MAX_NUMBER(=6)

  /* Configure the system clock to 4 MHz */
  SystemClock_Config();
 
	HAL_InitTick(0x0000); //set the systick interrupt priority to the highest

	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
	
	BSP_JOY_Init(JOY_MODE_EXTI);

	

	BSP_LCD_GLASS_Init();
	

	//BSP_LCD_GLASS_ScrollSentence((uint8_t*) "  mt3ta4 lab2 starter", 1, 200);
	
	
	TIM3_Config();
	TIM4_Config();
	Tim4_CCR=20000;       //2 s to fire an interrupt.
	TIM4_OC_Config();







	
//******************* use emulated EEPROM ====================================
	//First, Unlock the Flash Program Erase controller 
	HAL_FLASH_Unlock();
		
// EEPROM Init 
	EE_status=EE_Init();
	if(EE_status != HAL_OK)
  {
		Error_Handler();
  }
// then can write to or read from the emulated EEPROM
	
	//uint16_t k = EE_WriteVariable(VirtAddVarTab[0],T_BEST);
	
	
	
	
	
	
//*********************use RNG ================================  
Rng_Handle.Instance=RNG;  //Everytime declare a Handle, need to assign its Instance a base address. like the timer handles.... 													
	
	Hal_status=HAL_RNG_Init(&Rng_Handle);   //go to msp.c to see further low level initiation.
	
	if( Hal_status != HAL_OK)
  {
    Error_Handler();
  }
//then can use RNG
	
	

  /* Infinite loop */
  while (1)
  {
		//manages state of system
		switch (state)
		{
			case S_FLASHLED:
				//first state on entering, LEDs flash pressing a button creates random number for timer
				//goes to state to turn off leds
				BSP_LCD_GLASS_DisplayString((uint8_t*)"CHEAT");	
				L_FLASH=1;
			  B_ALLOW=1;
				if(B_PRESS==1)
				{
					B_PRESS=0;
					gen_rand();
					B_ALLOW=0;
					T_START=getTime();
					state=S_LEDOFF;
				}
			  break;
			case S_LEDOFF:
				//LEDs off until "random time passes"
				//moves to previous state if button is pressed too early
			  //after random time moves to state that times reaction of user
				L_FLASH=0;
			  T_CURR=getTime();
				if(T_RAND<getTime()-T_START){
					B_ALLOW=1;
					L_FLASH=2;
					T_START=getTime();
					state=S_LEDTIMER;
				}
			  if(B_PRESS==1&&B_ALLOW==0)
				{
					B_PRESS=0;
					state=S_FLASHLED;
				}
				break;
			case S_LEDTIMER:
				//times user and waits until button is pressed
			  //LEDs are on
			  B_ALLOW=1;
			  dispCurrTime();
			  if(B_PRESS==1)
				{
					T_END=getTime();
					B_PRESS=0;
					state=S_DISPTIME;
				}
				break;
			case S_DISPTIME:
				//shows user's current score and checks if it is the best score
			  //LEDs are off, pressing button goes to next state
				L_FLASH=0;
			  B_ALLOW=1;
			  T_SCORE=T_END-T_START;
			  showTime(T_SCORE);
			  if(B_PRESS==1)
				{
					B_PRESS=0;
					checkbest();
					state=S_LEDDISPTIME;
				}
				break;
			case S_LEDDISPTIME:
				//display's user's best score
			  //LED's flash 
				L_FLASH=1;
				B_ALLOW=1;
			  showTime(T_BEST);
			  if(B_PRESS==1)
				{
					B_PRESS=0;
					B_ALLOW=0;
					gen_rand();
					T_START=getTime();
					state=S_LEDOFF;
				}
				break;
		}

	}
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = MSI
  *            SYSCLK(Hz)                     = 4000000
  *            HCLK(Hz)                       = 4000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            Flash Latency(WS)              = 0
  * @param  None
  * @retval None
  */

void gen_rand (void)
{
	//generate random time-------
	HAL_RNG_GenerateRandomNumber(&Rng_Handle,&T_RAND);
	T_RAND %=3500;
	T_RAND += 500;

}


uint32_t getTime ( void )
{
  return TIM4->CNT;
}


void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  // The following clock configuration sets the Clock configuration sets after System reset                
  // It could be avoided but it is kept to illustrate the use of HAL_RCC_OscConfig and HAL_RCC_ClockConfig 
  // and to be eventually adapted to new clock configuration                                               

  // MSI is enabled after System reset at 4Mhz, PLL not used 
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // RCC_MSIRANGE_6 is for 4Mhz. _7 is for 8 Mhz, _9 is for 16..., _10 is for 24 Mhz, _11 for 48Hhz
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  
//	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40; 
  RCC_OscInitStruct.PLL.PLLR = 2;  //2,4,6 or 8
  RCC_OscInitStruct.PLL.PLLP = 7;   // or 17.
  RCC_OscInitStruct.PLL.PLLQ = 4;   //2, 4,6, 0r 8  ===the clock for RNG will be 4Mhz *N /M/Q =40Mhz. which is nearly 48
	
	
	
	
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers 
  // Set 0 Wait State flash latency for 4Mhz 
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // The voltage scaling allows optimizing the power consumption when the device is
  //   clocked below the maximum system frequency, to update the voltage scaling value
  //   regarding system frequency refer to product datasheet.  

  // Enable Power Control clock 
  __HAL_RCC_PWR_CLK_ENABLE();

  if(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // Disable Power Control clock 
  __HAL_RCC_PWR_CLK_DISABLE();
}
//after RCC configuration, for timmer 2---7, which are one APB1, the TIMxCLK from RCC is 4MHz

void checkbest(void)
{
	uint16_t i = EE_ReadVariable(VirtAddVarTab[0],&T_BEST);
	if (T_SCORE<T_BEST){
		T_BEST=T_SCORE;
		i=EE_WriteVariable(VirtAddVarTab[0],T_BEST);
	}
}

void showTime ( uint16_t val )
{
  val %= MAX_REACTION_TIME;
  char str[6] = "      ";
  snprintf(str, 7, "%dMS   ", val);
  BSP_LCD_GLASS_DisplayString((uint8_t*) str);
}

void  TIM3_Config(void)
{

		/* -----------------------------------------------------------------------
    Tim3 is of 16 bits. Timer 2..7 is on APB1.
	
		Since the clock source is MSI, and the clock range is RCC_MSIRANGE_6, SystemCoreClock=4Mhz.
	
		Since RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1, HCLK=4Mhz.
	
		Since RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1, PCLK1=4MHz, and TIM3CLK=4Mhz.
		(if the APB1 prescaler is not 1, the timer clock frequency will be 2 times of APB1 frequency)
	
		 that is, for current RCC config, the the Timer3 frequency=SystemCoreClock.
	
		To get TIM3's counter clock at 10 KHz, for example, the Prescaler is computed as following:
    Prescaler = (TIM3CLK / TIM3 counter clock) - 1
	
		i.e: Prescaler = (SystemCoreClock /10 KHz) - 1
       
  ----------------------------------------------------------------------- */  
  
  /* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
  Tim3_PrescalerValue = (uint16_t) (SystemCoreClock/ 10000) - 1;
  
  /* Set TIM3 instance */
  Tim3_Handle.Instance = TIM3; //TIM3 is defined in stm32f429xx.h
 
  Tim3_Handle.Init.Period = 5000 - 1;
  Tim3_Handle.Init.Prescaler = Tim3_PrescalerValue;
  Tim3_Handle.Init.ClockDivision = 0;
  Tim3_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&Tim3_Handle) != HAL_OK) // this line need to call the callback function _MspInit() in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC..
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if(HAL_TIM_Base_Start_IT(&Tim3_Handle) != HAL_OK)   //the TIM_XXX_Start_IT function enable IT, and also enable Timer
																											//so do not need HAL_TIM_BASE_Start() any more.
  {
    /* Starting Error */
    Error_Handler();
  }
	
	
	
}

// configure Timer4 base.
void  TIM4_Config(void)
{
  
  /* Compute the prescaler value to have TIM3 counter clock equal to 1 KHz */
  Tim4_PrescalerValue = (uint16_t) (SystemCoreClock/ 1000) - 1;
  
  /* Set TIM3 instance */
  Tim4_Handle.Instance = TIM4; 
	Tim4_Handle.Init.Period = 40000;
  Tim4_Handle.Init.Prescaler = Tim4_PrescalerValue;
  Tim4_Handle.Init.ClockDivision = 0;
  Tim4_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  //if(HAL_TIM_Base_Init(&Tim4_Handle) != HAL_OK)
  //{
    /* Initialization Error */
  //  Error_Handler();
  //} 
}



void  TIM4_OC_Config(void)
{
		Tim4_OCInitStructure.OCMode=  TIM_OCMODE_TIMING;
		Tim4_OCInitStructure.Pulse=Tim4_CCR;
		Tim4_OCInitStructure.OCPolarity=TIM_OCPOLARITY_HIGH;
		
		HAL_TIM_OC_Init(&Tim4_Handle); // if the TIM4 has not been set, then this line will call the callback function _MspInit() 
													//in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC.
	
		HAL_TIM_OC_ConfigChannel(&Tim4_Handle, &Tim4_OCInitStructure, TIM_CHANNEL_1); //must add this line to make OC work.!!!
	
	   /* **********see the top part of the hal_tim.c**********
		++ HAL_TIM_OC_Init and HAL_TIM_OC_ConfigChannel: to use the Timer to generate an 
              Output Compare signal. 
			similar to PWD mode and Onepulse mode!!!
	
	*******************/
	
	 	HAL_TIM_OC_Start_IT(&Tim4_Handle, TIM_CHANNEL_1); //this function enable IT and enable the timer. so do not need
				//HAL_TIM_OC_Start() any more
				
		
}


/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin) {
			case GPIO_PIN_0: 		               //SELECT button					
						B_PRESS=1;						
						break;	
			case GPIO_PIN_1:     //left button						
							break;
			case GPIO_PIN_2:    //right button						  to play again.
					
							break;
			case GPIO_PIN_3:    //up button							
					
							break;
			
			case GPIO_PIN_5:    //down button						
					
							break;
			default://
						//default
						break;
	  } 
}
void dispCurrTime ( void )
{
	uint32_t react = getTime() - T_START;

  //uint32_t reactionTime = react * 1000 / (SystemCoreClock / 100);

  showTime(react);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)   //see  stm32lxx_hal_tim.c for different callback function names. 
																															//for timer 3 , Timer 3 use update event initerrupt
{
			//clear the timer counter at the end of call back to avoid interrupt interval variation!  in stm32l4xx_hal_tim.c, the counter is not cleared after  OC interrupt
		__HAL_TIM_SET_COUNTER(htim, 0x0000);   //this macro is defined in stm32l4xx_hal_tim.h
	 
	  if(L_FLASH==0)
		{
			BSP_LED_Off(LED4);
			BSP_LED_Off(LED5);
		}
		else if (L_FLASH==1)
		{
			BSP_LED_Toggle(LED4);
			BSP_LED_Toggle(LED5);
		}
		else if(L_FLASH==2)
		{
			BSP_LED_On(LED4);
			BSP_LED_On(LED5);
		}	
}


void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) //see  stm32fxx_hal_tim.c for different callback function names. 
{																																//for timer4 
	

		//clear the timer counter at the end of call back to avoid interrupt interval variation!  in stm32l4xx_hal_tim.c, the counter is not cleared after  OC interrupt
		__HAL_TIM_SET_COUNTER(htim, 0x0000);   //this macro is defined in stm32l4xx_hal_tim.h
	
		
}


static void Error_Handler(void)
{
 
 
}



#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

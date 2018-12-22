/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stdbool.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


void RGB(uint8_t ch,uint8_t d, uint8_t e, uint8_t f);
void initRGB(void);
void setBuzz(uint8_t bzz);
uint8_t byteMap(uint8_t x);
bool digitalRead(uint8_t pin);
void digitalWrite(uint8_t pinz, bool statx);
void processSwitch(void);
void fadeColor(void);
void processInputs(void);
uint8_t crc(void);
void processSerial(void);
bool sw1,sw2,sw3,sw4;
bool t1 = false;
bool t2 = false;
bool t3 = false;
bool t4 = false;
uint8_t onColor[] = {255,165,0};
uint8_t offColor[] = {0,0,255};
uint8_t inBuff[11];
uint8_t outBuff[] = {0xCC,0xAA,0xCC,0xAA,0x00};
bool inBuff_isValid = false;



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  initRGB();
   RGB(1,0,0,0);
   RGB(2,0,0,0);
   RGB(3,0,0,0);
   RGB(4,0,0,0);
   sw1 = false;
   sw2 = false;
   sw3 = false;
   sw4 = false;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    processInputs();
   processSerial();
   processSwitch();
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

void RGB(uint8_t ch,uint8_t d,uint8_t e, uint8_t f)
{
  uint8_t red = byteMap(d);
  uint8_t grn = byteMap(e);
  uint8_t blu = byteMap(f);

  if(ch == 1)
  {
    htim2.Instance->CCR1 = red;
    htim2.Instance->CCR2 = grn;
    htim4.Instance->CCR1 = blu;
  }
  else if(ch == 2)
  {
    htim3.Instance->CCR4 = red;
    htim2.Instance->CCR3 = grn;
    htim2.Instance->CCR4 = blu;

  }
  else if(ch == 3)
  {
    htim3.Instance->CCR1 = red;
    htim3.Instance->CCR2 = grn;
    htim3.Instance->CCR3 = blu;
  }
  else if(ch == 4)
  {
    htim4.Instance->CCR2 = red;
    htim4.Instance->CCR3 = grn;
    htim4.Instance->CCR4 = blu;
  }

}

void initRGB()
{
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1 );
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1 );
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2 );
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3 );
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4 );
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1 );
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2 );
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3 );
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4 );
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1 );
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2 );
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3 );
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4 );
}

void setBuzz(uint8_t bzz)
{
  htim1.Instance->CCR1 = bzz;
}


uint8_t byteMap(uint8_t x)
{
  uint8_t in_min = 0;
  uint8_t in_max = 255;
  uint8_t out_min = 255;
  uint8_t out_max = 0;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


bool digitalRead(uint8_t pin)
{
  GPIO_PinState dx = GPIO_PIN_RESET;
  bool ret = false;

  if(pin == 1)
  {
      dx = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);
  }
  else if(pin == 2)
  {
        dx = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);
  }

  else if(pin == 3)
  {
    dx = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);

  }

  else if(pin == 4)
  {
    dx = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

  }

  if(dx == GPIO_PIN_SET)
  {
    ret = true;
  }
  return ret;
}

void digitalWrite(uint8_t pinz, bool statx)
{
  GPIO_PinState dd = GPIO_PIN_RESET;
  if(statx == true)
  {
    dd = GPIO_PIN_SET;
  }
  if(pinz == 1)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, dd);
  }
  else if(pinz == 2)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, dd);
  }
  else if(pinz == 3)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, dd);
  }
  else if(pinz == 4)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, dd);
  }

}






void processSwitch()
{
  if(sw1 == true)
  {
    RGB(1,onColor[0],onColor[1],onColor[2]);
    digitalWrite(1, true);
  }
  else
  {
    RGB(1,offColor[0],offColor[1],offColor[2]);
    digitalWrite(1, false);
  }


  if(sw2 == true)
  {
    RGB(2,onColor[0],onColor[1],onColor[2]);
    digitalWrite(2, true);
  }
  else
  {
    RGB(2,offColor[0],offColor[1],offColor[2]);
    digitalWrite(2, false);
  }

  if(sw3 == true)
  {
    RGB(3,onColor[0],onColor[1],onColor[2]);
    digitalWrite(3, true);
  }
  else
  {
    RGB(3,offColor[0],offColor[1],offColor[2]);
    digitalWrite(3, false);
  }


  if(sw4 == true)
  {
    RGB(4,onColor[0],onColor[1],onColor[2]);
    digitalWrite(4, true);
  }
  else
  {
    RGB(4,offColor[0],offColor[1],offColor[2]);
    digitalWrite(4, false);
  }
}


void fadeColor()
{
  uint32_t delval = 20;
  HAL_Delay(500);

  for(int i=0;i<255;i++)
  {
    for(int j=0;j<5;j++)
    {
      RGB(j,i,0,0);
    }
    HAL_Delay(delval);
  }

  for(int i=255;i>0;i--)
  {
    for(int j=0;j<5;j++)
    {
      RGB(j,i,0,0);
    }
    HAL_Delay(delval);
  }



  //------------------------------------Red OVER------------------//
  for(int i=0;i<255;i++)
  {
    for(int j=0;j<5;j++)
    {
      RGB(j,0,i,0);
    }
    HAL_Delay(delval);
  }


  for(int i=255;i>0;i--)
  {
    for(int j=0;j<5;j++)
    {
      RGB(j,0,i,0);
    }
    HAL_Delay(delval);
  }



//------------------------------------------Green OVER ---  -------//
for(int i=0;i<255;i++)
{
for(int j=0;j<5;j++)
{
  RGB(j,0,0,i);
}
HAL_Delay(delval);
}

  for(int i=255;i>0;i--)
  {
    for(int j=0;j<5;j++)
    {
      RGB(j,0,0,i);
    }
    HAL_Delay(delval);
  }

}
//----------------------------------------------------------------------------

void processInputs()
{
  bool s1,s2,s3,s4;
  s1 = digitalRead(1);
  s2 = digitalRead(2);
  s3 = digitalRead(3);
  s4 = digitalRead(4);
//----------------------------------------H1----------------------------------
  if(s1 == true)
  {
    if((t1==false)&&(sw1==false))
    {
      sw1 = true;
      t1 = true;
    }
    else if((t1==false)&&(sw1==true))
    {
      sw1 = false;
      t1 = true;
    }
  }
  else
  {
    t1 = false;
  }
  //----------------------------------------H2----------------------------------
  if(s2 == true)
  {
    if((t2==false)&&(sw2==false))
    {
      sw2 = true;
      t2 = true;
    }
    else if((t2==false)&&(sw2==true))
    {
      sw2 = false;
      t2 = true;
    }
  }
  else
  {
    t2 = false;
  }
  //----------------------------------------H3----------------------------------
  if(s3 == true)
  {
    if((t3==false)&&(sw3==false))
    {
      sw3 = true;
      t3 = true;
    }
    else if((t3==false)&&(sw3==true))
    {
      sw3 = false;
      t3 = true;
    }
  }
  else
  {
    t3 = false;
  }
  //----------------------------------------H4----------------------------------
  if(s4 == true)
  {
    if((t4==false)&&(sw4==false))
    {
      sw4 = true;
      t4 = true;
    }
    else if((t4==false)&&(sw4==true))
    {
      sw4 = false;
      t4 = true;
    }
  }
  else
  {
    t4 = false;
  }

}


uint8_t crc()
{
  uint8_t cdc = 0x00;
  for(int i=0;i<9;i++)
  {
    cdc ^= inBuff[i];
  }
  return cdc;
}

void processSerial()
{
  for(int i=0;i<10;i++)
  {
    inBuff[i] = 0x00;
  }
  inBuff_isValid = false;
  HAL_UART_Receive(&huart1, inBuff, 10,10);

  if((inBuff[0]==0xCC)&&(inBuff[1]==0xAA)&&(inBuff[2]==0xCC)&&(inBuff[3]==0xAA))
  {
    outBuff[0] = crc();
    outBuff[1] = inBuff[9];
    HAL_UART_Transmit(&huart1, outBuff, 5,50);

    if(inBuff[9] == crc())
    {

      inBuff_isValid = true;
    }
  }

  if(inBuff_isValid == true)
  {
    if(inBuff[4] == 0x01)
    {
      if(inBuff[5] == 0x01)
      {
        sw1 = true;
      }
      else if(inBuff[5] == 0x00)
      {
        sw1 = false;
      }


      if(inBuff[6] == 0x01)
      {
        sw2 = true;
      }
      else if(inBuff[6] == 0x00)
      {
        sw2 = false;
      }


      if(inBuff[7] == 0x01)
      {
        sw3 = true;
      }
      else if(inBuff[7] == 0x00)
      {
        sw3 = false;
      }

      if(inBuff[8] == 0x01)
      {
        sw4 = true;
      }
      else if(inBuff[8] == 0x00)
      {
        sw4 = false;
      }
    }
//--------------------------CMD 0x02------------------------------

else if(inBuff[4]==0x02)
{
  onColor[0] = inBuff[5];
  onColor[1] = inBuff[6];
  onColor[2] = inBuff[7];
}

//--------------------------CMD 0x03------------------------------
if(inBuff[4]==0x03)
{
  offColor[0] = inBuff[5];
  offColor[1] = inBuff[6];
  offColor[2] = inBuff[7];
}

  }
}




/************************ (C) COPYRIGHT HatchPrototype *****END OF FILE****/

/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

/*
 * Arquivo:D:\BackupCesar\C_bk\My_Documents\Pesquisa\Robo_UDESC\PIC30F\Proposta13_a\Robo_UTFPR.c
 * C?digo Gerado para a familia MSP430Gxx com sa?da Mealy
 * Foi utilizado listas encadeadas como jogador de aut?mato
 * Desenvolvido por C?sar Rafael Claure Torrico
 */

//Dados do aut?mato (N?o pode ser declarado dentro da fun??o main por ser const)
#define NTRANS 76   //N?mero de Transi??es
#define NESTADOS 22 //N?mero de Estados
#define BUFFER 10   //M?ximo N?mero de Eventos no Buffer
const unsigned int event[NTRANS]={6,4,2,1,10,18,16,8,4,2,5,10,12,4,2,9,4,2,17,4,2,11,14,8,4,3,2,3,4,2,21,14,12,4,2,10,19,12,4,2,14,19,8,13,10,18,16,8,3,7,10,18,16,8,3,4,2,14,12,5,10,12,9,17,11,14,8,1,10,18,16,8,21,14,12,20};
const unsigned int in_state[NTRANS]={1,7,6,1,5,4,3,2,7,6,9,8,1,7,6,1,7,6,1,7,6,10,1,8,12,11,14,13,7,6,15,2,5,7,6,8,1,1,7,6,1,1,8,1,19,18,17,16,11,1,19,18,17,16,13,7,6,2,5,9,21,20,1,1,10,20,21,1,19,18,17,16,15,16,19,1};
const unsigned int rfirst[NESTADOS] = {76,8,13,16,19,24,26,28,33,38,43,48,49,54,55,59,62,63,64,67,72,75};
const unsigned int rnext[NTRANS] = {0,0,2,3,4,5,6,7,0,9,10,11,12,0,14,15,0,17,18,0,20,21,22,23,0,25,0,27,0,29,30,31,32,0,34,35,36,37,0,39,40,41,42,0,44,45,46,47,0,0,50,51,52,53,0,0,56,57,58,0,60,61,0,0,0,65,66,0,68,69,70,71,0,73,74,1};

//mapeamento de eventos n?o control?veis como entradas
#define   Bd 	20
#define   Be 	6
#define   s1    2
#define   s2    4
#define   p1a   8
#define   p2a   10
#define   p1p   12
#define   p2p   14
#define   p3a   16
#define   p4a   18


#define TAM 8


//definição dos eventos de saída
#define EF_50  __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,20);	//Saida 0 PWM 50%
#define EF_100 __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,40);	//Saida 0 PWM 100%
#define EF_0 __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,0);	//Saida 0 PWM 0%
#define ER_50   __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,20);	//Saida 1 PWM 50%
#define ER_100 __HAL_TIM_SetCompare(&htim2	,TIM_CHANNEL_1,40);	//Saida 1 PWM 100%
#define ER_0  __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,0);	//Saida 1 PWM 0%
#define DF_50  __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,20);	//Saida 2 PWM 50%
#define DF_100 __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,40);	//Saida 2 PWM 100%
#define DF_0 __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,0);	//Saida 2 PWM 0%
#define DR_50   __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,20);	//Saida 3 PWM 50%
#define DR_100 __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,40);	//Saida 3 PWM 100%
#define DR_0  __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,0);	//Saida 3 PWM 0%

#define LD_ON  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);	//Saida 4 ON
#define LD_OFF HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);	//Saida 4 OFF
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/



/* USER CODE BEGIN PV */
char *bufftr;
uint8_t buffrc[2];
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;

/* Private variables ---------------------------------------------------------*/
unsigned char buffer[BUFFER];		//Buffer para armazenar a fila de enventos externos
unsigned char n_buffer=0;		//Número de eventos no Buffer
unsigned int i;
unsigned int tempo=0;
char buf[32];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void GPIO_INIT(void);
void Temporizador_ms(unsigned int TEMPO);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);



/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
 // SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  	SystemInit();
    SystemCoreClockUpdate();
   // SystemClock_Config();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();


  /* USER CODE BEGIN 2 */
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_TC);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
 // HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  //HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);

  GPIO_INIT();

  	unsigned int k;
	int occur_event=-1;			//Evento ocorrido
	unsigned int current_state = 0;	//Estado atual inicializado com estado inicial
	int g=0; 			//Flag para gerador aleatório de eventos
	int gerar_evento=1;			//Flag para habilitar a temporização de eventos controláveis
	int mealy_output = 0;		//Inicializa saída periférica

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

	  if(n_buffer == 0)//se não existir evento no buffer então gerar um evento interno(evento controlável)
	  		{
	  			if(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE)!= RESET)	//Se o timer estourar, habilita a geração de eventos
	  			{
	  				__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
	  				HAL_TIM_Base_Stop_IT(&htim1);
	  				gerar_evento=1;
	  			}
	  			if(gerar_evento==1)
	  			{
	  				switch(g)	//Aqui é implementado um gerador automático de eventos controláveis
	  				{
	  				case(0):
	  					occur_event=1;
	  					g++;
	  					break;
	  				case(1):
	  					occur_event=3;
	  					g++;
	  					break;
	  				case(2):
	  					occur_event=5;
	  					g++;
	  					break;
	  				case(3):
	  					occur_event=7;
	  					g++;
	  					break;
	  				case(4):
	  					occur_event=9;
	  					g++;
	  					break;
	  				case(5):
	  					occur_event=11;
	  					g++;
	  					break;
	  				case(6):
	  					occur_event=13;
	  					g++;
	  					break;
	  				case(7):
	  					occur_event=15;
	  					g++;
	  					break;
	  				case(8):
	  					occur_event=17;
	  					g++;
	  					break;
	  				case(9):
	  					occur_event=19;
	  					g++;
	  					break;
	  				case(10):
	  					occur_event =21;
	  					g=0;
	  					break;
	  				}
	  			}
	  		}
	  		else 	//se existir evento não controlável pegar do buffer
	  		{
	  			occur_event = buffer[0];
	  			n_buffer--;
	  			k = 0;
	  			while(k<n_buffer)
	  			{
	  				buffer[k] = buffer[k+1];
	  				k++;
	  			}
	  		}

	  		//Jogador de autômato
	  		k = rfirst[current_state];
	  		if(k==0)
	  		{
	  			return 0;     //Dead Lock!!!
	  		}
	  		else
	  		{
	  			while(k>0)
	  			{
	  				k--;
	  				if(event[k] == occur_event)
	  				{
	  					current_state = in_state[k];
	  					mealy_output = 1;
	  					break;
	  				}
	  				k = rnext[k];
	  			}
	  		}

	  		if(mealy_output) //Se o evento ocorrido for válido, então imprimir saída física
	  		{
	  			switch(occur_event)
	  			{
	  			 	 	  //eventos control?veis
					case(1):    //Adicionar A??o para o Evento 1; -> Frente
						GPIO_INIT();
						EF_50;
						DF_50;
						ER_0;
						DR_0;
						break;
					case(3):    //Adicionar A??o para o Evento 3; -> Re
						GPIO_INIT();
						EF_0;
						DF_0;
						ER_0;
						DR_0;
						gerar_evento=0; //para de gerar eventos control?veis
						Temporizador_ms(1000);
						break;
					case(5):    //Adicionar A??o para o Evento 5; -> Ge1
						GPIO_INIT();
						EF_0;
						DF_50;
						ER_0;
						DR_0;
						break;
					case(7):    //Adicionar A??o para o Evento 7; -> Ge2
						GPIO_INIT();
						EF_0;
						DF_50;
						ER_50;
						DR_0;
						gerar_evento=0; //para de gerar eventos control?veis
						Temporizador_ms(800);
						break;
					case(9):    //Adicionar A??o para o Evento 9; -> Ge3
						GPIO_INIT();
						EF_0;
						DF_50;
						ER_50;
						DR_0;
						gerar_evento=0; //para de gerar eventos control?veis
						Temporizador_ms(1000);
						break;
					case(11):   //Adicionar A??o para o Evento 11; -> Gd1
						GPIO_INIT();
						EF_50;
						DF_0;
						ER_0;
						DR_0;
						break;
					case(13):   //Adicionar A??o para o Evento 13; -> Gd2
						GPIO_INIT();
						EF_50;
						DF_0;
						ER_0;
						DR_50;
						gerar_evento=0; //para de gerar eventos control?veis
						Temporizador_ms(800);
						break;
					case(17):   //Adicionar A??o para o Evento 17; -> Gd3
						GPIO_INIT();
						EF_50;
						DF_0;
						ER_0;
						DR_50;
						gerar_evento=0; //para de gerar eventos control?veis
						Temporizador_ms(1000);
						break;
					case(19):   //Adicionar A??o para o Evento 19; ->Ts

						break;
					case(21):   //Adicionar A??o para o Evento 21; -> Frente Turbo
						GPIO_INIT();
						EF_100;
						DF_100;
						ER_0;
						DR_0;
						break;
					//eventos n?o control?veis
					case(2):    //Adicionar A??o para o Evento 2; -> S1
						gerar_evento=1;
						break;
					case(4):    //Adicionar A??o para o Evento 4; -> S2
						gerar_evento=1;
						break;
					case(6):    //Adicionar A??o para o Evento 6; -> Be bot?o come?ando pela esquerda
						LD_OFF;
						Temporizador_ms(500);
						while(!(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE))!= RESET); // fica presso enquanto o timer n?o estura
						LD_ON;
						Temporizador_ms(500);
						while(!(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE))!= RESET); // fica presso enquanto o timer n?o estura
						LD_OFF;
						Temporizador_ms(500);
						while(!(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE))!= RESET); // fica presso enquanto o timer n?o estura
						LD_ON;
						Temporizador_ms(500);
						while(!(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE))!= RESET); // fica presso enquanto o timer n?o estura
						LD_OFF;
						Temporizador_ms(500);
						while(!(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE))!= RESET); // fica presso enquanto o timer n?o estura
						LD_ON;
						Temporizador_ms(500);
						while(!(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE))!= RESET);// fica presso enquanto o timer n?o estura
						LD_OFF;
						Temporizador_ms(500);
						while(!(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE))!= RESET);// fica presso enquanto o timer n?o estura
						LD_ON;
						Temporizador_ms(500);
						while(!(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE))!= RESET);// fica presso enquanto o timer n?o estura
						LD_OFF;
						Temporizador_ms(500);
						while(!(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE))!= RESET); // fica presso enquanto o timer n?o estura
						LD_ON;
						Temporizador_ms(500); //fica preso enquanto n?o estourar o timer
						while(!(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE))!= RESET); // fica presso enquanto o timer n?o estura
						EF_50;
						DF_50;
						ER_0;
						DR_0;
						gerar_evento=0;
						Temporizador_ms(750); //Tempo de giro
						break;
					case(8):    //Adicionar A??o para o Evento 8; -> P1a

						gerar_evento=1;
						break;
					case(10):   //Adicionar A??o para o Evento 10; -> P2a

						gerar_evento=1;
						break;
					case(12):   //Adicionar A??o para o Evento 12; -> P1p

						gerar_evento=1;
						break;
					case(14):   //Adicionar A??o para o Evento 14; -> P2p

						gerar_evento=1;
						break;
					case(16):   //Adicionar A??o para o Evento 16; -> P3a

						gerar_evento=1;
						break;
					case(18):   //Adicionar A??o para o Evento 18; -> P4a

						gerar_evento=1;
						break;
					case(20):   //Adicionar A??o para o Evento 20; -> Bd bot?o come?ando pela direita
						LD_OFF;
						Temporizador_ms(500);
						while(!(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE))!= RESET); // fica presso enquanto o timer n?o estura
						LD_ON;
						Temporizador_ms(500);
						while(!(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE))!= RESET); // fica presso enquanto o timer n?o estura
						LD_OFF;
						Temporizador_ms(500);
						while(!(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE))!= RESET); // fica presso enquanto o timer n?o estura
						LD_ON;
						Temporizador_ms(500);
						while(!(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE))!= RESET); // fica presso enquanto o timer n?o estura
						LD_OFF;
						Temporizador_ms(500);
						while(!(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE))!= RESET); // fica presso enquanto o timer n?o estura
						LD_ON;
						Temporizador_ms(500);
						while(!(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE))!= RESET);// fica presso enquanto o timer n?o estura
						LD_OFF;
						Temporizador_ms(500);
						while(!(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE))!= RESET);// fica presso enquanto o timer n?o estura
						LD_ON;
						Temporizador_ms(500);
						while(!(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE))!= RESET);// fica presso enquanto o timer n?o estura
						LD_OFF;
						Temporizador_ms(500);
						while(!(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE))!= RESET); // fica presso enquanto o timer n?o estura
						LD_ON;
						Temporizador_ms(500); //fica preso enquanto n?o estourar o timer
						while(!(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE))!= RESET); // fica presso enquanto o timer n?o estura
						EF_50;
						DF_50;
						ER_0;
						DR_0;
						gerar_evento=0;
						Temporizador_ms(750); //Tempo de giro
						break;
						}//fim switch
	  			mealy_output = 0;
	  			occur_event = -1;
	  		}//fim if(mealy_output)
	  	}//fim while(1)

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */


/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void Temporizador_ms(unsigned int TEMPO)
{
	__HAL_TIM_SET_AUTORELOAD(&htim1, TEMPO-1);
	HAL_TIM_Base_Start_IT(&htim1);
}

void GPIO_INIT(void){
	EF_0;
	ER_0;
	DF_0;
	DR_0;
	LD_OFF;
}



/* TIM1 init function */
static void MX_TIM1_Init(void)
{

	__HAL_RCC_TIM3_CLK_ENABLE();

	 htim1.Instance = TIM1;
	 htim1.Init.Period = 0;
	 htim1.Init.Prescaler = 16500-1;
	 htim1.Init.ClockDivision = 0;
	 htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	 htim1.Init.RepetitionCounter = 0;
	 HAL_TIM_Base_Init(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 24;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 200;
  htim2.Init.ClockDivision = 0;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler =24;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 200;
  htim3.Init.ClockDivision = 0;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();



  // Initialize PA5

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA,&GPIO_InitStruct);
  /*Configure GPIO pins : PD8 PD9 PD10 PD11
                           PD12 PD13 PD14 PD15
                           PD0 PD1 PD2 PD3
                           PD4 PD5 PD6 PD7 */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                            |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                            |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                            |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);


  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
      HAL_NVIC_EnableIRQ(EXTI0_IRQn);
      HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 1);
      HAL_NVIC_EnableIRQ(EXTI1_IRQn);
      HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 2);
      HAL_NVIC_EnableIRQ(EXTI2_IRQn);
      HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 3);
      HAL_NVIC_EnableIRQ(EXTI3_IRQn);
      HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 4);
      HAL_NVIC_EnableIRQ(EXTI4_IRQn);
      HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 5);
      HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
      HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 5);
      HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
      HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 6);
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void EXTI0_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);

}
void EXTI1_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);

}
void EXTI2_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);

}
void EXTI3_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);

}
void EXTI4_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);

}
void EXTI9_5_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_5) != RESET)
		{
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
		}
	else if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_6) != RESET)
		{
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
		}
	else if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_7) != RESET)
			{
				HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
			}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin & GPIO_PIN_2)
	{
		if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_2) == GPIO_PIN_RESET)
		{
			snprintf(buf,TAM,"\n\rp1p\n\r");
			HAL_UART_Transmit_IT(&huart1, buf ,strlen(buf));
			strcpy(buf,"");
			buffer[n_buffer]=p1p;	//Atribuir evento a GPIOC_Pin_3
					n_buffer++;
		}
		else
		{
			snprintf(buf,TAM,"\n\rp1a\n\r");
			HAL_UART_Transmit_IT(&huart1, buf ,strlen(buf));
			strcpy(buf,"");
			buffer[n_buffer]=p1a;	//Atribuir evento a GPIOC_Pin_3
			n_buffer++;
		}
	}
	else if(GPIO_Pin & GPIO_PIN_1)
		{
			if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_1) == GPIO_PIN_RESET)
				{
					snprintf(buf,TAM,"\n\rp2p\n\r");
					HAL_UART_Transmit_IT(&huart1, buf ,strlen(buf));
					strcpy(buf,"");
					buffer[n_buffer]=p2p;	//Atribuir evento a GPIOC_Pin_3
							n_buffer++;
				}
				else
				{
					snprintf(buf,TAM,"\n\rp2a\n\r");
					HAL_UART_Transmit_IT(&huart1, buf ,strlen(buf));
					strcpy(buf,"");
					buffer[n_buffer]=p2a;	//Atribuir evento a GPIOC_Pin_3
					n_buffer++;
				}
		}
	else if(GPIO_Pin & GPIO_PIN_3)
		{
			if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_3) == GPIO_PIN_SET){

				snprintf(buf,TAM,"\n\rp3a\n\r");
				HAL_UART_Transmit_IT(&huart1, buf ,strlen(buf));
				strcpy(buf,"");
				buffer[n_buffer]=p3a;	//Atribuir evento a GPIOC_Pin_2
				n_buffer++;
			}
		}
	else if(GPIO_Pin & GPIO_PIN_4)
		{
		if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_4) == GPIO_PIN_SET){
			snprintf(buf,TAM,"\n\rp4a\n\r");
			HAL_UART_Transmit_IT(&huart1, buf ,strlen(buf));
			strcpy(buf,"");
			buffer[n_buffer]=p4a;	//Atribuir evento a GPIOC_Pin_2
			n_buffer++;
		}
		}
	else if(GPIO_Pin & GPIO_PIN_5)
		{
		if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_5) == GPIO_PIN_RESET){
			snprintf(buf,TAM,"\n\rs1 \n\r");
			HAL_UART_Transmit_IT(&huart1, buf ,strlen(buf));
			strcpy(buf,"");
			buffer[n_buffer]=s1;	//Atribuir evento a GPIOC_Pin_2
			n_buffer++;
			}
		}
	else if(GPIO_Pin & GPIO_PIN_6)
		{
		if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_6) == GPIO_PIN_RESET){
			snprintf(buf,TAM,"\n\rs2 \n\r");
			HAL_UART_Transmit_IT(&huart1, buf ,strlen(buf));
			strcpy(buf,"");
			buffer[n_buffer]=s2;	//Atribuir evento a GPIOC_Pin_2
			n_buffer++;
		}
		}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

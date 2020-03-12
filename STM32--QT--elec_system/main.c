/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <MPU6050.h>
#include <gui.h>
#include <ESP8266.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId KeyTaskHandle;
osThreadId DispTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define MAX_RECV_LEN 128
extern GUI_CONST_STORAGE GUI_BITMAP bmbptjl;
extern GUI_FLASH const GUI_FONT GUI_FontHZ_SimSun_12;
extern GUI_FLASH const GUI_FONT GUI_FontHZ_KaiTi_20;
extern GUI_FLASH const GUI_FONT GUI_FontHZ_NewSimSun_16;
extern GUI_FLASH const GUI_FONT GUI_FontHZ_NewSimSun_8;
extern GUI_FLASH const GUI_FONT GUI_FontHZ_NewSimSun_802;

int g_bMpuok = 0;						//MPU6050初始化成功标志
uint16_t g_Keypres = 0;			//按键状态记录[按位记录]
uint8_t g_Ledsta = 0x01;		//LED灯记录[按位记录]
uint8_t g_xx=0x00;
uint8_t g_bUsePWM = 1;
uint16_t g_val[20];
uint16_t g_adval = 0;

uint8_t rx1_buff[MAX_RECV_LEN*4] = {0};  // 串口接收数据缓冲
uint8_t rx1_show[MAX_RECV_LEN*4] = {0};	//显示接收到的数据
uint8_t * pBuf = rx1_buff;  // 当前接收字节存放位置指针
uint8_t line_flag = 0;      // 一行数据接收标志
HAL_StatusTypeDef rxit_ok; 	// 接收中断是否开启
uint8_t rx2_buff[MAX_RECV_LEN] = {0};  // 串口接收数据缓冲
uint8_t rx2_show[MAX_RECV_LEN*2] = {0};	//显示接收到的数据
uint8_t * pBuf2 = rx2_buff; // 当前接收字节存放位置指针
uint8_t line_flag2 = 0;     // 一行数据接收标志
HAL_StatusTypeDef rxit_ok2; // 接收中断是否开启
uint8_t uart1rxtick = 0;
uint8_t uart2rxtick = 0;
char uart2displine[50]={0}; 		// 串口2接收数据显示行
uint8_t Esp8266Err = 0;

int g_nScreen = 0;			// 页面索引，-1：副界面 0：主界面 1：系统测试 2：姿态解算 3：串口通信 4：WIFI通信 5：2d 6：飞机游戏
int g_nSel = 0;					// 准备选择项
int g_nSendTime = 1000;	// 送间隔
int g_bSend = 0;


//新加全局变量
uint8_t mpuf=1;		//MPU换页
uint8_t syslsdf=0;//流水灯控制
uint8_t sysf1=0;		//系统测试换页
uint8_t sysf2=0;
uint8_t mpu2df=0;	//MPU6050的2d功能
uint8_t initf=1;	//启动界面按键控制
uint8_t rxfk1=0;
uint8_t rxfk2=0;
uint8_t rxfk3=0;
uint8_t wifik1=0;
uint8_t wifik2=0;
uint8_t wifif=0;
char tmp[10];
char *s,c[5];
int px=24,py=0;
int mpx,mpy;
char tmp_esp[150];
char buf_esp[150];
uint8_t usart1f=0;
uint8_t fw=0;
int summ=0;

int yi=0;
int yj=0;
int tx,ty;

uint8_t flyf=0;
uint8_t over=0;
uint8_t zantin=0;
int randx[100];
int randy[100];
int randi=0;
int sssx=0;
int clockt=0;
int speed=120;
int flycnt=0;
int i,j,k;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/


//初始化
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void const * argument);
void StartKeyTask(void const * argument);
void StartDispTask(void const * argument);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void LED_GPIO_Init(void);
/* USER CODE END PFP */

//相关实现函数
void Uart2SendStr(char *str);	//串口发来的数据再发给串口
int fputc(int ch, FILE *f);		//向串口1输出一个字符
void SetLEDS(uint8_t sta);		//设置LED[一次设置四个]
void RunLsd(void);						//运行流水灯
void SetPWMLight(uint8_t light);
uint8_t ScanOneKey(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint8_t ScanKey(void);				//按键检测
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void Uart2SendStr(char *str);
uint8_t InitESP8266(void);




int main(void)
{
	//auto create
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
	

	
/* USER CODE BEGIN 2 */
	rxit_ok = HAL_UART_Receive_IT(&huart1, pBuf, 1);    // 串口1开启第一次中断，每次接收1字节
	rxit_ok2 = HAL_UART_Receive_IT(&huart2, pBuf2, 1);  // 串口2开启第一次中断，每次接收1字节	
	printf("\nTengJiaLu\n16073212\n\n");         // 向串口1发送一行字符串
	Esp8266Err = InitESP8266();		//WIFI初始化

	
	
  /* USER CODE END 2 */

  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of KeyTask */
  osThreadDef(KeyTask, StartKeyTask, osPriorityIdle, 0, 128);
  KeyTaskHandle = osThreadCreate(osThread(KeyTask), NULL);

  /* definition and creation of DispTask */
  osThreadDef(DispTask, StartDispTask, osPriorityIdle, 0, 128);
  DispTaskHandle = osThreadCreate(osThread(DispTask), NULL);
  osKernelStart();
	
	while(1);
}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void){

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 719;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
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

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* DMA    */
static void MX_DMA_Init(void) {
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/* GPIO  */
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OLED_SCL_Pin|OLED_SDA_Pin|EN_Pin|MPU6050_SCL_Pin 
                          |MPU6050_SDA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : K1_Pin K2_Pin K3_Pin K4_Pin */
  GPIO_InitStruct.Pin = K1_Pin|K2_Pin|K3_Pin|K4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin MPU6050_SCL_Pin MPU6050_SDA_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|MPU6050_SCL_Pin|MPU6050_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_Pin */
  GPIO_InitStruct.Pin = EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : STATE_Pin */
  GPIO_InitStruct.Pin = STATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(STATE_GPIO_Port, &GPIO_InitStruct);

}




void AddJiaoYan(char *buf){
	char tmp[20],*p=buf;
	uint16_t tcnt=0,tm;
	while(*p!='\0'){
		tcnt+=*p;
		++p;
	}
	tm=tcnt%16;
	tcnt>>=4;
	sprintf(tmp,"%0x%0x\n",tcnt%16,tm);
	strcat(buf,tmp);			
}






//
/* USER CODE BEGIN 4 */
void DrawScreen0(void){
	GUI_SetFont(&GUI_FontHZ_NewSimSun_16);
	GUI_SetColor(GUI_COLOR_BLACK);
	GUI_ClearRect(0, 0, 20, 64);
	GUI_DispStringAt("主", 2,6);
	GUI_DispStringAt("菜", 2, 24);
	GUI_DispStringAt("单", 2, 42);
	GUI_SetColor(GUI_COLOR_WHITE);
	GUI_DispStringAt("MPU6050", 50, 16);
	GUI_DispStringAt("WIFI测试", 50, 48);
	GUI_DispStringAt("系统测试", 50, 0);
	GUI_DispStringAt("串口通信", 50, 32);
	GUI_DispStringAt("■", 27, 16*g_nSel);
}
void DrawScreen00(void){
	GUI_SetFont(&GUI_FontHZ_NewSimSun_16);
	GUI_SetColor(GUI_COLOR_BLACK);
	GUI_ClearRect(0, 0, 20, 64);
	GUI_DispStringAt("拓", 2,14);
	GUI_DispStringAt("展", 2, 34);
	GUI_SetColor(GUI_COLOR_WHITE);
	GUI_DispStringAt("2D模型", 50, 0);
	GUI_DispStringAt("飞机游戏", 50, 16);
	GUI_DispStringAt("■", 27, 16*g_nSel-64);
}

void DrawScreen1(void){
  char buf[50];
	GUI_SetFont(&GUI_FontHZ_NewSimSun_16);
	GUI_SetColor(GUI_COLOR_BLACK);
	GUI_ClearRect(0, 0, 20, 64);
	GUI_DispStringAt("系", 2,0);
	GUI_DispStringAt("统", 2,16);
	GUI_DispStringAt("测", 2,32);
	GUI_DispStringAt("试", 2,48);
	GUI_SetColor(GUI_COLOR_WHITE);
	GUI_SetFont(&GUI_FontHZ_SimSun_12);
	if((sysf1==0)&&(sysf2==0))	// LED、按键、AD
	{
		memset(buf, sizeof(buf), 0);
		sprintf(buf, "%s %s %s %s",
								(g_Ledsta & 0x01) ? "●" : "○", 
								(g_Ledsta & 0x02) ? "●" : "○", 
								(g_Ledsta & 0x04) ? "●" : "○", 
								(g_Ledsta & 0x08) ? "●" : "○");
		GUI_DispStringAt(buf, 40, 0);
		memset(buf, sizeof(buf), 0);
		sprintf(buf, "%s %s %s %s", 
								(g_Keypres & K1_Pin) ? "■" : "□",
								(g_Keypres & K2_Pin) ? "■" : "□", 
								(g_Keypres & K3_Pin) ? "■" : "□", 
								(g_Keypres & K4_Pin) ? "■" : "□");
		GUI_DispStringAt(buf, 40, 26);
		GUI_DispStringAt(" L1", 35, 13);
		GUI_DispStringAt(" L2", 52, 13);
		GUI_DispStringAt(" L3", 70, 13);
		GUI_DispStringAt(" L4", 87, 13);
		GUI_DispStringAt(" K1", 35, 39);
		GUI_DispStringAt(" K2", 52, 39);
		GUI_DispStringAt(" K3", 70, 39);
		GUI_DispStringAt(" K4", 87, 39);
		GUI_DispStringAt("AD:", 40, 52);
		//GUI_FillRect(30, 60, 30 + (g_adval % 1000) * 96 / 1000, 62);
		GUI_FillRect(62, 58, 62 + (int)((g_adval-1200)/3000.0*120), 60);
		sprintf(buf, "%d", g_adval);
		GUI_DispStringAt(buf, 100, 52);
	}
	else if(sysf1){
		if (!g_bMpuok)
			GUI_DispStringAt("MPU6050连接失败！", 22, 20);
		else {
			//六轴数据
			sprintf(buf, "%6d, %6d, %6d, %6d, %6d, %6d\n", ax, ay, az, gx, gy, gz);
			sprintf(buf, "ax:%6d gx:%3d",ax, gx);
			GUI_DispStringAt(buf, 25, 6);
			sprintf(buf, "ay:%6d gy:%3d",ay, gy);
			GUI_DispStringAt(buf, 25, 24);	
			sprintf(buf, "az:%6d gz:%3d",az, gz);
			GUI_DispStringAt(buf, 25, 42);	
		}
	}
	else{
		sprintf(buf, "ax ay az gx gy gz");
		GUI_DispStringAt(buf, 22,50);
		if(ax>=0)
			GUI_FillRect(28, 23-(int)(ax/20000.0*23),30,23);
		else
			GUI_FillRect(28, 23,30,23+(int)(-1.0*ax/20000.0*23));
		if(ay>=0)
			GUI_FillRect(28+3*6,23-(int)(ay/10000.0*23),30+3*6,23);
		else
			GUI_FillRect(28+3*6,23,30+3*6,23+(int)(-1.0*ay/10000.0*23));
		if(az>=0)
			GUI_FillRect(28+6*6,23-(int)(az/20000.0*23),30+6*6,23);
		else
			GUI_FillRect(28+6*6,23,30+6*6,23+(int)(-1.0*az/20000.0*23));
		if(gx>=0)
			GUI_FillRect(28+9*6,23-(int)(gx/500.0*23),30+9*6,23);
		else
			GUI_FillRect(28+9*6, 23,30+9*6,23+(int)(-1.0*gx/500.0*23));
		if(gy>=0){
			if(gy>1500)
				gy=1500;
			GUI_FillRect(28+12*6,23-(int)(gy/1500.0*23),30+12*6,23);
		}
		else{
			if(gy<-1500)
				gy=-1500;
			GUI_FillRect(28+12*6, 23,30+12*6,23+(int)(-1.0*gy/1500.0*23));
		}
		if(gz>=0){
			if(gz>500)
					gz=500;
			GUI_FillRect(28+15*6,23-(int)(gz/500.0*23),30+15*6,23);
		}
		else{
			if(gz<-500)
				gz=-500;
			GUI_FillRect(28+15*6, 23,30+15*6,23+(int)(-1.0*gz/500.0*23));
		}
	}
}

void DrawScreen2(void){
	if(mpu2df){	//2d运动
		if(fAZ>0)
			mpx=64-(165-(int)fAZ);
		else
			mpx=64+(180-(int)(-fAZ));
		if(fAY>150)
			mpy=32+(int)fAY-150;
		else
			mpy=32-(155-(int)fAY)*0.2;
		if(mpy<0)mpy=0;
		if(mpy>64)mpy=64;
		if(mpx>128)mpx=128;
		if(mpx<0)mpx=0;
		GUI_SetFont(&GUI_FontHZ_NewSimSun_8);
		GUI_DispStringAt("大", mpx,mpy);
	}
	else{	
		char buf[100];
		GUI_SetFont(&GUI_FontHZ_NewSimSun_16);
		GUI_SetColor(GUI_COLOR_BLACK);
		GUI_ClearRect(0, 0, 20, 64);
		GUI_DispStringAt("姿", 2,0);
		GUI_DispStringAt("态", 2,16);
		GUI_DispStringAt("解", 2,32);
		GUI_DispStringAt("算", 2,48);
		GUI_SetColor(GUI_COLOR_WHITE);
		GUI_SetFont(&GUI_FontHZ_SimSun_12);

		if (!g_bMpuok)
			GUI_DispStringAt("MPU6050连接失败！", 22, 20);
		else {
			//六轴数据
			if(mpuf){
				sprintf(buf, "%6d, %6d, %6d, %6d, %6d, %6d\n", ax, ay, az, gx, gy, gz);
				sprintf(buf, "ax:%6d gx:%3d",ax, gx);
				GUI_DispStringAt(buf, 25, 6);
				sprintf(buf, "ay:%6d gy:%3d",ay, gy);
				GUI_DispStringAt(buf, 25, 24);	
				sprintf(buf, "az:%6d gz:%3d",az, gz);
				GUI_DispStringAt(buf, 25, 42);
			}
			else{
			sprintf(buf, "俯仰角:%6.1f°", fAX);
			GUI_DispStringAt(buf, 30, 6);
			sprintf(buf, "横滚角:%6.1f°", fAY);
			GUI_DispStringAt(buf, 30, 24);
			sprintf(buf, "航向角:%6.1f°", fAZ);
			GUI_DispStringAt(buf, 30, 42);
			}
		}
	}
}

void DrawScreen3(void){
	GUI_SetFont(&GUI_FontHZ_NewSimSun_16);
	GUI_SetColor(GUI_COLOR_BLACK);
	GUI_ClearRect(0, 0, 20, 64);
	GUI_DispStringAt("串", 2,0);
	GUI_DispStringAt("口", 2,16);
	GUI_DispStringAt("通", 2,32);
	GUI_DispStringAt("信", 2,48);
	GUI_SetColor(GUI_COLOR_WHITE);
	sprintf(tmp,"cnt:%5d",summ);
	GUI_DispStringAt(tmp, 24,0);
	
	
	
	
	
	
	
	px=24;py=16;
	if(rxfk1){	//切换显示
		if(yj>=strlen(rx1_show))
			yj=0;
		s=rx1_show+yj;
		ty=yj;
		while(*s!='\0'&&(ty-yi)<15){
			if(px>=116){
				px=24;
				py+=16;
			}
			sprintf(c,"%0x",*s);
			GUI_DispStringAt(c, px,py);
			px+=20;s++;ty++;
		}
	}
	else{
		if(yi>=strlen(rx1_show))
			yi=0;
		s=rx1_show+yi;
		tx=yi;
		while(*s!='\0'&&(tx-yi)<39){
			if(px>=128){
				px=24;
				py+=16;
			}
			c[0]=*s;c[1]='\0';
			GUI_DispStringAt(c, px,py);
			px+=8;s++;tx++;
		}
	}
	
	
	
	
	// 显示发送时间间隔、数据格式、发送计数、接收计数
}


void UARTDataProc(void){
	if (!line_flag2 && rx2_buff[0] && HAL_GetTick() > uart2rxtick + 100)
	line_flag2 = 1;
  if (line_flag2) {    // 如果串口2接收到一行数据
    printf("Fron WIFI: %s", rx2_buff);       // 打印接收内容到串口1    
    memset(rx2_buff, 0, sizeof(rx2_buff));   // 清空串口2缓存区
    pBuf2 = rx2_buff;// 重新将串口2接收数据的存放指针指向接收缓存的头部
    (&huart2)->pRxBuffPtr = pBuf2;    // 重新将串口2结构体中的接收缓冲指针指向缓冲数组头部
    line_flag2 = 0;  // 串口2接收标志清零
  }
  if (rxit_ok2 != HAL_OK)		// 如果串口2接收中断还没有启动，尝试再次启动
    rxit_ok2 = HAL_UART_Receive_IT(&huart2, pBuf2, 1);
}


void DrawScreen4(void){
	fw=1;
	uint8_t ex=1;
	int t=0;
	memset(uart2displine,0,sizeof(uart2displine));
	
	while (ex){
		GUI_Clear();
		GUI_SetFont(&GUI_FontHZ_NewSimSun_16);
		GUI_SetColor(GUI_COLOR_BLACK);
		GUI_ClearRect(0, 0, 20, 64);
		GUI_DispStringAt("W", 6,0);
		GUI_DispStringAt("I", 6,16);
		GUI_DispStringAt("F", 6,32);
		GUI_DispStringAt("I", 6,48);
		GUI_SetColor(GUI_COLOR_WHITE);
		GUI_SetFont(&GUI_FontHZ_SimSun_12);
		int key=ScanKey();
		switch(key){
			case 1:;
				break;
			case 2:wifik2=1;break;
			case 3:ESP8266_StartOrShutServer(0,User_ESP8266_TcpServer_Port,"2");break;
			case 4:ex=0;break;
			default:break;
		}
		
		//显示
		if(Esp8266Err==1)
			GUI_DispStringAt("ESP8266 not find...", 24, 0);
		else{
			GUI_DispStringAt(Esp8266SSID, 24,0);
			if(Esp8266Err>0){
				char ts[10];
				sprintf(ts, "Err%d", Esp8266Err);
				GUI_DispStringAt(ts, 128 - 4 * 6, 0);
			}
			else{
				GUI_DispStringAt("OK",128-2*6, 0);
				GUI_DispStringAt("Link to ...", 24,14);
				sprintf(tmp_esp, "%d.%d.%d.%d  %s\n",
					User_ESP8266_TcpServer_IP[0], User_ESP8266_TcpServer_IP[1], 
					User_ESP8266_TcpServer_IP[2], User_ESP8266_TcpServer_IP[3], User_ESP8266_TcpServer_Port);
				GUI_DispStringAt(tmp_esp, 24,28);		
			}
		}
		
		
		if (wifik1 && Esp8266Err != 1)
    {
			wifif=1;wifik1=0;
			GUI_DispStringAt("Connecting...", 24, 42);
			GUI_Update();
			memset(uart2displine, 0, sizeof(uart2displine));
			// 重新连接服务器
			if (0 == Esp8266Err)
			{
				HAL_Delay(1000);
				printf("1111\n");
				Uart2SendStr("+++");
				printf("2222\n");
				HAL_Delay(500);
			}
			printf("3333\n");
			Esp8266Err = 4;
			if(ESP8266_Link_Server(enumTCP,User_ESP8266_TcpServer_IP,User_ESP8266_TcpServer_Port,5))
			{
				printf("4444\n");
				if (ESP8266_UnvarnishSend())
				{
					printf("5555\n");
					Esp8266Err = 0;
					memset(uart2displine, 0, sizeof(uart2displine));
				}
			}
			printf("6666\n");
			memset(uart2displine, 0, sizeof(uart2displine));
			wifif=0;ex=0;
			printf("7777\n");
     }
		
		
		
		
		GUI_DispStringAt("Data:", 24, 42);
		if(strstr(uart2displine,"ERROR"))
			memset(uart2displine,0,sizeof(uart2displine));
		GUI_DispStringAt(uart2displine, 60, 42);	
		GUI_Update();	// 刷新屏幕
		UARTDataProc();
		GUI_Delay(50);
	}
	fw=wifik1=wifik2=0;
	g_nScreen=0;
	memset(uart2displine,0,sizeof(uart2displine));
}
void DrawScreen5(void){
	
	GUI_SetFont(&GUI_FontHZ_NewSimSun_16);
	GUI_SetColor(GUI_COLOR_BLACK);
	GUI_ClearRect(0, 0, 20, 64);
	GUI_DispStringAt("6", 6,0);
	GUI_DispStringAt("6", 6,16);
	GUI_DispStringAt("6", 6,32);
	GUI_DispStringAt("6", 6,48);
	GUI_SetColor(GUI_COLOR_WHITE);



}
void DrawScreen6(void){
	//判断是否碰撞
	if(over==0){	//游戏未结束
		for(i=0;i<randi;i++){
			if(randx[i]>=0&&randy[i]>=50){
				if(abs(randx[i]-sssx)<=6){
					over=1;
					break;
				}
				else{
					if(randy[i]>=60){
						randx[i]=-1;	//消掉这个点
						randy[i]=-1;
						++flycnt;
					}
				}
			}
		}
	}
	
	//生成点
	if (HAL_GetTick() > clockt + speed){
		if(j==1&&(zantin==0)&&(over==0)){
			if(randi>=100)
				randi=0;
			randx[randi]=(int)(ax*1.7)%101;
			randy[randi]=0;
			randi++;
			j=0;
		}
		if((int)(ax*1.7)%101>92)
			j=1;
		clockt = HAL_GetTick();
		if(zantin==0){
			for(i=0;i<randi;i++)
				++randy[i];
		}
	}
	
	
	//显示图形界面
	sssx=fAY;
	if(sssx>170)
		sssx=170;
	else if(sssx<10)
		sssx=10;
	sssx= (int)((sssx-10)*120.0/160);
	GUI_SetFont(&GUI_FontHZ_NewSimSun_16);
	if(over){
		GUI_DispStringAt("OVER", 64-16,16);
		sprintf(tmp,"cnt: %2d",flycnt);
		GUI_DispStringAt(tmp, 64-16,32);
	}
	else{
		sprintf(tmp,"%2d",flycnt);
		GUI_DispStringAt(tmp,110,0);
		if(speed<=20)
			sprintf(tmp,"%2d",6+(20-speed)/5);
		else
			sprintf(tmp,"%2d",(140-speed)/20);
			
		GUI_DispStringAt(tmp,110,16);
		GUI_SetFont(&GUI_FontHZ_NewSimSun_8);
		for(i=0;i<randi;i++){
			if(randx[i]>=0)
				GUI_DispStringAt("*",  randx[i],randy[i]);
		}
		GUI_DispStringAt("大", sssx, 56);
	}
}

/* USER CODE END 4 */



/* StartDefaultTask function */
void StartDefaultTask(void const * argument){
 /* USER CODE BEGIN 5 */
	char buf[100];
	uint32_t oldtick = 0;
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_val, 20);

	g_bMpuok = MPU_init();
	if (!g_bMpuok)
		printf("MPU6050 init error!\n");
	
	/* Infinite loop */
	while(1){
		osDelay(10);
		if (line_flag){     // 如果串口1接收到一行数据
			printf("Data = %s", rx1_buff);        // 打印输出接收内容
			int t=atoi(rx1_buff);
			if(t>20&&t<1200)
				g_nSendTime=t;
			if(strlen(rx1_show)<MAX_RECV_LEN*2){
				int i=0,j=0;
				for(;rx1_buff[j]!='\0';j++){
					if(rx1_buff[j]!='\n'&&rx1_buff[j]!=' '&&rx1_buff[j]>32)
						rx1_show[i++]=rx1_buff[j];
				}
				summ+=i;
				rx1_show[i]='\0';
			}
			memset(rx1_buff, 0, sizeof(rx1_buff));   // 清空串口1缓存区
			pBuf = rx1_buff;// 重新将串口1接收数据的存放指针指向接收缓存的头部
			(&huart1)->pRxBuffPtr = pBuf;    // 重新将串口1结构体中的接收缓冲指针指向缓冲数组头部
			line_flag = 0;  // 串口1接收标志清零
		}
		if (rxit_ok != HAL_OK)		// 如果串口1接收中断还没有启动，尝试再次启动
			rxit_ok = HAL_UART_Receive_IT(&huart1, pBuf, 1);

		
		
		
		
		
		
		if (g_bMpuok) 	// 如果MPU6050初始化成功
			MPU_getdata();	// 读取传感器数据
		//g_adval * 3.3 / 4096
		if ((HAL_GetTick() >= oldtick + g_nSendTime) && g_bSend)
		{
			oldtick = HAL_GetTick();
			if (g_bMpuok)
			{				
				sprintf(buf, "7MYD%4d%6d%6d%6d%6d%6d%6d", g_adval, ax, ay, az, gx, gy, gz);
				AddJiaoYan(buf);	//添加校验和
				printf("%s", buf);

				sprintf(buf, "7MRD%4.2f%6.1f%6.1f%6.1f", g_adval * 3.3 / 4096, fAX, fAY, fAZ);
				AddJiaoYan(buf);	//添加校验和
				printf("%s", buf);

				
			}
			// 格式化状态帧
			if(syslsdf)
				g_xx=g_Ledsta;
			else
				g_xx=0x00;
			sprintf(buf, "7MBD%c%c%c%c%c%c%c%c", 
									(g_xx & 0x01) ? '1' : '0',
									(g_xx & 0x02) ? '1' : '0',
									(g_xx & 0x04) ? '1' : '0',
									(g_xx & 0x08) ? '1' : '0',
									(g_Keypres & K1_Pin) ? 'D' : 'U',
									(g_Keypres & K2_Pin) ? 'D' : 'U',
									(g_Keypres & K3_Pin) ? 'D' : 'U',
									(g_Keypres & K4_Pin) ? 'D' : 'U');

			AddJiaoYan(buf);
			printf("%s", buf);
		}
		if(wifik2){
			wifik2=0;
			if (0 == Esp8266Err){
				sprintf(buf_esp, "7MYD%4d%6d%6d%6d%6d%6d%6d\n", g_adval, ax, ay, az, gx, gy, gz);
				printf("%s", buf_esp);
				Uart2SendStr(buf_esp);		// 同时将字符串发送到串口2
				sprintf(buf_esp, "7MRD%4.2f%6.1f%6.1f%6.1f\n", g_adval * 3.3 / 4096, fAX, fAY, fAZ);
				printf("%s", buf_esp);
				Uart2SendStr(buf_esp);		// 同时将字符串发送到串口2
				if(syslsdf)
					g_xx=g_Ledsta;
				else
					g_xx=0x00;
				sprintf(buf_esp, "7MBD%c%c%c%c%c%c%c%c\n\n", 
										(g_xx & 0x01) ? '1' : '0',
										(g_xx & 0x02) ? '1' : '0',
										(g_xx & 0x04) ? '1' : '0',
										(g_xx & 0x08) ? '1' : '0',
										(g_Keypres & K1_Pin) ? 'D' : 'U',
										(g_Keypres & K2_Pin) ? 'D' : 'U',
										(g_Keypres & K3_Pin) ? 'D' : 'U',
										(g_Keypres & K4_Pin) ? 'D' : 'U');
				printf("%s", buf_esp);	
				Uart2SendStr(buf_esp);		// 同时将字符串发送到串口2
			}
		}
	}
  /* USER CODE END 5 */ 
}

/* StartKeyTask function */
void StartKeyTask(void const * argument){
  /* USER CODE BEGIN StartKeyTask */
	
	uint32_t oldtick = 0;
  /* Infinite loop */
  while(1){
		osDelay(5);
		if(wifif||initf||g_nScreen==4)		//等待进入系统后再开始按键侦听
			continue;		
    uint8_t key = ScanKey();
		switch (g_nScreen){
			default:
			case -1:	// 副菜单 [g_nSel对应值为4，5]
				switch (key){
					case 1:			//向下
						if(g_nSel<5)
							++g_nSel;
						break;
					case 2:			//向上
							-- g_nSel;
						if(g_nSel<=3)
							g_nScreen = 0;						
						break;
					case 3:			//确认
						g_nScreen = g_nSel + 1;	
						break;
					case 4:
						g_nScreen = -1;	//返回副界面	
						break;
				}
				break;
			case 0:	// 主菜单
				mpuf=1;syslsdf=0;mpu2df=0;//全部重置
				switch (key){
					case 1:			//向下
						++ g_nSel;
						if(g_nSel>=4)
							g_nScreen=-1;
						break;
					case 2:			//向上
						if(g_nSel>0)
							--g_nSel;
						break;
					case 3:			//确认
						g_nScreen = g_nSel + 1;	
						break;
					case 4:
						g_nScreen = 0;	//返回主界面	
						break;
				}
				break;
			case 1:	// 系统测试
				if(syslsdf){
					if (HAL_GetTick() > oldtick + 200){
						oldtick = HAL_GetTick();
						RunLsd();	// 流水灯
					}
				}
				switch (key)
				{
					default:	break;
					case 1:sysf1=!sysf1;sysf2=0;break;
					case 2:sysf1=0;sysf2=!sysf2;break;	//切换页面
					case 3:syslsdf=!syslsdf;break;	//流水灯开关
					case 4:
						g_nScreen = 0;
						SetLEDS(0x00);sysf1=sysf2=0;
						break;
				}
				break;
			case 2:	// 姿态解算
				switch (key){
					default:	break;
					case 1:mpuf=!mpuf;mpu2df=0;break;	//切换页面
					case 2:	// 启动/暂停发送
						g_bSend = !g_bSend;
						if(!g_bSend)
							printf("  STOP\n");
						mpu2df=0;
						break;
					case 3:mpu2df=!mpu2df;break;
					case 4:g_nScreen = 0;mpu2df=0;break;
				}
				break;
			case 3:	// 串口通信
				if(!usart1f){
					memset(rx1_show,0,sizeof(rx1_show));
					summ=0;
					usart1f=1;
				}
				switch (key){
					default:	break;
					case 1:	rxfk1=!rxfk1;break;// 切换十六进制
					case 2:	yi+=39;yj+=15;break;			//翻页
					case 3:	memset(rx1_show,0,sizeof(rx1_show));summ=0;break;
					case 4:
						g_nScreen = 0;usart1f=0;;
						rxfk1=rxfk2=0;yi=yj=0;
						break;
				}
				break;
			case 4:	// WIFI通信
				break;
			case 5:	// 2d模型
				switch (key){
					default:	break;
					case 1:		
					case 2:
					case 3:break;
					case 4:
						g_nScreen = -1;
						break;
				}
				break;
			case 6:	// 飞机游戏
				switch (key){
					default:	break;
					case 1:if(speed<=20){
										speed-=5;
										if(speed<=5)
											speed=120;
									}
									else
										speed-=20;
									break;	
					case 2:zantin=!zantin;break;
					case 3:over=0;flycnt=0;speed=120;randi=0;zantin=0;break;
					case 4:
						g_nScreen = -1;over=0;zantin=0;speed=120;clockt=0;flycnt=0;
						randi=0;
						break;
				}
				break;				
				
				
				
				
				
				
		}
  }
  /* USER CODE END StartKeyTask */
}

/* StartDispTask function */
void StartDispTask(void const * argument){
  /* USER CODE BEGIN StartDispTask */
	LED_GPIO_Init();
	//开机初始状态

	
	//LED闪烁
	for(int i=0;i<5;i++){
		SetLEDS(0x0f);
		osDelay(160);
		SetLEDS(0x00);
		osDelay(160);
	}
	SetLEDS(0x00);
	//显示课程姓名学号
	GUI_Init();
	GUI_Clear();
	GUI_SetFont(&GUI_FontHZ_NewSimSun_16);
	GUI_DispStringAt("电子系统设计", 16, 0);
	GUI_SetFont(&GUI_FontHZ_KaiTi_20);
	GUI_DispStringAt("滕佳禄", 34, 20);		//128-48=80
	GUI_DispStringAt("16073212", 24, 40);	//128-64=64
	GUI_Update();
	osDelay(2000);		//延时2秒
	GUI_Clear();
  GUI_DrawBitmap(&bmbptjl, 22, 0);
  GUI_Update();
	while(!ScanKey());		//等待按键按下
	initf=0;
	
  
  /* Infinite loop */
  for(;;)
  {
		osDelay(5);
		if(fw||wifif)
			continue;
    GUI_Clear();	// OLED清屏，准备绘图
		switch (g_nScreen)
		{
			case -1://副界面
				DrawScreen00();break;
			case 0:	// 主界面
				DrawScreen0();break;
			case 1:	// 测试界面
				DrawScreen1();break;
			case 2:	// MPU6050界面
				DrawScreen2();break;
			case 3:	// 串口通信
				DrawScreen3();break;
			case 4:	// WIFI通信
				DrawScreen4();break;
			case 5:	//MPU6050-2D
				DrawScreen5();break;
			case 6:	//飞机游戏
				DrawScreen6();break;
		}
    GUI_Update();	// 刷新屏幕

  }
  /* USER CODE END StartDispTask */
}




//
//
//




/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f) {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff); // 向串口1发送一个字符
    return 0;
}

void SetLEDS(uint8_t sta){
	if (g_bUsePWM)
	{
		LED_GPIO_Init();
		g_bUsePWM = 0;
	}
	
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, (sta & 0x01) ? GPIO_PIN_RESET : GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, (sta & 0x02) ? GPIO_PIN_RESET : GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, (sta & 0x04) ? GPIO_PIN_RESET : GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, (sta & 0x08) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void RunLsd(void){
	g_Ledsta <<= 1;
	if (g_Ledsta > 0x08)
		g_Ledsta = 0x01;
  SetLEDS(g_Ledsta);
}

void SetPWMLight(uint8_t light){
	if (!g_bUsePWM)
	{
		MX_TIM3_Init();
		g_bUsePWM = 1;
	}

  htim3.Instance->CCR1 = light;
  htim3.Instance->CCR2 = light;
  htim3.Instance->CCR3 = light;
  htim3.Instance->CCR4 = light;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

uint8_t ScanOneKey(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
  uint8_t bact = 0;
  if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET) 
  {
    if (!(g_Keypres & GPIO_Pin)) 
    {
      osDelay(50);
      if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET) 
      {
        g_Keypres |= GPIO_Pin;
//        bact = 1;		// 按键按下认为是有效动作
      }
    }
  }
  else  
  {
    if (g_Keypres & GPIO_Pin) 
    {
      osDelay(50);
      if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET)
      {
        g_Keypres &= ~GPIO_Pin;
				bact = 1;		// 按键放开认为是有效动作
      }
    }
  }
  return bact;
}

uint8_t ScanKey(void) {
  if (ScanOneKey(K1_GPIO_Port, K1_Pin))
    return 1;
  if (ScanOneKey(K2_GPIO_Port, K2_Pin))
    return 2;
  if (ScanOneKey(K3_GPIO_Port, K3_Pin))
    return 3;
  if (ScanOneKey(K4_GPIO_Port, K4_Pin))
    return 4;
  return 0;
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
  int i;
  uint16_t sum = 0;
  for (i = 0; i < 20; ++i)
    sum += g_val[i];
  g_adval = sum / 20;
}

void Uart2SendStr(char *str){
   HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);
}

//接收中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
  uint8_t cnt = 0; // 临时变量，用于重复计数
  if (UartHandle->Instance == USART1)
  {
    ++ pBuf;  // 已接收一个字节数据，当前存储位置指针后移
    if(pBuf == rx1_buff + MAX_RECV_LEN)  // 如果指针已移出数组边界
        pBuf = rx1_buff;    // 重新指向数组开头
    else if(*(pBuf - 1) == '\n')  // 如果之前接收到‘\n’换行符，则表示接收完成
        line_flag  = 1;
    uart1rxtick = HAL_GetTick();
    // 重新开启接收中断
    do {
        rxit_ok = HAL_UART_Receive_IT(UartHandle, pBuf, 1);
        if (++cnt >= 5)
            break;
    } while(rxit_ok != HAL_OK);
  }
  else if(UartHandle->Instance == USART2)
  {
    esp8266_rxdata = *pBuf2;
    ++ pBuf2;  // 已接收一个字节数据，当前存储位置指针后移
    if(pBuf2 == rx2_buff + MAX_RECV_LEN)  // 如果指针已移出数组边界
        pBuf2 = rx2_buff;    // 重新指向数组开头
    else if(*(pBuf2 - 1) == '\n')  // 如果之前接收到‘\n’换行符，则表示接收完成
    {
//    printf("%s", rx2_buff);       // 打印接收内容到串口1    
      line_flag2  = 1;
      memcpy(uart2displine, rx2_buff, 40);
    }
    uart2rxtick = HAL_GetTick();
    if(strEsp8266_Fram_Record.InfBit.FramLength<(RX_BUF_MAX_LEN-1))                       //预留1个字节写结束符
      strEsp8266_Fram_Record.Data_RX_BUF[strEsp8266_Fram_Record.InfBit.FramLength++] = esp8266_rxdata;
    // 重新开启接收中断
    do {
        rxit_ok2 = HAL_UART_Receive_IT(UartHandle, pBuf2, 1);
        if (++cnt >= 5)
            break;
    } while(rxit_ok2 != HAL_OK);
  }
}

static void LED_GPIO_Init(void){
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin, GPIO_PIN_SET);
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED3_Pin|LED4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED3_Pin LED4_Pin EN_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED4_Pin|EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

uint8_t InitESP8266(void)
{
  int ret = 1;
	ESP8266_Usart("+++");	
	HAL_Delay(500);
  if(ESP8266_AT_Test())
  {
    printf("AT test OK\n");
  }
  else
    return ret;
  
  ret = 2;
  printf("<1> Set 8266 net mode...\n");
	if(ESP8266_Net_Mode_Choose(STA_AP))
  {
    printf("ESP8266_Net_Mode_Choose OK\n");
  }
  
  if (ESP8266_Inquire_ApMac(Esp8266IPAddress, 30))
  {
    sprintf(Esp8266SSID, "ESP8266_%s", Esp8266IPAddress + 8);
    ESP8266_BuildAP(Esp8266SSID, Esp8266PWD, WPA2_PSK);
  }
//  ret = 2;
//  printf("<2> Jion Wifi AP:%s, use password:%s\n", User_ESP8266_ApSsid, User_ESP8266_ApPwd);
  uint8_t cnt = 0;
//  while(!ESP8266_JoinAP(User_ESP8266_ApSsid, User_ESP8266_ApPwd))
//    if (++cnt > 1)
//      return ret;
  
  ret = 3;
	printf("<3> Set single link\n");
  ESP8266_Enable_MultipleId(DISABLE);
  
  if (ESP8266_Inquire_ApIp(Esp8266IPAddress, 30))
    printf("ESP8266 IP Address:%s\n", Esp8266IPAddress);
    
  ret = 4;
	printf("<4> Link to tcp server %d.%d.%d.%d.:%s\n", 
        User_ESP8266_TcpServer_IP[0], User_ESP8266_TcpServer_IP[1], 
        User_ESP8266_TcpServer_IP[2], User_ESP8266_TcpServer_IP[3], User_ESP8266_TcpServer_Port);
  cnt = 0;
	while(!ESP8266_Link_Server(enumTCP,User_ESP8266_TcpServer_IP,User_ESP8266_TcpServer_Port,Single_ID_0))
    if (++cnt > 1)
      return ret;
  
  ret = 5;
	printf("<5> Begin unvarnishSend...\n");
  cnt = 0;
  while(!ESP8266_UnvarnishSend())
    if (++cnt > 10)
      return ret;
	printf("Config ESP8266 OK...\n");
  return 0;
}

/* USER CODE END 0 */


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}


void _Error_Handler(char *file, int line){
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line){ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



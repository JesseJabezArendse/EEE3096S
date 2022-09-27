/* USER CODE BEGIN Header */
/**
*************************************
Info:		STM32 I2C with DS3231 HAL
Author:		Amaan Vally
*************************************
In this practical you will learn to use I2C on the STM32 using the HAL. Here, we will
be interfacing with a DS3231 RTC. We also create functions to convert the data between Binary
Coded Decimal (BCD) and decimal.

Code is also provided to send data from the STM32 to other devices using UART protocol
by using HAL. You will need Putty or a Python script to read from the serial port on your PC.

UART Connections are as follows: red->5V black->GND white(TX)->PA2 green(RX;unused)->PA3.
Open device manager and go to Ports. Plug in the USB connector with the STM powered on. Check the port number (COMx).
Open up Putty and create a new Serial session on that COMx with baud rate of 9600.

https://www.youtube.com/watch?v=EEsI9MxndbU&list=PLfIJKC1ud8ghc4eFhI84z_3p3Ap2MCMV-&index=4

RTC Connections: (+)->5V (-)->GND D->PB7 (I2C1_SDA) C->PB6 (I2C1_SCL)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {//creates a time structure
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hour;
	uint8_t dayofweek;
	uint8_t dayofmonth;
	uint8_t month;
	uint8_t year;
} TIME;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//TO DO:
//TASK 2
//Give DELAY1 and DELAY2 sensible values
#define DELAY1 376 //1 second
#define DELAY2 7996 //60 seconds

//TO DO:
//TASK 4

//Define the RTC slave address
#define DS3231_ADDRESS 0b1101000

#define FIRST_REG 0x0
#define REG_SIZE 7

#define EPOCH_2022 1640988000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
char buffer[14]; //creates the buffer array
uint8_t data [] = "Hello from STM32!\r\n"; //creates the data array and stores Hello from STM32!\r\n
TIME time;// declares the time variable using the TIME structure
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void HAL_UART_TxCpltCllback(UART_HandleTypeDef *huart);
void pause_sec(float x);


uint8_t decToBcd(int val); //declares a function to convert decimal to BCD
int bcdToDec(uint8_t val);//declares a function to convert BCD to decimal
void setTime (uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month, uint8_t year);//declares a function to set the time
void getTime (void);//declares a function to return the time
int epochFromTime(TIME time); //declares a function to convert time to epoch time

void test();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){

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
  MX_I2C1_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */


  //TO DO
  //TASK 6
  //YOUR CODE HERE
//  set time
  setTime(0, 0, 0b00000000, 0 , 0, 0 ,0); //sets time to 0
  sprintf(buffer, "%d \r\n", 55555555555555); //stores a temporary value in buffer
  debugPrintln(&huart2, "Hello, this is STMF0 Discovery board: "); //Code to display through putty to show the connection is working

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	//TO DO:
	//TASK 1
	//First run this with nothing else in the loop and scope pin PC8 on an oscilloscope
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);

	//TO DO:
	//TASK 6


//	This creates a string "55555555555555" with a pointer called buffer

	//Transmit data via UART
	//Blocking! fine for small buffers




	//YOUR CODE HERE
//	read from RTC
	getTime(); //gets the time
	uint currentTime = epochFromTime(time); //sets the current time as the epoch time
	sprintf(buffer, "%d \r", currentTime);//formats the epoch time value and stores it in buffer
	  debugPrintln(&huart2, buffer); // displays buffer


	int ss = bcdToDec(time.seconds); //sets the ss value to the number of seconds by converting the time value into decimals and calling seconds
	int mm = bcdToDec(time.minutes);//sets the mm value to the number of seconds by converting the time value into decimals and calling minutes
	int hh = bcdToDec(time.hour); //sets the hh value to the number of seconds by converting the time value into decimals and calling hours
	//int dow = bcdToDec(time.dayofweek);
	//int dom = bcdToDec(time.dayofmonth);
	//int m = bcdToDec(time.month);
	//int y = bcdToDec(time.year);

	if(ss<10)//sets the buffer value to the correctly formatted time
		{
			if(mm<10)
			{
				if(hh<10)
				{
					sprintf(buffer, "0%i:0%i:0%i",hh,mm,ss);
				}
				else
				{
					sprintf(buffer, "%i:0%i:0%i",hh,mm,ss);
				}
			}
			else
			{
				if(hh<10)
				{
					sprintf(buffer, "0%i:%i:0%i",hh,mm,ss);
				}
				else
				{
					sprintf(buffer, "%i:%i:0%i",hh,mm,ss);
				}
			}
	  }else{
		  if(mm<10)
		  		{
		  			if(hh<10)
		  			{
		  				sprintf(buffer, "0%i:0%i:%i",hh,mm,ss);
		  			}
		  			else
		  			{
		  				sprintf(buffer, "%i:0%i:%i",hh,mm,ss);
		  			}
		  		}
		  		else
		  		{
		  			if(hh<10)
		  			{
		  				sprintf(buffer, "0%i:%i:%i",hh,mm,ss);
		  			}
		  			else
		  			{
		  				sprintf(buffer, "%i:%i:%i",hh,mm,ss);
		  			}
		  		}
		}



	//sprintf(buffer, "%2i:%2i:%2i",hh,mm,ss);
	debugPrintln(&huart2, buffer);// displays the buffer


	pause_sec(1); //add a 1s delay

	test(); // test the conversions



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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void debugPrintln(UART_HandleTypeDef *uart_handle, char _out[]) {
	HAL_UART_Transmit(uart_handle, (uint8_t*) _out, strlen(_out), 60);
	char newline[2] = "\r\n";
	HAL_UART_Transmit(uart_handle, (uint8_t*) newline, 2, 10);
}

void pause_sec(float x)
{
	/* Delay program execution for x seconds */
	//TO DO:
	//TASK 2
	//Make sure you've defined DELAY1 and DELAY2 in the private define section
	//YOUR CODE HERE

		for(int i =0; i<x;i++){ //sets the delay time to the x value
		for(int j =0; j<DELAY1;j++){ //loops with j and k set a 1s delay
			for(int k =0; k<DELAY2;k++){
			};
		};
	};

}


uint8_t decToBcd(int val)
{
    /* Convert normal decimal numbers to binary coded decimal*/
	//TO DO:
	//TASK 3
	//YOUR CODE HERE

	return (  (val/10*16) + (val%10)  );// converts decimal to BCD
}

int bcdToDec(uint8_t val)
{
    /* Convert binary coded decimal to normal decimal numbers */
	//TO DO:
	//TASK 3
	//Complete the BCD to decimal function
	//YOUR CODE HERE

	return (  (val/16*10) + (val%16)  );// converts BCD to decimal
}

void setTime (uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month, uint8_t year)
{
    /* Write the time to the RTC using I2C */
	//TO DO:
	//TASK 4

	uint8_t set_time[7]; // creates a set time array

	//YOUR CODE HERE

	set_time[0] = sec;//sets seconds as the first item in the set time array
	set_time[1] = min;//sets minutes as the second item in the set time array
	set_time[2] = hour;//sets hours as the third item in the set time array
	set_time[3] = dow;//sets day of the week as the fourth item in the set time array
	set_time[4] = dom;//sets day of the month as the fifth item in the set time array
	set_time[5] = month;//sets months as the sixth item in the set time array
	set_time[6] = year;//sets years as the seventh item in the set time array

	//sets the value of time to the values in the set time array
	time.seconds    = set_time[0];
	time.minutes    = set_time[1];
	time.hour       = set_time[2];
	time.dayofweek  = set_time[3];
	time.dayofmonth = set_time[4];
	time.month      = set_time[5];
	time.year       = set_time[6];


	//fill in the address of the RTC, the address of the first register to write anmd the size of each register
	//The function and RTC supports multiwrite. That means we can give the function a buffer and first address
	//and it will write 1 byte of data, increment the register address, write another byte and so on

	HAL_I2C_Mem_Write(&hi2c1, 0b11010000, FIRST_REG, I2C_MEMADD_SIZE_8BIT, set_time, 7, 1000);
}

void getTime (void)
{
    /* Get the time from the RTC using I2C */
	//TO DO:
	//TASK 4
	//Update the global TIME time structure

	uint8_t get_time[7];

	//fill in the address of the RTC, the address of the first register to write anmd the size of each register
	//The function and RTC supports multiread. That means we can give the function a buffer and first address
	//and it will read 1 byte of data, increment the register address, write another byte and so on
	HAL_I2C_Mem_Read(&hi2c1, 0b11010001, FIRST_REG, I2C_MEMADD_SIZE_8BIT, get_time, 7, 1000);
//                   p

	//returns the time value
	time.seconds    = get_time[0];
	time.minutes    = get_time[1];
	time.hour       = get_time[2];
	time.dayofweek  = get_time[3];
	time.dayofmonth = get_time[4];
	time.month      = get_time[5];
	time.year       = get_time[6];


	//YOUR CODE HERE



}

int epochFromTime(TIME time){
    /* Convert time to UNIX epoch time */
	//TO DO:
	//TASK 5
	//You have been given the epoch time for Saturday, January 1, 2022 12:00:00 AM GMT+02:00
	//It is define above as EPOCH_2022. You can work from that and ignore the effects of leap years/seconds

	//YOUR CODE HERE




	/*
	 *COMPLETE THE SWITCH CASE OR INSERT YOUR OWN LOGIC
	 */




	//converts each part of the time structure to decimals and store them in their respective variables
	int seconds    = bcdToDec(time.seconds);
	int minutes    = bcdToDec(time.minutes);
	int hour       = bcdToDec(time.hour);
	int dayofweek  = bcdToDec(time.dayofweek);
	int dayofmonth = bcdToDec(time.dayofmonth);
	int month      = bcdToDec(time.month);
	int year       = bcdToDec(time.year);

//	sprintf(buffer, "%d \r\n", seconds);
//	debugPrintln(&huart2, buffer);
//	sprintf(buffer, "%d \r\n", minutes);
//	debugPrintln(&huart2, buffer);
//	sprintf(buffer, "%d \r\n", hour);
//	debugPrintln(&huart2, buffer);
//	sprintf(buffer, "%d \r\n", dayofweek);
//	debugPrintln(&huart2, buffer);
//	sprintf(buffer, "%d \r\n", dayofmonth);
//	debugPrintln(&huart2, buffer);
//	sprintf(buffer, "%d \r\n", month);
//	debugPrintln(&huart2, buffer);
//	sprintf(buffer, "%d \r\n", year);
//	debugPrintln(&huart2, buffer);


	int day = dayofmonth;


	switch(month){//sets the number of days depending on the month
	case 1:
		day = day;
	case 2:
		day = day + 31;
	case 3:
		day = day + 31 + 28;
	case 4:
		day = day + 31 + 28 + 31;
	case 5:
		day = day + 31 + 28 + 31 + 30;
	case 6:
		day = day + 31 + 28 + 31 + 30 + 31;
	case 7:
		day = day + 31 + 28 + 31 + 30 + 31 + 30;
	case 8:
		day = day + 31 + 28 + 31 + 30 + 31 + 30 + 31;
	case 9:
		day = day + 31 + 28 + 31 + 30 + 31 + 30 + 31 + 31;
	case 10:
		day = day + 31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30;
	case 11:
		day = day + 31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31;
	case 12:
		day = day + 31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31 + 30;
	}


	int current = 0;//variable current is set as the epoch time since 2022
	current = year * 365;
	current = current + day;
	current = current * 24;
	current = current + hour;
	current = current*60;
	current = current + minutes;
	current = current * 60;
	current = current + seconds;



	return EPOCH_2022 + current; // the epoch time is returned
}

void test(){ //test to show that decToBcd and bcdToDec works
	int dec = 51; //0b00110011
	uint8_t convertedDec = decToBcd(dec); //0b 0101 0001 = 81
		sprintf(buffer, "%d \r\n", convertedDec);
		debugPrintln(&huart2, buffer);


	uint8_t bcd = 81;
	uint8_t convertedBcd = bcdToDec(bcd);
			sprintf(buffer, "%d \r\n", convertedBcd);
			debugPrintln(&huart2, buffer);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

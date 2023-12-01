/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// my lib
#include "my_gpio_lib_v3_2.h"

#define My_Uart_LIB_EN		2
#define Console_Ch2			2
#define USART_Ch_2_EN		2
#define DMA_EN_Uart_2		2
#define USART_Ch_3_EN		3
#define IRQ_EN_Uart_3		5
#include "my_uart_lib_v4_4.h"

#include "color_sensor.h"
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
// ultrasonic wave sensor distance
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint32_t Difference = 0;
uint8_t sts_flag = 0;  // is the first value captured ?
uint8_t Distance  = 0, cm = 0, inch=0;
float mm = 0.0;
uint8_t wave_loop = 0;

// color sensor variable
color_value col;
char cur_color= 'x'; //color sensing nothing == default;
char color;

// raspi buffer
uint8_t RasPi_Buf[20];  // uart 3 rx_buf
uint16_t uart_loop = 0;

uint8_t Rotate_flag = 0;
uint16_t Rotate_Count = 0;

// state
uint8_t RC_Step = 1;
uint8_t LineTrace_Mode = 3; // Mode : 0(Sensor), 1(Cam), 2(Color), 3(wait)
uint8_t Old_LineTrace_Mode = 3; // Mode : 0(Sensor), 1(Cam), 2(Color), 3(wait)
uint8_t Mode_Chage_flag = 0;
uint16_t Mode_Chage_Wait = 0;

uint8_t Uart_ena_flag = 1;
uint8_t LineTrace_Run_flag = 0;

// infrared sensor flag
uint8_t Infrared_Check_flag = 1;
uint8_t LineLimit_Enable_flag = 0;
uint8_t L_LineLimit_Detect_flag = 0;
uint8_t R_LineLimit_Detect_flag = 0;

// ultrasonic wave flag
uint8_t SonicWave_Enable_flag = 1;
uint8_t SonicWave_Detect_flag = 0;


uint8_t Motor_Old_State = 0;
uint8_t Motor_Cur_State = 0;
uint8_t Motor_Spd_State = 0;

#define Motor_Move_FWD 	 1
#define Motor_Move_STOP  2
#define Motor_Move_BWD   3
#define Motor_Move_LEFT  4
#define Motor_Move_RIGHT 5

#define Motor_SPD_HIGH 	 6
#define Motor_SPD_LOW  	 7
#define Motor_SPD_ROTATE 8

#define SPD_HIGH 		2300
#define SPD_NORMAL 		1300
#define SPD_ROTATE 		3000
#define SPD_NORMAL_2	700

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_us(uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER (&htim1) < time);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034/2; // cm
			cm = Difference / 58;  // cm
			inch = Difference / 148; // inch
			mm = cm * 10.0; //mm

			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC4);
		}

		SonicWave_Enable_flag = 1;

	}
}

void Chk_Infrared(){
	uint16_t buf;
	buf = PortC_rd_L() & 0x00f0;
	buf >>= 4;

	if (buf == 0){	// fwd
		LineTrace_Run_flag = 1;
		LineLimit_Enable_flag = 0;
	}
	else if (buf == 9 || buf == 1 || buf == 3  || buf == 8 || buf == 12){	// fwd + limit enable
		LineTrace_Run_flag = 1;
		LineLimit_Enable_flag = 1;
	}
	else if (buf == 7 || buf == 14){ // fwd + limit enable
		LineTrace_Run_flag = 1;
		LineLimit_Enable_flag = 1;
	}
	else if (buf == 15){	// stop
		LineTrace_Run_flag = 0;
		LineLimit_Enable_flag = 0;
	}
	else {	// stop
		LineTrace_Run_flag = 0;
		LineLimit_Enable_flag = 0;
	}
}

void Chk_Infrared_Limit(){
	uint16_t buf;
	buf = PortC_rd_H() & 0x0300;
	buf >>= 8;
	if (buf == 1) {
		L_LineLimit_Detect_flag = 1;
		R_LineLimit_Detect_flag = 0;
	}
	else if(buf == 2) {
		L_LineLimit_Detect_flag = 0;
		R_LineLimit_Detect_flag = 1;
	}
	else {
		L_LineLimit_Detect_flag = 0;
		R_LineLimit_Detect_flag = 0;
	}
}

void  chk_CamLineTrace(){
	if (RasPi_Buf[0] != 'G' && RasPi_Buf[0] != 'L' && RasPi_Buf[0] != 'R' && RasPi_Buf[0] != '0'){
		return;
	}
	if(RasPi_Buf[0] == 'G'){
		// fwd
		LineTrace_Run_flag = 1;
		Motor_Cur_State = Motor_Move_FWD;
		L_LineLimit_Detect_flag = 0;
		R_LineLimit_Detect_flag = 0;
	}
	else if(RasPi_Buf[0] == 'L'){
		// left
		LineTrace_Run_flag = 1;

		L_LineLimit_Detect_flag = 1;
		R_LineLimit_Detect_flag = 0;
	}
	else if(RasPi_Buf[0] == 'R'){
		// right
		LineTrace_Run_flag = 1;

		L_LineLimit_Detect_flag = 0;
		R_LineLimit_Detect_flag = 1;
	}
	else if(RasPi_Buf[0] == '0'){
			// right
		if(LineTrace_Mode == 2){
			Motor_Cur_State = Motor_Move_STOP;
			L_LineLimit_Detect_flag = 0;
			R_LineLimit_Detect_flag = 0;
		}
		else{
			L_LineLimit_Detect_flag = 0;
			R_LineLimit_Detect_flag = 0;
		}

	}
	if (LineTrace_Mode != 2){
		my_printf(Uart_Ch_3, RasPi_Buf);
	}
	Uart_ena_flag = 0;
}

void Run_Mode_chk(){
	if (RasPi_Buf[1] == '0'){
		LineTrace_Mode = 0;
	}
	else if(RasPi_Buf[1] == '1'){
		LineTrace_Mode = 1;
	}
	else if(RasPi_Buf[1] == '2'){
		LineTrace_Mode = 2;
	}
	else if(RasPi_Buf[1] == '3'){
		LineTrace_Mode = 3;
	}
	else{
		return;
	}
	if (Old_LineTrace_Mode != LineTrace_Mode){
		Old_LineTrace_Mode = LineTrace_Mode;
		Mode_Chage_flag = 1;
		Mode_Chage_Wait = 0;
	}
}

void Motor1_CW(){
	Out_Bit(GPIOA, M1_CW_Pin, 1);
	Out_Bit(GPIOA, M1_CCW_Pin, 0);
}
void Motor1_CCW(){
	Out_Bit(GPIOA, M1_CW_Pin, 0);
	Out_Bit(GPIOA, M1_CCW_Pin, 1);
}
void Motor2_CW(){
	Out_Bit(GPIOA, M2_CW_Pin, 1);
	Out_Bit(GPIOB, M2_CCW_Pin, 0);
}
void Motor2_CCW(){
	Out_Bit(GPIOA, M2_CW_Pin, 0);
	Out_Bit(GPIOB, M2_CCW_Pin, 1);
}
void Motor1_Stop(){
	Out_Bit(GPIOA, M1_CW_Pin, 0);
	Out_Bit(GPIOA, M1_CCW_Pin, 0);

}
void Motor2_Stop(){
	Out_Bit(GPIOA, M2_CW_Pin, 0);
	Out_Bit(GPIOB, M2_CCW_Pin, 0);
}


// function
void Moter_Control(uint8_t Direction){
	// motor move >> cw ccw fwd bwd stop
	if (Direction == 1){ // FWD
		Motor1_CCW();
		Motor2_CCW();
	}
	else if (Direction == 2) { // STOP
		Motor1_Stop();
		Motor2_Stop();
	}
	else if (Direction == 3){ // BWD
		Motor1_CW();
		Motor2_CW();
	}
	else if (Direction == 4){ // LEFT

		Motor1_CW();
		Motor2_CCW();

	}
	else if (Direction == 5){ // RIGHT
		Motor1_CCW();
		Motor2_CW();
	}
	else { // STOP
		Motor1_Stop();
		Motor2_Stop();
	}
}

void HCSR04_Read (void)
{
	HAL_GPIO_WritePin(SONIC_TRIG_GPIO_Port, SONIC_TRIG_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay_us(10);  // wait for 10 us
	HAL_GPIO_WritePin(SONIC_TRIG_GPIO_Port, SONIC_TRIG_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);
}

void SonicWave_Read(){
	if (wave_loop > 200){
		if(SonicWave_Enable_flag == 0){
			HCSR04_Read();
			wave_loop = 0;
		}
	}
	else{
		wave_loop++;
	}
}

uint8_t Distance_Chk(){
	if (mm < 150){
		SonicWave_Detect_flag = 1;
	}
	else{
		SonicWave_Detect_flag = 0;
	}
	SonicWave_Enable_flag = 0;


	return SonicWave_Detect_flag;
}

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
  MX_DMA_Init();
  MX_I2C1_Init();

  MX_USART2_UART_Init();
  MX_USART3_UART_Init();

  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
#if My_Uart_LIB_EN
   memset(Rx_data_1, 0, 20 *sizeof(uint8_t));
   memset(Rx_data_2, 0, sizeof(Rx_data_2));
   memset(Rx_data_3, 0, 20 * sizeof(unsigned char));

   HAL_UART_Receive_DMA(&huart2, (uint8_t *)Rx_data_2, 7);
   HAL_UART_Receive_IT(&huart3,   (uint8_t *)Rx_data_3, 2);
#endif
   if(TCS34725_Init() != 0){
      	  printf("TCS34725 initialization error!!\r\n");
        	  return 0;
    }
    else{
  	  printf("TCS34725 initialization success!!\r\n");
    }
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2, SPD_NORMAL);
  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1, SPD_NORMAL);

  // color sensor init

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (Rotate_flag == 1){

		  if (Rotate_Count > 3000){
			  Rotate_flag = 0;
			  Rotate_Count = 0;
		  }
		  else{
			  Rotate_Count++;
			  continue;
		  }
	  }

	  if(uart_loop > 20){
//	      printf("R:%d G:%d B:%d, %c\r\n", col.r, col.g, col.b, color);
//	      printf("%d\r\n", RC_Step);
//	      printf("Motor State :%d, Motor Spd:%d\r\n", Motor_Old_State, Motor_Spd_State);
//	  	  printf("dist :%d mm\r\n", mm);
		  printf("Mode :%d\r\n", LineTrace_Mode);
		  if(rx_end_flag_3 == 1)
		  {
			  strcpy(RasPi_Buf, Rx_data_3);
			  rx_end_flag_3 = 0;
			  Run_Mode_chk();	// Mode Check
			  Uart_ena_flag = 1;
		  }
		  uart_loop = 0;
	  }
	  else{
		  uart_loop++;
	  }

	  if (Mode_Chage_flag == 1){

		if (Mode_Chage_Wait > 5000){
			Mode_Chage_flag = 0;
			Mode_Chage_Wait = 0;
		}
		else{
			Moter_Control(Motor_Move_STOP);
			Mode_Chage_Wait++;
			printf("%d\r\n", Mode_Chage_Wait);
			continue;
		}
	  }
	  if (LineTrace_Mode == 3){
	 	  		  Moter_Control(Motor_Move_STOP);
	 	  		  continue;
	 	  }

	  switch(RC_Step){

		  case 1:
			  if (LineTrace_Mode == 1 || LineTrace_Mode == 2){
				  if (Uart_ena_flag)
				  chk_CamLineTrace();
			  }
			  else{
				  // Line check
				  if(Infrared_Check_flag){
					  Chk_Infrared();
				  }
				  if (LineTrace_Run_flag){
					  RC_Step = 10; // Line o
				  }
				  else{
					  // color sensor check
					  LineLimit_Enable_flag = 1;
					  RC_Step = 10; // Line x or color line
				  }
			  }
			  RC_Step = 10;
			  break;

		  case 10:
			  if (LineTrace_Mode == 1){

				  Chk_Infrared_Limit();
				  if (L_LineLimit_Detect_flag || R_LineLimit_Detect_flag){
					  if (L_LineLimit_Detect_flag) Motor_Cur_State = Motor_Move_LEFT;
					  else Motor_Cur_State = Motor_Move_RIGHT;
					  RC_Step = 100; // line out
				  }
				  else{
					  RC_Step = 20; // line in
				  }
			  }
			  else if (LineTrace_Mode == 2){
				  if (L_LineLimit_Detect_flag || R_LineLimit_Detect_flag){
					  if (L_LineLimit_Detect_flag) Motor_Cur_State = Motor_Move_LEFT;
					  else Motor_Cur_State = Motor_Move_RIGHT;
				  }
				  RC_Step = 100;
			  }
			  else{
				  // Line Limit detect
				  if(LineTrace_Run_flag == 1 || LineLimit_Enable_flag == 1){
					  Motor_Spd_State = Motor_SPD_HIGH;
					  Chk_Infrared_Limit();
				  }
				  else{
					  Motor_Spd_State = Motor_SPD_LOW;
				  }
				  if (L_LineLimit_Detect_flag || R_LineLimit_Detect_flag){
					  if (L_LineLimit_Detect_flag) Motor_Cur_State = Motor_Move_LEFT;
					  else Motor_Cur_State = Motor_Move_RIGHT;
					  RC_Step = 100; // line out
				  }
				  else{
					  RC_Step = 20; // line in
				  }
			  }

			  break;

		  case 20:
			  // ultra sonicwave detect
			  if(Distance_Chk()){
				  Motor_Cur_State = Motor_Move_STOP;
				  RC_Step = 100; // ultra sonicwave detect
			  }
			  else{
				  RC_Step = 30;
			  }


			  break;

		  case 30:
			  // color sensor
			  if (LineTrace_Mode == 2){
				  RC_Step = 40;
			  }
			  else{
				  col = color_get();
				  if(col.col_flag){

					  color = cur_color = color_chk(col);
				  }
				  else{
					  col.col_flag=0;
					  color = 'x';
				  }
				  RC_Step = 40;
			  }

			  break;

		  case 40:
			  // color sensor
			  if(color=='r'){
				  // stop
				  Motor_Cur_State = Motor_Move_STOP;

			  }
			  else if(color=='g'){
				  // high speed
				  Motor_Spd_State = Motor_SPD_HIGH;

			  }
			  else if(color=='b'){
				  // normal speed
				  Motor_Spd_State = Motor_SPD_LOW;
				  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1, SPD_NORMAL);
				  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2, SPD_NORMAL);
			  }
			  else{
				  Motor_Cur_State = Motor_Move_FWD;
			  }
			  RC_Step = 100;
			  break;

		  case 100:
			  if (Motor_Old_State != Motor_Cur_State){
				  Motor_Old_State = Motor_Cur_State;
			  }
			  RC_Step = 110;
			  break;

		  case 110:
			  // motor speed set
			  if (Motor_Spd_State == Motor_SPD_HIGH){
				  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1, SPD_HIGH);
				  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2, SPD_HIGH);
			  }
			  else if (Motor_Spd_State == Motor_SPD_LOW){
				  if (LineTrace_Mode == 2){
					  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1, SPD_NORMAL_2);
					  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2, SPD_NORMAL_2);
				  }
				  else{
					  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1, SPD_NORMAL);
					  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2, SPD_NORMAL);
				  }


			  }
			  else if (Motor_Spd_State == Motor_SPD_ROTATE){
				  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1, SPD_ROTATE);
				  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2, SPD_ROTATE);
				  if (LineTrace_Mode == 1 || LineTrace_Mode == 2) Rotate_flag = 1;
			  }
			  else{
				  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1, SPD_NORMAL);
				  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2, SPD_NORMAL);
			  }
			  // motor direction set
			  if (Motor_Old_State == Motor_Move_STOP){
				  // stop
				  Moter_Control(Motor_Move_STOP);
			  }
			  else if(Motor_Old_State == Motor_Move_LEFT){
				  // move left
				  Moter_Control(Motor_Move_LEFT);
			  }
			  else if(Motor_Old_State == Motor_Move_RIGHT){
				  // move right
				  Moter_Control(Motor_Move_RIGHT);
			  }
			  else if(Motor_Old_State == Motor_Move_FWD){
				  // move fwd
				  Moter_Control(Motor_Move_FWD);
			  }
			  else if(Motor_Old_State == Motor_Move_BWD){
				  // move bwd
				  Moter_Control(Motor_Move_BWD);
			  }
			  RC_Step = 1; //
			  break;
	  }
	  HAL_Delay(1);
    /* USER CODE END WHILE */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

#include "main.h"
#include "stm32f4xx_hal.h"

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

#define freq_source 80000000 //internal clock source freq

uint16_t prescaler = 40; //prescaler used by timer
uint32_t freq_counter; //the freq of the timer calculated from freq_source and prescaler
uint16_t init_speed = 10000; //this sets the acceleration by setting the timing of first step, the smaller the number the faster the acceleration
uint16_t SPR = 3200; //steps per revolution of the stepper motor
float tick_freq[3]; //the freq that the steps need to be calculated from frq_counter RPM and SPR
float speed[3]; //the current speed measured by timer ticks in ARR value to count up to
float target_speed[3]; //the target speed that speed is accelerating towards

int32_t n[3];
int8_t curret_dir[3], target_dir[3], RPM_zero[3];


void stepper_setup(void){

	freq_counter = freq_source / (prescaler + 1); //calculate the timer freq

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIODEN; //enable port A and port D clock

	//setup timer pins
	GPIOA->MODER |= GPIO_MODER_MODE0_1 | GPIO_MODER_MODE6_1; //setup pin A0 and pin A6 to AF
	GPIOD->MODER |= GPIO_MODER_MODER12_1; //setup pin D12 to AF mode
	GPIOA->AFR[0] = (GPIOD->AFR[1] &  ~(0b1111 | (0b1111<<(6*4)))) | 0b0010 | (0b0010<<(6*4)); //set pin A0 and pin A6 to AF timer mode       ;
	GPIOD->AFR[1] = (GPIOD->AFR[1] & ~(0b1111<<(4*(12-8)))) | 0b0010<<(4*(12-8)); //set pin D12 to AF timer mode

	//setup direction and enable pins
	GPIOA->MODER |= GPIO_MODER_MODE1_0 | GPIO_MODER_MODE2_0 |  GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0;
	GPIOD->MODER |= GPIO_MODER_MODE10_0 | GPIO_MODER_MODE11_0;

	//set all 3 enable pins to low
	GPIOA->ODR &= ~(GPIO_ODR_OD1 | GPIO_ODR_OD4);
	GPIOD->ODR &= ~GPIO_ODR_OD10;

	//set all 3 dir pins
	GPIOA->ODR &= ~(GPIO_ODR_OD2 | GPIO_ODR_OD5);
	GPIOD->ODR &= ~GPIO_ODR_OD11;

	//setup all 3 timers

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //enable the timer4 clock
	TIM3->CR1 &= ~TIM_CR1_CEN; //disable channel 1.
	TIM3->PSC = prescaler;   //set prescale
	TIM3->CCMR1 = (TIM3->CCMR1 & ~(0b111<<4)) | (0b110<<4); //set PWM mode 110
	TIM3->CCR1 = 10; //set to min rise time
	TIM3->ARR = init_speed; //set to timing
	TIM3->CCER |= TIM_CCER_CC1E; //enable output to pin.
	TIM3->CR1 |= TIM_CR1_ARPE; //buffer ARR
	TIM3->DIER |= TIM_DIER_UIE; //enable interupt
	NVIC_EnableIRQ(TIM3_IRQn); // Enable interrupt(NVIC level)

	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; //enable the timer4 clock
	TIM4->CR1 &= ~TIM_CR1_CEN; //disable channel 1.
	TIM4->PSC = prescaler;   //set prescale
	TIM4->CCMR1 = (TIM4->CCMR1 & ~(0b111<<4)) | (0b110<<4); //set PWM mode 110
	TIM4->CCR1 = 10; //set to min rise time
	TIM4->ARR = init_speed; //set to timing
	TIM4->CCER |= TIM_CCER_CC1E; //enable output to pin.
	TIM4->CR1 |= TIM_CR1_ARPE; //buffer ARR
	TIM4->DIER |= TIM_DIER_UIE; //enable interupt
	NVIC_EnableIRQ(TIM4_IRQn); // Enable interrupt(NVIC level)


	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; //enable the timer4 clock
	TIM5->CR1 &= ~TIM_CR1_CEN; //disable channel 1.
	TIM5->PSC = prescaler;   //set prescale
	TIM5->CCMR1 = (TIM4->CCMR1 & ~(0b111<<4)) | (0b110<<4); //set PWM mode 110
	TIM5->CCR1 = 10; //set to min rise time
	TIM5->ARR = init_speed; //set to timing
	TIM5->CCER |= TIM_CCER_CC1E; //enable output to pin.
	TIM5->CR1 |= TIM_CR1_ARPE; //buffer ARR
	TIM5->DIER |= TIM_DIER_UIE; //enable interupt
	NVIC_EnableIRQ(TIM5_IRQn); // Enable interrupt(NVIC level)

	//initialize variables
	speed[0] = init_speed;
	speed[1] = init_speed;
	speed[2] = init_speed;
	RPM_zero[0] = 1;
	RPM_zero[1] = 1;
	RPM_zero[2] = 1;
	curret_dir[0] = 1;
	curret_dir[1] = 1;
	curret_dir[2] = 1;
	target_dir[0] =1;
	target_dir[2] =1;
	target_dir[3] =1;
}

void disable_steppers(void){
	//set all 3 enable pins to low
	GPIOA->ODR &= ~(GPIO_ODR_OD1 | GPIO_ODR_OD4);
	GPIOD->ODR &= ~GPIO_ODR_OD10;
}

void enable_steppers(void){
	//set all 3 enable pins to high
	GPIOA->ODR |= GPIO_ODR_OD1 | GPIO_ODR_OD4;
	GPIOD->ODR |= GPIO_ODR_OD10;
}

void set_speed(uint8_t motor_num, float RPM){

	if(RPM==0){
		RPM_zero[motor_num] = 1;
		target_speed[motor_num] = init_speed;
	}else{
		RPM_zero[motor_num] = 0;
		if(RPM>0)target_dir[motor_num] = 1;
		else{
			target_dir[motor_num] = 0;
			RPM = RPM *-1;
		}
		tick_freq[motor_num] = SPR * RPM / 60;
		target_speed[motor_num] = freq_counter / tick_freq[motor_num];
	}
	if(motor_num==0)TIM3->CR1 |= TIM_CR1_CEN; //enable channel 1 of timer 3.
	if(motor_num==1)TIM4->CR1 |= TIM_CR1_CEN; //enable channel 1 of timer 4.
	if(motor_num==2)TIM5->CR1 |= TIM_CR1_CEN; //enable channel 1 of timer 5.
}





int main(void){

  HAL_Init();
  SystemClock_Config();

  stepper_setup();


  enable_steppers();


  set_speed(2,400);
  HAL_Delay(3000);

  set_speed(2,-210);
  HAL_Delay(3000);

  set_speed(2,1);
  HAL_Delay(3000);

  set_speed(2,300);
  HAL_Delay(3000);


  disable_steppers();


  while (1){


  }
}

void TIM3_IRQHandler(void){

	TIM3->SR &= ~TIM_SR_UIF; // clear UIF flag

	//if the target speed is zero & current speed is slower init speed the disable the channel
	if (RPM_zero[0] && (speed[0] >= init_speed))TIM3->CR1 &= ~TIM_CR1_CEN;

	//if the current direction is same as target direction
	if(target_dir[0] == curret_dir[0]){

		//if target current speed is slower than init speed then set to init and reset n
		if (speed[0]>=init_speed){
			speed[0] = init_speed;
			n[0]=0;
		}

		//if target speed is slower than init and current speed is slower than init speed then set current to target and reset n
		if((target_speed[0] >= init_speed) && (speed[0] >= init_speed)){
			speed[0] = target_speed[0];
			n[0]=0;

			//if current speed is slower than target then speed up else slow down
		}else if(speed[0]>target_speed[0]){
					n[0]++;
					speed[0] = speed[0] - ( (2 * speed[0]) / (4 * n[0] + 1) );
		  	  }else{
		  		  n[0]--;
		  		  speed[0] = (speed[0] * (4 * n[0] + 1) / (4 * n[0] - 1));
		  	  }

	//else the current direction is not same as target direction
	}else{

		//if the current speed is slower than init speed then flip the direction pin and reset
		if(speed[0] > init_speed-100){
			if(target_dir[0])GPIOA->ODR &= ~GPIO_ODR_OD5; //set direction pin
			else GPIOA->ODR |= GPIO_ODR_OD5; //set direction pin
			curret_dir[0] = target_dir[0];
			speed[0] = init_speed;
			n[0] = 0;

		//else slow down
		}else{
			n[0]--;
			speed[0] = (speed[0] * (4 * n[0] + 1) / (4 * n[0] - 1));
		}
	}

	TIM3->ARR = (uint32_t)speed[0];//update ARR
}



void TIM4_IRQHandler(void){

	TIM4->SR &= ~TIM_SR_UIF; // clear UIF flag

	//if the target speed is zero & current speed is slower init speed the disable the channel
	if (RPM_zero[1] && (speed[1] >= init_speed))TIM4->CR1 &= ~TIM_CR1_CEN;

	//if the current direction is same as target direction
	if(target_dir[1] == curret_dir[1]){

		//if target current speed is slower than init speed then set to init and reset n
		if (speed[1]>=init_speed){
			speed[1] = init_speed;
			n[1]=0;
		}

		//if target speed is slower than init and current speed is slower than init speed then set current to target and reset n
		if((target_speed[1] >= init_speed) && (speed[1] >= init_speed)){
			speed[1] = target_speed[1];
			n[1]=0;

			//if current speed is slower than target then speed up else slow down
		}else if(speed[1]>target_speed[1]){
					n[1]++;
					speed[1] = speed[1] - ( (2 * speed[1]) / (4 * n[1] + 1) );
		  	  }else{
		  		  n[1]--;
		  		  speed[1] = (speed[1] * (4 * n[1] + 1) / (4 * n[1] - 1));
		  	  }

	//else the current direction is not same as target direction
	}else{

		//if the current speed is slower than init speed then flip the direction pin and reset
		if(speed[1] > init_speed-100){
			if(target_dir[1])GPIOD->ODR &= ~GPIO_ODR_OD11; //set direction pin
			else GPIOD->ODR |= GPIO_ODR_OD11; //set direction pin
			curret_dir[1] = target_dir[1];
			speed[1] = init_speed;
			n[1] = 0;

		//else slow down
		}else{
			n[1]--;
			speed[1] = (speed[1] * (4 * n[1] + 1) / (4 * n[1] - 1));
		}
	}

	TIM4->ARR = (uint32_t)speed[1];//update ARR
}


void TIM5_IRQHandler(void){

	TIM5->SR &= ~TIM_SR_UIF; // clear UIF flag

	//if the target speed is zero & current speed is slower init speed the disable the channel
	if (RPM_zero[2] && (speed[2] >= init_speed))TIM5->CR1 &= ~TIM_CR1_CEN;

	//if the current direction is same as target direction
	if(target_dir[2] == curret_dir[2]){

		//if target current speed is slower than init speed then set to init and reset n
		if (speed[2]>=init_speed){
			speed[2] = init_speed;
			n[2]=0;
		}

		//if target speed is slower than init and current speed is slower than init speed then set current to target and reset n
		if((target_speed[2] >= init_speed) && (speed[2] >= init_speed)){
			speed[2] = target_speed[2];
			n[2]=0;

			//if current speed is slower than target then speed up else slow down
		}else if(speed[2]>target_speed[2]){
					n[2]++;
					speed[2] = speed[2] - ( (2 * speed[2]) / (4 * n[2] + 1) );
		  	  }else{
		  		  n[2]--;
		  		  speed[2] = (speed[2] * (4 * n[2] + 1) / (4 * n[2] - 1));
		  	  }

	//else the current direction is not same as target direction
	}else{

		//if the current speed is slower than init speed then flip the direction pin and reset
		if(speed[2] > init_speed-100){
			if(target_dir[2])GPIOA->ODR &= ~GPIO_ODR_OD2; //set direction pin
			else GPIOA->ODR |= GPIO_ODR_OD2; //set direction pin
			curret_dir[2] = target_dir[2];
			speed[2] = init_speed;
			n[2] = 0;

		//else slow down
		}else{
			n[2]--;
			speed[2] = (speed[2] * (4 * n[2] + 1) / (4 * n[2] - 1));
		}
	}

	TIM5->ARR = (uint32_t)speed[2];//update ARR
}




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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

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

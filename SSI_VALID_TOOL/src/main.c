/**
 ******************************************************************************
 * @file    main.c
 * @author  Igor Kantorski
 * @version V1.0
 * @date    13-May-2018
 * @brief   Entry point of SSI encoders validation tool.
 ******************************************************************************
 */


#include "stm32f1xx.h"
#include "LCD_HW.h"


#define SSI_CLK_PIN GPIO_PIN_5
#define SSI_DTA_PIN GPIO_PIN_6
#define SSI_MIN_WAIT_TIME 21 //us



uint32_t SSI_CLK_FREQ = 350000; 		//SSI clokc frequency on HZ, max 350000
uint8_t SSI_MT_BITS = 13;		//Number of bits per turns count
uint8_t SSI_ST_BITS = 12;		//Number of bits per turn
uint8_t SSI_ALL_BITS = 25;		//Nuber of all transision bits
uint32_t GLOBAL_RESPONSE = 2000;



//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
/*			TYPEDEFS AND CLASSES							*/

typedef struct SSIConf_s
{
	uint8_t C_MT_Bits;
	uint8_t C_ST_Bits;
	uint8_t C_all_Bits;
	uint32_t C_ClockFreq;
	uint32_t C_MinWaitTime;
	uint32_t S_all_Hold_Reg;
	uint32_t S_MT_Hold_Reg;
	uint32_t S_ST_Hold_Reg;
	uint32_t h_HoldUpClkTicks;
	uint32_t h_DblBits;
	uint32_t h_DblBits_pp;
	uint32_t h_DblBits_pp_wait_t;
	uint32_t h_valid_Bit_Mask;
} SSIConfState;

//////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
/*					GLOBAL INSTANCES						*/


GPIO_InitTypeDef gpioLcdBus;
TON timer_1 = { .elapsed = 0, .T_ms = 0, .start = 0, .Q = 0};
TON timer_HW = { .elapsed = 0, .T_ms = 0, .start = 0, .Q = 0};
TIM_HandleTypeDef t1;
Screen mainScreen;
uint32_t GL_READ_BUF[48];
uint32_t GL_WRITE_BUF[48];
uint32_t counter = 0;
SSIConfState GlobalSSIConfig =
{
	.C_MT_Bits = 13,
	.C_ST_Bits = 12,
	.C_all_Bits = 25,
	.C_ClockFreq = 350000,
	.C_MinWaitTime = 21,
	.S_all_Hold_Reg = 0,
	.S_MT_Hold_Reg = 0,
	.S_ST_Hold_Reg = 0,
	.h_HoldUpClkTicks = 7,
	.h_DblBits = 50,
	.h_DblBits_pp = 51,
	.h_DblBits_pp_wait_t = 59,
	.h_valid_Bit_Mask = 0x1FFFFFF
};
uint8_t reconfSSITaskDone = 0;

//////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
/*					SSI FUNCTIONS UTILITIES					*/


/**
 * It is required for this function to run in cyclic interrupt of frequency
 * equal to SSI_CLK_FREQ.
 *
 * Function checks the condition for starting SSI transactions. It should be called
 * before startSSIMaster(...).
 *
 * Function sets the initial condition of CLK line to 1 and holds it.
 *
 * Returns 3 if init method in idlenes state
 * Returns 2 if busy, i.e. waiting for 21us to elapse
 * Returns 1 if slave sets DTA to HIGH after required 21us
 * Returns 0 if slave didn't set DTA to HIGH after required 21us
 *
 */
uint8_t initSSIMaster(uint8_t start, SSIConfState *config)
{
	static uint32_t count_int_tick = 0;  			//Counter -> wait for 21us
	static uint8_t initInProgress = 0;
	static uint8_t lastOccuredStart = 0;
	uint8_t retval = 3;
	//RisingEdge on start input
	if(start && !lastOccuredStart && !initInProgress)
	{
		config->h_HoldUpClkTicks = (((config->C_ClockFreq)*(config->C_MinWaitTime))/1000000);
		config->C_all_Bits = config->C_ST_Bits + config->C_MT_Bits;
		config->h_DblBits = 2*(config->C_all_Bits);
		config->h_DblBits_pp = config->h_DblBits + 1;
		config->h_DblBits_pp_wait_t = config->h_DblBits + 2 +config->h_HoldUpClkTicks;
		config->h_valid_Bit_Mask = 0xFFFFFFFF >> (32 - config->C_all_Bits);
		HAL_GPIO_WritePin(GPIOA, SSI_CLK_PIN, GPIO_PIN_SET);
		initInProgress = 1;
	}
	else if (initInProgress)
	{
		if(count_int_tick++ > (config->h_HoldUpClkTicks))
		{
			initInProgress = 0;
			count_int_tick = 0;
			if(HAL_GPIO_ReadPin(GPIOA, SSI_DTA_PIN) == GPIO_PIN_SET)
				retval = 1;	//Init done correctly
			else
				retval = 0;	//Slave didn't respond to Init
		}
		else
			retval = 2; //Waiting for time to elapse
	}


	lastOccuredStart = start;
	return retval;


}

uint8_t startSSIMaster(uint32_t *response, uint8_t start_transaction, SSIConfState *config)
{
	static uint8_t edge_cnt = 0;					//When bit_cnt is even then we are after falling edge of clk
												//When bit_cnt is odd then we are after rising edge of clk
	//static uint8_t hold_toggle = 0;
	//static uint8_t trans_in_progress = 0;
	static uint8_t wsk_trans = 0;
	uint8_t retval = 200;

	//uint32_t curr_bit_mask = 0;					//Current bit mask

	switch (wsk_trans)
	{
		case 0:		//prepare
		{
			edge_cnt = 0;
			if(start_transaction)	wsk_trans = 10;
			retval = 0; //not started yet or transmision done
			break;
		}
		case 10:	//First falling edge, set FE,
		{
			HAL_GPIO_WritePin(GPIOA, SSI_CLK_PIN, GPIO_PIN_RESET);
			edge_cnt++;
			wsk_trans = 20;
			retval = 1; //transmision in progerss
			break;
		}
		case 20:	//First Rising edge, set RE
		{
			HAL_GPIO_WritePin(GPIOA, SSI_CLK_PIN, GPIO_PIN_SET);
			edge_cnt++;
			wsk_trans = 30;
			retval = 1; //transmision in progerss
			break;
		}
		case 30:	//Falling edge, set FE, time to read
		{
			HAL_GPIO_WritePin(GPIOA, SSI_CLK_PIN, GPIO_PIN_RESET);
			if (!(edge_cnt % 2))
				*response = ((*response) << 1) | ((uint32_t) HAL_GPIO_ReadPin(GPIOA, SSI_DTA_PIN));
			edge_cnt++;
			wsk_trans = 40;
			retval = 1; //transmision in progerss
			break;
		}
		case 40:	//Rising edge, check if maybe end of transmission and hold high?
		{
			HAL_GPIO_WritePin(GPIOA, SSI_CLK_PIN, GPIO_PIN_SET);
			if(edge_cnt >= config->h_DblBits_pp)
				wsk_trans = 50;
			else
				wsk_trans = 30;
			edge_cnt++;
			retval = 1; //transmision in progerss
			break;
		}
		case 50:	//Hold line high
		{
			if(edge_cnt >= config->h_DblBits_pp_wait_t)		//transaction done, return to start
				wsk_trans = 0;
			edge_cnt++;
			retval = 2; //transmision done, transaction in progress (waiting high)
			break;
		}

	}

	/*

	if((start_transaction) && !(trans_in_progress))				// Start of transaction moment
	{
		trans_in_progress = 1;									//indicate start of transaction
		hold_toggle = 0;										//take off toggle holding
		bit_cnt = 0;											//reset the bit counter
		//HAL_GPIO_WritePin(GPIOA, SSI_CLK_PIN, GPIO_PIN_SET);	// hold HIGH on CLK
	}
	else if (!trans_in_progress)								// No signal to start and end of transaction
	{
		bit_cnt = 0;											// hold 0 in the bit counter
		hold_toggle = 1;										// set the toggle holder
		HAL_GPIO_WritePin(GPIOA, SSI_CLK_PIN, GPIO_PIN_SET);	// hold HIGH on CLK
	}


	if(!hold_toggle)											// Toggle the CLK line only when bit transfer is in progress
	{
		HAL_GPIO_TogglePin(GPIOA, SSI_CLK_PIN);		//Generate clock pulses
	}
	// Last (SSI_ALL_BITS) bit is acquired when i = 2*SSI_ALL_BITS - 1 (RE)
	// Then it is required to perform one more falling and rising
	if (((bit_cnt % 2) == 1) && (bit_cnt <= (config->h_DblBits) - 1) && trans_in_progress)	//Rising edge, read value from DTA while transfer is in progress
	{
		curr_bit_mask = (uint32_t) HAL_GPIO_ReadPin(GPIOA, SSI_DTA_PIN);	//Read value from DTA line
		*response = ((*response) << 1) | curr_bit_mask;
		bit_cnt++;
		return 0;						//Transfer in progress
	}

	if (bit_cnt >= (config->h_DblBits + 2 + config->C_MinWaitTime))
	{
		hold_toggle = 1;
		trans_in_progress = 0;
		bit_cnt++;
		return 2; 						//Transaction done and ready for next one
	}

	if (bit_cnt >= config->h_DblBits + 1 )	//Additional cycle done, DTA should remain high
	{
		hold_toggle = 1;
		bit_cnt++;
		return 1; 						//Register filled but not ready for next transaction
	}

	bit_cnt++;
	return 200;							//Undefined state
*/
	return retval;
}




void HAL_SYSTICK_Callback()
{

	uint32_t respMaster = 0;
	uint8_t stateMaster = 0;
	uint8_t stateInit = 0;

	stateInit = initSSIMaster(reconfSSITaskDone, &GlobalSSIConfig);  //#3 = StartRxTx
	stateMaster = startSSIMaster(&respMaster, /*(stateInit == 1) &&*/ GL_READ_BUF[3], &GlobalSSIConfig);  //#3 = StartRxTx
	if (stateMaster==0)
	{
		GL_WRITE_BUF[5] += 1 + (GlobalSSIConfig.h_valid_Bit_Mask & respMaster);  //#5 = PV value from encoder
	}
}

void TIM1_UP_TIM10_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&t1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	timerON(&timer_HW);
	timerON(&timer_1);
	//GL_WRITE_BUF[5] = counter++;
	//HAL_GPIO_TogglePin(GPIOA, SSI_CLK_PIN);
}

void SystemClock_Config(uint32_t clk)
{
	  RCC_OscInitTypeDef RCC_OscInitStruct;
	  RCC_ClkInitTypeDef RCC_ClkInitStruct;

	    /**Initializes the CPU, AHB and APB busses clocks
	    */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	  RCC_OscInitStruct.HSICalibrationValue = 16;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	  HAL_RCC_OscConfig(&RCC_OscInitStruct);


	    /**Initializes the CPU, AHB and APB busses clocks
	    */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);



	    /**Configure the Systick interrupt time
	    */
	  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/(clk));

	    /**Configure the Systick
	    */
	  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	  /* SysTick_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
 * reconfigures SSI hardware clocks on RE of controlByte
 * join ctrlByte to START(ON/OFF) status from GL_READ_BUFFER
 * function sets its return DONE flag, immediately after done cofiguration.
 * Function hold low DONE flag only if ctrlByte is holding low.
 */
uint8_t reconfigureSSITask(uint8_t ctrlByte, SSIConfState *config)
{
	static uint8_t lastCtrlByte = 0;
	static uint8_t retval = 0;

	//RE on ctrl byte
	if(ctrlByte && !lastCtrlByte)
	{
		//update ssi config struct (2 = SSI_CLK_FRQ
		config->C_ClockFreq = 2*GL_READ_BUF[2];
		config->C_MT_Bits = GL_READ_BUF[0];
		config->C_MinWaitTime = 21;
		config->C_ST_Bits = GL_READ_BUF[1];
		config->C_all_Bits = config->C_MT_Bits + config->C_ST_Bits;

		//HAL_NVIC_DisableIRQ(SysTick_IRQn);

		HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/(config->C_ClockFreq));
		HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
		HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

		//HAL_NVIC_EnableIRQ(SysTick_IRQn);

		retval = 1;
	}
	if(!ctrlByte)
	{
		retval = 0;
	}

	lastCtrlByte = ctrlByte;

	return retval;
}

int main(void)
{

	SystemCoreClock = 64000000;

	HAL_Init();
	SystemClock_Config(GlobalSSIConfig.C_ClockFreq);

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_TIM1_CLK_ENABLE();	//8MHz

	GPIO_InitTypeDef gpioOut;
	GPIO_InitTypeDef gpioIn;
	GPIO_InitTypeDef gpioLcdCtrl;
	GPIO_InitTypeDef gpioButtons;

	//Configure Output CLK
	gpioOut.Mode = GPIO_MODE_OUTPUT_PP;
	gpioOut.Pin = SSI_CLK_PIN;
	gpioOut.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &gpioOut);

	//Configure Input DTA
	gpioIn.Mode = GPIO_MODE_INPUT;
	gpioIn.Pin = SSI_DTA_PIN;
	gpioIn.Pull = GPIO_PULLUP;
	gpioIn.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &gpioIn);

	HAL_GPIO_WritePin(GPIOA, SSI_CLK_PIN, GPIO_PIN_SET);

	//Configure LCD Control registers
	gpioLcdCtrl.Mode = GPIO_MODE_OUTPUT_OD;
	gpioLcdCtrl.Pin = 	RS | RW | E;
	gpioLcdCtrl.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &gpioLcdCtrl);

	//Configure LCD Bus registers
	//Default conf. is output. Please change back to Out if conf is changed on the fly
	gpioLcdBus.Mode = GPIO_MODE_OUTPUT_OD;
	gpioLcdBus.Pin = DB0 | DB1 | DB2 | DB3 | DB4 | DB5 | DB6 | DB7;
	gpioLcdBus.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &gpioLcdBus);

	//Configure button registers:
	gpioButtons.Mode = GPIO_MODE_INPUT;
	gpioButtons.Pin = NEXT_PAR_PIN;
	gpioButtons.Pull = GPIO_NOPULL;
	gpioButtons.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &gpioButtons);

	gpioButtons.Mode = GPIO_MODE_INPUT;
	gpioButtons.Pin = NEXT_PAG_PIN | NEXT_VAL_PIN;
	gpioButtons.Pull = GPIO_PULLDOWN;
	gpioButtons.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &gpioButtons);




	//Timer conf. for cyclic interrupt
	t1.Instance = TIM1;
	t1.Init.Prescaler = 64 - 1; // fclk/(prescaler+1) = overal freq
	t1.Init.CounterMode = TIM_COUNTERMODE_UP;
	t1.Init.Period = (1000000/1000) - 1; // Overflow by every two ticks Overflow = interrupt <=> max++
	// Overal freq = 500kHz
	t1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	t1.Init.RepetitionCounter = 0;
	t1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&t1);

	//HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);

	HAL_NVIC_SetPriority(TIM1_UP_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);


	HAL_TIM_Base_Start_IT(&t1);

	SCREEN_init_all(&mainScreen, 2, GL_READ_BUF, &timer_HW);

	SCREEN_fill_with_page(&mainScreen, &timer_HW);

	while (1)
	{
		PAGE_continously_write(mainScreen.PV_pag, GL_WRITE_BUF, &timer_HW);
		PAGE_continously_read(mainScreen.PV_pag, GL_READ_BUF, &timer_HW);

		reconfSSITaskDone = reconfigureSSITask((uint8_t) (GL_READ_BUF[3]), &GlobalSSIConfig); 		//#3 = StartTxRx

		SCREEN_buttonNextPage_RE(&mainScreen, &timer_HW);
		SCREEN_buttonPrevPage_RE(&mainScreen, &timer_HW);
		SCREEN_buttonNextParam_RE(&mainScreen, &timer_HW);
		SCREEN_buttonPrevParam_RE(&mainScreen, &timer_HW);
		SCREEN_buttonNextValue_RE(&mainScreen, &timer_HW);
		SCREEN_buttonPrevValue_RE(&mainScreen, &timer_HW);
		wait(&timer_1, 100);
	}

}


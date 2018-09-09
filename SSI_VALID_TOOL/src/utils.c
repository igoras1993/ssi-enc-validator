/*
 * utils.c
 *
 *  Created on: 02.05.2018
 *      Author: Igor
 */

#include "stm32f1xx.h"
#include "utils.h"



char int2charLUT[10] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};



/**
 * @brief sets (TON) .Q if (TON) .start stay H for longer than (TON) .T_ms
 * @param TON *timer
 * This function should be called only from cyclic interrupt.
 * Cyclic interrupt freq (SSI_CLK_FREQ) should be > 1000Hz.
 * Otherwise, T_ms in TON instance should be greater than 1000
 */
void timerON(volatile TON *timer)
{

	if (timer->start)
	{
		(timer->Q) = (uint8_t) ((timer->elapsed)++ >= (1000*(timer->T_ms))/1000 );
	}
	else
	{
		(timer->elapsed) = 0;
		(timer->Q) = 0;
	}
}


void wait(volatile TON *timer, volatile uint32_t time_ms)
{
	//Set blocking mode
	timer->T_ms = time_ms; 	//on init procedure wait 100 ms
	timer->start = 1;		//start counting
	while (!(timer->Q))
	{
		;
	}
	timer->T_ms = 0;
	timer->start = 0;
	timer->Q = 0;
	timer->elapsed = 0;	//reset internal state of counter
}

uint8_t uint2str(uint32_t num, char *buffer)
{
	uint8_t i = 0;
	uint8_t i_max = 0;
	uint32_t rest = 0;

	//buffer = (char*) malloc( 11 * sizeof(char));
	rest = num;
	for(i=0; i<10; i++)
	{
		if(rest == 0) break;
		rest /= 10;
	}
	rest = num;
	i_max = i;

	for(i=0; i<i_max; i++)
	{
		*(buffer+(i_max - 1 - i)) = int2charLUT[rest % 10];
		rest /= 10;
	}
	//*(buffer+i_max) = '\0';

	return i_max;
}

void copyCharBuf(char *src, char *dst, uint8_t len)
{
	uint8_t i = 0;

	for(i = 0; i < len; i++)
	{
		*(dst + i) = *(src + i);
	}
}

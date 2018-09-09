/*
 * utils.h
 *
 *  Created on: 02.05.2018
 *      Author: Igor
 */

#ifndef UTILS_H_
#define UTILS_H_
#endif /* UTILS_H_ */

/**
 *  Class TON along with method invoked in cyclic interrupt
 *  Always remember to initialize all members to 0
 */
typedef struct TON_s
{
	uint32_t elapsed;
	uint32_t T_ms;
	uint8_t start;
	uint8_t Q;
} TON;



/**
 * @brief sets (TON) .Q if (TON) .start stay H for longer than (TON) .T_ms
 * @param TON *timer
 * This function should be called only from cyclic interrupt.
 * Cyclic interrupt freq (SSI_CLK_FREQ) should be > 1000Hz.
 * Otherwise, T_ms in TON instance should be greater than 1000
 */
void timerON(volatile TON *timer);


void wait(volatile TON *timer, volatile uint32_t time_ms);

uint8_t uint2str(uint32_t num, char *buffer);

void copyCharBuf(char *src, char *dst, uint8_t len);


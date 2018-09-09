/*
 * LCD_HW.c
 *
 *  Created on: 01.05.2018
 *      Author: Igor
 */

#include "LCD_HW.h"

/* Values list */
Value all_values[48] =
{
	//Value no #0 = MT value of encoder
	{
		.PV			= 13,			// Multi turn bbits
		.enum_range = 11,			// 11 possible MT values
		.val_enum	= 8,			// (idx of lut table)
		.LUT_val	= {5,	6,	7,	8,	9,	10,	11,	12,	13,	14,	15},
		.LUT_view	= {" 5",	" 6",	" 7",	" 8",	" 9",	"10",	"11",	"12",	"13",	"14",	"15"},
		.view		= "13"
	},

	//Value no #1 = ST value of encoder
	{
		.PV			= 12,			// Single turn bbits
		.enum_range = 11,			// 11 possible MT values
		.val_enum	= 7,			// (idx of lut table)
		.LUT_val	= {5,	6,	7,	8,	9,	10,	11,	12,	13,	14,	15},
		.LUT_view	= {" 5",	" 6",	" 7",	" 8",	" 9",	"10",	"11",	"12",	"13",	"14",	"15"},
		.view		= "12"
	},

	//Value no #2 = SSI_CLK_FREQ, modal values have to have LUT enumerations
	{
		.PV 		= 60000,	// in Hz
		.enum_range = 6,		// 6 possible values in this parameter value
		.val_enum	= 0,		// enumerator value for default 75 (idx of lut table)
		.LUT_val 	= {60000,	75000,	100000,	125000,	150000,	175000	},
		.LUT_view	= {" 60",	" 75",	"100",	"125",	"150",	"175"	},
		.view 		= " 60"
	},

	//Value no #3 = SSI_Start_RxTx, modal
	{
		.PV 		= 0,	// in Hz
		.enum_range = 2,		// 2 possible values in this parameter value
		.val_enum	= 0,		// enumerator value for default 0 (idx of table)
		.LUT_val 	= {0, 1},
		.LUT_view	= {"OFF",	" ON"},
		.view 		= "OFF"
	},

	//Value no #4 = TEST, non-modal
	{
		.PV 		= 0,	// in Hz
		//.enum_range = 6,		// 6 possible values in this parameter value
		//.val_enum	= 0,		// enumerator value for default 75
		//.LUT_val 	= {60000,	75000,	100000,	125000,	150000,	175000	},
		//.LUT_view	= {" 60",	" 75",	"100",	"125",	"150",	"175"	},
		.view 		= "8885"
	},

	//Value no #5 = PV value of encoder, non-modal
	{
		.PV 		= 12,	// in Hz
		//.enum_range = 6,		// 6 possible values in this parameter value
		//.val_enum	= 0,		// enumerator value for default 75
		//.LUT_val 	= {60000,	75000,	100000,	125000,	150000,	175000	},
		//.LUT_view	= {" 60",	" 75",	"100",	"125",	"150",	"175"	},
		.view 		= "0000"
	}


};

/* Parameters List */

Param all_params[48] =
{

		//Param no #0 = MT bits
		{
			.name 		= "MT",		//MT bits value
			.modality 	= 1,		//MT is modal
			.digits		= 2,		//3 digits
			.param_idx	= 0,		//this param default position is 0
			.val_adr	= 3,		//this param default value adr is 3
			.val 		= &(all_values[0])

		},

		//Param no #1 = ST bits
		{
			.name 		= "ST",		//ST bits value
			.modality 	= 1,		//ST is modal
			.digits		= 2,		//in kHz, 3 digits
			.param_idx	= 1,		//this param default position is 1
			.val_adr	= 11,		//this param default value adr is 11
			.val 		= &(all_values[1])

		},

		//Param no #2 = SSI CLK FREQUENCY
		{
			.name 		= "FQ",		//SSI CLK frequency
			.modality 	= 1,		//freq is modal
			.digits		= 3,		//in kHz, 3 digits
			.param_idx	= 2,		//this param default position is 2
			.val_adr	= 67,		//this param default value adr is 67
			.val 		= &(all_values[2])

		},

		//Param no #3 = SSI_Start_RxTx, modal
		{
			.name 		= "Rx",		//SSI start read process
			.modality 	= 1,		//Start is modal
			.digits		= 3,		//On/Off, 3 digits
			.param_idx	= 0,		//this param default position is 0
			.val_adr	= 3,		//this param default value adr is 3
			.val 		= &(all_values[3])

		},
		//Param no #4 = Test non-modal
		{
			.name 		= "T2",		//SSI CLK frequency
			.modality 	= 0,		//freq is modal
			.digits		= 4,		//in kHz, 3 digits
			.param_idx	= 1,		//this param default position is 1
			.val_adr	= 11,		//this param default value adr is 11
			.val 		= &(all_values[4])

		},

		//Param no #5 = PV encoder value non-modal
		{
			.name 		= "PV",		//PV value from encoder
			.modality 	= 0,		//PV value is non-modal
			.digits		= 10,		//32bits max, 10 digits
			.param_idx	= 2,		//this param default position is 2
			.val_adr	= 67,		//this param default value adr is 67
			.val 		= &(all_values[5])

		}



};


/* Hardware */

/*
 * Clear entire screen with whitespace characters, disables current shift,
 * sets Addr counte to 0(first line firs char). Sets I_D to 1 (increment).
 * Leaves Shift mode S alone.
 */
void HW_clr_scr(TON *own_tim)
{
	HAL_GPIO_WritePin(GPIOB, RS, 0);
	HAL_GPIO_WritePin(GPIOB, RW, 0);
	HAL_GPIO_WritePin(GPIOB, DB7, 0);
	HAL_GPIO_WritePin(GPIOB, DB6, 0);
	HAL_GPIO_WritePin(GPIOB, DB5, 0);
	HAL_GPIO_WritePin(GPIOB, DB4, 0);
	HAL_GPIO_WritePin(GPIOB, DB3, 0);
	HAL_GPIO_WritePin(GPIOB, DB2, 0);
	HAL_GPIO_WritePin(GPIOB, DB1, 0);
	HAL_GPIO_WritePin(GPIOB, DB0, 1);

	//Command set
	HAL_GPIO_WritePin(GPIOB, E, 1);
	wait(own_tim, 2);
	HAL_GPIO_WritePin(GPIOB, E, 0);
	wait(own_tim, 20);

	//Clear done

}

/**
 * Performs an initialization of LCD display
 */
void HW_init_lcd(TON *own_tim)
{
	volatile GPIO_PinState DL = 1;	//8-bits
	volatile GPIO_PinState N = 1;	//2 disp lines
	volatile GPIO_PinState F = 0;	//5x8 font
	volatile GPIO_PinState D = 1;	//disp on
	volatile GPIO_PinState C = 1;	//cursor on
	volatile GPIO_PinState B = 1;	//blinking on
	volatile GPIO_PinState I_D = 1;	//Increment DDRAM addres by 1(=cursor moves 1 forward) when data is writen
	volatile GPIO_PinState S = 0;	//Do not shift display when data is writen





	wait(own_tim, 5); //wait 100 ms in blocking mode
	//Init wait done
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);


	/* Function set:
	 * Data length DL = 1 (8 bits)
	 * Number of display lines N = 1 (2 lines)
	 * Font F = 0 (5x8 dots)
	 */
	//Command set
	HAL_GPIO_WritePin(GPIOB, RS, 0);
	HAL_GPIO_WritePin(GPIOB, RW, 0);
	HAL_GPIO_WritePin(GPIOB, DB7, 0);
	HAL_GPIO_WritePin(GPIOB, DB6, 0);
	HAL_GPIO_WritePin(GPIOB, DB5, 1);
	HAL_GPIO_WritePin(GPIOB, DB4, DL);
	HAL_GPIO_WritePin(GPIOB, DB3, N);
	HAL_GPIO_WritePin(GPIOB, DB2, F);

	//write cycle:
	HAL_GPIO_WritePin(GPIOB, E, 1);
	wait(own_tim, 2);
	HAL_GPIO_WritePin(GPIOB, E, 0); //write occurence
	wait(own_tim, 100); //wait for instr to be done (4.1ms in datasheet)

	//Function set done



	/* Display on/off control:
	 * Display on/off D = 1: Display on
	 * Cursor on/off C = 1: Cursor on
	 * Blinking on/off B = 1: Blinking on
	 */

	//Command set
	HAL_GPIO_WritePin(GPIOB, RS, 0);
	HAL_GPIO_WritePin(GPIOB, RW, 0);
	HAL_GPIO_WritePin(GPIOB, DB7, 0);
	HAL_GPIO_WritePin(GPIOB, DB6, 0);
	HAL_GPIO_WritePin(GPIOB, DB5, 0);
	HAL_GPIO_WritePin(GPIOB, DB4, 0);
	HAL_GPIO_WritePin(GPIOB, DB3, 1);
	HAL_GPIO_WritePin(GPIOB, DB2, D);
	HAL_GPIO_WritePin(GPIOB, DB1, C);
	HAL_GPIO_WritePin(GPIOB, DB0, B);

	//write cycle:
	HAL_GPIO_WritePin(GPIOB, E, 1);
	wait(own_tim, 2);
	HAL_GPIO_WritePin(GPIOB, E, 0);	//write occurence
	wait(own_tim, 2);	//wait for instr to be done (37us in datasheet, 2ms is enough)

	//Display on control done



	/* Entry mode set:
	 * Incr/decr I_D = 1: Increment DDRAM addr when char is writen (moves cursor as well)
	 * Shift S = 0: Do not shift display when write
	 */

	//Command set
	HAL_GPIO_WritePin(GPIOB, RS, 0);
	HAL_GPIO_WritePin(GPIOB, RW, 0);
	HAL_GPIO_WritePin(GPIOB, DB7, 0);
	HAL_GPIO_WritePin(GPIOB, DB6, 0);
	HAL_GPIO_WritePin(GPIOB, DB5, 0);
	HAL_GPIO_WritePin(GPIOB, DB4, 0);
	HAL_GPIO_WritePin(GPIOB, DB3, 0);
	HAL_GPIO_WritePin(GPIOB, DB2, 1);
	HAL_GPIO_WritePin(GPIOB, DB1, I_D);
	HAL_GPIO_WritePin(GPIOB, DB0, S);

	//write cycle
	HAL_GPIO_WritePin(GPIOB, E, 1);
	wait(own_tim, 2);
	HAL_GPIO_WritePin(GPIOB, E, 0); //write occurence
	wait(own_tim, 2); //wait for instr to be done

	//Entry mode set done

	/* Clear display
	 *
	 */
	HW_clr_scr(own_tim);
	//done
}

/*
 * Sets the DDRAM address. This sets Address counter too, and
 * because of that also cursor position.
 *
 * Addressing for covinence
 *
 * 	\col|| 0  | 1  | 2  | 3  | 4  | 5  | 6  | 7  | 8  | 9  | 10 | 11 | 12 | 13 | 14 | 15 ||
 * 	row\||____|____|____|____|____|____|____|____|____|____|____|____|____|____|____|____||
 * 	    || 00 | 01 | 02 | 03 | 04 | 05 | 06 | 07 | 08 | 09 | 10 | 11 | 12 | 13 | 14 | 15 ||
 * 	_0__||____|____|____|____|____|____|____|____|____|____|____|____|____|____|____|____||
 * 	    || 64 | 65 | 66 | 67 | 68 | 69 | 70 | 71 | 72 | 73 | 74 | 75 | 76 | 77 | 78 | 79 ||
 * 	_1__||____|____|____|____|____|____|____|____|____|____|____|____|____|____|____|____||
 *
 *
 */
void HW_set_cursor_DDRAM_adr(uint8_t adr, TON *own_tim)
{
	//Command set
	HAL_GPIO_WritePin(GPIOB, RS, 0);
	HAL_GPIO_WritePin(GPIOB, RW, 0);
	HAL_GPIO_WritePin(GPIOB, DB7, 1);
	//7 bits left for the ADD adres
	HAL_GPIO_WritePin(GPIOB, DB6, 0x40 & adr);
	HAL_GPIO_WritePin(GPIOB, DB5, 0x20 & adr);
	HAL_GPIO_WritePin(GPIOB, DB4, 0x10 & adr);
	HAL_GPIO_WritePin(GPIOB, DB3, 0x08 & adr);
	HAL_GPIO_WritePin(GPIOB, DB2, 0x04 & adr);
	HAL_GPIO_WritePin(GPIOB, DB1, 0x02 & adr);
	HAL_GPIO_WritePin(GPIOB, DB0, 0x01 & adr);

	//write cycle
	HAL_GPIO_WritePin(GPIOB, E, 1);
	wait(own_tim, 1);
	HAL_GPIO_WritePin(GPIOB, E, 0); //write occurence
	wait(own_tim, 1);

	//DDRAM Adress setting done

}

/**
 * Writes the char bufer "str" of length "len" into the lcd
 * position given by "adr".
 * Function cares that the custor is set on position "curr_DDRAM_ADR",
 * so it remembers it, and restoring after "str" is writen.
 *
 */
void HW_write_str_at_adr(char *str, uint8_t len, uint8_t adr, uint8_t curr_DDRAM_adr, TON *own_tim)
{
	uint8_t cnt = 0;
	uint8_t curr_chr = ' ';
	uint8_t last_DDRAM_adr = 0;

	//1.
	//First, remember last addres position (to restore it at the end of operation)
	last_DDRAM_adr = curr_DDRAM_adr;

	//2.
	//Disable cursor display at the time of writing
	/* Display on/off control:
	 * Display on/off D = 1: Display on
	 * Cursor on/off C = 0: Cursor off
	 * Blinking on/off B = 0: Blinking off
	 */

	//Command set
	HAL_GPIO_WritePin(GPIOB, RS, 0);
	HAL_GPIO_WritePin(GPIOB, RW, 0);
	HAL_GPIO_WritePin(GPIOB, DB7, 0);
	HAL_GPIO_WritePin(GPIOB, DB6, 0);
	HAL_GPIO_WritePin(GPIOB, DB5, 0);
	HAL_GPIO_WritePin(GPIOB, DB4, 0);
	HAL_GPIO_WritePin(GPIOB, DB3, 1);
	HAL_GPIO_WritePin(GPIOB, DB2, 1); //D
	HAL_GPIO_WritePin(GPIOB, DB1, 0); //C
	HAL_GPIO_WritePin(GPIOB, DB0, 0); //B

	//write cycle:
	HAL_GPIO_WritePin(GPIOB, E, 1);
	wait(own_tim, 1);
	HAL_GPIO_WritePin(GPIOB, E, 0);	//write occurence
	wait(own_tim, 1);	//wait for instr to be done (37us in datasheet, 1ms is enough)

	//disabling done

	//3.
	//Next, lets set the DDRAM addres:
	HW_set_cursor_DDRAM_adr(adr, own_tim);

	//4.
	//Now, when the DDRAM adr is set on place, perform
	//continous read-from-buffer operation and write
	//this characters into lcd. Remember, that the AC
	//will increment when writing into lcd DDRAM, so
	//that there is no need to do it manualy
	while(cnt < len)
	{
		//obtain current character from buffer
		curr_chr = (uint8_t) *(str + cnt);

		//Write data to DDRAM
		HAL_GPIO_WritePin(GPIOB, RS, 1);
		HAL_GPIO_WritePin(GPIOB, RW, 0);
		HAL_GPIO_WritePin(GPIOB, DB7, 0x80 & curr_chr);
		HAL_GPIO_WritePin(GPIOB, DB6, 0x40 & curr_chr);
		HAL_GPIO_WritePin(GPIOB, DB5, 0x20 & curr_chr);
		HAL_GPIO_WritePin(GPIOB, DB4, 0x10 & curr_chr);
		HAL_GPIO_WritePin(GPIOB, DB3, 0x08 & curr_chr);
		HAL_GPIO_WritePin(GPIOB, DB2, 0x04 & curr_chr);
		HAL_GPIO_WritePin(GPIOB, DB1, 0x02 & curr_chr);
		HAL_GPIO_WritePin(GPIOB, DB0, 0x01 & curr_chr);

		//write cycle:
		HAL_GPIO_WritePin(GPIOB, E, 1);
		wait(own_tim, 1);
		HAL_GPIO_WritePin(GPIOB, E, 0);	//write occurence
		wait(own_tim, 1);	//wait for instr to be done (41us in datasheet, 1ms is enough)

		//increment for next char
		cnt++;
	}

	//5.
	//Return with cursor on last position (refer to 1.)
	HW_set_cursor_DDRAM_adr(last_DDRAM_adr, own_tim);

	//6.
	//Enable cursor and blinking
	/* Display on/off control:
	 * Display on/off D = 1: Display on
	 * Cursor on/off C = 1: Cursor on
	 * Blinking on/off B = 1: Blinking on
	 */

	//Command set
	HAL_GPIO_WritePin(GPIOB, RS, 0);
	HAL_GPIO_WritePin(GPIOB, RW, 0);
	HAL_GPIO_WritePin(GPIOB, DB7, 0);
	HAL_GPIO_WritePin(GPIOB, DB6, 0);
	HAL_GPIO_WritePin(GPIOB, DB5, 0);
	HAL_GPIO_WritePin(GPIOB, DB4, 0);
	HAL_GPIO_WritePin(GPIOB, DB3, 1);
	HAL_GPIO_WritePin(GPIOB, DB2, 1); //D
	HAL_GPIO_WritePin(GPIOB, DB1, 1); //C
	HAL_GPIO_WritePin(GPIOB, DB0, 1); //B

	//write cycle:
	HAL_GPIO_WritePin(GPIOB, E, 1);
	wait(own_tim, 1);
	HAL_GPIO_WritePin(GPIOB, E, 0);	//write occurence
	wait(own_tim, 1);	//wait for instr to be done (37us in datasheet, 1ms is enough)

	//ALL DONE

}


/* LCD ABSTRACTION LAYER */

/**
 * Calculates lcd position address from
 * given row and column (counted from 0).
 */
uint8_t IS_calc_adr(uint8_t row, uint8_t col)
{
	if(row)	//row == 1
	{
		return 64 + col;
	}
	else 	//row == 0
	{
		return col;
	}
}

/**
 * This method sets and views next param value by:
 *
 * - Checking if given parameter is modal
 * - setting next PV value
 * - setting next viev str from LUT
 * - updating val_enum (current LUT idx)
 * - writing parameter on lcd
 */
void PAGE_next_par_value(Page *pag, TON *own_tim)
{
	uint8_t curr_val_idx = 0;
	uint8_t next_val_idx = 0;

	if(pag->state.PV_param->modality) //if param is modal
	{
		//switch to the next param...
		curr_val_idx = pag->state.PV_param->val->val_enum;
		next_val_idx = (curr_val_idx + 1) % (pag->state.PV_param->val->enum_range);

		//...PV
		pag->state.PV_param->val->PV = pag->state.PV_param->val->LUT_val[next_val_idx];

		//...view
		copyCharBuf(pag->state.PV_param->val->LUT_view[next_val_idx], pag->state.PV_param->val->view, pag->state.PV_param->digits);

		//...enum value (curr LUT idx)
		pag->state.PV_param->val->val_enum = next_val_idx;

		//Write parameter on LCD
		HW_write_str_at_adr(pag->state.PV_param->val->view, pag->state.PV_param->digits, pag->state.PV_param->val_adr, pag->state.PV_param->val_adr, own_tim);


	}
}

/**
 * This method sets and views prev param value by:
 * - Checking if given parameter is modal
 * - setting prev PV value
 * - setting prev viev str from LUT
 * - updating val_enum (current LUT idx)
 * - writing parameter on lcd
 */
void PAGE_prev_par_value(Page *pag, TON *own_tim)
{
	uint8_t curr_val_idx = 0;
	uint8_t prev_val_idx = 0;

	if(pag->state.PV_param->modality) //if param is modal
	{
		//switch to the prev param...
		curr_val_idx = pag->state.PV_param->val->val_enum;
		if(curr_val_idx == 0)
		{
			prev_val_idx = (pag->state.PV_param->val->enum_range) - 1;
		}
		else
		{
			prev_val_idx = (pag->state.PV_param->val->val_enum - 1);
		}

		//...PV
		pag->state.PV_param->val->PV = pag->state.PV_param->val->LUT_val[prev_val_idx];

		//...view
		copyCharBuf(pag->state.PV_param->val->LUT_view[prev_val_idx], pag->state.PV_param->val->view, pag->state.PV_param->digits);

		//...enum value (curr LUT idx)
		pag->state.PV_param->val->val_enum = prev_val_idx;

		//Write parameter on LCD
		HW_write_str_at_adr(pag->state.PV_param->val->view, pag->state.PV_param->digits, pag->state.PV_param->val_adr, pag->state.PV_param->val_adr, own_tim);
	}
}

/**
 * Fills the IS.PV_scr member with present values of
 * all three parameters and all signatures and names
 */
void PAGE_generate_scr(Page *pag)
{
	uint8_t i = 0;



	//First param writing on page
	*(pag->state.PV_scr)   = pag->params[0]->name[0];
	*(pag->state.PV_scr+1) = pag->params[0]->name[1];
	*(pag->state.PV_scr+2) = '=';

	//if param is non-modal (dont have LUTs, value is clearly numerical)
	if(!(pag->params[0]->modality))
	{
		//update value view state from PV
		pag->params[0]->digits = uint2str(pag->params[0]->val->PV, pag->params[0]->val->view);
	}

	for(i = 0; i < 5; i++)
	{
		if (i < pag->params[0]->digits)
			*(pag->state.PV_scr + 3 + i) = pag->params[0]->val->view[i];
		else
			*(pag->state.PV_scr + 3 + i) = ' ';
	}

	//second param writing
	*(pag->state.PV_scr+8)  = pag->params[1]->name[0];
	*(pag->state.PV_scr+9)  = pag->params[1]->name[1];
	*(pag->state.PV_scr+10) = '=';

	//if param is non-modal (dont have LUTs, value is clearly numerical)
	if(!(pag->params[1]->modality))
	{
		//update value view state from PV
		pag->params[1]->digits = uint2str(pag->params[1]->val->PV, pag->params[1]->val->view);
	}

	for(i = 0; i < 5; i++)
	{
		if (i < pag->params[1]->digits)
			*(pag->state.PV_scr + 11 + i) = pag->params[1]->val->view[i];
		else
			*(pag->state.PV_scr + 11 + i) = ' ';
	}

	//Third param writing
	*(pag->state.PV_scr+16)  = pag->params[2]->name[0];
	*(pag->state.PV_scr+17)  = pag->params[2]->name[1];
	*(pag->state.PV_scr+18) = '=';

	//if param is non-modal (dont have LUTs, value is clearly numerical)
	if(!(pag->params[2]->modality))
	{
		//update value view state from PV
		pag->params[2]->digits = uint2str(pag->params[2]->val->PV, pag->params[2]->val->view);
	}

	for(i = 0; i < 13; i++)
	{
		if (i < pag->params[2]->digits)
			*(pag->state.PV_scr + 19 + i) = pag->params[2]->val->view[i];
		else
			*(pag->state.PV_scr + 19 + i) = ' ';
	}

}

/**
 * -selects PV_param as next param
 * -updates internal state of PV_page
 * -generates whole screen once
 * -puts the DDRAM adres on next param adress
 * -writes whole screen to lcd
 *
 */
void PAGE_next_param(Page *pag, TON *own_tim)
{
	//select new PV_param
	pag->state.PV_param = pag->state.PV_param->next;

	//update IS
	pag->state.PV_adr = pag->state.PV_param->val_adr;
	if(pag->state.PV_adr > 64)
	{
		pag->state.PV_row = 1;
		pag->state.PV_col = 3;
	}
	else
	{
		pag->state.PV_row = 0;
		pag->state.PV_col = pag->state.PV_adr;
	}

	//Update PV_scr
	PAGE_generate_scr(pag);

	//Move cursor
	HW_set_cursor_DDRAM_adr(pag->state.PV_adr, own_tim);

	//Write whole screen
	HW_write_str_at_adr(pag->state.PV_scr, 16, 0, pag->state.PV_adr, own_tim);
	HW_write_str_at_adr((char*) (pag->state.PV_scr+16), 16, 64, pag->state.PV_adr, own_tim);

}

/**
 * -selects PV_param as prev param
 * -updates internal state of PV_page
 * -generates whole screen once
 * -puts the DDRAM adres on prev param adress
 * -writes whole screen to lcd
 *
 */
void PAGE_prev_param(Page *pag, TON *own_tim)
{
	//select new PV_param
	pag->state.PV_param = pag->state.PV_param->prev;

	//update IS
	pag->state.PV_adr = pag->state.PV_param->val_adr;
	if(pag->state.PV_adr > 64)
	{
		pag->state.PV_row = 1;
		pag->state.PV_col = 3;
	}
	else
	{
		pag->state.PV_row = 0;
		pag->state.PV_col = pag->state.PV_adr;
	}

	//Update PV_scr
	PAGE_generate_scr(pag);

	//Move cursor
	HW_set_cursor_DDRAM_adr(pag->state.PV_adr, own_tim);

	//Write whole screen
	HW_write_str_at_adr(pag->state.PV_scr, 16, 0, pag->state.PV_adr, own_tim);
	HW_write_str_at_adr((char*) (pag->state.PV_scr+16), 16, 64, pag->state.PV_adr, own_tim);
}

/*
 * This method reads pv values of parameters of
 * current page.
 * - Rode parameters are inserted into global read buffer
 *   at the positions with respect to global param table
 */
void PAGE_continously_read(Page *pag, uint32_t *global_read_buffer, TON *own_tim)
{
	//modality does not mater for reading parameters
	*(global_read_buffer + (pag->params[0]->param_idx) + 3*(pag->page_idx)) = pag->params[0]->val->PV;
	*(global_read_buffer + (pag->params[1]->param_idx) + 3*(pag->page_idx)) = pag->params[1]->val->PV;
	*(global_read_buffer + (pag->params[2]->param_idx) + 3*(pag->page_idx)) = pag->params[2]->val->PV;
}

/*
 * This method reads pv values of non-modal parameters of
 * current page from global_write_buffer.
 * - Rode parameters are inserted into corresponding
 *   val.PV fields of params on current page
 * - whole param member is updated, along with its val member
 * - updated parameter values are writen into lcd
 */
void PAGE_continously_write(Page *pag, uint32_t *global_write_buffer, TON *own_tim)
{
	uint32_t par_PV_val[3] = {0,0,0};
	uint8_t i = 0;

	//Store all valuest to be written in temp containters for clarity
	par_PV_val[0] = *(global_write_buffer + (pag->params[0]->param_idx) + 3*(pag->page_idx));
	par_PV_val[1] = *(global_write_buffer + (pag->params[1]->param_idx) + 3*(pag->page_idx));
	par_PV_val[2] = *(global_write_buffer + (pag->params[2]->param_idx) + 3*(pag->page_idx));

	//for every parameter on page...
	for(i = 0; i < 3; i++)
	{
		if(!(pag->params[i]->modality))		//if param is non-modal
		{
			pag->params[i]->val->PV = par_PV_val[i];
			pag->params[i]->digits = uint2str(par_PV_val[i], pag->params[i]->val->view);

			//terminate length of view in case of to long str
			if( (i == 0)  && ((pag->params[i]->digits) > 4))
			{
				pag->params[i]->digits = 4;
				pag->params[i]->val_adr = 3; 		//in case of my stupidyty...
			}
			if( (i == 1)  && ((pag->params[i]->digits) > 5))
			{
				pag->params[i]->digits = 5;
				pag->params[i]->val_adr = 11; 		//in case of my stupidyty...
			}

			if( (i == 2)  && ((pag->params[i]->digits) > 13))
			{
				pag->params[i]->digits = 13;
				pag->params[i]->val_adr = 67; 		//in case of my stupidyty...
			}

			//write parameters on page
			HW_write_str_at_adr(pag->params[i]->val->view, pag->params[i]->digits, pag->params[i]->val_adr, pag->state.PV_adr, own_tim);

		}
	}
}

/**
 * Fills screen with content stored in PV_scr of
 * PV_page internal state. It cares of updating
 * PV_screen before writing.
 */
void SCREEN_fill_with_page(Screen *scr, TON *own_tim)
{
	char *whole_content;// =   	"_FUCK OBJECT_____ORIENTED PROG._";
							//   0123456789ABCDEF0123456789ABCDEF

	//First, update screen field
	PAGE_generate_scr(scr->PV_pag);

	//store screen coontent
	whole_content = scr->PV_pag->state.PV_scr;

	//write first line
	HW_write_str_at_adr(whole_content, 16, 0, scr->PV_pag->state.PV_adr, own_tim);

	//write second line
	HW_write_str_at_adr((char*) (whole_content+16), 16, 64, scr->PV_pag->state.PV_adr, own_tim);


}

/**
 * Selects next page as visible page by updating
 * whole internal state of next page and setting screens
 * pv page as next page. At the end writes whole page on screen
 */
void SCREEN_next_page(Screen *scr, TON *own_tim)
{
	Page *next_pag = scr->PV_pag->next;				//new page preparation
	uint8_t curr_adr = scr->PV_pag->state.PV_adr;
	uint8_t curr_param_idx = scr->PV_pag->state.PV_param->param_idx;

	/* Start updating internal state of next page */

	//Leave cursor adr unchanged:
	next_pag->state.PV_adr = curr_adr;
	next_pag->state.PV_row = scr->PV_pag->state.PV_row;
	next_pag->state.PV_col = scr->PV_pag->state.PV_col;

	//set next page PV param as last pv param by idx
	next_pag->state.PV_param = (next_pag->params[curr_param_idx]);

	//Generate screen:
	PAGE_generate_scr(next_pag);

	/* Updating internal state of next page DONE */

	//scroll to next page
	scr->PV_pag = next_pag;

	//write screen content
	SCREEN_fill_with_page(scr, own_tim);


}

/**
 * Selects next page as visible page by updating
 * whole internal state of next page and setting screens
 * pv page as next page. At the end writes whole page on screen
 */
void SCREEN_prev_page(Screen *scr, TON *own_tim)
{
	Page *prev_pag = scr->PV_pag->next;				//new page preparation
	uint8_t curr_adr = scr->PV_pag->state.PV_adr;
	uint8_t curr_param_idx = scr->PV_pag->state.PV_param->param_idx;

	/* Start updating internal state of next page */

	//Leave cursor adr unchanged:
	prev_pag->state.PV_adr = curr_adr;
	prev_pag->state.PV_row = scr->PV_pag->state.PV_row;
	prev_pag->state.PV_col = scr->PV_pag->state.PV_col;

	//set next page PV param as last pv param by idx
	prev_pag->state.PV_param = (prev_pag->params[curr_param_idx]);

	//Generate screen:
	PAGE_generate_scr(prev_pag);

	/* Updating internal state of next page DONE */

	//scroll to next page
	scr->PV_pag = prev_pag;

	//write screen content
	SCREEN_fill_with_page(scr, own_tim);

}

/**
 * This method:
 * - initializes lcd display
 * - creates links between all pages linked list
 * - sets correct indices to every page
 * - links params and its values to global table of params and values
 * - creates links between all parameters
 * - creates valid internal state of every page
 * - reads all default values to the global read buffer
 * - selects first page as PV page and writes its content on lcd
 * - places cursor and sets DDRAM address on first parameter
 */
void SCREEN_init_all(Screen *scr, uint8_t pages_count, uint32_t *global_read_buffer, TON *own_tim)
{
	uint8_t i = 0;
	uint8_t j = 0;


	//Initialize LCD
	HW_init_lcd(own_tim);


	//For every page...
	for(i = 0; i < pages_count; i++)
	{
		//set the page index
		scr->pages[i].page_idx = i;

		//Set the liked list links for every page
		scr->pages[i].next = &(scr->pages[(i+1)%(pages_count)]);
		if(i==0)
		{
			scr->pages[i].prev = &(scr->pages[pages_count-1]);
		}
		else
		{
			scr->pages[i].prev = &(scr->pages[i-1]);
		}

		//for each param in every page...

		// link params to global table of params. Values are linked alredy
		scr->pages[i].params[0] = &(all_params[3*i + 0]);
		scr->pages[i].params[1] = &(all_params[3*i + 1]);
		scr->pages[i].params[2] = &(all_params[3*i + 2]);

		/* LINKING PARAMS NEIGHBOURS BEFORE LINKING PARAMS TO THEIR
		 * GLOBAL TABLE, RESUTS IN HARDWARE ERROR (STOP). LINK PARAMS TO THEIR
		 * GLOBAL TABLE FIRST, TO ENSURE THAT ALL PARAMETERS THAT ARE POINTERS
		 * AND HAS NOT RESERVED MEMORY, POINTS TO MEMORY LOCATION WITH RESERVED
		 * SPACE (E.G. GLOBAL LIST OF ALL PARAMS).
		 */
		for(j = 0; j < 3; j++)
		{
			//set the links between params
			scr->pages[i].params[j]->next = (scr->pages[i].params[(j+1)%3]);

			if(j==0)
			{
				scr->pages[i].params[j]->prev = (scr->pages[i].params[2]);
			}
			else
			{
				scr->pages[i].params[j]->prev = (scr->pages[i].params[j-1]);
			}


		}

		//create valid internal state of page
		scr->pages[i].state.PV_row = 0;
		scr->pages[i].state.PV_col = 3;
		scr->pages[i].state.PV_adr = scr->pages[i].params[0]->val_adr = IS_calc_adr(scr->pages[i].state.PV_row, scr->pages[i].state.PV_col);
		scr->pages[i].state.PV_param = scr->pages[i].params[0];
		PAGE_generate_scr(&(scr->pages[i]));  //this will update state.PV_scr

		//Fill the global read buffer with initial values
		PAGE_continously_read(&(scr->pages[i]), global_read_buffer, own_tim);
	}

	//Select PV page as first page
	scr->PV_pag = &(scr->pages[0]);

	//Set cursor and DDRAM addres to adres of first parameter
	HW_set_cursor_DDRAM_adr(scr->PV_pag->params[0]->val_adr, own_tim);

}

/*
 * This function performs Next_param
 * action at the event of rising edge on pin
 * NEXT_PAR_PIN
 */
void SCREEN_buttonNextParam_RE(Screen *scr, TON *own_tim)
{
	static GPIO_PinState lastBtnState = GPIO_PIN_RESET;
	GPIO_PinState curr_btnState = GPIO_PIN_RESET;

	curr_btnState = HAL_GPIO_ReadPin(GPIOC, NEXT_PAR_PIN);
	//Rising edge!
	if((lastBtnState == GPIO_PIN_RESET) && (curr_btnState == GPIO_PIN_SET))
	{
		PAGE_next_param(scr->PV_pag, own_tim);
	}
	lastBtnState = curr_btnState;
}

/*
 * This function performs Prev_param
 * action at the event of rising edge on pin
 * PREV_PAR_PIN
 */
void SCREEN_buttonPrevParam_RE(Screen *scr, TON *own_tim)
{
	static GPIO_PinState lastBtnState = GPIO_PIN_RESET;
	GPIO_PinState curr_btnState = GPIO_PIN_RESET;

	curr_btnState = HAL_GPIO_ReadPin(GPIOC, PREV_PAR_PIN);
	//Rising edge!
	if((lastBtnState == GPIO_PIN_RESET) && (curr_btnState == GPIO_PIN_SET))
	{
		PAGE_prev_param(scr->PV_pag, own_tim);
	}
	lastBtnState = curr_btnState;
}

/*
 * This function performs Next_page action
 * at the event of rising edge on pin
 * NEXT_PAG_PIN
 */
void SCREEN_buttonNextPage_RE(Screen *scr, TON *own_tim)
{
	static GPIO_PinState lastBtnState = GPIO_PIN_RESET;
	GPIO_PinState curr_btnState = GPIO_PIN_RESET;

	curr_btnState = HAL_GPIO_ReadPin(GPIOC, NEXT_PAG_PIN);
	//Rising edge!
	if((lastBtnState == GPIO_PIN_RESET) && (curr_btnState == GPIO_PIN_SET))
	{
		SCREEN_next_page(scr, own_tim);
	}
	lastBtnState = curr_btnState;
}

/*
 * This function performs Next_page action
 * at the event of rising edge on pin
 * NEXT_PAG_PIN
 */
void SCREEN_buttonPrevPage_RE(Screen *scr, TON *own_tim)
{
	static GPIO_PinState lastBtnState = GPIO_PIN_RESET;
	GPIO_PinState curr_btnState = GPIO_PIN_RESET;

	curr_btnState = HAL_GPIO_ReadPin(GPIOC, PREV_PAG_PIN);
	//Rising edge!
	if((lastBtnState == GPIO_PIN_RESET) && (curr_btnState == GPIO_PIN_SET))
	{
		SCREEN_prev_page(scr, own_tim);
	}
	lastBtnState = curr_btnState;
}

/*
 * This function performs action Next_value on current page
 * at the event of rising edge on pin NEXT_VAL_PIN.
 */
void SCREEN_buttonNextValue_RE(Screen *scr, TON *own_tim)
{
	static GPIO_PinState lastBtnState = GPIO_PIN_RESET;
	GPIO_PinState curr_btnState = GPIO_PIN_RESET;

	curr_btnState = HAL_GPIO_ReadPin(GPIOC, NEXT_VAL_PIN);
	//Rising edge!
	if((lastBtnState == GPIO_PIN_RESET) && (curr_btnState == GPIO_PIN_SET))
	{
		PAGE_next_par_value(scr->PV_pag, own_tim);
	}
	lastBtnState = curr_btnState;
}

/*
 * This function performs action Prev_value on current page
 * at the event of rising edge on pin PREV_VAL_PIN.
 */
void SCREEN_buttonPrevValue_RE(Screen *scr, TON *own_tim)
{
	static GPIO_PinState lastBtnState = GPIO_PIN_RESET;
	GPIO_PinState curr_btnState = GPIO_PIN_RESET;

	curr_btnState = HAL_GPIO_ReadPin(GPIOC, PREV_VAL_PIN);
	//Rising edge!
	if((lastBtnState == GPIO_PIN_RESET) && (curr_btnState == GPIO_PIN_SET))
	{
		PAGE_prev_par_value(scr->PV_pag, own_tim);
	}
	lastBtnState = curr_btnState;
}



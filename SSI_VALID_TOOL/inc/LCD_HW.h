/*
 * LCD_HW.h
 *
 *  Created on: 01.05.2018
 *      Author: Igor
 */

#ifndef LCD_HW_H_
#define LCD_HW_H_
#endif /* LCD_HW_H_ */

#include "stm32f1xx.h"
#include "utils.h"

#define DB0 GPIO_PIN_0
#define DB1 GPIO_PIN_1
#define DB2 GPIO_PIN_2
#define DB3 GPIO_PIN_14
#define DB4 GPIO_PIN_15
#define DB5 GPIO_PIN_5
#define DB6 GPIO_PIN_6
#define DB7 GPIO_PIN_7

#define RS GPIO_PIN_8
#define RW GPIO_PIN_9
#define E GPIO_PIN_10

#define NEXT_PAR_PIN GPIO_PIN_13
#define NEXT_PAG_PIN GPIO_PIN_5
#define NEXT_VAL_PIN GPIO_PIN_6
#define PREV_PAR_PIN GPIO_PIN_0
#define PREV_PAG_PIN GPIO_PIN_0
#define PREV_VAL_PIN GPIO_PIN_0


/**
 * PV: numerical/boolean value of param value
 * view: string viewing the value on screen - has len of its param .digit member
 * next: linked list next value if enum suported, if not it is NULL
 * prev: linked list prev value if enum suported, if not it is NULL
 */
typedef struct val_s
{
	uint32_t PV;
	char view[13];
	uint8_t val_enum;
	uint8_t enum_range;
	uint32_t LUT_val[16];
	char LUT_view[16][13];
} Value;

/**
 * name: 2 char name seen on LCD
 * val:  value of parameter
 * modality: 0 - read only, !0 - read/write
 * digits: number of digits on LCD
 * val_adr: LCD RAM address on which param value starts to apear
 * next: linked list next param
 * prev: linked list prev param
 */
typedef struct param_s
{
	char name[2];
	Value *val;
	uint8_t modality;
	uint8_t digits;
	uint8_t val_adr;
	uint8_t param_idx;
	struct param_s *next;
	struct param_s *prev;
} Param;

typedef struct internal_state_s
{
	uint8_t PV_row;
	uint8_t PV_col;
	uint8_t PV_adr;
	Param *PV_param;
	char PV_scr[32];
} Internal_state;

typedef struct page_s
{
	Param *(params[3]);
	Internal_state state;
	uint8_t page_idx;
	struct page_s *next;
	struct page_s *prev;
} Page;

typedef struct scr_s
{
	Page pages[16];
	Page *PV_pag;
} Screen;

void PAGE_generate_scr(Page *pag);
uint8_t IS_calc_adr(uint8_t row, uint8_t col);

void PAGE_next_par_value(Page *pag, TON *own_tim);
void PAGE_prev_par_value(Page *pag, TON *own_tim);

void PAGE_next_param(Page *pag, TON *own_tim);
void PAGE_prev_param(Page *pag, TON *own_tim);
void PAGE_continously_read(Page *pag, uint32_t *global_read_buffer, TON *own_tim);
void PAGE_continously_write(Page *pag, uint32_t *global_write_buffer, TON *own_tim);

void SCREEN_next_page(Screen *scr, TON *own_tim);
void SCREEN_prev_page(Screen *scr, TON *own_tim);
void SCREEN_fill_with_page(Screen *scr, TON *own_tim);
void SCREEN_init_all(Screen *scr, uint8_t pages_count, uint32_t *global_read_buffer, TON *own_tim);

void SCREEN_buttonNextParam_RE(Screen *scr, TON *own_tim);
void SCREEN_buttonPrevParam_RE(Screen *scr, TON *own_tim);
void SCREEN_buttonNextPage_RE(Screen *scr, TON *own_tim);
void SCREEN_buttonPrevPage_RE(Screen *scr, TON *own_tim);
void SCREEN_buttonNextValue_RE(Screen *scr, TON *own_tim);
void SCREEN_buttonPrevValue_RE(Screen *scr, TON *own_tim);

void HW_init_lcd(TON *own_tim);
void HW_clr_scr(TON *own_tim);
void HW_set_cursor_DDRAM_adr(uint8_t adr, TON *own_tim);
void HW_write_str_at_adr(char *str, uint8_t len, uint8_t adr, uint8_t curr_DDRAM_adr, TON *own_tim);







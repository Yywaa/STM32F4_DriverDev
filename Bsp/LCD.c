/*
 * LCD.c
 *
 *  Created on: Nov 23, 2024
 *      Author: yywvi
 */

#include "LCD.h"


static void write_4_bits(uint8_t value);
static void lcd_enable(void);
static void mdelay(uint32_t cnt);
static void udelay(uint32_t cnt);
//static void lcd_display_clear(void);

void lcd_send_command(uint8_t cmd)
{
	/*RS = 0 for LCD command*/
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

	/*R/W = 0, for write*/
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	write_4_bits(cmd >> 4);
	write_4_bits(cmd & 0xf);

}

/*
 * This function sends data to LCD
 * Here we used 4 bits parallel data transmission
 * First higher nibble of the data will be sent on to data line D4, D5, D6,D7
 * Then lower nibble of the data will be sent to the data lines, D4, D5, D6, D7
 */

void lcd_print_char(uint8_t data)
{
	/*RS = 1 for LCD user data*/
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_SET);

	/*R/w = 0,for write*/
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);
	write_4_bits(data >> 4); /*Higher nibble*/
	write_4_bits(data & 0x0F); /*Lower nibble*/


}

void lcd_print_string(char *message)
{
	do{
		lcd_print_char((uint8_t)*message++);
	}while(*message != '\0');
}


void lcd_init(void)
{
	//1,configure the gpio pins which are used for
	GPIO_Handle_t lcd_signal;

	lcd_signal.pGPIOx = LCD_GPIO_PORT;
	lcd_signal.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	lcd_signal.GPIO_PinConfig.GPIO_PinNum = LCD_GPIO_RS;
	lcd_signal.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	lcd_signal.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;
	lcd_signal.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&lcd_signal);


	lcd_signal.GPIO_PinConfig.GPIO_PinNum = LCD_GPIO_RW;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNum = LCD_GPIO_EN;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNum = LCD_GPIO_D4;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNum = LCD_GPIO_D5;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNum = LCD_GPIO_D6;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNum = LCD_GPIO_D7;
	GPIO_Init(&lcd_signal);

    //Keep all the pin in logic 0
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, GPIO_PIN_RESET);

	//2,Do the LCD initialization
	mdelay(40); //40ms

	/*RS = 0, For LCD command*/
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

	/*Rw = 0,writing to LCD*/
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	write_4_bits(0x3);// 0 0 1 1,

	mdelay(5);//wait 5ms

	write_4_bits(0x3);// 0 0 1 1,

	udelay(150);//wait 100us

	write_4_bits(0x3);
	write_4_bits(0x2);

	//function set command
	lcd_send_command(LCD_CMD_4DL_2N_5X8F);

	//display ON and cursor ON
	lcd_send_command(LCD_CMD_DON_CURON);

	//clear
	lcd_display_clear();

	//entry mode set
	lcd_send_command(LCD_CMD_INCADD);

}

//writes 4 bits of data/command on to D4,D5,D6,D7 lines
static void write_4_bits(uint8_t value)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, ((value >> 0)& 0x1));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, ((value >> 1)& 0x1));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, ((value >> 2)& 0x1));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, ((value >> 3)& 0x1));

	lcd_enable();
}


void lcd_display_clear(void)
{
	//display clear
	lcd_send_command(LCD_CMD_DIS_CLEAR);

	mdelay(2);

}

/*Cursor returns to home position*/
void lcd_display_return_home(void)
{
	lcd_send_command(LCD_CMD_DIS_RETURN_HOME);
	/*
	 * return home command wait time is around 2ms
	 */
	mdelay(2);

}

static void lcd_enable(void)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_SET);
	udelay(10);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	udelay(100);//wait time should be larger than 38 micro secondss
}

static void mdelay(uint32_t cnt)
{
	for(uint32_t i = 0;i < (cnt * 1000);i++);
}

static void udelay(uint32_t cnt)
{
	for(uint32_t i = 0; i <(cnt *1); i++);
}

/*
 * set Lcd to a specified location given by row and column information
 * row number (1 or 2)
 * Column number (1 to 16),Assuming a 2*16 characters display
 */

void lcd_set_cursor(uint8_t row,uint8_t column)
{
	column--;
	switch(row)
	{
	case 1:
		/*Set the cursor to 1st row address and add index*/
		lcd_send_command((column|=0x80));
		break;
	case 2:
		/*Set the cursor to 2nd row address and add index*/
		lcd_send_command((column|=0xC0));
		break;
	default:
		break;
	}

}



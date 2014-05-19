/**
 * @file LCD2x16_driver.c
 * @brief LCD2x16 Driver for Microchip PIC18F4550 MCU.
 *
 * Date: 02-02-2012
 * License: BSD 3-Clause
 * 
 * Copyright (c) 2012-2014, Pedro A. Hortas (pah@ucodev.org)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  - Neither the name of the ucodev.org nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * 
 *
 * Project details:
 *
 *  Home Page:   http://www.ucodev.org
 *  Version:     0.1
 *  Portability: Microchip PIC* MCUs
 *  Description: LCD (2x16) Driver
 *  Notes:       Delay functions must be implemented based on the instruction set documentation and must match the
 *               frequency of the chipset being used.
 *               PORT definitions must be changed on LCD2x16_driver.h file.
 *
 */

#include "LCD2x16_driver.h"

BYTE LCD_get_BF(void) {
	BYTE low,high;
	
	lcd_DB4_tris = 1;
	lcd_DB5_tris = 1;
	lcd_DB6_tris = 1;
	lcd_DB7_tris = 1;

	lcd_RS = 0;
	lcd_RW = 1;

	delay_1us();

	lcd_E = 1;

	delay_1us();
	delay_1us();
	
	high = ((lcd_DB7_in << 3) | (lcd_DB6_in << 2) |
			(lcd_DB5_in << 1) | lcd_DB4_in);

	lcd_E = 0;
	
	delay_1us();
	delay_1us();

	lcd_E = 1;

	delay_1us();
	delay_1us();

	low = ((lcd_DB7_in << 3) | (lcd_DB6_in << 2) |
			(lcd_DB5_in << 1) | lcd_DB4_in);

	lcd_E = 0;

	delay_1us();
	delay_1us();

	lcd_DB4_tris = 0;
	lcd_DB5_tris = 0;
	lcd_DB6_tris = 0;
	lcd_DB7_tris = 0;

	delay_10us(5);

	return ((high << 4) | low);
}

void LCD_4write(BYTE RS, BYTE RW, BYTE DB7, BYTE DB6, BYTE DB5, BYTE DB4) {
	lcd_DB4_tris = 0;
	lcd_DB5_tris = 0;
	lcd_DB6_tris = 0;
	lcd_DB7_tris = 0;

	lcd_RS_tris = 0;
	lcd_RW_tris = 0;
	lcd_E_tris = 0;

	lcd_RW = RW;
	lcd_RS = RS;

	lcd_DB7_out = DB7;
	lcd_DB6_out = DB6;
	lcd_DB5_out = DB5;
	lcd_DB4_out = DB4;

	delay_1us();
	delay_1us();

	lcd_E = 1;
	delay_1us();
	delay_1us();
	lcd_E = 0;

	delay_1us();
	delay_1us();
}

void LCD_8write(BYTE RS, BYTE RW, BYTE DB7, BYTE DB6, BYTE DB5, BYTE DB4,
                BYTE DB3, BYTE DB2, BYTE DB1, BYTE DB0) {
	lcd_DB4_tris = 0;
	lcd_DB5_tris = 0;
	lcd_DB6_tris = 0;
	lcd_DB7_tris = 0;

	lcd_RS_tris = 0;
	lcd_RW_tris = 0;
	lcd_E_tris = 0;

	lcd_RW = RW;
	lcd_RS = RS;

	lcd_DB7_out = DB7;
	lcd_DB6_out = DB6;
	lcd_DB5_out = DB5;
	lcd_DB4_out = DB4;

	delay_1us();
	delay_1us();

	lcd_E = 1;

	delay_1us();
	delay_1us();

	lcd_E = 0;

	delay_1us();
	delay_1us();

	lcd_DB7_out = DB3;
	lcd_DB6_out = DB2;
	lcd_DB5_out = DB1;
	lcd_DB4_out = DB0;

	delay_1us();
	delay_1us();

	lcd_E = 1;

	delay_1us();
	delay_1us();

	lcd_E = 0;

	delay_1us();
	delay_1us();
}

void LCD_write_byte(BYTE RS, BYTE ch) {
	/* Wait for any instruction being performed on LCD */
	while ((LCD_get_BF() & 0x80) == 1) ;

	/* Wait done... continuing ... */
	lcd_DB4_tris = 0;
	lcd_DB5_tris = 0;
	lcd_DB6_tris = 0;
	lcd_DB7_tris = 0;

	lcd_RS_tris = 0;
	lcd_RW_tris = 0;
	lcd_E_tris = 0;

	lcd_RW = 0;
	lcd_RS = RS;

	lcd_DB7_out = ((ch & 0x80) == 0x80);
	lcd_DB6_out = ((ch & 0x40) == 0x40);
	lcd_DB5_out = ((ch & 0x20) == 0x20);
	lcd_DB4_out = ((ch & 0x10) == 0x10);

	delay_1us();
	delay_1us();

	lcd_E = 1;

	delay_1us();
	delay_1us();

	lcd_E = 0;

	delay_1us();
	delay_1us();

	lcd_DB7_out = ((ch & 0x08) == 0x08);
	lcd_DB6_out = ((ch & 0x04) == 0x04);
	lcd_DB5_out = ((ch & 0x02) == 0x02);
	lcd_DB4_out = ((ch & 0x01) == 0x01);

	delay_1us();
	delay_1us();

	lcd_E = 1;

	delay_1us();
	delay_1us();

	lcd_E = 0;

	delay_10us(5);
}

void LCD_goto_xy(BYTE x, BYTE y) {
	LCD_write_byte(0, (0x80 | (((y > 1) ? 0x40 : 0) + (x - 1))));
	delay_10us(5);
}

void LCD_write_line(BYTE nr, unsigned char *line) {
	BYTE i;

	LCD_goto_xy(1, nr);

	for (i = 0; (i < 16) && line[i]; i++) {
		LCD_write_byte(1, line[i]);
	}

}

void LCD_clear(void) {
	LCD_8write(0, 0, 0, 0, 0, 0, 0, 0, 0, 1);
	delay_1ms(5);
}

void LCD_init(void) {
        lcd_DB0_tris = 0;
	lcd_DB1_tris = 0;
	lcd_DB2_tris = 0;
	lcd_DB3_tris = 0;
	lcd_DB4_tris = 0;	// Data0
	lcd_DB5_tris = 0;	// Data1
	lcd_DB6_tris = 0;	// Data2
	lcd_DB7_tris = 0;	// Data3
	lcd_RW_tris = 0;
	lcd_RS_tris = 0;
	lcd_E_tris = 0;

	lcd_E = 0;
	lcd_RW = 0;

	delay_1ms(40);	// > 40ms Init

	LCD_4write(0, 0, 0, 0, 1, 1);

	delay_1ms(5);

	LCD_4write(0, 0, 0, 0, 1, 1);

	delay_1ms(5);

	LCD_4write(0, 0, 0, 0, 1, 1); // Function set - 8 bits interface

	delay_1ms(5);

	LCD_4write(0, 0, 0, 0, 1, 0); 

	delay_10us(25);

        /* DL is 4bits, N is 2 row, F is 0 */
	LCD_8write(0, 0, 0, 0, 1, 0, 1, 0, 0, 0);

	delay_10us(25);

	/* Diplay on, Cursor, Blink */
	LCD_8write(0, 0, 0, 0, 0, 0, 1, 1, 1, 1);

	delay_10us(25);

	/* Clear LCD */
	LCD_8write(0, 0, 0, 0, 0, 0, 0, 0, 0, 1);

	delay_1ms(25);

	/* Set Entry Mode */
	LCD_8write(0, 0, 0, 0, 0, 0, 0, 1, 1, 0);

	delay_1ms(25);

	while ((LCD_get_BF() & 0x80) == 1) ;

	delay_1ms(40);
}

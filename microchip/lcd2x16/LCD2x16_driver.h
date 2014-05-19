/**
 * @file LCD2x16_driver.h
 * @brief Header file for LCD2x16 Driver for Microchip PIC18F4550 MCU.
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


#ifndef LCD_DRIVER_H
#define LCD_DRIVER_H

#include "GenericTypeDefs.h"

/* Port Configuration */
#define init_LCD_pins()			TRISB &= 0xC0; LATB &= 0xC0; TRISC &= 0x38; LATC &= 0x38;

#define lcd_RS				LATBbits.LATB0
#define lcd_RW				LATBbits.LATB1
#define lcd_E				LATCbits.LATC2

#define lcd_DB4_out			LATBbits.LATB2
#define lcd_DB5_out			LATBbits.LATB3
#define lcd_DB6_out			LATBbits.LATB4
#define lcd_DB7_out			LATBbits.LATB5

#define lcd_DB4_in			PORTBbits.RB2
#define lcd_DB5_in			PORTBbits.RB3
#define lcd_DB6_in			PORTBbits.RB4
#define lcd_DB7_in			PORTBbits.RB5

#define lcd_RS_tris			TRISBbits.TRISB0
#define lcd_RW_tris			TRISBbits.TRISB1
#define lcd_E_tris			TRISCbits.TRISC2
#define lcd_DB0_tris			TRISCbits.TRISC6
#define lcd_DB1_tris			TRISCbits.TRISC7
#define lcd_DB2_tris			TRISBbits.TRISB0
#define lcd_DB3_tris			TRISBbits.TRISB1
#define lcd_DB4_tris			TRISBbits.TRISB2
#define lcd_DB5_tris			TRISBbits.TRISB3
#define lcd_DB6_tris			TRISBbits.TRISB4
#define lcd_DB7_tris			TRISBbits.TRISB5

/* Prototypes */
BYTE LCD_get_BF(void);
void LCD_4write(BYTE RS, BYTE RW, BYTE DB7, BYTE DB6, BYTE DB5, BYTE DB4);
void LCD_8write(BYTE RS, BYTE RW, BYTE DB7, BYTE DB6, BYTE DB5, BYTE DB4, BYTE DB3, BYTE DB2, BYTE DB1, BYTE DB0);
void LCD_write_byte(BYTE RS, BYTE ch);
void LCD_goto_xy(BYTE x, BYTE y);
void LCD_write_line(BYTE nr, unsigned char *line);
void LCD_clear(void);
void LCD_init(void);


#endif

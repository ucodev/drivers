/**
 * @file SRAM4040_driver.h
 * @brief Header for SRAM4040 Driver for Microchip PIC18F4550 MCU.
 *
 * Date: 15-03-2012
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
 *  Description: SRAM (4040 as controller) Driver
 *  Notes:       Delay functions must be implemented based on the instruction set documentation and must match the
 *               frequency of the chipset being used.
 *               PORT definitions must be changed on SRAM4040_driver.h file.
 *
 */


#ifndef SRAM_DRIVER_H_
#define SRAM_DRIVER_H_

/* Port Configuration */
#define SRAM_D0_out			LATAbits.LATA1
#define SRAM_D1_out			LATAbits.LATA2
#define SRAM_D2_out			LATAbits.LATA3
#define SRAM_D3_out			LATAbits.LATA4
#define SRAM_D4_out			LATAbits.LATA5
#define SRAM_D5_out			LATEbits.LATE0
#define SRAM_D6_out			LATEbits.LATE1
#define SRAM_D7_out			LATEbits.LATE2

#define SRAM_D0_in			PORTAbits.RA1
#define SRAM_D1_in			PORTAbits.RA2
#define SRAM_D2_in			PORTAbits.RA3
#define SRAM_D3_in			PORTAbits.RA4
#define SRAM_D4_in			PORTAbits.RA5
#define SRAM_D5_in			PORTEbits.RE0
#define SRAM_D6_in			PORTEbits.RE1
#define SRAM_D7_in			PORTEbits.RE2

#define SRAM_D0_tris			TRISAbits.TRISA1
#define SRAM_D1_tris			TRISAbits.TRISA2
#define SRAM_D2_tris			TRISAbits.TRISA3
#define SRAM_D3_tris			TRISAbits.TRISA4
#define SRAM_D4_tris			TRISAbits.TRISA5
#define SRAM_D5_tris			TRISEbits.TRISE0
#define SRAM_D6_tris			TRISEbits.TRISE1
#define SRAM_D7_tris			TRISEbits.TRISE2
	
/** SRAM address counter ticker */
#define SRAM_ADDR_CNTR_TCK_pin		LATCbits.LATC6
#define SRAM_ADDR_CNTR_TCK_tris		TRISCbits.TRISC6

/** SRAM address counter reset */
#define SRAM_ADDR_CNTR_RST_pin          LATCbits.LATC7
#define SRAM_ADDR_CNTR_RST_tris         TRISCbits.TRISC7
	
/** SRAM control pins */
#define SRAM_CS_pin			LATDbits.LATD7
#define SRAM_OE_pin			LATDbits.LATD0
#define SRAM_WE_pin			LATDbits.LATD1
	
#define SRAM_CS_tris			TRISDbits.TRISD7
#define SRAM_OE_tris			TRISDbits.TRISD0
#define SRAM_WE_tris			TRISDbits.TRISD1


/* Prototypes */
void SRAM_init(void);
void SRAM_reset_addr(void);
void SRAM_inc_addr(void);
void SRAM_set_addr(unsigned int addr);
BYTE SRAM_read_addr(unsigned int addr);
BYTE SRAM_read_cur_addr(void);
void SRAM_write_addr(unsigned int addr, BYTE b);
void SRAM_write_cur_addr(BYTE b);

#endif

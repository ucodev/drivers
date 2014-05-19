/**
 * @file SRAM4040_driver.c
 * @brief SRAM4040 Driver for Microchip PIC18F4550 MCU.
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


#include "SRAM_driver.h"

void SRAM_init(void) {
	SRAM_D0_tris = 0;
	SRAM_D1_tris = 0;
	SRAM_D2_tris = 0;
	SRAM_D3_tris = 0;
	SRAM_D4_tris = 0;
	SRAM_D5_tris = 0;
	SRAM_D6_tris = 0;
	SRAM_D7_tris = 0;

	SRAM_D0_out = 0;
	SRAM_D1_out = 0;
	SRAM_D2_out = 0;
	SRAM_D3_out = 0;
	SRAM_D4_out = 0;
	SRAM_D5_out = 0;
	SRAM_D6_out = 0;
	SRAM_D7_out = 0;

	SRAM_ADDR_CNTR_TCK_tris = 0;
	SRAM_ADDR_CNTR_TCK_pin = 0;

	SRAM_ADDR_CNTR_RST_tris = 0;
	SRAM_ADDR_CNTR_RST_pin = 0;

	SRAM_CS_tris = 0;
	SRAM_OE_tris = 0;
	SRAM_WE_tris = 0;

	SRAM_CS_pin = 1;	// When CS=1 -> I/O=HighZ, OE=deselect, WE=standby
	SRAM_OE_pin = 1;
	SRAM_WE_pin = 1;

	/* Set address pointer to 0 */
	SRAM_set_addr(0);
}

void SRAM_reset_addr(void) {
	/* Reset the CMOS 4040 counter */
	SRAM_ADDR_CNTR_RST_pin = 1;

	/* 3 nop + 1 instruction (var assignment):
	 * (1 / (4 * 0.08333us)) * 2 = 1.5MHz
	 * Note that we multiply by 2 because the value is for
	 * half period of the square wave.
	 */
	Nop();
	Nop();
	Nop();

	SRAM_ADDR_CNTR_RST_pin = 0;

	Nop();
}

void SRAM_inc_addr(void) {
	SRAM_ADDR_CNTR_TCK_pin = 1;

	Nop();
	Nop();
	Nop();

	SRAM_ADDR_CNTR_TCK_pin = 0;

	Nop();
}

void SRAM_set_addr(unsigned int addr) {
	unsigned int i;

	/* Reset the CMOS 4040 counter */
	SRAM_ADDR_CNTR_RST_pin = 1;

	/* 3 nop + 1 instruction (var assignment):
	 * (1 / (4 * 0.08333us)) * 2 = 1.5MHz
	 * Note that we multiply by 2 because the value is for
	 * half period of the square wave.
	 */
	Nop();
	Nop();
	Nop();

	SRAM_ADDR_CNTR_RST_pin = 0;

	/* 1 Nop + 1 assignment (i = 0) + 1 comparator (i != addr)
	 * + 1 assignment (SRAM_ADDR_CNTR_TCK_pin = 1)
         * = 4 instuctions * 0.08333us = half wave of 1.5MHz
	 */
	Nop();

	/* Start counting. Each count is updated on the falling-edge
	 * of the clock (normal operation of ripple counters).
	 * So we need to bring the pin ON and then OFF to increment
	 * the CMOS 4040 counter by 1.
	 * This allow us to connect multiple counters in series by
	 * directly connect the MSB of the first counter to the LSB
	 * of the second, and so on...
	 */
	for (i = 0; i != addr; i++) {
		SRAM_ADDR_CNTR_TCK_pin = 1;

		/* 7 nop = 7 * 0.08333us + 1 instruction
                 * for the var assignment we have:
		 * (7 * 0.08333us) + (1 * 0.08333us) = 0.66666us
		 * Which is a frequency of 1.5MHz for the 4040.
		 * At 5V the minimum freq should be 1.5MHz so we should
		 * have no problem here with this latency.
		 */
		Nop();
		Nop();
		Nop();
	
		SRAM_ADDR_CNTR_TCK_pin = 0;

		/* 1 Nop + 1 increment (i++) + 1 comparator (i != addr)
		 * + 1 assignment (SRAM_ADDR_CNTR_TCK_pin = 1)
		 * = 4 instuctions * 0.08333us = half wave of 1.5MHz 
		 */

		Nop();
	}
}

BYTE SRAM_read_addr(unsigned int addr) {
	BYTE ret;

	SRAM_WE_pin = 1;
	SRAM_OE_pin = 0;

	SRAM_set_addr(addr);

	SRAM_D0_tris = 1;
	SRAM_D1_tris = 1;
	SRAM_D2_tris = 1;
	SRAM_D3_tris = 1;
	SRAM_D4_tris = 1;
	SRAM_D5_tris = 1;
	SRAM_D6_tris = 1;
	SRAM_D7_tris = 1;

	SRAM_CS_pin = 0;

	Nop();
	Nop();

	ret = ((SRAM_D7_in << 7) | (SRAM_D6_in << 6) |
			(SRAM_D5_in << 5) | (SRAM_D4_in << 4) |
			(SRAM_D3_in << 3) | (SRAM_D2_in << 2) |
			(SRAM_D1_in << 1) | (SRAM_D0_in));

	SRAM_CS_pin = 1;
	SRAM_OE_pin = 1;

	return ret;
}

BYTE SRAM_read_cur_addr(void) {
	BYTE ret;

	SRAM_WE_pin = 1;
	SRAM_OE_pin = 0;

	SRAM_D0_tris = 1;
	SRAM_D1_tris = 1;
	SRAM_D2_tris = 1;
	SRAM_D3_tris = 1;
	SRAM_D4_tris = 1;
	SRAM_D5_tris = 1;
	SRAM_D6_tris = 1;
	SRAM_D7_tris = 1;

	SRAM_CS_pin = 0;

	Nop();
	Nop();

	ret = ((SRAM_D7_in << 7) | (SRAM_D6_in << 6) |
			(SRAM_D5_in << 5) | (SRAM_D4_in << 4) |
			(SRAM_D3_in << 3) | (SRAM_D2_in << 2) |
			(SRAM_D1_in << 1) | (SRAM_D0_in));

	SRAM_CS_pin = 1;
	SRAM_OE_pin = 1;

	return ret;
}

void SRAM_write_addr(unsigned int addr, BYTE b) {
	SRAM_OE_pin = 1;

	SRAM_D0_tris = 0;
	SRAM_D1_tris = 0;
	SRAM_D2_tris = 0;
	SRAM_D3_tris = 0;
	SRAM_D4_tris = 0;
	SRAM_D5_tris = 0;
	SRAM_D6_tris = 0;
	SRAM_D7_tris = 0;

	SRAM_D7_out = ((b & 0x80) == 0x80);
	SRAM_D6_out = ((b & 0x40) == 0x40);
	SRAM_D5_out = ((b & 0x20) == 0x20);
	SRAM_D4_out = ((b & 0x10) == 0x10);
	SRAM_D3_out = ((b & 0x08) == 0x08);
	SRAM_D2_out = ((b & 0x04) == 0x04);
	SRAM_D1_out = ((b & 0x02) == 0x02);
	SRAM_D0_out = ((b & 0x01) == 0x01);

	SRAM_set_addr(addr);

	Nop();
	Nop();

	SRAM_CS_pin = 0;
	SRAM_WE_pin = 0;

	Nop();
	Nop();

	SRAM_WE_pin = 1;
	SRAM_CS_pin = 1;
}

void SRAM_write_cur_addr(BYTE b) {
	SRAM_OE_pin = 1;

	SRAM_D0_tris = 0;
	SRAM_D1_tris = 0;
	SRAM_D2_tris = 0;
	SRAM_D3_tris = 0;
	SRAM_D4_tris = 0;
	SRAM_D5_tris = 0;
	SRAM_D6_tris = 0;
	SRAM_D7_tris = 0;

	SRAM_D7_out = ((b & 0x80) == 0x80);
	SRAM_D6_out = ((b & 0x40) == 0x40);
	SRAM_D5_out = ((b & 0x20) == 0x20);
	SRAM_D4_out = ((b & 0x10) == 0x10);
	SRAM_D3_out = ((b & 0x08) == 0x08);
	SRAM_D2_out = ((b & 0x04) == 0x04);
	SRAM_D1_out = ((b & 0x02) == 0x02);
	SRAM_D0_out = ((b & 0x01) == 0x01);

	SRAM_CS_pin = 0;
	SRAM_WE_pin = 0;

	Nop();
	Nop();

	SRAM_WE_pin = 1;
	SRAM_CS_pin = 1;
}

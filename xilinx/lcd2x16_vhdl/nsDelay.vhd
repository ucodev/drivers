---
-- @file nsDelay.vhd
-- @brief LCD 2x16 Driver for Xilinx FPGAs.
--
-- Date: 10-10-2010
-- License: BSD 3-Clause
-- 
-- Copyright (c) 2010-2014, Pedro A. Hortas (pah@ucodev.org)
-- All rights reserved.
--
-- Redistribution and use in source and binary forms, with or without
-- modification, are permitted provided that the following conditions are met:
--
--  - Redistributions of source code must retain the above copyright notice,
--    this list of conditions and the following disclaimer.
--
--  - Redistributions in binary form must reproduce the above copyright notice,
--    this list of conditions and the following disclaimer in the documentation
--    and/or other materials provided with the distribution.
--
--  - Neither the name of the ucodev.org nor the names of its contributors
--    may be used to endorse or promote products derived from this software
--    without specific prior written permission.
--
-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
-- AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
-- IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
-- ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
-- LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
-- CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
-- SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
-- INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
-- CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
-- ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
-- POSSIBILITY OF SUCH DAMAGE.
-- 
-- Company:	uCodev 
-- Engineer: 	Pedro A. Hortas
-- 
-- Create Date:    00:52:25 10/10/2010 
-- Design Name: 
-- Module Name:    nsDelay - Time_delay 
-- Project Name: 
-- Target Devices: 
-- Tool versions: 
-- Description: 
--
-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
--
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Use MATH_REAL for LOG() and CEIL() functions
use IEEE.MATH_REAL.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity tickDelay is
	generic(
		
		-- CLOCK: integer := 50000000;	-- Clock given in Hz. 50MHz default value
		-- STEP: integer := integer(CEIL(real((1 / M) * 1000000000)));	-- Delay Step. 20ns default value (minimum unit and base step)
		-- DELAY: integer := 20;	-- Number of nanoseconds to delay
		-- TICKS: integer := integer(CEIL(real(DELAY / (real((1 / CLOCK) * 1000000000)))))		-- Number of ticks to elapse before port 'tick' is set to 1.
		N: integer := 4;			-- Number of bits needed to store TICKS value.
		TICKS: integer := 10
	);

	port(
		clk, reset: in std_logic;
		en: in std_logic;							-- When set to 1, the counter will begin to count and will set 'tick' to 1 when N nanoseconds are elapsed.
		tick: out std_logic						-- "ticks" (is set to 1) once N nanoseconds are elapsed.
	);
end tickDelay;

architecture Time_delay of tickDelay is
	signal r_reg: unsigned(N-1 downto 0);
	signal r_next: unsigned(N-1 downto 0);
begin
	-- register
	process(clk, reset)
	begin
		if (reset = '1') then
			r_reg <= (others => '0');	-- Reset 'r_reg' value to 0. This will cause 'r_next' to be 1 (r_next = r_reg + 1. See next-state logic) and 'r_reg' will be updated with
							-- the 'r_next' value (in this case: 1)  on the next rising edge of the clock signal.
		elsif (clk'event and clk = '1') then
			r_reg <= r_next;
		end if;
	end process;

	-- next-state logic
	r_next <= (others => '0') when (r_reg = (TICKS - 1)) or en = '0' else (r_reg + 1);	-- Increment 'r_next' while 'r_reg' don't reach it's maximum allowed value (TICKS-1). Otherwise, reset 'r_next'.
												-- Reseting 'r_next' here will cause 'r_reg' to be 0 on the next rising edge of the clock signal.

	-- output logic
	tick <= '1' when (r_reg = (TICKS - 1)) and en = '1' else '0';	-- If 'r_reg' reaches it's maximum value (definied by generic 'TICKS' minus 1) then set 'max_tick' output to 1 for a clock cycle.
									-- Otherwise, set 'max_tick' to '0' for one clock cycle.	
end Time_delay;


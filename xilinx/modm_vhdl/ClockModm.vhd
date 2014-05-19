---
-- @file ClockModm.vhd
-- @brief Modm Counter for Xilinx FPGAs.
--
-- Date: 06-10-2010
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
-- Create Date:    01:43:09 10/06/2010 
-- Design Name: 
-- Module Name:    ClockModm - Clock 
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
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity ClockModm is
	generic(
		M: integer := 10
	);
	
	port(
		clk_in, reset: in std_logic;
		clk_out: out std_logic
	);
end ClockModm;

architecture DivClock of ClockModm is
	signal max_tick: std_logic;
	signal d_clk: std_logic;		-- derivative clock. This signal is inverted (d_clk <= not d_clk) every time max_tick is on it's rising edge.
begin
	-- instantiate Mod-m counter
	modm_counter0: entity work.Modm(Counter)
		generic map(N => integer(CEIL(LOG(real(M)) / LOG(real(2)))), M => (M / 2))	-- M/2 because we need 'max_tick' to pulse twice every M counts, to generate a 50% duty cycle waveform with the Mod-m couter period.
																										   -- N is calculated using the LOG(M) / LOG(2) (rounded up) which gives the number of bits needed to store the numeric value of M
		port map(clk => clk_in, reset => reset, max_tick => max_tick, q => open);	-- 'open' means 'not connected'

	-- process max_tick and reset
	process(max_tick, reset)
	begin
		if (reset = '1') then
			d_clk <= '0';		-- reset clk_out to 0 when reset is 1;
		elsif (max_tick'event and max_tick = '1') then
			d_clk <= not d_clk;
		end if;
	end process;

	-- derivate a 50% duty cycle clock from the 'max_tick' pulse
	clk_out <= d_clk;

end DivClock;


---
-- @file Modm.vhd
-- @brief Modm Counter for Xilinx FPGAs.
--
-- Date: 05-10-2010
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
-- Create Date:    21:35:50 10/05/2010 
-- Design Name: 
-- Module Name:    Modm - Behavioral 
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

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

-- This model enables the 'max_tick' signal once for one clock cycle for every M clock cycles.
-- More explanations about this Mod-m counter architecture can be found on page 83 of the book FPGA PROTOTYPING BY VHDL EXAMPLES by Pong P. Chu.

entity Modm is
	generic(
		N: integer := 4;
		M: integer := 10
	);
	
	port(
		clk, reset: in std_logic;
		max_tick: out std_logic;
		q: out std_logic_vector(N-1 downto 0)
	);
end Modm;

architecture Counter of Modm is
	signal r_reg: unsigned(N-1 downto 0);
	signal r_next: unsigned(N-1 downto 0);
begin
	-- register
	process(clk,reset)
	begin
		if (reset='1') then
			r_reg <= (others=>'0');	-- Reset 'r_reg' value to 0. This will cause 'r_next' to be 1 (r_next = r_reg + 1. See next-state logic) and 'r_reg' will be updated with
											-- the 'r_next' value (in this case: 1)  on the next rising edge of the clock signal.
		elsif (clk'event and clk='1') then
			r_reg <= r_next;			-- Update 'r_reg' signal with the value of the 'r_next' signal on each rising edge of the clock signal.
		end if;
	end process;
	
	-- next-state logic
	r_next <= (others=>'0') when r_reg=(M-1) else r_reg + 1;		-- Increment 'r_next' while 'r_reg' don't reach it's maximum allowed value (M-1). Otherwise, reset 'r_next'.
																					-- Reseting 'r_next' here will cause 'r_reg' to be 0 on the next rising edge of the clock signal.
	
	-- output logic
	q <= std_logic_vector(r_reg);		-- Convert the unsigned type 'r_reg' into std_logic_vector type to associate the value to 'q'.
	
	max_tick <= '1' when r_reg=(M-1) else '0';	-- If 'r_reg' reaches it's maximum value (definied by generic 'M' minus 1) then set 'max_tick' output to 1 for a clock cycle.
																-- Otherwise, set 'max_tick' to '0' for one clock cycle.

end Counter;


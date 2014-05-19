---
-- @file LCDdriver.vhd
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
--
-- Company:	uCodev 
-- Engineer: 	Pedro A. Hortas
-- 
-- Create Date:    17:45:08 10/10/2010 
-- Design Name: 
-- Module Name:    LCDdriver - lcd_handler
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

-- Use MATH_REAL for LOG() and CEIL() functions
use IEEE.MATH_REAL.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity LCDdriver is
	port(
		clk, reset: in std_logic;
		wr: in std_logic;					-- Writes the char set in the port 'byte'
		clr: in std_logic;				-- Clears the LCD
		in_byte: in std_logic_vector(7 downto 0); -- Input byte (char) to be written on the LCD
		
		-- LCD Specific pinout
		d: inout std_logic_vector(3 downto 0);
		rs: out std_logic;
		rw: out std_logic;
		e: out std_logic
	);
end LCDdriver;

architecture lcd_handler of LCDdriver is
	---------------------------
	-- Constant Declarations --
	---------------------------
	constant CLOCK_C: integer := 50000000;	-- 50MHz clock

	-----------------------
	-- Type Declarations --
	-----------------------
	type lcd_state_type is (idle,		-- LCD controller is idle
									try_idle,	-- LCD Controller will try to enter in the idle state. A read operation will be performed to check if the LCD is busy.
									check_idle,	-- after performing a read opeartion to check the lcd status, this state will check if that byte indicates if LCD is idle or not.
									w_nibble_1,		-- Writting nibble 1
									w_nibble_2,		-- Writting nibble 2
									r_nibble_1,		-- Reading nibble 1
									r_nibble_2,		-- Reading nibble 2
									clear_1,	-- Clear LCD, phase 1
									clear_2,	-- Clear LCD, phase 2
									uninit,	-- Unitialized state
									init_1,		--
									init_2,		--
									init_3,		-- 3 x RS: 0, R/W: 0, DB7: 0, DB6: 0, DB5: 1, DB4: 1
									init_cfg_1,	--
									init_cfg_2_p1,
									init_cfg_2_p2, --
									init_cfg_3_p1, --
									init_cfg_3_p2, --
									init_cfg_4_p1,	--
									init_cfg_4_p2, --
									init_cfg_5_p1, --
									init_cfg_5_p2);	-- Last Configuration Options set in initializing state.
	type op_type is (do_wrt,	-- Perform write operation
						  do_rd,		-- Perform read operation
						  do_cfg,	-- Perform configure operation
						  do_idle);	
	
	-------------------------
	-- Signal Declarations --
	-------------------------
	signal wr_reg, wr_next, clr_reg, clr_next: std_logic;
	signal state_lcd_reg, state_lcd_next: lcd_state_type;	-- LCD state
	signal oper_reg, oper_next: op_type;
	signal e_reg, e_next, e_progress_reg, e_progress_next, rs_reg, rs_next, rw_reg, rw_next: std_logic;	-- e, rs and rw registers
																		-- e_reg is the signal that indicates when a E operation is being performed.
																		-- e_done is the signal that indicates when a E operation was performed. This signal must be cleared after e_reg = 0 and oper = op_idle
																		-- is verified.
	signal nibble_reg, nibble_next: std_logic_vector(3 downto 0);	-- Nibble that map 'in_byte' signal to port 'd' with two write operations.
	signal rbyte_reg, rbyte_next: std_logic_vector(7 downto 0);		-- Byte that will contain the two nibbles read from the op_rd operation.
	
	
	
	-- generic timers
	signal en_5ms_t, en_5ms_t_next, en_150us_t, en_150us_t_next: std_logic;
	signal tck_5ms_t, tck_150us_t: std_logic;
	
	-- operation timers
	signal en_op_42us_t, en_op_42us_t_next: std_logic;
	signal tck_op_42us_t: std_logic;
	
	-- Write mode timers
	signal en_w_tr_tpw_tf_t, en_w_tr_tpw_tf_t_next, en_w_tsp1_t, en_w_tsp1_t_next, en_w_500ns_t, en_w_500ns_t_next: std_logic;
	signal tck_w_tr_tpw_tf_t, tck_w_tsp1_t, tck_w_500ns_t: std_logic;
	
	-- Read mode timers
	signal en_r_rest_tw_tf_t, en_r_rest_tw_tf_t_next, en_r_tsp1_t, en_r_tsp1_t_next, en_r_tr_td_t, en_r_tr_td_t_next, en_r_500ns_t, en_r_500ns_t_next: std_logic;
	signal tck_r_rest_tw_tf_t, tck_r_tsp1_t, tck_r_tr_td_t, tck_r_500ns_t: std_logic;

begin
	
	
	-----------------------------
	-- Component Instantiation --
	-----------------------------
	
	
	-- Generic Timers

	-- 150us timer
	delay_gen_150us_0: entity work.tickDelay(Time_delay)
		generic map(
			N => integer(CEIL(LOG(real(CEIL(real(real(150000) / real((real(1000000000) / real(CLOCK_C))))))) / LOG(real(2)))) + 1,	-- 13 bits are needed to store the value 7500 -> log(7500) / log(2) = 12.8726748802706
			TICKS => integer(CEIL(real(real(150000) / real((real(1000000000) / real(CLOCK_C))))))  -- for a 50MHz clock, we need 7500 ticks to elapse 150us of delay
		)

		port map(
			clk => clk,
			reset => reset,
			en => en_150us_t,
			tick => tck_150us_t
		);

	-- 5ms timer
	delay_gen_5ms_0: entity work.tickDelay(Time_delay)
		generic map(
			N => integer(CEIL(LOG(real(CEIL(real(real(5000000) / real((real(1000000000) / real(CLOCK_C))))))) / LOG(real(2)))) + 1,		-- 18 bits are needed to store the value 250000 -> log(250000) / log(2) = 17.9315685693241
			TICKS => integer(CEIL(real(real(5000000) / real((real(1000000000) / real(CLOCK_C)))))) -- for a 50MHz clock, we need 250000 ticks to elapse 5ms of delay
		)

		port map(
			clk => clk,
			reset => reset,
			en => en_5ms_t,
			tick => tck_5ms_t
		);

	
	-- Operation Timer
	
	delay_op_42us_0: entity work.tickDelay(Time_delay)
		generic map(
			N => integer(CEIL(LOG(real(CEIL(real(real(42000) / real((real(1000000000) / real(CLOCK_C))))))) / LOG(real(2)))) + 1,
			TICKS => integer(CEIL(real(real(42000) / real((real(1000000000) / real(CLOCK_C))))))
		)

		port map(
			clk => clk,
			reset => reset,
			en => en_op_42us_t,
			tick => tck_op_42us_t
		);
		

	-- Write mode timers
	
	-- write tpw timer: 450ns (min) + 25ns * 2 (max) for tr and tf.
	delay_write_tr_tpw_tf_0: entity work.tickDelay(Time_delay)
		generic map(
			N => integer(CEIL(LOG(real(CEIL(real(real(500) / real((real(1000000000) / real(CLOCK_C))))))) / LOG(real(2)))) + 1,
			TICKS => integer(CEIL(real(real(500) / real((real(1000000000) / real(CLOCK_C))))))
		)
		
		port map(
			clk => clk,
			reset => reset,
			en => en_w_tr_tpw_tf_t,
			tick => tck_w_tr_tpw_tf_t
		);
	
	-- write tsp1 timer: 60ns (min)
	delay_write_tsp1_0: entity work.tickDelay(Time_delay)
		generic map(
			N => integer(CEIL(LOG(real(CEIL(real(real(60) / real((real(1000000000) / real(CLOCK_C))))))) / LOG(real(2)))) + 1,	-- +1 to avoid null array when wave period >= 60ns.
			TICKS => integer(CEIL(real(real(60) / real((real(1000000000) / real(CLOCK_C))))))
		)
		
		port map(
			clk => clk,
			reset => reset,
			en => en_w_tsp1_t,
			tick => tck_w_tsp1_t
		);

	-- write 500ns timer
	delay_write_500ns_0: entity work.tickDelay(Time_delay)
		generic map(
			N => integer(CEIL(LOG(real(CEIL(real(real(500) / real((real(1000000000) / real(CLOCK_C))))))) / LOG(real(2)))) + 1,
			TICKS => integer(CEIL(real(real(500) / real((real(1000000000) / real(CLOCK_C))))))
		)
		
		port map(
			clk => clk,
			reset => reset,
			en => en_w_500ns_t,
			tick => tck_w_500ns_t
		);

	-- Read mode timers
	
	-- read rest_tw timer: 450ns - 360ns = 90ns (min) + 25ns (max) for tf
	delay_read_rest_tw_tf_0: entity work.tickDelay(Time_delay)
		generic map(
			N => integer(CEIL(LOG(real(CEIL(real(real(115) / real((real(1000000000) / real(CLOCK_C))))))) / LOG(real(2)))) + 1,
			TICKS => integer(CEIL(real(real(115) / real((real(1000000000) / real(CLOCK_C))))))
		)
		
		port map(
			clk => clk,
			reset => reset,
			en => en_r_rest_tw_tf_t,
			tick => tck_r_rest_tw_tf_t
		);

	-- read tsp1 timer: 60ns (min)
	delay_read_tsp1_0: entity work.tickDelay(Time_delay)
		generic map(
			N => integer(CEIL(LOG(real(CEIL(real(real(60) / real((real(1000000000) / real(CLOCK_C))))))) / LOG(real(2)))) + 1,	-- +1 to avoid null array when wave period >= 60ns.
			TICKS => integer(CEIL(real(real(60) / real((real(1000000000) / real(CLOCK_C))))))
		)
		
		port map(
			clk => clk,
			reset => reset,
			en => en_r_tsp1_t,
			tick => tck_r_tsp1_t
		);

	-- read tr td timer: 360ns (min) + 25ns (max) for tr
	delay_read_tr_td_0: entity work.tickDelay(Time_delay)
		generic map(
			N => integer(CEIL(LOG(real(CEIL(real(real(360) / real((real(1000000000) / real(CLOCK_C))))))) / LOG(real(2)))) + 1,
			TICKS => integer(CEIL(real(real(360) / real((real(1000000000) / real(CLOCK_C))))))
		)
		
		port map(
			clk => clk,
			reset => reset,
			en => en_r_tr_td_t,
			tick => tck_r_tr_td_t
		);

	-- read 500ns timer
	delay_read_500ns_0: entity work.tickDelay(Time_delay)
		generic map(
			N => integer(CEIL(LOG(real(CEIL(real(real(500) / real((real(1000000000) / real(CLOCK_C))))))) / LOG(real(2)))) + 1,
			TICKS => integer(CEIL(real(real(500) / real((real(1000000000) / real(CLOCK_C))))))
		)
		
		port map(
			clk => clk,
			reset => reset,
			en => en_r_500ns_t,
			tick => tck_r_500ns_t
		);




	 --  ****************** --
	-- * LCD driver circuit * --
	 --  ****************** --

	
	------------------------------------------------------------------------------------------------------------
	-- Synchronous system core. Syncronization of states is performed on the rising-edge of the system clock. --
	------------------------------------------------------------------------------------------------------------
	
	process(clk, reset)
	begin
		if (reset = '1') then
			nibble_reg <= (others => '0');
			rbyte_reg <= (others => '0');
			oper_reg <= do_idle;
			rs_reg <= '0';
			rw_reg <= '0';
			e_reg <= '0';
			e_progress_reg <= '0';
			state_lcd_reg <= uninit;
			en_150us_t <= '0';
			en_5ms_t <= '0';
			en_op_42us_t <= '0';
			en_w_tr_tpw_tf_t <= '0';
			en_w_tsp1_t <= '0';
			en_w_500ns_t <= '0';
			en_r_rest_tw_tf_t <= '0';
			en_r_tr_td_t <= '0';
			en_r_tsp1_t <= '0';
			en_r_500ns_t <= '0';
			wr_reg <= '0';
			clr_reg <= '0';
		elsif (clk'event and clk = '1') then
			nibble_reg <= nibble_next;
			rbyte_reg <= rbyte_next;
			oper_reg <= oper_next;
			rs_reg <= rs_next;
			rw_reg <= rw_next;
			e_reg <= e_next;
			e_progress_reg <= e_progress_next;
			state_lcd_reg <= state_lcd_next;
			en_150us_t <= en_150us_t_next;
			en_5ms_t <= en_5ms_t_next;
			en_op_42us_t <= en_op_42us_t_next;
			en_w_tr_tpw_tf_t <= en_w_tr_tpw_tf_t_next;
			en_w_tsp1_t <= en_w_tsp1_t_next;
			en_w_500ns_t <= en_w_500ns_t_next;
			en_r_rest_tw_tf_t <= en_r_rest_tw_tf_t_next;
			en_r_tr_td_t <= en_r_tr_td_t_next;
			en_r_tsp1_t <= en_r_tsp1_t_next;
			en_r_500ns_t <= en_r_500ns_t_next;
			wr_reg <= wr_next;
			clr_reg <= clr_next;
		end if;
	end process;
	
	
	
	----------------------
	-- Next-State Logic --
	----------------------
	
	process (d, state_lcd_reg, wr, wr_reg, clr, clr_reg, oper_reg, tck_5ms_t, tck_op_42us_t, nibble_reg, rbyte_reg, rs_reg, rw_reg, in_byte,
				en_5ms_t, en_150us_t, en_op_42us_t, en_w_tr_tpw_tf_t, en_w_tsp1_t, en_r_rest_tw_tf_t, en_r_tr_td_t, en_r_tsp1_t, en_w_500ns_t, en_r_500ns_t,
				e_reg, e_progress_reg, tck_150us_t, tck_w_tr_tpw_tf_t, tck_w_tsp1_t, tck_w_500ns_t, tck_r_rest_tw_tf_t, tck_r_tr_td_t, tck_r_tsp1_t, tck_r_500ns_t
				)
	begin
		-- default values
		nibble_next <= nibble_reg;
		rbyte_next <= rbyte_reg;
		oper_next <= oper_reg;
		rs_next <= rs_reg;
		rw_next <= rw_reg;
		e_next <= e_reg;
		e_progress_next <= e_progress_reg;
		state_lcd_next <= state_lcd_reg;
		en_5ms_t_next <= en_5ms_t;
		en_150us_t_next <= en_150us_t;
		en_op_42us_t_next <= en_op_42us_t;
		en_w_tr_tpw_tf_t_next <= en_w_tr_tpw_tf_t;
		en_w_tsp1_t_next <= en_w_tsp1_t;
		en_w_500ns_t_next <= en_w_500ns_t;
		en_r_rest_tw_tf_t_next <= en_r_rest_tw_tf_t;
		en_r_tr_td_t_next <= en_r_tr_td_t;
		en_r_tsp1_t_next <= en_r_tsp1_t;
		en_r_500ns_t_next <= en_r_500ns_t;
		wr_next <= wr;
		clr_next <= clr;
		
		-- Process data, if any
		case state_lcd_reg is
			when uninit =>
				if (oper_reg = do_idle) then	-- Grant that there's no operation being performed on the LCD (this step may be skipped when LCD state is uninitialized).
					rs_next <= '0';				-- rs register should be '0' in the initialization process.
					rw_next <= '0';				-- rw register should be '0' in the initialization process.
					nibble_next <= "0011";		-- This nibble defines the states for the DB7, DB6, DB5 and DB4 respectively (0, 0, 1, 1).
					e_next <= '1';					-- Perform an enable instruction, respecting the timimg diagrams of the JHD162A (see datasheet).
					en_5ms_t_next <= '1';		-- Wait 5ms until perform the next initialization operation.
					oper_next <= do_cfg;			-- Set the oper_next state to configuration mode (do_cfg).
					state_lcd_next <= init_1;	-- Set the LCD state to being processed.
				end if;
			when init_1 =>
				if (tck_5ms_t = '1') then				-- Check if the 5ms have been elapsed
					en_5ms_t_next <= '0';				-- Stop timer
					
					if (oper_reg = do_idle) then
						rs_next <= '0';
						rw_next <= '0';
						nibble_next <= "0011";
						e_next <= '1';
						en_150us_t_next <= '1';
						oper_next <= do_cfg;
						state_lcd_next <= init_2;
					end if;
				end if;
			when init_2 =>
				if (tck_150us_t = '1') then			-- Check if the 150us have been elapsed
					en_150us_t_next <= '0';				-- Stop timer
					
					if (oper_reg = do_idle) then
						rs_next <= '0';
						rw_next <= '0';
						nibble_next <= "0011";
						e_next <= '1';
						en_op_42us_t_next <= '1';		-- Since this point, in the initialization process, there a need to wait between each operation sent to the LCD. So lets wait 41000ns which
																-- is the time needed to perform each Enable operation.
						oper_next <= do_cfg;
						state_lcd_next <= init_3;
					end if;
				end if;
			when init_3 =>
				if (tck_op_42us_t = '1') then
					en_op_42us_t_next <= '0';
					
					if (oper_reg = do_idle) then
						rs_next <= '0';
						rw_next <= '0';
						nibble_next <= "0010";
						e_next <= '1';
						en_op_42us_t_next <= '1';
						oper_next <= do_cfg;
						state_lcd_next <= init_cfg_1;
					end if;
				end if;
			when init_cfg_1 =>
				if (tck_op_42us_t = '1') then
					en_op_42us_t_next <= '0';
					
					if (oper_reg = do_idle) then
						rs_next <= '0';
						rw_next <= '0';
						nibble_next <= "0010";	-- High Order nibble: Function SET
						e_next <= '1';
						en_op_42us_t_next <= '1';
						oper_next <= do_cfg;
						state_lcd_next <= init_cfg_2_p1;
					end if;
				end if;
			when init_cfg_2_p1 =>
				if (tck_op_42us_t = '1') then
					en_op_42us_t_next <= '0';
					
					if (oper_reg = do_idle) then
						rs_next <= '0';
						rw_next <= '0';
						nibble_next <= "1100";	-- Low order nibble: N=1 (two rows), F=1 (5x10 style)
						e_next <= '1';
						en_op_42us_t_next <= '1';
						oper_next <= do_cfg;
						state_lcd_next <= init_cfg_2_p2;
					end if;
				end if;
			when init_cfg_2_p2 =>
				if (tck_op_42us_t = '1') then
					en_op_42us_t_next <= '0';
					
					if (oper_reg = do_idle) then
						rs_next <= '0';
						rw_next <= '0';
						nibble_next <= "0000";	-- High order nibble: Will be a Display Switch since 4th bit of low order nibble will be '1'.
						e_next <= '1';
						en_op_42us_t_next <= '1';
						oper_next <= do_cfg;
						state_lcd_next <= init_cfg_3_p1;
					end if;
				end if;
			when init_cfg_3_p1 =>
				if (tck_op_42us_t = '1') then
					en_op_42us_t_next <= '0';
					
					if (oper_reg = do_idle) then
						rs_next <= '0';
						rw_next <= '0';
						nibble_next <= "1111";	-- Display Switch: Display On (D=1), Cursor On (C=1), Blink On (B=1)
						e_next <= '1';
						en_op_42us_t_next <= '1';
						oper_next <= do_cfg;
						state_lcd_next <= init_cfg_3_p2;
					end if;
				end if;
			when init_cfg_3_p2 =>
				if (tck_op_42us_t = '1') then
					en_op_42us_t_next <= '0';
					
					if (oper_reg = do_idle) then
						rs_next <= '0';
						rw_next <= '0';
						nibble_next <= "0000";	-- High order nibble: Will be a display clear since 1st bit of low order nibble will be '1' with all others set to '0'.
						e_next <= '1';
						en_op_42us_t_next <= '1';
						oper_next <= do_cfg;
						state_lcd_next <= init_cfg_4_p1;
					end if;
				end if;
			when init_cfg_4_p1 =>
				if (tck_op_42us_t = '1') then
					en_op_42us_t_next <= '0';
					
					if (oper_reg = do_idle) then
						rs_next <= '0';
						rw_next <= '0';
						nibble_next <= "0001";	-- Display clear since all other bits are '0' (including the high order nibble).
						e_next <= '1';
						en_op_42us_t_next <= '1';
						oper_next <= do_cfg;
						state_lcd_next <= init_cfg_4_p2;
					end if;
				end if;
			when init_cfg_4_p2 =>
				if (tck_op_42us_t = '1') then
					en_op_42us_t_next <= '0';
					
					if (oper_reg = do_idle) then
						rs_next <= '0';
						rw_next <= '0';
						nibble_next <= "0000";	-- High order nibble: Will be an Input Set since 3rd bit of low order byte will be '1' with all other (from 3rd to 8th) being '0'.
						e_next <= '1';
						en_op_42us_t_next <= '1';
						oper_next <= do_cfg;
						state_lcd_next <= init_cfg_5_p1;
					end if;
				end if;
			when init_cfg_5_p1 =>
				if (tck_op_42us_t = '1') then
					en_op_42us_t_next <= '0';
					
					if (oper_reg = do_idle) then
						rs_next <= '0';
						rw_next <= '0';
						nibble_next <= "0110";	-- Low order nibble: Entry mode set since 3rd bit is high, Increment mode (I=1) and do not shift (S=0).
						e_next <= '1';
						en_op_42us_t_next <= '1';
						oper_next <= do_cfg;
						state_lcd_next <= init_cfg_5_p2;
					end if;
				end if;
			when init_cfg_5_p2 =>
				if (tck_op_42us_t = '1') then
					en_op_42us_t_next <= '0';
					
					if (oper_reg = do_idle) then
						state_lcd_next <= try_idle;
					end if;
				end if;
			when try_idle =>		-- try to enter in the idle state. for that, we need to check the status of the lcd to be sure that it's not performing any operation
				rs_next <= '1';
				rw_next <= '1';
				e_next <= '1';
				en_op_42us_t_next <= '1';
				oper_next <= do_rd;
				state_lcd_next <= r_nibble_1;
			when r_nibble_1 =>
				if (tck_op_42us_t = '1') then
					en_op_42us_t_next <= '0';
					
					if (oper_reg = do_idle) then
						rs_next <= '1';
						rw_next <= '1';
						rbyte_next(7 downto 4) <= nibble_reg;
						e_next <= '1';
						en_op_42us_t_next <= '1';
						oper_next <= do_rd;
						state_lcd_next <= r_nibble_2;
					end if;
				end if;
			when r_nibble_2 =>
				if (tck_op_42us_t = '1') then
					en_op_42us_t_next <= '0';
					
					if (oper_reg = do_idle) then
						rs_next <= '0';
						rw_next <= '0';
						rbyte_next(3 downto 0) <= nibble_reg;
						state_lcd_next <= check_idle;
					end if;
				end if;
			when check_idle =>
				if (rbyte_reg = "10000000") then
					state_lcd_next <= try_idle;	-- If the read byte is 0x80, then lcd is performing an operation and we should perform another read
				else
					state_lcd_next <= idle;			-- If the read byte is different than 0x80, then the LCD is idle
				end if;
			when idle =>						-- Here we trig for actions on the 'wr' and 'clr' input pins.
				wr_next <= wr;
				clr_next <= clr;
				
				if (wr_reg = '1') then
					rs_next <= '1';
					rw_next <= '0';
					nibble_next <= in_byte(7 downto 4);
					e_next <= '1';
					en_op_42us_t_next <= '1';
					oper_next <= do_wrt;
					state_lcd_next <= w_nibble_1;
				elsif (clr_reg = '1') then
					rs_next <= '0';
					rw_next <= '0';
					nibble_next <= "0000";		-- First nibble of the "00000001" byte which indicates a clear operation.
					e_next <= '1';
					en_op_42us_t_next <= '1';
					oper_next <= do_cfg;
					state_lcd_next <= clear_1;
				end if;
			when w_nibble_1 =>
				if (tck_op_42us_t = '1') then
					en_op_42us_t_next <= '0';
					
					if (oper_reg = do_idle) then
						rs_next <= '1';
						rw_next <= '0';
						nibble_next <= in_byte(3 downto 0);
						e_next <= '1';
						en_op_42us_t_next <= '1';
						oper_next <= do_wrt;
						state_lcd_next <= w_nibble_2;
					end if;
				end if;
			when w_nibble_2 =>
				if (tck_op_42us_t = '1') then
					en_op_42us_t_next <= '0';
					
					if (oper_reg = do_idle) then
						rs_next <= '0';
						rw_next <= '0';
						state_lcd_next <= try_idle;
					end if;
				end if;
			when clear_1 =>
				if (tck_op_42us_t = '1') then
					en_op_42us_t_next <= '0';
					
					if (oper_reg = do_idle) then
						rs_next <= '0';
						rw_next <= '0';
						nibble_next <= "0001";	-- Second nibble of the "00000001" byte which indicates a clear operation
						e_next <= '1';
						en_op_42us_t_next <= '1';
						oper_next <= do_cfg;
						state_lcd_next <= clear_2;
					end if;
				end if;
			when clear_2 =>
				if (tck_op_42us_t = '1') then
					en_op_42us_t_next <= '0';
					
					if (oper_reg = do_idle) then
						rs_next <= '0';
						rw_next <= '0';
						state_lcd_next <= try_idle;
					end if;
				end if;
			when others =>
				rs_next <= '0';	-- Remove this after case is complete.
		end case;
		

		-- Now lets process e_reg.
		-- This will enter in a delay loop that will respect all timming diagrams of the LCD datasheet (for the JHD162A and compatible LCDs) for write and read operations.
		-- An operation begins when e_reg is set to '1'. An operation is done when the signal 'e_done' is set to '1'.

		if (e_reg = '1') then		-- It's very important here to trigger ONLY the RISING EDGE of 'e_reg', because e_reg will be '1' for multiple checks of this condition
											-- and we don't want to enable tsp1 counter over and over.. only when e_reg changes from 0 to 1.
			if ((oper_reg = do_wrt or oper_reg = do_cfg) and e_progress_reg = '0') then
				e_progress_next <= '1';
				en_w_tsp1_t_next <= '1';		-- Start timer for TSP1 counter for write operation
			elsif (oper_reg = do_rd and e_progress_reg = '0') then
				e_progress_next <= '1';
				en_r_tsp1_t_next <= '1';		-- Start timer for TSP1 counter for read operation
			end if;
		end if;


		-- Write operation
		if (e_reg = '1' and tck_w_tsp1_t = '1') then
			en_w_tsp1_t_next <= '0';			-- Stop TSP1 timer
			en_w_tr_tpw_tf_t_next <= '1';	-- Start tpw timer... this will cause that port 'e' to be also '1' (see output logic).
		elsif (e_reg = '1' and tck_w_tr_tpw_tf_t = '1') then
			en_w_tr_tpw_tf_t_next <= '0';	-- Stop tpw timer... this will cause port 'e' to be also '0' (see next-state logic).
			en_w_500ns_t_next <= '1';			-- Start 500ns timer (this is TC - TPW - TR - TL = 500ns).
		elsif (e_reg = '1' and tck_w_500ns_t = '1') then
			en_w_500ns_t_next <= '0';			-- Stop 500ns timer
			e_next <= '0';		-- Operation is done.
			oper_next <= do_idle;
			e_progress_next <= '0';

		-- Read operation
		elsif (e_reg = '1' and tck_r_tsp1_t = '1') then
			en_r_tsp1_t_next <= '0';			-- Stop TSP1 timer
			en_r_tr_td_t_next <= '1';			-- Enable td timer... when this timer expires, data will be available to be read from the lcd
		elsif (e_reg = '1' and tck_r_tr_td_t = '1') then
			en_r_tr_td_t_next <= '0';
			en_r_rest_tw_tf_t_next <= '1';		-- Start tw timer... this will cause that port 'e' to be enable also (see next-state logic).
		elsif (e_reg = '1' and tck_r_rest_tw_tf_t = '1') then
			en_r_rest_tw_tf_t_next <= '0';	-- Stop tw timer... this will cause port 'e' to be also '0' (see next-state logic).
			nibble_next <= d;						-- Read one nibble from LCD.
			en_r_500ns_t_next <= '1';			-- Start 500ns timer (this is TC - TW - TR - TL = 500ns).
		elsif (e_reg = '1' and tck_r_500ns_t = '1') then
			en_r_500ns_t_next <= '0';			-- Stop 500ns timer
			e_next <= '0';		-- Operation is done.
			oper_next <= do_idle;
			e_progress_next <= '0';
		end if;
	end process;




	------------------
	-- Output Logic --
	------------------
	
	d <= nibble_reg when (oper_reg = do_wrt) or (oper_reg = do_cfg) else (others=>'Z');
	rs <= rs_reg when (oper_reg = do_wrt) or (oper_reg = do_rd) or (oper_reg = do_cfg) else '0';
	rw <= rw_reg when (oper_reg = do_wrt) or (oper_reg = do_rd) or (oper_reg = do_cfg) else '0';
	e <= en_w_tr_tpw_tf_t or en_r_rest_tw_tf_t or en_r_tr_td_t;	-- Enable signal should be positive while tr, tpw or tw, and tf are being elapsed.

end lcd_handler;


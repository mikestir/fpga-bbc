-- BBC Micro for Altera DE1
--
-- Copyright (c) 2011 Mike Stirling
--
-- All rights reserved
--
-- Redistribution and use in source and synthezised forms, with or without
-- modification, are permitted provided that the following conditions are met:
--
-- * Redistributions of source code must retain the above copyright notice,
--   this list of conditions and the following disclaimer.
--
-- * Redistributions in synthesized form must reproduce the above copyright
--   notice, this list of conditions and the following disclaimer in the
--   documentation and/or other materials provided with the distribution.
--
-- * Neither the name of the author nor the names of other contributors may
--   be used to endorse or promote products derived from this software without
--   specific prior written permission.
--
-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
-- AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
-- THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
-- PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE
-- LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
-- CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
-- SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
-- INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
-- CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
-- ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
-- POSSIBILITY OF SUCH DAMAGE.
--

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity bbc_micro_tb is
end entity;

architecture tb of bbc_micro_tb is
component bbc_micro_de1 is
port (
	-- Clocks
	CLOCK_24	:	in	std_logic_vector(1 downto 0);
	CLOCK_27	:	in	std_logic_vector(1 downto 0);
	CLOCK_50	:	in	std_logic;
	EXT_CLOCK	:	in	std_logic;
	
	-- Switches
	SW			:	in	std_logic_vector(9 downto 0);
	-- Buttons
	KEY			:	in	std_logic_vector(3 downto 0);
	
	-- 7 segment displays
	HEX0		:	out	std_logic_vector(6 downto 0);
	HEX1		:	out	std_logic_vector(6 downto 0);
	HEX2		:	out	std_logic_vector(6 downto 0);
	HEX3		:	out	std_logic_vector(6 downto 0);
	-- Red LEDs
	LEDR		:	out	std_logic_vector(9 downto 0);
	-- Green LEDs
	LEDG		:	out	std_logic_vector(7 downto 0);
	
	-- VGA
	VGA_R		:	out	std_logic_vector(3 downto 0);
	VGA_G		:	out	std_logic_vector(3 downto 0);
	VGA_B		:	out	std_logic_vector(3 downto 0);
	VGA_HS		:	out	std_logic;
	VGA_VS		:	out	std_logic;
	
	-- Serial
	UART_RXD	:	in	std_logic;
	UART_TXD	:	out	std_logic;
	
	-- PS/2 Keyboard
	PS2_CLK		:	inout	std_logic;
	PS2_DAT		:	inout	std_logic;
	
	-- I2C
	I2C_SCLK	:	inout	std_logic;
	I2C_SDAT	:	inout	std_logic;
	
	-- Audio
	AUD_XCK		:	out		std_logic;
	AUD_BCLK	:	out		std_logic;
	AUD_ADCLRCK	:	out		std_logic;
	AUD_ADCDAT	:	in		std_logic;
	AUD_DACLRCK	:	out		std_logic;
	AUD_DACDAT	:	out		std_logic;
	
	-- SRAM
	SRAM_ADDR	:	out		std_logic_vector(17 downto 0);
	SRAM_DQ		:	inout	std_logic_vector(15 downto 0);
	SRAM_CE_N	:	out		std_logic;
	SRAM_OE_N	:	out		std_logic;
	SRAM_WE_N	:	out		std_logic;
	SRAM_UB_N	:	out		std_logic;
	SRAM_LB_N	:	out		std_logic;
	
	-- SDRAM
	DRAM_ADDR	:	out		std_logic_vector(11 downto 0);
	DRAM_DQ		:	inout	std_logic_vector(15 downto 0);
	DRAM_BA_0	:	in		std_logic;
	DRAM_BA_1	:	in		std_logic;
	DRAM_CAS_N	:	in		std_logic;
	DRAM_CKE	:	in		std_logic;
	DRAM_CLK	:	in		std_logic;
	DRAM_CS_N	:	in		std_logic;
	DRAM_LDQM	:	in		std_logic;
	DRAM_RAS_N	:	in		std_logic;
	DRAM_UDQM	:	in		std_logic;
	DRAM_WE_N	:	in		std_logic;
	
	-- Flash
	FL_ADDR		:	out		std_logic_vector(21 downto 0);
	FL_DQ		:	inout	std_logic_vector(7 downto 0);
	FL_RST_N	:	in		std_logic;
	FL_OE_N		:	in		std_logic;
	FL_WE_N		:	in		std_logic;
	
	-- GPIO
	GPIO_0		:	inout	std_logic_vector(35 downto 0);
	GPIO_1		:	inout	std_logic_vector(35 downto 0)
	);
end component;


signal clock_24 	:	std_logic_vector(1 downto 0)	:= "00";
signal clock_27		:	std_logic_vector(1 downto 0)	:= "00";
signal clock_50		:	std_logic	:= '0';
signal ext_clock	:	std_logic	:= '0';
signal sw			:	std_logic_vector(9 downto 0);
signal key			:	std_logic_vector(3 downto 0);
signal hex0			:	std_logic_vector(6 downto 0);
signal hex1			:	std_logic_vector(6 downto 0);
signal hex2			:	std_logic_vector(6 downto 0);
signal hex3			:	std_logic_vector(6 downto 0);
signal ledr			:	std_logic_vector(9 downto 0);
signal ledg			:	std_logic_vector(7 downto 0);
signal vga_r		:	std_logic_vector(3 downto 0);
signal vga_g		:	std_logic_vector(3 downto 0);
signal vga_b		:	std_logic_vector(3 downto 0);
signal vga_hs		:	std_logic;
signal vga_vs		:	std_logic;
signal uart_rxd		:	std_logic;
signal uart_txd		:	std_logic;
signal ps2_clk		:	std_logic;
signal ps2_dat		:	std_logic;
signal i2c_sclk		:	std_logic;
signal i2c_sdat		:	std_logic;
signal aud_xck		:	std_logic;
signal aud_bclk		:	std_logic;
signal aud_adclrck	:	std_logic;
signal aud_adcdat	:	std_logic;
signal aud_daclrck	:	std_logic;
signal aud_dacdat	:	std_logic;
signal sram_addr	:	std_logic_vector(17 downto 0);
signal sram_dq		:	std_logic_vector(15 downto 0);
signal sram_ce_n	:	std_logic;
signal sram_oe_n	:	std_logic;
signal sram_we_n	:	std_logic;
signal sram_ub_n	:	std_logic;
signal sram_lb_n	:	std_logic;
signal dram_addr	:	std_logic_vector(11 downto 0);
signal dram_dq		:	std_logic_vector(15 downto 0);
signal dram_ba_0	:	std_logic;
signal dram_ba_1	:	std_logic;
signal dram_cas_n	:	std_logic;
signal dram_cke		:	std_logic;
signal dram_clk		:	std_logic;
signal dram_cs_n	:	std_logic;
signal dram_ldqm	:	std_logic;
signal dram_ras_n	:	std_logic;
signal dram_udqm	:	std_logic;
signal dram_we_n	:	std_logic;
signal fl_addr		:	std_logic_vector(21 downto 0);
signal fl_dq		:	std_logic_vector(7 downto 0);
signal fl_rst_n		:	std_logic;
signal fl_oe_n		:	std_logic;
signal fl_we_n		:	std_logic;
signal gpio_0		:	std_logic_vector(35 downto 0);
signal gpio_1		:	std_logic_vector(35 downto 0);

signal n_reset		:	std_logic	:= '0';
signal n_slow		:	std_logic	:= '1';

type ram_t is array(0 to 65535) of std_logic_vector(15 downto 0);
signal ram : ram_t;
signal ram_a : std_logic_vector(15 downto 0);
begin

	uut:	bbc_micro_de1 port map (
		clock_24, 	
		clock_27,		
		clock_50,		
		ext_clock,	
		sw,			
		key,			
		hex0,			
		hex1,			
		hex2,			
		hex3,			
		ledr,			
		ledg,			
		vga_r,		
		vga_g,		
		vga_b,		
		vga_hs,		
		vga_vs,	
		uart_rxd,		
		uart_txd,		
		ps2_clk,		
		ps2_dat,		
		i2c_sclk,		
		i2c_sdat,		
		aud_xck,		
		aud_bclk,		
		aud_adclrck,	
		aud_adcdat,	
		aud_daclrck,	
		aud_dacdat,	
		sram_addr,	
		sram_dq,		
		sram_ce_n,	
		sram_oe_n,	
		sram_we_n,	
		sram_ub_n,	
		sram_lb_n,	
		dram_addr,	
		dram_dq,		
		dram_ba_0,	
		dram_ba_1,	
		dram_cas_n,	
		dram_cke,		
		dram_clk,		
		dram_cs_n,	
		dram_ldqm,	
		dram_ras_n,	
		dram_udqm,	
		dram_we_n,	
		fl_addr,
		fl_dq,	
		fl_rst_n,		
		fl_oe_n,		
		fl_we_n,	
		gpio_0,		
		gpio_1		
		);
		
	sw <= n_reset & n_slow & "00000101";
	clock_50 <= not clock_50 after 10 ns;
	clock_27(0) <= not clock_27(0) after 18.5 ns;
	clock_27(1) <= not clock_27(1) after 18.5 ns;
	clock_24(0) <= not clock_24(0) after 20.8 ns;
	clock_24(1) <= not clock_24(1) after 20.8 ns;
		
	reset: process
	begin
		wait for 100 ns;
		n_reset <= '1';	
	end process;
	
	sram: process(sram_addr,sram_dq,sram_ce_n,sram_oe_n,sram_we_n,sram_ub_n,sram_lb_n)
	begin
		if sram_ce_n = '0' then
			if sram_oe_n = '0' and sram_we_n = '1' then
				if sram_ub_n = '0' then
					sram_dq(15 downto 8) <= ram(to_integer(unsigned(sram_addr(15 downto 0))))(15 downto 8);
				else
					sram_dq(15 downto 8) <= (others => 'Z');
				end if;
				if sram_lb_n = '0' then
					sram_dq(7 downto 0) <= ram(to_integer(unsigned(sram_addr(15 downto 0))))(7 downto 0);
				else
					sram_dq(7 downto 0) <= (others => 'Z');
				end if;
			else
				sram_dq(15 downto 0) <= (others => 'Z');
				if sram_we_n = '0' then
					if sram_ub_n = '0' then
						ram(to_integer(unsigned(sram_addr(15 downto 0))))(15 downto 8) <= sram_dq(15 downto 8);
					end if;
					if sram_lb_n = '0' then
						ram(to_integer(unsigned(sram_addr(15 downto 0))))(7 downto 0) <= sram_dq(7 downto 0);
					end if;
				end if;
			end if;
		else
			sram_dq <= (others => 'Z');
		end if;
	end process;
	

end architecture;

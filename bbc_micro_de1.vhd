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

-- BBC B Micro
--
-- Terasic DE1 top-level
--
-- (C) 2011 Mike Stirling

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- Generic top-level entity for Altera DE1 board
entity bbc_micro_de1 is
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
	FL_RST_N	:	out		std_logic;
	FL_OE_N		:	out		std_logic;
	FL_WE_N		:	out		std_logic;
	FL_CE_N		:	out		std_logic;
	
	-- SD card (SPI mode)
	SD_nCS		:	out		std_logic;
	SD_MOSI		:	out		std_logic;
	SD_SCLK		:	out		std_logic;
	SD_MISO		:	in		std_logic;
	
	-- GPIO
	GPIO_0		:	inout	std_logic_vector(35 downto 0);
	GPIO_1		:	inout	std_logic_vector(35 downto 0)
	);
end entity;

architecture rtl of bbc_micro_de1 is

------------------------------
-- PLL (32 MHz master clock)
------------------------------

component pll32 IS
	PORT
	(
		areset		: IN STD_LOGIC  := '0';
		inclk0		: IN STD_LOGIC  := '0';
		c0			: OUT STD_LOGIC ;
		locked		: OUT STD_LOGIC 
	);
end component;

------------------------------
-- Test UART for custom ROM
------------------------------

component simple_uart is
generic (
	f_clock		:	natural	:= 32000000;
	baud_rate	:	natural	:= 19200
	);
port (
	CLOCK		:	in	std_logic;
	nRESET		:	in	std_logic;
	
	ENABLE		:	in	std_logic;
	-- Read not write
	R_nW		:	in	std_logic;
	-- Data not status (address)
	S_nD		:	in	std_logic;
	-- Data bus in
	DI			:	in	std_logic_vector(7 downto 0);
	-- Data bus out
	DO			:	out	std_logic_vector(7 downto 0);
	
	-- Port pins
	RXD			:	in	std_logic;
	TXD			:	out	std_logic
	);
end component;

---------
-- CPU
---------

component T65 is
	port(
		Mode    : in  std_logic_vector(1 downto 0);      -- "00" => 6502, "01" => 65C02, "10" => 65C816
		Res_n   : in  std_logic;
		Enable  : in  std_logic;
		Clk     : in  std_logic;
		Rdy     : in  std_logic;
		Abort_n : in  std_logic;
		IRQ_n   : in  std_logic;
		NMI_n   : in  std_logic;
		SO_n    : in  std_logic;
		R_W_n   : out std_logic;
		Sync    : out std_logic;
		EF      : out std_logic;
		MF      : out std_logic;
		XF      : out std_logic;
		ML_n    : out std_logic;
		VP_n    : out std_logic;
		VDA     : out std_logic;
		VPA     : out std_logic;
		A       : out std_logic_vector(23 downto 0);
		DI      : in  std_logic_vector(7 downto 0);
		DO      : out std_logic_vector(7 downto 0)
	);
end component;

-----------------
-- MC6845 CRTC
-----------------

component mc6845 is
port (
	CLOCK		:	in	std_logic;
	CLKEN		:	in	std_logic;
	nRESET		:	in	std_logic;

	-- Bus interface
	ENABLE		:	in	std_logic;
	R_nW		:	in	std_logic;
	RS			:	in	std_logic;
	DI			:	in	std_logic_vector(7 downto 0);
	DO			:	out	std_logic_vector(7 downto 0);

	-- Display interface
	VSYNC		:	out	std_logic;
	HSYNC		:	out	std_logic;
	DE			:	out	std_logic;
	CURSOR		:	out	std_logic;
	LPSTB		:	in	std_logic;
	
	-- Memory interface
	MA			:	out	std_logic_vector(13 downto 0);
	RA			:	out	std_logic_vector(4 downto 0)
	);
end component;

-------------------------
-- "VIDPROC" Video ULA
-------------------------

component vidproc is
port (
	CLOCK		:	in	std_logic;
	-- Clock enable qualifies display cycles (interleaved with CPU cycles)
	CLKEN		:	in	std_logic;
	nRESET		:	in	std_logic;
	
	-- Clock enable output to CRTC
	CLKEN_CRTC	:	out	std_logic;
	
	-- Bus interface
	ENABLE		:	in	std_logic;
	A0			:	in	std_logic;
	-- CPU data bus (for register writes)
	DI_CPU		:	in	std_logic_vector(7 downto 0);
	-- Display RAM data bus (for display data fetch)
	DI_RAM		:	in	std_logic_vector(7 downto 0);
	
	-- Control interface
	nINVERT		:	in	std_logic;
	DISEN		:	in	std_logic;
	CURSOR		:	in	std_logic;
	
	-- Video in (teletext mode)
	R_IN		:	in	std_logic;
	G_IN		:	in	std_logic;
	B_IN		:	in	std_logic;
	
	-- Video out
	R			:	out	std_logic;
	G			:	out	std_logic;
	B			:	out	std_logic
	);
end component;

--------------------------------
-- SAA5050 Teletext Generator
--------------------------------

component saa5050 is
port (
	CLOCK		:	in	std_logic;	
	-- 6 MHz dot clock enable
	CLKEN		:	in	std_logic;
	-- Async reset
	nRESET		:	in	std_logic;
	
	-- Character data input (in the bus clock domain)
	DI_CLOCK	:	in	std_logic;
	DI_CLKEN	:	in	std_logic;
	DI			:	in	std_logic_vector(6 downto 0);
	
	-- Timing inputs
	-- General line reset (not used)
	GLR			:	in	std_logic; -- /HSYNC
	-- Data entry window - high during VSYNC.
	-- Resets ROM row counter and drives 'flash' signal
	DEW			:	in	std_logic; -- VSYNC
	-- Character rounding select - high during even field
	CRS			:	in	std_logic; -- FIELD
	-- Load output shift register enable - high during active video
	LOSE		:	in	std_logic; -- DE
	
	-- Video out
	R			:	out	std_logic;
	G			:	out	std_logic;
	B			:	out	std_logic;
	Y			:	out	std_logic
	);
end component;

--------------
-- 6522 VIA
--------------

component M6522 is
  port (

    I_RS              : in    std_logic_vector(3 downto 0);
    I_DATA            : in    std_logic_vector(7 downto 0);
    O_DATA            : out   std_logic_vector(7 downto 0);
    O_DATA_OE_L       : out   std_logic;

    I_RW_L            : in    std_logic;
    I_CS1             : in    std_logic;
    I_CS2_L           : in    std_logic;

    O_IRQ_L           : out   std_logic; -- note, not open drain
    -- port a
    I_CA1             : in    std_logic;
    I_CA2             : in    std_logic;
    O_CA2             : out   std_logic;
    O_CA2_OE_L        : out   std_logic;

    I_PA              : in    std_logic_vector(7 downto 0);
    O_PA              : out   std_logic_vector(7 downto 0);
    O_PA_OE_L         : out   std_logic_vector(7 downto 0);

    -- port b
    I_CB1             : in    std_logic;
    O_CB1             : out   std_logic;
    O_CB1_OE_L        : out   std_logic;

    I_CB2             : in    std_logic;
    O_CB2             : out   std_logic;
    O_CB2_OE_L        : out   std_logic;

    I_PB              : in    std_logic_vector(7 downto 0);
    O_PB              : out   std_logic_vector(7 downto 0);
    O_PB_OE_L         : out   std_logic_vector(7 downto 0);

	-- FIXME: Revisit timing in this component.  Does it really need a 4x clock?
    I_P2_H            : in    std_logic; -- high for phase 2 clock  ____----__
    RESET_L           : in    std_logic;
    ENA_4             : in    std_logic; -- clk enable (4x system clock rate)
    CLK               : in    std_logic
    );
end component;

-----------------------------
-- PS/2 Keyboard Emulation
-----------------------------

component keyboard is
port (
	CLOCK		:	in	std_logic;
	nRESET		:	in	std_logic;
	CLKEN_1MHZ	:	in	std_logic;
	
	-- PS/2 interface
	PS2_CLK		:	in	std_logic;
	PS2_DATA	:	in	std_logic;
	
	-- If 1 then column is incremented automatically at
	-- 1 MHz rate
	AUTOSCAN	:	in	std_logic;
	
	COLUMN		:	in	std_logic_vector(3 downto 0);
	ROW			:	in	std_logic_vector(2 downto 0);
	
	-- 1 when currently selected key is down (AUTOSCAN disabled)
	KEYPRESS	:	out	std_logic;
	-- 1 when any key is down (except row 0)
	INT			:	out	std_logic;
	-- BREAK key output - 1 when pressed
	BREAK_OUT	:	out	std_logic;
	
	-- DIP switch inputs
	DIP_SWITCH	:	in	std_logic_vector(7 downto 0)
	);
end component;

-----------------------------
-- SN76489 sound generator
-----------------------------

component sn76489_top is

  generic (
    clock_div_16_g : integer := 1
  );
  port (
    clock_i    : in  std_logic;
    clock_en_i : in  std_logic;
    res_n_i    : in  std_logic;
    ce_n_i     : in  std_logic;
    we_n_i     : in  std_logic;
    ready_o    : out std_logic;
    d_i        : in  std_logic_vector(0 to 7);
    aout_o     : out signed(0 to 7)
  );

end component;

component i2s_intf is
generic(
	mclk_rate : positive := 12000000;
	sample_rate : positive := 8000;
	preamble : positive := 1; -- I2S
	word_length : positive := 16
	);

port (
	-- 2x MCLK in (e.g. 24 MHz for WM8731 USB mode)
	CLK			:	in	std_logic;
	nRESET		:	in	std_logic;
	
	-- Parallel IO
	PCM_INL		:	out	std_logic_vector(word_length - 1 downto 0);
	PCM_INR		:	out	std_logic_vector(word_length - 1 downto 0);
	PCM_OUTL	:	in	std_logic_vector(word_length - 1 downto 0);
	PCM_OUTR	:	in	std_logic_vector(word_length - 1 downto 0);
	
	-- Codec interface (right justified mode)
	-- MCLK is generated at half of the CLK input
	I2S_MCLK	:	out	std_logic;
	-- LRCLK is equal to the sample rate and is synchronous to
	-- MCLK.  It must be related to MCLK by the oversampling ratio
	-- given in the codec datasheet.
	I2S_LRCLK	:	out	std_logic;
	
	-- Data is shifted out on the falling edge of BCLK, sampled
	-- on the rising edge.  The bit rate is determined such that
	-- it is fast enough to fit preamble + word_length bits into
	-- each LRCLK half cycle.  The last cycle of each word may be 
	-- stretched to fit to LRCLK.  This is OK at least for the 
	-- WM8731 codec.
	-- The first falling edge of each timeslot is always synchronised
	-- with the LRCLK edge.
	I2S_BCLK	:	out	std_logic;
	-- Output bitstream
	I2S_DOUT	:	out	std_logic;
	-- Input bitstream
	I2S_DIN		:	in	std_logic
	);
end component;

component i2c_loader is
generic (
	-- Address of slave to be loaded
	device_address : integer := 16#1a#;
	-- Number of retries to allow before stopping
	num_retries : integer := 0;
	-- Length of clock divider in bits.  Resulting bus frequency is
	-- CLK/2^(log2_divider + 2)
	log2_divider : integer := 6
);

port (
	CLK			:	in	std_logic;
	nRESET		:	in	std_logic;
	
	I2C_SCL		:	inout	std_logic;
	I2C_SDA		:	inout	std_logic;
	
	IS_DONE		:	out std_logic;
	IS_ERROR	:	out	std_logic
	);
end component;

component debugger is
generic (
	-- Set this for a reasonable half flash duration relative to the
	-- clock frequency
	flash_divider	:	natural := 24
	);
port (
	CLOCK		:	in	std_logic;
	nRESET		:	in	std_logic;
	-- CPU clock enable in
	CLKEN_IN	:	in	std_logic;
	-- Gated clock enable back out to CPU
	CLKEN_OUT	:	out	std_logic;
	-- CPU IRQ in
	nIRQ_IN		:	in	std_logic;
	-- Gated IRQ back out to CPU (no interrupts when single stepping)
	nIRQ_OUT	:	out	std_logic;
	
	-- CPU
	A_CPU		:	in	std_logic_vector(15 downto 0);
	R_nW		:	in	std_logic;
	SYNC		:	in	std_logic;
	
	-- Aux bus input for display in hex
	AUX_BUS		:	in	std_logic_vector(15 downto 0);	
	
	-- Controls
	-- RUN or HALT CPU
	RUN			:	in	std_logic;
	-- Push button to single-step in HALT mode
	nSTEP		:	in	std_logic;
	-- Push button to cycle display mode
	nMODE		:	in	std_logic;
	-- Push button to cycle display digit in edit mode
	nDIGIT		:	in	std_logic;
	-- Push button to cycle digit value in edit mode
	nSET		:	in	std_logic;
	
	-- Output to display
	DIGIT3		:	out	std_logic_vector(6 downto 0);
	DIGIT2		:	out	std_logic_vector(6 downto 0);
	DIGIT1		:	out	std_logic_vector(6 downto 0);
	DIGIT0		:	out	std_logic_vector(6 downto 0);
	
	LED_BREAKPOINT	:	out	std_logic;
	LED_WATCHPOINT	:	out	std_logic
	);
end component;

-------------
-- Signals
-------------

-- Master clock - 32 MHz
signal pll_reset	:	std_logic;
signal pll_locked	:	std_logic;
signal clock		:	std_logic;
signal hard_reset_n	:	std_logic;
signal reset_n		:	std_logic;

-- Clock enable counter
-- CPU and video cycles are interleaved.  The CPU runs at 2 MHz (every 16th
-- cycle) and the video subsystem is enabled on every odd cycle.
signal clken_counter	:	unsigned(4 downto 0);
signal cpu_cycle		:	std_logic; -- Qualifies all 2 MHz cycles
signal cpu_cycle_mask	:	std_logic; -- Set to mask CPU cycles until 1 MHz cycle is complete
signal cpu_clken		:	std_logic; -- 2 MHz cycles in which the CPU is enabled
signal cpu_debug_clken	:	std_logic; -- CPU clken return from hardware debugger
-- IO cycles are out of phase with the CPU
signal vid_clken		:	std_logic; -- 16 MHz video cycles
signal mhz4_clken		:	std_logic; -- Used by 6522
signal mhz2_clken		:	std_logic; -- Used for latching CPU address for clock stretch
signal mhz1_clken		:	std_logic; -- 1 MHz bus and associated peripherals, 6522 phase 2
-- SAA5050 needs a 6 MHz clock enable relative to a 24 MHz clock
signal ttxt_clken_counter	:	unsigned(1 downto 0);
signal ttxt_clken		:	std_logic;

-- Testing
signal test_uart_do		:	std_logic_vector(7 downto 0);

-- Debugger connections
signal debug_irq_in_n	:	std_logic;
signal debug_aux		:	std_logic_vector(15 downto 0);

-- CPU signals
signal cpu_mode			:	std_logic_vector(1 downto 0);
signal cpu_ready		:	std_logic;
signal cpu_abort_n		:	std_logic;
signal cpu_irq_n		:	std_logic;
signal cpu_nmi_n		:	std_logic;
signal cpu_so_n			:	std_logic;
signal cpu_r_nw			:	std_logic;
signal cpu_sync			:	std_logic;
signal cpu_ef			:	std_logic;
signal cpu_mf			:	std_logic;
signal cpu_xf			:	std_logic;
signal cpu_ml_n			:	std_logic;
signal cpu_vp_n			:	std_logic;
signal cpu_vda			:	std_logic;
signal cpu_vpa			:	std_logic;
signal cpu_a			:	std_logic_vector(23 downto 0);
signal cpu_di			:	std_logic_vector(7 downto 0);
signal cpu_do			:	std_logic_vector(7 downto 0);

-- CRTC signals
signal crtc_clken		:	std_logic;
signal crtc_do			:	std_logic_vector(7 downto 0);
signal crtc_vsync		:	std_logic;
signal crtc_hsync		:	std_logic;
signal crtc_de			:	std_logic;
signal crtc_cursor		:	std_logic;
signal crtc_lpstb		:	std_logic := '0';
signal crtc_ma			:	std_logic_vector(13 downto 0);
signal crtc_ra			:	std_logic_vector(4 downto 0);

-- Decoded display address after address translation for hardware
-- scrolling
signal display_a		:	std_logic_vector(14 downto 0);

-- "VIDPROC" signals
signal vidproc_invert_n	:	std_logic;
signal vidproc_disen	:	std_logic;
signal r_in				:	std_logic;
signal g_in				:	std_logic;
signal b_in				:	std_logic;
signal r_out			:	std_logic;
signal g_out			:	std_logic;
signal b_out			:	std_logic;

-- SAA5050 signals
signal ttxt_glr			:	std_logic;
signal ttxt_dew			:	std_logic;
signal ttxt_crs			:	std_logic;
signal ttxt_lose		:	std_logic;
signal ttxt_r			:	std_logic;
signal ttxt_g			:	std_logic;
signal ttxt_b			:	std_logic;
signal ttxt_y			:	std_logic;

-- System VIA signals
signal sys_via_do		:	std_logic_vector(7 downto 0);
signal sys_via_do_oe_n	:	std_logic;
signal sys_via_irq_n	:	std_logic;
signal sys_via_ca1_in	:	std_logic := '0';
signal sys_via_ca2_in	:	std_logic := '0';
signal sys_via_ca2_out	:	std_logic;
signal sys_via_ca2_oe_n	:	std_logic;
signal sys_via_pa_in	:	std_logic_vector(7 downto 0);
signal sys_via_pa_out	:	std_logic_vector(7 downto 0);
signal sys_via_pa_oe_n	:	std_logic_vector(7 downto 0);
signal sys_via_cb1_in	:	std_logic := '0';
signal sys_via_cb1_out	:	std_logic;
signal sys_via_cb1_oe_n	:	std_logic;
signal sys_via_cb2_in	:	std_logic := '0';
signal sys_via_cb2_out	:	std_logic;
signal sys_via_cb2_oe_n	:	std_logic;
signal sys_via_pb_in	:	std_logic_vector(7 downto 0);
signal sys_via_pb_out	:	std_logic_vector(7 downto 0);
signal sys_via_pb_oe_n	:	std_logic_vector(7 downto 0);

-- User VIA signals
signal user_via_do		:	std_logic_vector(7 downto 0);
signal user_via_do_oe_n	:	std_logic;
signal user_via_irq_n	:	std_logic;
signal user_via_ca1_in	:	std_logic := '0';
signal user_via_ca2_in	:	std_logic := '0';
signal user_via_ca2_out	:	std_logic;
signal user_via_ca2_oe_n	:	std_logic;
signal user_via_pa_in	:	std_logic_vector(7 downto 0);
signal user_via_pa_out	:	std_logic_vector(7 downto 0);
signal user_via_pa_oe_n	:	std_logic_vector(7 downto 0);
signal user_via_cb1_in	:	std_logic := '0';
signal user_via_cb1_out	:	std_logic;
signal user_via_cb1_oe_n	:	std_logic;
signal user_via_cb2_in	:	std_logic := '0';
signal user_via_cb2_out	:	std_logic;
signal user_via_cb2_oe_n	:	std_logic;
signal user_via_pb_in	:	std_logic_vector(7 downto 0);
signal user_via_pb_out	:	std_logic_vector(7 downto 0);
signal user_via_pb_oe_n	:	std_logic_vector(7 downto 0);

-- IC32 latch on System VIA
signal ic32				:	std_logic_vector(7 downto 0);
signal sound_enable_n	:	std_logic;
signal speech_read_n	:	std_logic;
signal speech_write_n	:	std_logic;
signal keyb_enable_n	:	std_logic;
signal disp_addr_offs	:	std_logic_vector(1 downto 0);
signal caps_lock_led_n	:	std_logic;
signal shift_lock_led_n	:	std_logic;

-- Keyboard
signal keyb_column		:	std_logic_vector(3 downto 0);
signal keyb_row			:	std_logic_vector(2 downto 0);
signal keyb_out			:	std_logic;
signal keyb_int			:	std_logic;
signal keyb_break		:	std_logic;

-- Sound generator
signal sound_ready		:	std_logic;
signal sound_di			:	std_logic_vector(7 downto 0);
signal sound_ao			:	signed(7 downto 0);
signal pcm_inl			:	std_logic_vector(15 downto 0);
signal pcm_inr			:	std_logic_vector(15 downto 0);

-- Memory enables
signal ram_enable		:	std_logic;		-- 0x0000
signal rom_enable		:	std_logic;		-- 0x8000 (BASIC/sideways ROMs)
signal mos_enable		:	std_logic;		-- 0xC000
-- IO region enables
signal io_fred			:	std_logic;		-- 0xFC00 (1 MHz bus)
signal io_jim			:	std_logic;		-- 0xFD00 (1 MHz bus)
signal io_sheila		:	std_logic;		-- 0xFE00 (System peripherals)
-- SHIELA
signal crtc_enable		:	std_logic;		-- 0xFE00-FE07
signal acia_enable		:	std_logic;		-- 0xFE08-FE0F
signal serproc_enable	:	std_logic;		-- 0xFE10-FE1F
signal vidproc_enable	:	std_logic;		-- 0xFE20-FE2F
signal romsel_enable	:	std_logic;		-- 0xFE30-FE3F
signal sys_via_enable	:	std_logic;		-- 0xFE40-FE5F
signal user_via_enable	:	std_logic;		-- 0xFE60-FE7F
signal fddc_enable		:	std_logic;		-- 0xFE80-FE9F
signal adlc_enable		:	std_logic;		-- 0xFEA0-FEBF (Econet)
signal adc_enable		:	std_logic;		-- 0xFEC0-FEDF
signal tube_enable		:	std_logic;		-- 0xFEE0-FEFF

-- ROM select latch
signal romsel			:	std_logic_vector(3 downto 0);

signal mhz1_enable		:	std_logic;		-- Set for access to any 1 MHz peripheral

begin
	-------------------------
	-- COMPONENT INSTANCES
	-------------------------

	-- 32 MHz master clock
	pll:	pll32 port map (
		pll_reset,
		CLOCK_24(0),
		clock,
		pll_locked );
		
	-- Hardware debugger block (single-step, breakpoints)
	debug:	debugger port map (
		clock,
		hard_reset_n,
		cpu_clken,
		cpu_debug_clken,
		debug_irq_in_n,
		cpu_irq_n,
		cpu_a(15 downto 0), cpu_r_nw, cpu_sync,
		debug_aux,
		SW(8), -- RUN
		KEY(3), -- STEP
		KEY(2), -- MODE
		KEY(1), -- DIGIT
		KEY(0), -- SET
		HEX3, HEX2, HEX1, HEX0,
		LEDR(3), -- BREAKPOINT
		LEDR(2) -- WATCHPOINT
		);
	
	-- Test UART
	test_uart : simple_uart port map (
		clock,
		reset_n,
		io_fred,
		cpu_r_nw,
		cpu_a(0),
		cpu_do,
		test_uart_do,
		UART_RXD,
		UART_TXD
		);

	-- 6502 CPU
	cpu : T65 port map (
		cpu_mode,
		reset_n,
		cpu_debug_clken,
		clock,
		cpu_ready,
		cpu_abort_n,
		cpu_irq_n,
		cpu_nmi_n,
		cpu_so_n,
		cpu_r_nw,
		cpu_sync,
		cpu_ef,
		cpu_mf,
		cpu_xf,
		cpu_ml_n,
		cpu_vp_n,
		cpu_vda,
		cpu_vpa,
		cpu_a,
		cpu_di,
		cpu_do );
		
	crtc : mc6845 port map (
		clock,
		crtc_clken,
		reset_n,
		crtc_enable,
		cpu_r_nw,
		cpu_a(0),
		cpu_do,
		crtc_do,
		crtc_vsync,
		crtc_hsync,
		crtc_de,
		crtc_cursor,
		crtc_lpstb,
		crtc_ma,
		crtc_ra );
		
	video_ula : vidproc port map (
		clock,
		vid_clken,
		reset_n,
		crtc_clken,
		vidproc_enable,
		cpu_a(0),
		cpu_do,
		SRAM_DQ(7 downto 0),
		vidproc_invert_n,
		vidproc_disen,
		crtc_cursor,
		r_in, g_in, b_in,
		r_out, g_out, b_out
		);
		
	teletext : saa5050 port map (
		CLOCK_24(0), -- This runs at 6 MHz, which we can't derive from the 32 MHz clock
		ttxt_clken,
		reset_n,
		clock, -- Data input is synchronised from the bus clock domain
		vid_clken,
		SRAM_DQ(6 downto 0),
		ttxt_glr,
		ttxt_dew,
		ttxt_crs,
		ttxt_lose,
		ttxt_r, ttxt_g, ttxt_b, ttxt_y
		);
		
	-- System VIA
	system_via : m6522 port map (
		cpu_a(3 downto 0),
		cpu_do,
		sys_via_do,
		sys_via_do_oe_n,
		cpu_r_nw,
		sys_via_enable,
		'0', -- nCS2
		sys_via_irq_n,
		sys_via_ca1_in,
		sys_via_ca2_in,
		sys_via_ca2_out,
		sys_via_ca2_oe_n,
		sys_via_pa_in,
		sys_via_pa_out,
		sys_via_pa_oe_n,
		sys_via_cb1_in,
		sys_via_cb1_out,
		sys_via_cb1_oe_n,
		sys_via_cb2_in,
		sys_via_cb2_out,
		sys_via_cb2_oe_n,
		sys_via_pb_in,
		sys_via_pb_out,
		sys_via_pb_oe_n,
		mhz1_clken,
		hard_reset_n, -- System VIA is reset by power on reset only
		mhz4_clken,
		clock
		);
	
	-- User VIA
	user_via : m6522 port map (
		cpu_a(3 downto 0),
		cpu_do,
		user_via_do,
		user_via_do_oe_n,
		cpu_r_nw,
		user_via_enable,
		'0', -- nCS2
		user_via_irq_n,
		user_via_ca1_in,
		user_via_ca2_in,
		user_via_ca2_out,
		user_via_ca2_oe_n,
		user_via_pa_in,
		user_via_pa_out,
		user_via_pa_oe_n,
		user_via_cb1_in,
		user_via_cb1_out,
		user_via_cb1_oe_n,
		user_via_cb2_in,
		user_via_cb2_out,
		user_via_cb2_oe_n,
		user_via_pb_in,
		user_via_pb_out,
		user_via_pb_oe_n,
		mhz1_clken,
		reset_n,
		mhz4_clken,
		clock
		);
		
	-- Keyboard
	keyb : keyboard port map (
		clock, hard_reset_n, mhz1_clken,
		PS2_CLK, PS2_DAT,
		keyb_enable_n,
		keyb_column,
		keyb_row,
		keyb_out,
		keyb_int,
		keyb_break,
		SW(7 downto 0)
		);
		
	-- Sound generator (and drive logic for I2S codec)
	sound : sn76489_top port map (
		clock, mhz4_clken,
		reset_n, '0', sound_enable_n,
		sound_ready, sound_di,
		sound_ao
		);
	i2s : i2s_intf port map (
		CLOCK_24(0), reset_n,
		pcm_inl, pcm_inr,
		std_logic_vector(sound_ao) & "00000000",
		std_logic_vector(sound_ao) & "00000000",
		AUD_XCK, AUD_DACLRCK,
		AUD_BCLK, AUD_DACDAT, AUD_ADCDAT
		);
	i2c : i2c_loader 
		generic map (
			log2_divider => 7
		)
		port map (
			clock, reset_n,
			I2C_SCLK, I2C_SDAT,
			LEDR(5), -- IS_DONE
			LEDR(4) -- IS_ERROR
		);
	
	-- Asynchronous reset
	-- PLL is reset by external reset switch
	pll_reset <= not SW(9);
	-- Keyboard and System VIA are reset by external reset switch or PLL being out of lock
	hard_reset_n <= not (pll_reset or not pll_locked);
	-- Rest of system is reset by all of the above plus the keyboard BREAK key
	reset_n <= hard_reset_n and not keyb_break;
		
	-- Clock enable generation - 32 MHz clock split into 32 cycles
	-- CPU is on 0 and 16 (but can be masked by 1 MHz bus accesses)
	-- Video is on all odd cycles (16 MHz)
	-- 1 MHz cycles are on cycle 31 (1 MHz)	
	vid_clken <= clken_counter(0); -- 1,3,5...
	mhz4_clken <= clken_counter(0) and clken_counter(1) and clken_counter(2); -- 7/15/23/31
	mhz2_clken <= mhz4_clken and clken_counter(3); -- 15/31
	mhz1_clken <= mhz2_clken and clken_counter(4); -- 31
	cpu_cycle <= not (clken_counter(0) or clken_counter(1) or clken_counter(2) or clken_counter(3)); -- 0/16
	cpu_clken <= cpu_cycle and not cpu_cycle_mask;
	
	clk_gen: process(clock,reset_n)
	begin
		if reset_n = '0' then
			clken_counter <= (others => '0');
		elsif rising_edge(clock) then
			clken_counter <= clken_counter + 1;
		end if;
	end process;
	
	cycle_stretch: process(clock,reset_n)
	begin
		if reset_n = '0' then
			cpu_cycle_mask <= '0';
		elsif rising_edge(clock) and mhz2_clken = '1' then
			if mhz1_enable = '1' and cpu_cycle_mask = '0' then
				-- Block CPU cycles until 1 MHz cycle has completed
				cpu_cycle_mask <= '1';
			end if;
			if mhz1_clken = '1' then
				-- CPU can run again
				-- FIXME: This may not be correct in terms of CPU cycles, but it
				-- should work
				cpu_cycle_mask <= '0';
			end if;
		end if;
	end process;
	
	ttxt_clk_gen: process(CLOCK_24(0),reset_n)
	begin
		if reset_n = '0' then
			ttxt_clken_counter <= (others => '0');
		elsif rising_edge(CLOCK_24(0)) then
			ttxt_clken_counter <= ttxt_clken_counter + 1;
		end if;
	end process;
	
	-- 6 MHz clock enable for SAA5050
	ttxt_clken <= '1' when ttxt_clken_counter = 0 else '0';

	-- CPU configuration and fixed signals
	cpu_mode <= "00"; -- 6502
	cpu_ready <= '1';
	cpu_abort_n <= '1';
	cpu_nmi_n <= '1';
	cpu_so_n <= '1';
	
	-- Address decoding
	-- 0x0000 = 32 KB SRAM
	-- 0x8000 = 16 KB BASIC/Sideways ROMs
	-- 0xC000 = 16 KB MOS ROM
	--
	-- IO regions are mapped into a hole in the MOS.  There are three regions:
	-- 0xFC00 = FRED
	-- 0xFD00 = JIM
	-- 0xFE00 = SHEILA
	ram_enable <= not cpu_a(15);
	rom_enable <= cpu_a(15) and not cpu_a(14);
	mos_enable <= cpu_a(15) and cpu_a(14) and not (io_fred or io_jim or io_sheila);
	io_fred <= '1' when cpu_a(15 downto 8) = "11111100" else '0';
	io_jim <= '1' when cpu_a(15 downto 8) = "11111101" else '0';
	io_sheila <= '1' when cpu_a(15 downto 8) = "11111110" else '0';
	-- The following IO regions are accessed at 1 MHz and hence will stall the
	-- CPU accordingly
	mhz1_enable <= io_fred or io_jim or
		adc_enable or sys_via_enable or user_via_enable or
		serproc_enable or acia_enable or crtc_enable;
	
	-- SHEILA address demux
	-- All the system peripherals are mapped into this page as follows:
	-- 0xFE00 - 0xFE07 = MC6845 CRTC
	-- 0xFE08 - 0xFE0F = MC6850 ACIA (Serial/Tape)
	-- 0xFE10 - 0xFE1F = Serial ULA
	-- 0xFE20 - 0xFE2F = Video ULA
	-- 0xFE30 - 0xFE3F = Paged ROM select latch
	-- 0xFE40 - 0xFE5F = System VIA (6522)
	-- 0xFE60 - 0xFE7F = User VIA (6522)
	-- 0xFE80 - 0xFE9F = 8271 Floppy disc controller
	-- 0xFEA0 - 0xFEBF = 68B54 ADLC for Econet
	-- 0xFEC0 - 0xFEDF = uPD7002 ADC
	-- 0xFEE0 - 0xFEFF = Tube ULA
	process(cpu_a,io_sheila)
	begin
		-- All regions normally de-selected
		crtc_enable <= '0';
		acia_enable <= '0';
		serproc_enable <= '0';
		vidproc_enable <= '0';
		romsel_enable <= '0';
		sys_via_enable <= '0';
		user_via_enable <= '0';
		fddc_enable <= '0';
		adlc_enable <= '0';
		adc_enable <= '0';
		tube_enable <= '0';
		
		if io_sheila = '1' then
			case cpu_a(7 downto 5) is
				when "000" =>
					-- 0xFE00
					if cpu_a(4) = '0' then
						if cpu_a(3) = '0' then
							-- 0xFE00
							crtc_enable <= '1';
						else
							-- 0xFE08
							acia_enable <= '1';
						end if;
					else
						-- 0xFE10
						serproc_enable <= '1';
					end if;
				when "001" =>
					-- 0xFE20
					if cpu_a(4) = '0' then
						-- 0xFE20
						vidproc_enable <= '1';
					else
						-- 0xFE30
						romsel_enable <= '1';
					end if;
				when "010" => sys_via_enable <= '1';	-- 0xFE40
				when "011" => user_via_enable <= '1';	-- 0xFE60
				when "100" => fddc_enable <= '1';		-- 0xFE80
				when "101" => adlc_enable <= '1';		-- 0xFEA0
				when "110" => adc_enable <= '1';		-- 0xFEC0
				when "111" => tube_enable <= '1';		-- 0xFEE0
				when others =>
					null;
			end case;
		end if;
	end process;
	
	-- CPU data bus mux and interrupts
	cpu_di <=
		SRAM_DQ(7 downto 0) when ram_enable = '1' else
		FL_DQ		when rom_enable = '1' else
		FL_DQ		when mos_enable = '1' else
		crtc_do		when crtc_enable = '1' else
		"00000010"	when acia_enable = '1' else
		sys_via_do	when sys_via_enable = '1' else
		user_via_do	when user_via_enable = '1' else
		test_uart_do when io_fred = '1' else
		(others => '0'); -- un-decoded locations are pulled down by RP1
	debug_irq_in_n <= sys_via_irq_n and user_via_irq_n; -- route IRQ through debugger
	--cpu_irq_n <= sys_via_irq_n and user_via_irq_n;
	
	-- ROMs are in external flash
	FL_RST_N <= reset_n;
	FL_CE_N <= '0';
	FL_OE_N <= '0';
	FL_WE_N <= '1';
	FL_ADDR(13 downto 0) <= cpu_a(13 downto 0);
	FL_ADDR(21 downto 16) <= (others => '0');
	FL_ADDR(15 downto 14) <= 
		"00" when mos_enable = '1' else
		"01" when rom_enable = '1' and romsel(1 downto 0) = "11" else	-- BASIC
		"10" when rom_enable = '1' and romsel(1 downto 0) = "00" else	-- DFS/MMC
		"11"; -- Spare
		
	-- SRAM bus
	SRAM_UB_N <= '1';
	SRAM_LB_N <= '0';
	SRAM_CE_N <= '0';
	SRAM_OE_N <= '0';
	SRAM_DQ(15 downto 8) <= (others => '0');
	
	-- Synchronous outputs to SRAM
	process(clock,reset_n)
	variable ram_write : std_logic;
	begin		
		ram_write := ram_enable and not cpu_r_nw;
	
		if reset_n = '0' then
			SRAM_WE_N <= '1';
			SRAM_DQ(7 downto 0) <= (others => 'Z');
		elsif rising_edge(clock) then
			-- Default to inputs
			SRAM_DQ(7 downto 0) <= (others => 'Z');
			
			-- Register SRAM signals to outputs (clock must be at least 2x CPU clock)
			if vid_clken = '1' then
				-- Fetch data from previous CPU cycle
				SRAM_WE_N <= not ram_write;
				SRAM_ADDR <= "00" & cpu_a(15 downto 0);
				if ram_write = '1' then
					SRAM_DQ(7 downto 0) <= cpu_do;
				end if;
			else
				-- Fetch data from previous display cycle
				SRAM_WE_N <= '1';
				SRAM_ADDR <= "000" & display_a;
			end if;
		end if;
	end process;
	
	-- Address translation logic for calculation of display address
	process(crtc_ma,crtc_ra,disp_addr_offs)
	variable aa : unsigned(3 downto 0);
	begin
		if crtc_ma(12) = '0' then
			-- No adjustment
			aa := unsigned(crtc_ma(11 downto 8));
		else
			-- Address adjusted according to screen mode to compensate for
			-- wrap at 0x8000.
			case disp_addr_offs is
			when "00" =>
				-- Mode 3 - restart at 0x4000
				aa := unsigned(crtc_ma(11 downto 8)) + 8;
			when "01" =>
				-- Mode 6 - restart at 0x6000
				aa := unsigned(crtc_ma(11 downto 8)) + 12;
			when "10" =>
				-- Mode 0,1,2 - restart at 0x3000
				aa := unsigned(crtc_ma(11 downto 8)) + 6;
			when "11" =>
				-- Mode 4,5 - restart at 0x5800
				aa := unsigned(crtc_ma(11 downto 8)) + 11;
			when others =>
				null;
			end case;
		end if;
		
		if crtc_ma(13) = '0' then
			-- HI RES
			display_a <= std_logic_vector(aa(3 downto 0)) & crtc_ma(7 downto 0) & crtc_ra(2 downto 0);
		else
			-- TTX VDU
			display_a <= std_logic(aa(3)) & "1111" & crtc_ma(9 downto 0);
		end if;
	end process;
	
	-- VIDPROC
	vidproc_invert_n <= '1';
	vidproc_disen <= crtc_de and not crtc_ra(3); -- DISEN is masked off by RA(3) for MODEs 3 and 6
	r_in <= ttxt_r;
	g_in <= ttxt_g;
	b_in <= ttxt_b;
	
	-- SAA5050
	ttxt_glr <= not crtc_hsync;
	ttxt_dew <= crtc_vsync;
	ttxt_crs <= not crtc_ra(0);
	ttxt_lose <= crtc_de;
	
	-- CRTC drives video out (CSYNC on HSYNC output, VSYNC high)
	VGA_HS <= not (crtc_hsync xor crtc_vsync);
	VGA_VS <= '1';
	VGA_R <= r_out & r_out & r_out & r_out;
	VGA_G <= g_out & g_out & g_out & g_out;
	VGA_B <= b_out & b_out & b_out & b_out;
	
	-- Connections to System VIA
	-- ADC
	sys_via_cb1_in <= '1'; -- /EOC
	-- CRTC
	sys_via_ca1_in <= crtc_vsync;
	sys_via_cb2_in <= crtc_lpstb;
	-- Keyboard
	sys_via_ca2_in <= keyb_int;
	sys_via_pa_in(7) <= keyb_out;
	sys_via_pa_in(6 downto 0) <= sys_via_pa_out(6 downto 0); -- Must loop back output pins or keyboard won't work
	keyb_column <= sys_via_pa_out(3 downto 0);
	keyb_row <= sys_via_pa_out(6 downto 4);
	-- Sound
	sound_di <= sys_via_pa_out;
	-- Others (idle until missing bits implemented)
	sys_via_pb_in(7 downto 4) <= (others => '1');
	
	-- Connections to User VIA (user port is output on green LEDs)
	user_via_ca1_in <= '1'; -- Pulled up
	--LEDG <= user_via_pb_out;
	
	-- MMBEEB
	user_via_cb1_in <= user_via_pb_out(1);
	SD_SCLK <= user_via_pb_out(1); -- SCLK
	SD_MOSI <= user_via_pb_out(0); -- SDO
	SD_nCS <= '0'; -- CS
	user_via_cb2_in <= SD_MISO; -- SDI
	user_via_pb_in <= user_via_pb_out;
	
	-- ROM select latch
	process(clock,reset_n)
	begin
		if reset_n = '0' then
			romsel <= (others => '0');
		elsif rising_edge(clock) then
			if romsel_enable = '1' and cpu_r_nw = '0' then
				romsel <= cpu_do(3 downto 0);
			end if;
		end if;
	end process;
	
	-- IC32 latch
	sound_enable_n <= ic32(0);
	speech_write_n <= ic32(1);
	speech_read_n <= ic32(2);
	keyb_enable_n <= ic32(3);
	disp_addr_offs <= ic32(5 downto 4);
	caps_lock_led_n <= ic32(6);
	shift_lock_led_n <= ic32(7);
	
	process(clock,reset_n)
	variable bit_num : integer;
	begin
		bit_num := to_integer(unsigned(sys_via_pb_out(2 downto 0)));
	
		if reset_n = '0' then
			ic32 <= (others => '0');
		elsif rising_edge(clock) then
			ic32(bit_num) <= sys_via_pb_out(3);
		end if;
	end process;
	
	-- Keyboard LEDs
	LEDR(0) <= not caps_lock_led_n;
	LEDR(1) <= not shift_lock_led_n;
	
	-----------------
	-- DEBUG STUFF
	-----------------
	
	GPIO_0(0) <= not (crtc_hsync xor crtc_vsync);
	GPIO_0(1) <= crtc_de;

end architecture;

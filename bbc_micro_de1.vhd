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
	FL_RST_N	:	in		std_logic;
	FL_OE_N		:	in		std_logic;
	FL_WE_N		:	in		std_logic;
	
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

-----------------------
-- 7 segment display
-----------------------

component seg7 is
port (
	D			: in std_logic_vector(3 downto 0);
	Q			: out std_logic_vector(6 downto 0)
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

-------------
-- MOS ROM
-------------

component os12 IS
	PORT
	(
		address		: IN STD_LOGIC_VECTOR (13 DOWNTO 0);
		clock		: IN STD_LOGIC ;
		q			: OUT STD_LOGIC_VECTOR (7 DOWNTO 0)
	);
end component;

--------------
-- Test ROM
--------------

component ehbasic IS
	PORT
	(
		address		: IN STD_LOGIC_VECTOR (13 DOWNTO 0);
		clock		: IN STD_LOGIC ;
		q			: OUT STD_LOGIC_VECTOR (7 DOWNTO 0)
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

-------------
-- Signals
-------------

-- Master clock - 32 MHz
signal pll_reset	:	std_logic;
signal pll_locked	:	std_logic;
signal clock		:	std_logic;
signal reset_n		:	std_logic;

-- Clock enable counter
-- CPU and video cycles are interleaved.  The CPU runs at 2 MHz (every 16th
-- cycle) and the video subsystem is enabled on every odd cycle.
signal clken_counter	:	unsigned(3 downto 0);
signal slomo_counter	:	unsigned(17 downto 0);
signal cpu_clken		:	std_logic;
signal vid_clken		:	std_logic;

-- Testing
signal test_uart_do		:	std_logic_vector(7 downto 0);

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

-- "VIDPROC" signals
signal vidproc_invert_n	:	std_logic;
signal r_in				:	std_logic;
signal g_in				:	std_logic;
signal b_in				:	std_logic;
signal r_out			:	std_logic;
signal g_out			:	std_logic;
signal b_out			:	std_logic;

-- MOS ROM signals
signal mos_d			:	std_logic_vector(7 downto 0);

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

-- Common VIA signals
signal via_clken		:	std_logic;
signal via_phase2		:	std_logic;

-- IC32 latch on System VIA
signal ic32				:	std_logic_vector(7 downto 0);
signal sound_enable_n	:	std_logic;
signal speech_read_n	:	std_logic;
signal speech_write_n	:	std_logic;
signal keyb_enable_n	:	std_logic;
signal disp_addr_offs	:	std_logic_vector(1 downto 0);
signal caps_lock_led_n	:	std_logic;
signal shift_lock_led_n	:	std_logic;

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
	
	-- Display of address bus	
	addr3 : seg7 port map (cpu_a(15 downto 12), HEX3);
	addr2 : seg7 port map (cpu_a(11 downto 8), HEX2);
	addr1 : seg7 port map (cpu_a(7 downto 4), HEX1);
	addr0 : seg7 port map (cpu_a(3 downto 0), HEX0);
	
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
		cpu_clken,
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
		crtc_de,
		crtc_cursor,
		r_in, g_in, b_in,
		r_out, g_out, b_out
		);
		
	-- MOS ROM
--	mos : os12 port map (
--		cpu_a(13 downto 0),
--		clock,
--		mos_d );
	test_rom : ehbasic port map (
		cpu_a(13 downto 0),
		clock, mos_d );
		
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
		via_phase2,
		reset_n,
		via_clken,
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
		via_phase2,
		reset_n,
		via_clken,
		clock
		);
	
	-- Asynchronous reset
	pll_reset <= not SW(9);
	reset_n <= not (pll_reset or not pll_locked);
		
	-- Clock enable generation
	-- CPU is on cycle 0 of 16 (2 MHz)
	-- Video is on all odd cycles
	cpu_clken <= '1' when clken_counter = 0 and (SW(8) = '1' or slomo_counter = 0) else '0';
	vid_clken <= clken_counter(0);
	-- FIXME: VIAs - this is wrong.  They should actually run at 1MHz (so 4 MHz clken in this
	-- case) and the CPU should be stalled until the 1 MHz bus cycle is complete
	via_clken <= clken_counter(0) and clken_counter(1);
	via_phase2 <= clken_counter(3);
	process(clock,reset_n)
	begin
		if reset_n = '0' then
			clken_counter <= (others => '0');
			slomo_counter <= (others => '0');
		elsif rising_edge(clock) then
			clken_counter <= clken_counter + 1;
			if clken_counter = 0 then
				slomo_counter <= slomo_counter + 1;
			end if;
		end if;
	end process;

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
		mos_d		when mos_enable = '1' else
		"11111111"	when rom_enable = '1' else
		crtc_do		when crtc_enable = '1' else
		sys_via_do	when sys_via_enable = '1' else
		user_via_do	when user_via_enable = '1' else
		test_uart_do when io_fred = '1' else
		SRAM_DQ(7 downto 0);
	cpu_irq_n <= sys_via_irq_n and user_via_irq_n;
	
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
				SRAM_ADDR <= "000" & crtc_ma(11 downto 0) & crtc_ra(2 downto 0);
			end if;
		end if;
	end process;
	
	-- VIDPROC
	vidproc_invert_n <= '1';
	r_in <= '0';
	g_in <= '0';
	b_in <= '0';
	
	GPIO_0(0) <= crtc_clken;
	GPIO_0(1) <= crtc_hsync;
	GPIO_0(2) <= crtc_vsync;
	GPIO_0(3) <= not (crtc_hsync xor crtc_vsync);
	
	-- CRTC drives video out (CSYNC on HSYNC output, VSYNC high)
	VGA_HS <= not (crtc_hsync xor crtc_vsync);
	VGA_VS <= '1';
	VGA_R <= r_out & r_out & r_out & r_out;
	VGA_G <= g_out & g_out & g_out & g_out;
	VGA_B <= b_out & b_out & b_out & b_out;
	
	-- Connections to System VIA
	sys_via_ca1_in <= crtc_vsync;
	sys_via_cb1_in <= crtc_lpstb;
	
	-- Connections to User VIA (user port is output on green LEDs)
	--LEDG <= user_via_pb_out;
	
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
	
	LEDG <= ic32;
	
	-- Keyboard LEDs
	LEDR(0) <= not caps_lock_led_n;
	LEDR(1) <= not shift_lock_led_n;
	
	-- Hack for jumper links
	process(sys_via_pa_out)
	begin
		sys_via_pa_in <= sys_via_pa_out;
		
		if sys_via_pa_out(6 downto 4) = "000" then
			case to_integer(unsigned(sys_via_pa_out(3 downto 0))) is
			when 2 => sys_via_pa_in(7) <= SW(7);
			when 3 => sys_via_pa_in(7) <= SW(6);
			when 4 => sys_via_pa_in(7) <= SW(5);
			when 5 => sys_via_pa_in(7) <= SW(4);
			when 6 => sys_via_pa_in(7) <= SW(3);
			when 7 => sys_via_pa_in(7) <= SW(2);
			when 8 => sys_via_pa_in(7) <= SW(1);
			when 9 => sys_via_pa_in(7) <= SW(0);
			when others => sys_via_pa_in(7) <= '1';
			end case;
		else
			sys_via_pa_in(7) <= '1';
		end if;
	end process;
	
--	process(clock,reset_n)
--	begin
--		if reset_n = '0' then
--			LEDG <= "00000000";
--		elsif rising_edge(clock) then
--			if cpu_a(15 downto 0) = "0000001010001111" and cpu_r_nw = '0' then
--				LEDG <= cpu_do;
--			end if;
--		end if;
--	end process;
		
end architecture;

-- Simple UART for debugging.  Fixed baud rate, no handshaking or parity
--
-- (C) 2011 Mike Stirling
--

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity simple_uart is
generic (
	f_clock		:	natural	:= 50000000;
	baud_rate	:	natural	:= 115200
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
end entity;

-- FIXME: RX really should have some mechanism for syncing with
-- the middle of the bit.  Currently it is just pot luck!

architecture rtl of simple_uart is
constant baud_const	: natural := (f_clock / baud_rate) - 1;
subtype baud_div_t is natural range 0 to baud_const;
subtype bitcount_t is natural range 0 to 7;

type state_t is (Idle, Sync, Start, Data, Stop);

-- Clock enables
signal rx_clken : std_logic;
signal tx_clken : std_logic;
-- Baud rate divider for receive side
signal rx_baud_div : baud_div_t;
-- Baud rate divider for transmit side
signal tx_baud_div: baud_div_t;
-- Receive holding register
signal rx_reg : std_logic_vector(7 downto 0);
-- Receive holding register full flag
signal rx_ready : std_logic;
-- Receive shift register
signal rx_sreg : std_logic_vector(7 downto 0);
-- Receive state
signal rx_state : state_t;
-- Receive bit count
signal rx_bitcount : bitcount_t;
-- Latched rxd for falling edge detection
signal rxd_latch : std_logic;
-- Transmit input register
signal tx_reg : std_logic_vector(7 downto 0);
-- Transmit input register full flag
signal tx_busy : std_logic;
-- Transmit shift register
signal tx_sreg : std_logic_vector(7 downto 0);
-- Transmit state
signal tx_state : state_t;
-- Transmit bit count
signal tx_bitcount : bitcount_t;
-- Internal (async) representation of txd
signal txd_i : std_logic;
-- Latched enable for rising edge detection
signal enable_latch : std_logic;
begin
	rx_clken <= '1' when rx_baud_div = 0 else '0';
	tx_clken <= '1' when tx_baud_div = 0 else '0';
	
	txd_i <=
		'0' when tx_state = Start else
		tx_sreg(0) when tx_state = Data else
		'1';

	process(CLOCK,nRESET)
	begin
		if nRESET = '0' then
			TXD <= '1';
		else
			-- Register TXD to output
			TXD <= txd_i;
		end if;
	end process;
	
	process(CLOCK,nRESET)
	begin
		if nRESET = '0' then
			rx_reg <= (others => '0');
			rx_sreg <= (others => '0');
			rx_ready <= '0';
			rx_state <= Idle;
			rx_bitcount <= 0;
			
			tx_reg <= (others => '0');
			tx_sreg <= (others => '0');
			tx_busy <= '0';
			tx_state <= Idle;
			tx_bitcount <= 0;

			rx_baud_div <= baud_const;
			tx_baud_div <= baud_const;
			enable_latch <= '0';
			DO <= (others => '0');
		elsif rising_edge(CLOCK) then
			-- Latch enable signal for edge detection
			enable_latch <= ENABLE;
			-- Latch rxd for falling edge (start) detection)
			rxd_latch <= RXD;
			
			-- Baud rate generation.  Tx and Rx are done separately so that
			-- the receive counter can be adjusted when the falling edge of a
			-- start bit is detected
			if rx_baud_div = 0 then
				-- Reload
				rx_baud_div <= baud_const;
			else
				rx_baud_div <= rx_baud_div - 1;
			end if;
			if rx_state = Idle and RXD = '0' and rxd_latch = '1' then
				-- Reset rx baud divider to mid point when a start bit occurs
				rx_baud_div <= baud_const / 2;
			end if;
			if tx_baud_div = 0 then
				-- Reload
				tx_baud_div <= baud_const;
			else
				tx_baud_div <= tx_baud_div - 1;
			end if;
			
			-- Handle bus writes - edge triggered to avoid unintentionally
			-- re-transmitting the same character
			if ENABLE = '1' and enable_latch = '0' and R_nW = '0' then
				if S_nD = '0' and tx_busy = '0' then
					-- Write to data register and tx register is empty
					tx_reg <= DI;
					tx_busy <= '1';
				end if;
				-- All other writes are discarded
			end if;
			-- Handle bus reads - edge triggered so DO is held for the
			-- duration of ENABLE, but a subsequent read with !rx_ready
			-- will return 0
			if ENABLE = '1' and enable_latch = '0' and R_nW = '1' then
				if S_nD = '0' then
					-- Receive data register read
					if rx_ready = '1' then
						DO <= rx_reg;
						rx_ready <= '0';
					else
						DO <= (others => '0');
					end if;
				else
					-- Status register read
					-- bit 0 = TX busy
					-- bit 1 = RX ready
					DO <= "000000" & rx_ready & tx_busy;
				end if;
			end if;
			
			if tx_busy = '1' and (tx_state = Idle or tx_state = Stop) then
				-- Start a new transmit cycle.  TXD remains idle until
				-- the start of the next baud period, hence the additional
				-- 'Sync' state.
				tx_sreg <= tx_reg;
				tx_busy <= '0';
				tx_state <= Sync;
				tx_bitcount <= 7;
			end if;
			
			-- Transmitter
			if tx_clken = '1' and not (tx_state = Idle) then
				case tx_state is
				when Sync =>
					tx_state <= Start;
				when Start =>
					tx_state <= Data;				
				when Data =>
					tx_sreg <= '0' & tx_sreg(7 downto 1);
					if tx_bitcount = 0 then
						tx_state <= Stop;
					else
						tx_bitcount <= tx_bitcount - 1;
					end if;				
				when others =>
					-- Return to idle
					tx_state <= Idle;
				end case;
			end if;
			
			-- Receiver
			if rx_clken = '1' then
				case rx_state is
				when Idle =>
					if RXD = '0' then
						-- Start bit detected - next bit is data, so we
						-- skip the start state on the rx side
						rx_state <= Data;
						rx_bitcount <= 7;
					end if;
				when Data =>
					rx_sreg <= RXD & rx_sreg(7 downto 1);
					if rx_bitcount = 0 then
						rx_state <= Stop;
					else
						rx_bitcount <= rx_bitcount - 1;
					end if;
				when others =>
					-- Only latch output if stop bit matches and the
					-- rx register is empty
					if RXD = '1' and rx_ready = '0' then
						rx_reg <= rx_sreg;
						rx_ready <= '1';
					end if;
					-- Return to idle
					rx_state <= Idle;
				end case;
			end if;
		end if;
	end process;

end architecture;

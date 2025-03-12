library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL;

entity tb_pluto_hodgart_msk_system is
-- Testbench has no ports
end tb_pluto_hodgart_msk_system;

architecture Behavioral of tb_pluto_hodgart_msk_system is
    -- Component declarations
    component pluto_hodgart_msk_top_with_decimator
        Port ( 
            -- System signals
            sys_clk      : in STD_LOGIC;
            sys_resetn   : in STD_LOGIC;
            
            -- AXI-Stream interface for IQ samples from ADC
            s_axis_aclk     : in STD_LOGIC;
            s_axis_tvalid   : in STD_LOGIC;
            s_axis_tready   : out STD_LOGIC;
            s_axis_tdata    : in STD_LOGIC_VECTOR(31 downto 0);
            
            -- AXI-Stream interface for demodulated data output
            m_axis_aclk     : in STD_LOGIC;
            m_axis_tvalid   : out STD_LOGIC;
            m_axis_tready   : in STD_LOGIC;
            m_axis_tdata    : out STD_LOGIC_VECTOR(7 downto 0);
            
            -- AXI-Lite slave interface for configuration
            s_axi_aclk      : in STD_LOGIC;
            s_axi_aresetn   : in STD_LOGIC;
            s_axi_awaddr    : in STD_LOGIC_VECTOR(7 downto 0);
            s_axi_awvalid   : in STD_LOGIC;
            s_axi_awready   : out STD_LOGIC;
            s_axi_wdata     : in STD_LOGIC_VECTOR(31 downto 0);
            s_axi_wvalid    : in STD_LOGIC;
            s_axi_wready    : out STD_LOGIC;
            s_axi_bresp     : out STD_LOGIC_VECTOR(1 downto 0);
            s_axi_bvalid    : out STD_LOGIC;
            s_axi_bready    : in STD_LOGIC;
            s_axi_araddr    : in STD_LOGIC_VECTOR(7 downto 0);
            s_axi_arvalid   : in STD_LOGIC;
            s_axi_arready   : out STD_LOGIC;
            s_axi_rdata     : out STD_LOGIC_VECTOR(31 downto 0);
            s_axi_rresp     : out STD_LOGIC_VECTOR(1 downto 0);
            s_axi_rvalid    : out STD_LOGIC;
            s_axi_rready    : in STD_LOGIC;
            
            -- Status outputs
            status_lock     : out STD_LOGIC;
            debug_decim     : out STD_LOGIC
        );
    end component;
    
    -- Constants
    constant CLK_PERIOD : time := 16.276 ns;  -- 61.44 MHz
    constant SAMPLES_PER_SYMBOL : integer := 4;   -- After decimation
    
    -- Test data parameters
    constant MSK_FREQ_HZ : real := 447150.0;  -- MSK center frequency
    constant SYMBOL_RATE_HZ : real := 27100.0; -- Symbol rate
    constant AMPLITUDE : integer := 20000;     -- Signal amplitude
    constant NUM_TEST_BITS : integer := 64;    -- Number of test bits
    
    -- Clock and reset signals
    signal clk : std_logic := '0';
    signal resetn : std_logic := '0';
    
    -- AXI-Stream input signals
    signal s_axis_tvalid : std_logic := '0';
    signal s_axis_tready : std_logic;
    signal s_axis_tdata : std_logic_vector(31 downto 0) := (others => '0');
    
    -- AXI-Stream output signals
    signal m_axis_tvalid : std_logic;
    signal m_axis_tready : std_logic := '1';  -- Always ready to receive
    signal m_axis_tdata : std_logic_vector(7 downto 0);
    
    -- AXI-Lite signals
    signal s_axi_awaddr : std_logic_vector(7 downto 0) := (others => '0');
    signal s_axi_awvalid : std_logic := '0';
    signal s_axi_awready : std_logic;
    signal s_axi_wdata : std_logic_vector(31 downto 0) := (others => '0');
    signal s_axi_wvalid : std_logic := '0';
    signal s_axi_wready : std_logic;
    signal s_axi_bresp : std_logic_vector(1 downto 0);
    signal s_axi_bvalid : std_logic;
    signal s_axi_bready : std_logic := '1';  -- Always ready for response
    signal s_axi_araddr : std_logic_vector(7 downto 0) := (others => '0');
    signal s_axi_arvalid : std_logic := '0';
    signal s_axi_arready : std_logic;
    signal s_axi_rdata : std_logic_vector(31 downto 0);
    signal s_axi_rresp : std_logic_vector(1 downto 0);
    signal s_axi_rvalid : std_logic;
    signal s_axi_rready : std_logic := '1';  -- Always ready for response
    
    -- Status signals
    signal status_lock : std_logic;
    signal debug_decim : std_logic;
    
    -- Test data signals
    signal test_phase : real := 0.0;
    signal test_symbol_phase : real := 0.0;
    signal test_bit_index : integer := 0;
    signal test_bits : std_logic_vector(NUM_TEST_BITS-1 downto 0) := (others => '0');
    signal current_test_bit : std_logic := '0';
    signal prev_test_bit : std_logic := '0';
    signal symbol_count : integer := 0;
    signal phase_inc_multiplier : real := 1.0;  -- For frequency variation
    
    -- Sine/cosine functions
    function sin_wave(phase : real) return integer is
        variable result : integer;
    begin
        result := integer(real(AMPLITUDE) * sin(phase));
        return result;
    end function;
    
    function cos_wave(phase : real) return integer is
        variable result : integer;
    begin
        result := integer(real(AMPLITUDE) * cos(phase));
        return result;
    end function;
    
    -- MSK modulation function
    procedure generate_msk_sample(
        signal phase : inout real;
        signal symbol_phase : inout real;
        current_bit : in std_logic;
        prev_bit : in std_logic;
        signal i_out : out integer;
        signal q_out : out integer
    ) is
        variable i_val, q_val : integer;
        variable phase_inc : real;
        constant PHASE_INC_CONSTANT : real := 2.0 * MATH_PI * MSK_FREQ_HZ / (61.44e6);
        constant SYMBOL_PHASE_INC : real := MATH_PI / 2.0 / (61.44e6 / SYMBOL_RATE_HZ);
    begin
        -- Determine phase increment based on current bit
        if current_bit = '1' then
            phase_inc := PHASE_INC_CONSTANT + SYMBOL_PHASE_INC;
        else
            phase_inc := PHASE_INC_CONSTANT - SYMBOL_PHASE_INC;
        end if;
        
        -- Update phase
        phase <= phase + phase_inc;
        symbol_phase <= symbol_phase + SYMBOL_PHASE_INC;
        
        -- Generate I/Q components
        i_val := cos_wave(phase);
        q_val := sin_wave(phase);
        
        i_out <= i_val;
        q_out <= q_val;
    end procedure;
    
    -- Test bit pattern generation
    procedure generate_test_bits(signal test_bits : out std_logic_vector) is
        variable temp_bits : std_logic_vector(NUM_TEST_BITS-1 downto 0);
    begin
        -- Generate a mix of 0s and 1s with some bit transitions
        -- Using a known pattern for easier verification
        temp_bits := x"A55A" & x"3C3C" & x"F00F" & x"5555";
        test_bits <= temp_bits;
    end procedure;
    
    -- AXI-Lite register write procedure
    procedure axi_lite_write(
        signal awaddr : out std_logic_vector(7 downto 0);
        signal awvalid : out std_logic;
        signal wdata : out std_logic_vector(31 downto 0);
        signal wvalid : out std_logic;
        signal bready : out std_logic;
        signal awready : in std_logic;
        signal wready : in std_logic;
        signal bvalid : in std_logic;
        addr : in std_logic_vector(7 downto 0);
        data : in std_logic_vector(31 downto 0)
    ) is
    begin
        -- Address phase
        awaddr <= addr;
        awvalid <= '1';
        wait until awready = '1';
        wait for CLK_PERIOD;
        awvalid <= '0';
        
        -- Data phase
        wdata <= data;
        wvalid <= '1';
        wait until wready = '1';
        wait for CLK_PERIOD;
        wvalid <= '0';
        
        -- Response phase
        bready <= '1';
        wait until bvalid = '1';
        wait for CLK_PERIOD;
        bready <= '1';
    end procedure;
    
    -- Signal for modulation
    signal i_sample, q_sample : integer := 0;
    
begin
    -- Clock generation
    process
    begin
        clk <= '0';
        wait for CLK_PERIOD/2;
        clk <= '1';
        wait for CLK_PERIOD/2;
    end process;
    
    -- Reset generation
    process
    begin
        resetn <= '0';
        wait for 10 * CLK_PERIOD;
        resetn <= '1';
        wait;
    end process;
    
    -- DUT instantiation
    uut: pluto_hodgart_msk_top_with_decimator
    port map (
        sys_clk => clk,
        sys_resetn => resetn,
        
        s_axis_aclk => clk,
        s_axis_tvalid => s_axis_tvalid,
        s_axis_tready => s_axis_tready,
        s_axis_tdata => s_axis_tdata,
        
        m_axis_aclk => clk,
        m_axis_tvalid => m_axis_tvalid,
        m_axis_tready => m_axis_tready,
        m_axis_tdata => m_axis_tdata,
        
        s_axi_aclk => clk,
        s_axi_aresetn => resetn,
        s_axi_awaddr => s_axi_awaddr,
        s_axi_awvalid => s_axi_awvalid,
        s_axi_awready => s_axi_awready,
        s_axi_wdata => s_axi_wdata,
        s_axi_wvalid => s_axi_wvalid,
        s_axi_wready => s_axi_wready,
        s_axi_bresp => s_axi_bresp,
        s_axi_bvalid => s_axi_bvalid,
        s_axi_bready => s_axi_bready,
        s_axi_araddr => s_axi_araddr,
        s_axi_arvalid => s_axi_arvalid,
        s_axi_arready => s_axi_arready,
        s_axi_rdata => s_axi_rdata,
        s_axi_rresp => s_axi_rresp,
        s_axi_rvalid => s_axi_rvalid,
        s_axi_rready => s_axi_rready,
        
        status_lock => status_lock,
        debug_decim => debug_decim
    );
    
    -- Test process
    process
    begin
        -- Wait for reset to complete
        wait until resetn = '1';
        wait for 10 * CLK_PERIOD;
        
        -- Generate test bit pattern
        generate_test_bits(test_bits);
        
        -- Configure the system
        -- Write samples_per_symbol register (0x04) with value 4
        axi_lite_write(s_axi_awaddr, s_axi_awvalid, s_axi_wdata, s_axi_wvalid, s_axi_bready,
                      s_axi_awready, s_axi_wready, s_axi_bvalid,
                      x"04", x"00000004");
                      
        -- Enable decimation (0x0C)
        axi_lite_write(s_axi_awaddr, s_axi_awvalid, s_axi_wdata, s_axi_wvalid, s_axi_bready,
                      s_axi_awready, s_axi_wready, s_axi_bvalid,
                      x"0C", x"00000001");
        
        -- Wait a bit
        wait for 100 * CLK_PERIOD;
        
        -- Start sending MSK modulated data
        current_test_bit <= test_bits(0);
        
        -- Main test loop - send MSK modulated samples 
        for i in 0 to 200000 loop  -- Send many samples to allow decimation and demodulation
            -- Update test bit every symbol period
            if symbol_count >= integer(61.44e6 / SYMBOL_RATE_HZ) then
                symbol_count <= 0;
                prev_test_bit <= current_test_bit;
                
                -- Get next test bit
                if test_bit_index < NUM_TEST_BITS-1 then
                    test_bit_index <= test_bit_index + 1;
                else
                    test_bit_index <= 0;  -- Wrap around
                end if;
                
                current_test_bit <= test_bits(test_bit_index);
            else
                symbol_count <= symbol_count + 1;
            end if;
            
            -- Generate MSK modulated sample
            generate_msk_sample(test_phase, test_symbol_phase, current_test_bit, prev_test_bit, i_sample, q_sample);
            
            -- Pack I/Q data into AXIS data word
            s_axis_tdata <= std_logic_vector(to_signed(q_sample, 16)) & std_logic_vector(to_signed(i_sample, 16));
            s_axis_tvalid <= '1';
            
            -- Wait for ready
            wait until s_axis_tready = '1';
            wait for CLK_PERIOD;
            
            -- Every 10 symbols, introduce a small frequency variation to test tracking
            if i mod 10000 = 0 then
                phase_inc_multiplier <= 1.0 + 0.0001 * real((i mod 30000) - 15000) / 15000.0;
            end if;
        end loop;
        
        -- Finish
        s_axis_tvalid <= '0';
        wait for 10000 * CLK_PERIOD;
        
        -- Check if lock was achieved
        if status_lock = '1' then
            report "TEST PASSED: Demodulator achieved lock" severity note;
        else
            report "TEST FAILED: Demodulator did not achieve lock" severity error;
        end if;
        
        -- End simulation
        report "Test completed" severity note;
        wait;
    end process;
    
    -- Test process for disabling/enabling decimation (stress test)
    process
    begin
        wait until resetn = '1';
        wait for 50000 * CLK_PERIOD;
        
        -- Toggle decimation a few times to test robustness
        for i in 1 to 3 loop
            -- Disable decimation
            axi_lite_write(s_axi_awaddr, s_axi_awvalid, s_axi_wdata, s_axi_wvalid, s_axi_bready,
                          s_axi_awready, s_axi_wready, s_axi_bvalid,
                          x"0C", x"00000000");
                          
            -- Wait a while
            wait for 10000 * CLK_PERIOD;
            
            -- Re-enable decimation
            axi_lite_write(s_axi_awaddr, s_axi_awvalid, s_axi_wdata, s_axi_wvalid, s_axi_bready,
                          s_axi_awready, s_axi_wready, s_axi_bvalid,
                          x"0C", x"00000001");
                          
            -- Wait a while
            wait for 20000 * CLK_PERIOD;
        end loop;
        
        wait;
    end process;
    
end Behavioral;
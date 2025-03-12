library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity pluto_hodgart_msk_top_with_decimator is
    Port ( 
        -- System signals
        sys_clk      : in STD_LOGIC;  -- PLUTO system clock
        sys_resetn   : in STD_LOGIC;  -- System reset, active low
        
        -- AXI-Stream interface for IQ samples from ADC
        s_axis_aclk     : in STD_LOGIC;
        s_axis_tvalid   : in STD_LOGIC;
        s_axis_tready   : out STD_LOGIC;
        s_axis_tdata    : in STD_LOGIC_VECTOR(31 downto 0);  -- Packed IQ samples
        
        -- AXI-Stream interface for demodulated data output
        m_axis_aclk     : in STD_LOGIC;
        m_axis_tvalid   : out STD_LOGIC;
        m_axis_tready   : in STD_LOGIC;
        m_axis_tdata    : out STD_LOGIC_VECTOR(7 downto 0);  -- Demodulated data bytes
        
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
end pluto_hodgart_msk_top_with_decimator;

architecture Behavioral of pluto_hodgart_msk_top_with_decimator is
    -- Component declarations
    component hodgart_msk_demodulator
        Port ( 
            clk         : in STD_LOGIC;
            reset_n     : in STD_LOGIC;
            i_in        : in STD_LOGIC_VECTOR(15 downto 0);
            q_in        : in STD_LOGIC_VECTOR(15 downto 0);
            sample_valid: in STD_LOGIC;
            bit_out     : out STD_LOGIC;
            bit_valid   : out STD_LOGIC;
            samples_per_symbol : in STD_LOGIC_VECTOR(7 downto 0);
            lock_detect : out STD_LOGIC
        );
    end component;
    
    component pluto_hodgart_msk_decimator
        Port ( 
            clk         : in STD_LOGIC;
            reset_n     : in STD_LOGIC;
            i_in        : in STD_LOGIC_VECTOR(15 downto 0);
            q_in        : in STD_LOGIC_VECTOR(15 downto 0);
            sample_valid_in : in STD_LOGIC;
            i_out       : out STD_LOGIC_VECTOR(15 downto 0);
            q_out       : out STD_LOGIC_VECTOR(15 downto 0);
            sample_valid_out : out STD_LOGIC;
            enable      : in STD_LOGIC;
            stage1_valid : out STD_LOGIC
        );
    end component;
    
    -- Configuration registers
    signal reg_control      : std_logic_vector(31 downto 0) := (others => '0');
    signal reg_samples_per_symbol : std_logic_vector(7 downto 0) := x"04";  -- Default 4 samples per symbol (after decimation)
    signal reg_doppler_shift : std_logic_vector(15 downto 0) := (others => '0'); -- Doppler compensation
    signal reg_decimation_enable : std_logic := '1';  -- Enable decimation by default
    
    -- AXI-Lite interface signals
    type axi_state_t is (IDLE, WRITE_DATA, WRITE_RESP, READ_DATA);
    signal axi_state : axi_state_t := IDLE;
    signal axi_addr_reg : std_logic_vector(7 downto 0);
    
    -- Internal signals
    signal i_raw : std_logic_vector(15 downto 0);
    signal q_raw : std_logic_vector(15 downto 0);
    signal sample_valid_raw : std_logic;
    
    -- Decimated signals
    signal i_decimated : std_logic_vector(15 downto 0);
    signal q_decimated : std_logic_vector(15 downto 0);
    signal sample_valid_decimated : std_logic;
    signal stage1_valid_debug : std_logic;
    
    -- Doppler-compensated signals (after decimation)
    signal i_compensated : std_logic_vector(15 downto 0);
    signal q_compensated : std_logic_vector(15 downto 0);
    signal sample_valid_compensated : std_logic;
    
    -- Demodulator outputs
    signal bit_out : std_logic;
    signal bit_valid : std_logic;
    signal bit_buffer : std_logic_vector(7 downto 0) := (others => '0');
    signal bit_count : integer range 0 to 7 := 0;
    signal byte_valid : std_logic := '0';
    signal lock_detect : std_logic;
    
    -- Clock domain crossing signals
    signal bit_valid_sync : std_logic_vector(2 downto 0) := (others => '0');
    signal bit_out_sync : std_logic_vector(2 downto 0) := (others => '0');
    signal lock_detect_sync : std_logic_vector(2 downto 0) := (others => '0');
    
    -- Doppler compensation signals
    signal doppler_phase : signed(15 downto 0) := (others => '0');
    signal doppler_phase_inc : signed(15 downto 0) := (others => '0');
    
    -- Sine/cosine lookup table for Doppler compensation
    type sin_table_t is array(0 to 63) of integer range -32768 to 32767;
    constant SIN_TABLE : sin_table_t := (
         0,  3212,  6393,  9512, 12539, 15446, 18204, 20787,
     23170, 25329, 27245, 28898, 30273, 31356, 32137, 32609,
     32767, 32609, 32137, 31356, 30273, 28898, 27245, 25329,
     23170, 20787, 18204, 15446, 12539,  9512,  6393,  3212,
         0, -3212, -6393, -9512,-12539,-15446,-18204,-20787,
    -23170,-25329,-27245,-28898,-30273,-31356,-32137,-32609,
    -32767,-32609,-32137,-31356,-30273,-28898,-27245,-25329,
    -23170,-20787,-18204,-15446,-12539, -9512, -6393, -3212
    );
    
    -- Function to look up sine value from table
    function sin_lookup(phase: signed) return signed is
        variable index : integer;
        variable sine_val : signed(15 downto 0);
    begin
        index := to_integer(phase(13 downto 8));  -- Use 6 bits for lookup
        sine_val := to_signed(SIN_TABLE(index), 16);
        return sine_val;
    end function;
    
    -- Function to look up cosine value from table
    function cos_lookup(phase: signed) return signed is
        variable index : integer;
        variable cos_val : signed(15 downto 0);
    begin
        index := to_integer(phase(13 downto 8) + 16) mod 64;  -- Offset by π/2
        cos_val := to_signed(SIN_TABLE(index), 16);
        return cos_val;
    end function;
    
begin
    -- Decimator instance
    decimator_inst: pluto_hodgart_msk_decimator
    port map (
        clk => s_axis_aclk,
        reset_n => sys_resetn,
        i_in => i_raw,
        q_in => q_raw,
        sample_valid_in => sample_valid_raw,
        i_out => i_decimated,
        q_out => q_decimated,
        sample_valid_out => sample_valid_decimated,
        enable => reg_decimation_enable,
        stage1_valid => stage1_valid_debug
    );
    
    -- MSK demodulator instance
    msk_demod_inst: hodgart_msk_demodulator
    port map (
        clk => s_axis_aclk,
        reset_n => sys_resetn,
        i_in => i_compensated,
        q_in => q_compensated,
        sample_valid => sample_valid_compensated,
        bit_out => bit_out,
        bit_valid => bit_valid,
        samples_per_symbol => reg_samples_per_symbol,
        lock_detect => lock_detect
    );
    
    -- Debug output - expose stage1_valid for debugging with scope
    debug_decim <= stage1_valid_debug;
    
    -- Input data extraction
    process(s_axis_aclk)
    begin
        if rising_edge(s_axis_aclk) then
            if sys_resetn = '0' then
                i_raw <= (others => '0');
                q_raw <= (others => '0');
                sample_valid_raw <= '0';
            else
                if s_axis_tvalid = '1' then
                    i_raw <= s_axis_tdata(15 downto 0);
                    q_raw <= s_axis_tdata(31 downto 16);
                    sample_valid_raw <= '1';
                else
                    sample_valid_raw <= '0';
                end if;
            end if;
        end if;
    end process;
    
end Behavioral;
    
    -- Doppler compensation (after decimation)
    process(s_axis_aclk)
        variable i_raw_signed, q_raw_signed : signed(15 downto 0);
        variable doppler_cos, doppler_sin : signed(15 downto 0);
        variable i_comp, q_comp : signed(31 downto 0);
    begin
        if rising_edge(s_axis_aclk) then
            if sys_resetn = '0' then
                sample_valid_compensated <= '0';
                doppler_phase <= (others => '0');
                i_compensated <= (others => '0');
                q_compensated <= (others => '0');
                doppler_phase_inc <= (others => '0');
            else
                -- Calculate adjusted Doppler shift for decimated rate
                -- Divide Doppler shift by total decimation factor (approx 568)
                doppler_phase_inc <= shift_right(signed(reg_doppler_shift), 9);  -- Divide by 512 (close to 568)
                
                -- Apply Doppler compensation to decimated data
                if sample_valid_decimated = '1' then
                    -- Convert to signed
                    i_raw_signed := signed(i_decimated);
                    q_raw_signed := signed(q_decimated);
                    
                    -- Update Doppler phase
                    doppler_phase <= doppler_phase + doppler_phase_inc;
                    
                    -- Get sine and cosine for Doppler compensation
                    doppler_cos := cos_lookup(doppler_phase);
                    doppler_sin := sin_lookup(doppler_phase);
                    
                    -- Perform complex multiplication for frequency correction
                    -- (i_comp + j*q_comp) = (i_dec + j*q_dec) * (cos - j*sin)
                    i_comp := (i_raw_signed * doppler_cos + q_raw_signed * doppler_sin) / 32768;
                    q_comp := (q_raw_signed * doppler_cos - i_raw_signed * doppler_sin) / 32768;
                    
                    -- Scale back to 16-bit
                    i_compensated <= std_logic_vector(resize(i_comp, 16));
                    q_compensated <= std_logic_vector(resize(q_comp, 16));
                    
                    sample_valid_compensated <= '1';
                else
                    sample_valid_compensated <= '0';
                end if;
            end if;
        end if;
    end process;
    
    -- Always ready to receive data
    s_axis_tready <= '1';
    
    -- Bit to byte conversion
    process(s_axis_aclk)
    begin
        if rising_edge(s_axis_aclk) then
            if sys_resetn = '0' then
                bit_count <= 0;
                bit_buffer <= (others => '0');
                byte_valid <= '0';
            else
                byte_valid <= '0';
                
                if bit_valid = '1' then
                    -- Shift in new bit
                    bit_buffer <= bit_buffer(6 downto 0) & bit_out;
                    
                    -- Increment bit counter
                    if bit_count = 7 then
                        bit_count <= 0;
                        byte_valid <= '1';
                    else
                        bit_count <= bit_count + 1;
                    end if;
                end if;
            end if;
        end if;
    end process;
    
    -- Clock domain crossing for output axis
    process(m_axis_aclk)
    begin
        if rising_edge(m_axis_aclk) then
            -- Two-stage synchronizer for bit_valid and bit_out
            bit_valid_sync <= bit_valid_sync(1 downto 0) & byte_valid;
            lock_detect_sync <= lock_detect_sync(1 downto 0) & lock_detect;
        end if;
    end process;
    
    -- Output data handling (demodulator to AXIS)
    process(m_axis_aclk)
    begin
        if rising_edge(m_axis_aclk) then
            if sys_resetn = '0' then
                m_axis_tvalid <= '0';
                m_axis_tdata <= (others => '0');
            else
                -- Detect rising edge of synchronized bit_valid
                if bit_valid_sync(2) = '0' and bit_valid_sync(1) = '1' then
                    m_axis_tdata <= bit_buffer;
                    m_axis_tvalid <= '1';
                elsif m_axis_tready = '1' and m_axis_tvalid = '1' then
                    m_axis_tvalid <= '0';
                end if;
            end if;
        end if;
    end process;
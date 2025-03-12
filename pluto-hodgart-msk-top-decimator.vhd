library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity pluto_hodgart_msk_top_decimated is
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
        status_lock     : out STD_LOGIC
    );
end pluto_hodgart_msk_top_decimated;

architecture Behavioral of pluto_hodgart_msk_top_decimated is
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
    
    -- Configuration registers
    signal reg_control      : std_logic_vector(31 downto 0) := (others => '0');
    signal reg_samples_per_symbol : std_logic_vector(7 downto 0) := x"04";  -- Default 4 samples per symbol after decimation
    signal reg_doppler_shift : std_logic_vector(15 downto 0) := (others => '0'); -- Doppler compensation
    signal reg_decimation_factor : std_logic_vector(15 downto 0) := x"0238"; -- Default 568 decimation factor
    
    -- AXI-Lite interface signals
    type axi_state_t is (IDLE, WRITE_DATA, WRITE_RESP, READ_DATA);
    signal axi_state : axi_state_t := IDLE;
    signal axi_addr_reg : std_logic_vector(7 downto 0);
    
    -- Internal signals
    signal i_sample, q_sample : std_logic_vector(15 downto 0);
    signal sample_valid : std_logic;
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
    signal i_compensated : signed(15 downto 0);
    signal q_compensated : signed(15 downto 0);
    
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
    
    -- Decimation signals and constants
    -- Stage 1 filter coefficients (32 taps, symmetric FIR)
    type coeff_array_t is array (0 to 31) of signed(15 downto 0);
    constant STAGE1_COEFFS : coeff_array_t := (
        to_signed(12,    16),
        to_signed(24,    16),
        to_signed(45,    16),
        to_signed(78,    16),
        to_signed(125,   16),
        to_signed(187,   16),
        to_signed(265,   16),
        to_signed(358,   16),
        to_signed(463,   16),
        to_signed(576,   16),
        to_signed(693,   16),
        to_signed(807,   16),
        to_signed(912,   16),
        to_signed(1001,  16),
        to_signed(1068,  16),
        to_signed(1110,  16),
        to_signed(1124,  16),  -- Center coefficient
        to_signed(1110,  16),
        to_signed(1068,  16),
        to_signed(1001,  16),
        to_signed(912,   16),
        to_signed(807,   16),
        to_signed(693,   16),
        to_signed(576,   16),
        to_signed(463,   16),
        to_signed(358,   16),
        to_signed(265,   16),
        to_signed(187,   16),
        to_signed(125,   16),
        to_signed(78,    16),
        to_signed(45,    16),
        to_signed(24,    16)
    );
    
    -- Stage 2 filter coefficients (24 taps, symmetric FIR)
    type coeff_array2_t is array (0 to 23) of signed(15 downto 0);
    constant STAGE2_COEFFS : coeff_array2_t := (
        to_signed(15,    16),
        to_signed(36,    16),
        to_signed(72,    16),
        to_signed(128,   16),
        to_signed(204,   16),
        to_signed(301,   16),
        to_signed(412,   16),
        to_signed(532,   16),
        to_signed(652,   16),
        to_signed(764,   16),
        to_signed(857,   16),
        to_signed(924,   16),
        to_signed(958,   16),  -- Center coefficient
        to_signed(924,   16),
        to_signed(857,   16),
        to_signed(764,   16),
        to_signed(652,   16),
        to_signed(532,   16),
        to_signed(412,   16),
        to_signed(301,   16),
        to_signed(204,   16),
        to_signed(128,   16),
        to_signed(72,    16),
        to_signed(36,    16)
    );
    
    -- Decimation constants and signals
    constant STAGE1_DECIM_FACTOR : integer := 8;
    signal stage1_count : integer range 0 to STAGE1_DECIM_FACTOR-1 := 0;
    signal stage2_count : integer range 0 to 127 := 0; -- Support up to 128 for second stage
    signal stage2_decim_factor : integer range 1 to 128 := 71; -- Default 71 (8*71=568)
    
    -- Stage 1 decimation signals
    type shift_reg_t is array (0 to 31) of signed(15 downto 0);
    signal i_shift_reg1 : shift_reg_t := (others => (others => '0'));
    signal q_shift_reg1 : shift_reg_t := (others => (others => '0'));
    signal i_stage1, q_stage1 : signed(15 downto 0) := (others => '0');
    signal valid_stage1 : std_logic := '0';
    signal i_acc1, q_acc1 : signed(32 downto 0) := (others => '0');
    
    -- Stage 2 decimation signals
    type shift_reg2_t is array (0 to 23) of signed(15 downto 0);
    signal i_shift_reg2 : shift_reg2_t := (others => (others => '0'));
    signal q_shift_reg2 : shift_reg2_t := (others => (others => '0'));
    signal i_acc2, q_acc2 : signed(32 downto 0) := (others => '0');
    signal i_decimated, q_decimated : signed(15 downto 0) := (others => '0');
    signal decimated_valid : std_logic := '0';
    
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
    -- MSK demodulator instance
    msk_demod_inst: hodgart_msk_demodulator
    port map (
        clk => s_axis_aclk,
        reset_n => sys_resetn,
        i_in => std_logic_vector(i_compensated),
        q_in => std_logic_vector(q_compensated),
        sample_valid => sample_valid,
        bit_out => bit_out,
        bit_valid => bit_valid,
        samples_per_symbol => reg_samples_per_symbol,
        lock_detect => lock_detect
    );
    
    -- Input data handling with two-stage decimation and Doppler compensation
    -- Stage 1: FIR filter and decimation by 8
    process(s_axis_aclk)
        variable i_raw, q_raw : signed(15 downto 0);
    begin
        if rising_edge(s_axis_aclk) then
            if sys_resetn = '0' then
                stage1_count <= 0;
                i_shift_reg1 <= (others => (others => '0'));
                q_shift_reg1 <= (others => (others => '0'));
                i_stage1 <= (others => '0');
                q_stage1 <= (others => '0');
                valid_stage1 <= '0';
                i_acc1 <= (others => '0');
                q_acc1 <= (others => '0');
            elsif s_axis_tvalid = '1' then
                -- Extract raw I/Q samples
                i_raw := signed(s_axis_tdata(15 downto 0));
                q_raw := signed(s_axis_tdata(31 downto 16));
                
                -- Shift in new samples
                i_shift_reg1 <= i_raw & i_shift_reg1(0 to i_shift_reg1'length-2);
                q_shift_reg1 <= q_raw & q_shift_reg1(0 to q_shift_reg1'length-2);
                
                -- Calculate FIR output (MAC operation)
                i_acc1 <= (others => '0');
                q_acc1 <= (others => '0');
                
                for i in 0 to STAGE1_COEFFS'length-1 loop
                    i_acc1 <= i_acc1 + i_shift_reg1(i) * STAGE1_COEFFS(i);
                    q_acc1 <= q_acc1 + q_shift_reg1(i) * STAGE1_COEFFS(i);
                end loop;
                
                -- Decimate by STAGE1_DECIM_FACTOR
                if stage1_count = STAGE1_DECIM_FACTOR-1 then
                    stage1_count <= 0;
                    -- Extract the result with scaling (right shift to adjust bit width)
                    i_stage1 <= resize(shift_right(i_acc1, 16), 16);
                    q_stage1 <= resize(shift_right(q_acc1, 16), 16);
                    valid_stage1 <= '1';
                else
                    stage1_count <= stage1_count + 1;
                    valid_stage1 <= '0';
                end if;
            else
                valid_stage1 <= '0';
            end if;
        end if;
    end process;
    
    -- Stage 2: FIR filter and variable decimation
    process(s_axis_aclk)
    begin
        if rising_edge(s_axis_aclk) then
            if sys_resetn = '0' then
                stage2_count <= 0;
                i_shift_reg2 <= (others => (others => '0'));
                q_shift_reg2 <= (others => (others => '0'));
                i_decimated <= (others => '0');
                q_decimated <= (others => '0');
                decimated_valid <= '0';
                i_acc2 <= (others => '0');
                q_acc2 <= (others => '0');
                
                -- Update decimation factor from register (with bounds check)
                if unsigned(reg_decimation_factor(15 downto 8)) = 0 then
                    stage2_decim_factor <= to_integer(unsigned(reg_decimation_factor(6 downto 0)));
                else
                    stage2_decim_factor <= 71; -- Default to 71 if out of range
                end if;
            elsif valid_stage1 = '1' then
                -- Shift in new sample from stage 1
                i_shift_reg2 <= i_stage1 & i_shift_reg2(0 to i_shift_reg2'length-2);
                q_shift_reg2 <= q_stage1 & q_shift_reg2(0 to q_shift_reg2'length-2);
                
                -- Calculate FIR output (MAC operation)
                i_acc2 <= (others => '0');
                q_acc2 <= (others => '0');
                
                for i in 0 to STAGE2_COEFFS'length-1 loop
                    i_acc2 <= i_acc2 + i_shift_reg2(i) * STAGE2_COEFFS(i);
                    q_acc2 <= q_acc2 + q_shift_reg2(i) * STAGE2_COEFFS(i);
                end loop;
                
                -- Apply variable decimation factor
                if stage2_count = stage2_decim_factor-1 then
                    stage2_count <= 0;
                    -- Extract the result with scaling
                    i_decimated <= resize(shift_right(i_acc2, 16), 16);
                    q_decimated <= resize(shift_right(q_acc2, 16), 16);
                    decimated_valid <= '1';
                else
                    stage2_count <= stage2_count + 1;
                    decimated_valid <= '0';
                end if;
            else
                decimated_valid <= '0';
            end if;
        end if;
    end process;
    
    -- Doppler compensation process
    process(s_axis_aclk)
        variable doppler_cos, doppler_sin : signed(15 downto 0);
        variable i_comp, q_comp : signed(31 downto 0);
    begin
        if rising_edge(s_axis_aclk) then
            if sys_resetn = '0' then
                sample_valid <= '0';
                doppler_phase <= (others => '0');
                i_compensated <= (others => '0');
                q_compensated <= (others => '0');
            else
                -- Process decimated data with Doppler compensation
                if decimated_valid = '1' then
                    -- Update Doppler phase - scale for new sample rate
                    -- (Doppler shift needs to be adjusted for decimated sample rate)
                    doppler_phase <= doppler_phase + shift_right(signed(reg_doppler_shift), 9); -- Approx. divide by 568
                    
                    -- Get sine and cosine for Doppler compensation
                    doppler_cos := cos_lookup(doppler_phase);
                    doppler_sin := sin_lookup(doppler_phase);
                    
                    -- Perform complex multiplication for frequency correction
                    -- (i_comp + j*q_comp) = (i_decimated + j*q_decimated) * (cos - j*sin)
                    i_comp := (i_decimated * doppler_cos + q_decimated * doppler_sin) / 32768;
                    q_comp := (q_decimated * doppler_cos - i_decimated * doppler_sin) / 32768;
                    
                    -- Scale back to 16-bit
                    i_compensated <= resize(i_comp, 16);
                    q_compensated <= resize(q_comp, 16);
                    
                    sample_valid <= '1';
                else
                    sample_valid <= '0';
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
    
    -- Status output
    status_lock <= lock_detect_sync(2);
    
    -- AXI-Lite Slave Interface for configuration
    process(s_axi_aclk)
    begin
        if rising_edge(s_axi_aclk) then
            if s_axi_aresetn = '0' then
                s_axi_awready <= '0';
                s_axi_wready <= '0';
                s_axi_bvalid <= '0';
                s_axi_bresp <= "00";
                s_axi_arready <= '0';
                s_axi_rvalid <= '0';
                s_axi_rdata <= (others => '0');
                s_axi_rresp <= "00";
                axi_state <= IDLE;
                reg_control <= (others => '0');
                reg_samples_per_symbol <= x"04";  -- Default: 4 samples per symbol
                reg_doppler_shift <= (others => '0'); -- No Doppler shift by default
                reg_decimation_factor <= x"0238"; -- Default: 568 (0x238)
            else
                case axi_state is
                    when IDLE =>
                        -- Write address channel
                        if s_axi_awvalid = '1' then
                            s_axi_awready <= '1';
                            axi_addr_reg <= s_axi_awaddr;
                            axi_state <= WRITE_DATA;
                        -- Read address channel
                        elsif s_axi_arvalid = '1' then
                            s_axi_arready <= '1';
                            axi_addr_reg <= s_axi_araddr;
                            axi_state <= READ_DATA;
                        end if;
                        
                    when WRITE_DATA =>
                        s_axi_awready <= '0';
                        
                        if s_axi_wvalid = '1' then
                            s_axi_wready <= '1';
                            
                            -- Decode address and write to appropriate register
                            case axi_addr_reg(7 downto 2) is
                                when "000000" =>  -- 0x00: Control register
                                    reg_control <= s_axi_wdata;
                                when "000001" =>  -- 0x04: Samples per symbol
                                    reg_samples_per_symbol <= s_axi_wdata(7 downto 0);
                                when "000010" =>  -- 0x08: Doppler shift compensation
                                    reg_doppler_shift <= s_axi_wdata(15 downto 0);
                                when "000011" =>  -- 0x0C: Decimation factor
                                    reg_decimation_factor <= s_axi_wdata(15 downto 0);
                                when others =>
                                    null;
                            end case;
                            
                            axi_state <= WRITE_RESP;
                        end if;
                        
                    when WRITE_RESP =>
                        s_axi_wready <= '0';
                        s_axi_bvalid <= '1';
                        s_axi_bresp <= "00";  -- OKAY response
                        
                        if s_axi_bready = '1' then
                            s_axi_bvalid <= '0';
                            axi_state <= IDLE;
                        end if;
                        
                    when READ_DATA =>
                        s_axi_arready <= '0';
                        s_axi_rvalid <= '1';
                        s_axi_rresp <= "00";  -- OKAY response
                        
                        -- Decode address and read from appropriate register
                        case axi_addr_reg(7 downto 2) is
                            when "000000" =>  -- 0x00: Control register
                                s_axi_rdata <= reg_control;
                            when "000001" =>  -- 0x04: Samples per symbol
                                s_axi_rdata <= x"000000" & reg_samples_per_symbol;
                            when "000010" =>  -- 0x08: Doppler shift
                                s_axi_rdata <= x"0000" & reg_doppler_shift;
                            when "000011" =>  -- 0x0C: Decimation factor
                                s_axi_rdata <= x"0000" & reg_decimation_factor;
                            when "000100" =>  -- 0x10: Status register
                                s_axi_rdata <= x"0000000" & "000" & lock_detect;
                            when others =>
                                s_axi_rdata <= (others => '0');
                        end case;
                        
                        if s_axi_rready = '1' then
                            s_axi_rvalid <= '0';
                            axi_state <= IDLE;
                        end if;
                end case;
            end if;
        end if;
    end process;
    
end Behavioral;
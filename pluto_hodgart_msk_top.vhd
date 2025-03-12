library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity pluto_hodgart_msk_top is
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
end pluto_hodgart_msk_top;

architecture Behavioral of pluto_hodgart_msk_top is
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
    signal reg_samples_per_symbol : std_logic_vector(7 downto 0) := x"08";  -- Default 8 samples per symbol
    signal reg_doppler_shift : std_logic_vector(15 downto 0) := (others => '0'); -- Doppler compensation
    
    -- AXI-Lite interface signals
    type axi_state_t is (IDLE, WRITE_DATA, WRITE_RESP, READ_DATA);
    signal axi_state : axi_state_t := IDLE;
    signal axi_addr_reg : std_logic_vector(7 downto 0);
    
    -- Internal signals
    signal i_sample : std_logic_vector(15 downto 0);
    signal q_sample : std_logic_vector(15 downto 0);
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
    
    -- Input data handling with Doppler compensation (AXIS to demodulator)
    process(s_axis_aclk)
        variable i_raw, q_raw : signed(15 downto 0);
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
                -- Extract I/Q samples from packed AXIS data
                if s_axis_tvalid = '1' then
                    -- Extract raw I/Q samples
                    i_raw := signed(s_axis_tdata(15 downto 0));
                    q_raw := signed(s_axis_tdata(31 downto 16));
                    
                    -- Update Doppler phase
                    doppler_phase <= doppler_phase + signed(reg_doppler_shift);
                    
                    -- Get sine and cosine for Doppler compensation
                    doppler_cos := cos_lookup(doppler_phase);
                    doppler_sin := sin_lookup(doppler_phase);
                    
                    -- Perform complex multiplication for frequency correction
                    -- (i_comp + j*q_comp) = (i_raw + j*q_raw) * (cos - j*sin)
                    i_comp := (i_raw * doppler_cos + q_raw * doppler_sin) / 32768;
                    q_comp := (q_raw * doppler_cos - i_raw * doppler_sin) / 32768;
                    
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
                reg_samples_per_symbol <= x"08";  -- Default: 8 samples per symbol
                reg_doppler_shift <= (others => '0'); -- No Doppler shift by default
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
                            when "000011" =>  -- 0x0C: Status register
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
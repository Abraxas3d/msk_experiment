library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL;

entity hodgart_msk_demodulator is
    Port ( 
        -- Clock and reset
        clk         : in STD_LOGIC;
        reset_n     : in STD_LOGIC;
        
        -- Input IQ samples from ADC (16-bit signed)
        i_in        : in STD_LOGIC_VECTOR(15 downto 0);
        q_in        : in STD_LOGIC_VECTOR(15 downto 0);
        sample_valid: in STD_LOGIC;
        
        -- Output demodulated bit and valid signal
        bit_out     : out STD_LOGIC;
        bit_valid   : out STD_LOGIC;
        
        -- Configuration
        samples_per_symbol : in STD_LOGIC_VECTOR(7 downto 0);  -- Number of samples per symbol
        
        -- Debug outputs
        lock_detect : out STD_LOGIC
    );
end hodgart_msk_demodulator;

architecture Behavioral of hodgart_msk_demodulator is
    -- Constants
    constant PI : integer := 32768;  -- PI in Q16 format (32768 = pi in Q15)
    
    -- Types
    type iq_sample_t is record
        i : signed(15 downto 0);
        q : signed(15 downto 0);
    end record;

    -- Reference oscillator signals
    signal ref1_i : signed(15 downto 0);
    signal ref1_q : signed(15 downto 0);
    signal ref2_i : signed(15 downto 0);
    signal ref2_q : signed(15 downto 0);
    
    -- Costas loop signals for frequency f1
    signal loop1_phase : signed(15 downto 0);
    signal loop1_freq : signed(15 downto 0);
    signal loop1_error : signed(15 downto 0);
    signal loop1_i_corr : signed(31 downto 0);
    signal loop1_q_corr : signed(31 downto 0);
    signal loop1_enabled : std_logic;
    
    -- Costas loop signals for frequency f2
    signal loop2_phase : signed(15 downto 0);
    signal loop2_freq : signed(15 downto 0);
    signal loop2_error : signed(15 downto 0);
    signal loop2_i_corr : signed(31 downto 0);
    signal loop2_q_corr : signed(31 downto 0);
    signal loop2_enabled : std_logic;
    
    -- Decision signals
    signal decision_bit : std_logic;
    signal prev_decision_bit : std_logic;
    
    -- Correlation signals (two-bit interval detection)
    signal corr1_i : signed(31 downto 0);
    signal corr1_q : signed(31 downto 0);
    signal corr2_i : signed(31 downto 0);
    signal corr2_q : signed(31 downto 0);
    signal corr1_acc : signed(31 downto 0);
    signal corr2_acc : signed(31 downto 0);
    
    -- Correlation and decision logic
    signal corr_result1 : signed(31 downto 0);
    signal corr_result2 : signed(31 downto 0);
    signal decision_metric : signed(31 downto 0);
    
    -- Symbol timing recovery
    signal symbol_counter : unsigned(7 downto 0);
    signal symbol_center : std_logic;
    signal bit_center : std_logic;
    signal symbol_clock : std_logic;
    signal bit_clock : std_logic;
    signal toggle_bit : std_logic;
    
    -- Lock detection
    signal lock_counter : unsigned(7 downto 0);
    signal lock_state : std_logic;
    
    -- Loop filter coefficients (adjusted for decimated rate)
    constant LOOP_GAIN_ALPHA : integer := 7;      -- Proportional (increased slightly from 8)
    constant LOOP_GAIN_BETA : integer := 2;       -- Integral (increased slightly from 3)
    
    -- Sine/cosine lookup table
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
    -- Main process for decision-switched Costas loops and MSK demodulation
    process(clk, reset_n)
        variable phase_inc1 : signed(15 downto 0);
        variable phase_inc2 : signed(15 downto 0);
    begin
        if reset_n = '0' then
            -- Reset all signals
            loop1_phase <= (others => '0');
            loop1_freq <= (others => '0');
            loop1_error <= (others => '0');
            loop1_i_corr <= (others => '0');
            loop1_q_corr <= (others => '0');
            loop1_enabled <= '0';
            
            loop2_phase <= (others => '0');
            loop2_freq <= (others => '0');
            loop2_error <= (others => '0');
            loop2_i_corr <= (others => '0');
            loop2_q_corr <= (others => '0');
            loop2_enabled <= '0';
            
            symbol_counter <= (others => '0');
            corr1_acc <= (others => '0');
            corr2_acc <= (others => '0');
            decision_bit <= '0';
            prev_decision_bit <= '0';
            bit_valid <= '0';
            bit_out <= '0';
            lock_counter <= (others => '0');
            lock_state <= '0';
            symbol_clock <= '0';
            bit_clock <= '0';
            toggle_bit <= '0';
            
        elsif rising_edge(clk) then
            bit_valid <= '0';  -- Default state
            
            if sample_valid = '1' then
                -- Generate reference oscillator signals for both frequencies
                -- f1 = fc + 1/4T and f2 = fc - 1/4T
                -- Implemented as phase accumulator with different frequencies
                
                -- Calculate phase increments based on frequency
                -- For samples_per_symbol = 4, this gives ±π/8 which is optimal for MSK
                phase_inc1 := shift_right(to_signed(PI/2, 16), to_integer(unsigned(samples_per_symbol)));
                phase_inc2 := shift_right(to_signed(-PI/2, 16), to_integer(unsigned(samples_per_symbol)));
                
                -- Update the loop phases
                loop1_phase <= loop1_phase + loop1_freq + phase_inc1;
                loop2_phase <= loop2_phase + loop2_freq + phase_inc2;
                
                -- Generate reference signals using sine/cosine lookup
                ref1_i <= cos_lookup(loop1_phase);
                ref1_q <= sin_lookup(loop1_phase);
                ref2_i <= cos_lookup(loop2_phase);
                ref2_q <= sin_lookup(loop2_phase);
                
                -- Correlate input signal with reference signals (multiply and accumulate)
                loop1_i_corr <= signed(i_in) * ref1_i;
                loop1_q_corr <= signed(i_in) * ref1_q;
                loop2_i_corr <= signed(i_in) * ref2_i;
                loop2_q_corr <= signed(i_in) * ref2_q;
                
                -- Decision-switched Costas loop error detection
                -- Only one loop is active at a time based on the decision bit
                if decision_bit = '1' then
                    -- Use first frequency (f1)
                    loop1_enabled <= '1';
                    loop2_enabled <= '0';
                    loop1_error <= resize(shift_right(loop1_i_corr * loop1_q_corr, 16), 16);
                else
                    -- Use second frequency (f2)
                    loop1_enabled <= '0';
                    loop2_enabled <= '1';
                    loop2_error <= resize(shift_right(loop2_i_corr * loop2_q_corr, 16), 16);
                end if;
                
                -- Loop filters (PI controllers) for tracking frequency
                -- Gain values adjusted for decimated rate (fewer samples per loop)
                if loop1_enabled = '1' then
                    -- Proportional term
                    loop1_freq <= loop1_freq - shift_right(loop1_error, LOOP_GAIN_ALPHA);
                    -- Integral term
                    loop1_freq <= loop1_freq - shift_right(loop1_error, LOOP_GAIN_BETA);
                end if;
                
                if loop2_enabled = '1' then
                    -- Proportional term
                    loop2_freq <= loop2_freq - shift_right(loop2_error, LOOP_GAIN_ALPHA);
                    -- Integral term
                    loop2_freq <= loop2_freq - shift_right(loop2_error, LOOP_GAIN_BETA);
                end if;
                
                -- Symbol timing recovery based on counter
                if symbol_counter = unsigned(samples_per_symbol) - 1 then
                    symbol_counter <= (others => '0');
                    symbol_clock <= '1';
                    
                    -- Toggle bit for alternating sequence
                    toggle_bit <= not toggle_bit;
                    
                    -- Every other symbol is a bit transition
                    if toggle_bit = '1' then
                        bit_clock <= '1';
                    else
                        bit_clock <= '0';
                    end if;
                else
                    symbol_counter <= symbol_counter + 1;
                    symbol_clock <= '0';
                    bit_clock <= '0';
                end if;
                
                -- Correlation accumulation for two-symbol interval detection (Massey's approach)
                if symbol_clock = '1' then
                    -- First correlator using f1
                    corr1_i <= resize(shift_right(signed(i_in) * ref1_i, 8), 32);
                    corr1_q <= resize(shift_right(signed(i_in) * ref1_q, 8), 32);
                    
                    -- Second correlator using f2
                    corr2_i <= resize(shift_right(signed(i_in) * ref2_i, 8), 32);
                    corr2_q <= resize(shift_right(signed(i_in) * ref2_q, 8), 32);
                    
                    -- Accumulate correlations over two bit intervals
                    if toggle_bit = '0' then  -- First symbol of the bit
                        corr1_acc <= corr1_i;
                        corr2_acc <= corr2_i;
                    else  -- Second symbol of the bit
                        -- Complete the two-symbol correlation
                        corr_result1 <= corr1_acc + corr1_i;
                        corr_result2 <= corr2_acc + corr2_i;
                        
                        -- Decision metric based on Hodgart's paper
                        decision_metric <= corr_result1 - corr_result2;
                        
                        -- Make bit decision
                        prev_decision_bit <= decision_bit;
                        if decision_metric > 0 then
                            decision_bit <= '1';
                            bit_out <= '1';
                        else
                            decision_bit <= '0';
                            bit_out <= '0';
                        end if;
                        
                        bit_valid <= '1';
                        
                        -- Update lock detection
                        if (prev_decision_bit /= decision_bit) then
                            -- Transitions indicate signal presence
                            if lock_counter < 255 then
                                lock_counter <= lock_counter + 1;
                            end if;
                        else
                            -- Long periods without transitions indicate loss of signal
                            if lock_counter > 0 then
                                lock_counter <= lock_counter - 1;
                            end if;
                        end if;
                        
                        -- Update lock state based on counter threshold
                        if lock_counter > 128 then
                            lock_state <= '1';
                        else
                            lock_state <= '0';
                        end if;
                    end if;
                end if;
            end if;
        end if;
    end process;
    
    -- Output lock detection
    lock_detect <= lock_state;

end Behavioral;

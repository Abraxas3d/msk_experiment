library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL;

entity hodgart_msk_demodulator_tb is
end hodgart_msk_demodulator_tb;

architecture Behavioral of hodgart_msk_demodulator_tb is
    -- Component declaration
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
    
    -- Clock and reset signals
    signal clk : std_logic := '0';
    signal reset_n : std_logic := '0';
    
    -- Test data signals
    signal i_in : std_logic_vector(15 downto 0) := (others => '0');
    signal q_in : std_logic_vector(15 downto 0) := (others => '0');
    signal sample_valid : std_logic := '0';
    
    -- Output signals
    signal bit_out : std_logic;
    signal bit_valid : std_logic;
    signal lock_detect : std_logic;
    
    -- Configuration
    signal samples_per_symbol : std_logic_vector(7 downto 0) := std_logic_vector(to_unsigned(8, 8));
    
    -- Clock period definition
    constant clk_period : time := 10 ns;
    
    -- Test data generation
    signal test_bit_sequence : std_logic_vector(31 downto 0) := x"A5A5A5A5";  -- Test pattern
    signal bit_counter : integer := 0;
    signal phase : real := 0.0;
    signal phase_inc : real := 0.0;
    
    -- Function to generate MSK signal
    function generate_msk_sample(phase : real; bit_value : std_logic) return real is
        variable phase_shift : real;
    begin
        if bit_value = '1' then
            phase_shift := MATH_PI / 2.0;  -- +90 degrees for '1'
        else
            phase_shift := -MATH_PI / 2.0;  -- -90 degrees for '0'
        end if;
        
        return phase_shift;
    end function;
    
begin
    -- Instantiate UUT (Unit Under Test)
    uut: hodgart_msk_demodulator 
    port map (
        clk => clk,
        reset_n => reset_n,
        i_in => i_in,
        q_in => q_in,
        sample_valid => sample_valid,
        bit_out => bit_out,
        bit_valid => bit_valid,
        samples_per_symbol => samples_per_symbol,
        lock_detect => lock_detect
    );
    
    -- Clock process
    clk_process: process
    begin
        clk <= '0';
        wait for clk_period/2;
        clk <= '1';
        wait for clk_period/2;
    end process;
    
    -- Stimulus process
    stim_proc: process
        variable current_bit : std_logic;
        variable i_val, q_val : real;
        variable sample_count : integer := 0;
        variable sps : integer := 8;  -- Samples per symbol
    begin
        -- Initialize
        reset_n <= '0';
        sample_valid <= '0';
        wait for 100 ns;
        reset_n <= '1';
        wait for clk_period;
        
        -- Test sequence
        for i in 0 to 127 loop
            -- Get current bit (change every symbol time)
            if sample_count = 0 then
                current_bit := test_bit_sequence(bit_counter);
                bit_counter <= (bit_counter + 1) mod 32;
                
                -- Calculate phase increment based on bit value
                phase_inc <= generate_msk_sample(phase, current_bit);
            end if;
            
            -- Update phase
            phase <= phase + phase_inc / real(sps);
            
            -- Generate I/Q samples
            i_val := 32767.0 * cos(phase);
            q_val := 32767.0 * sin(phase);
            
            -- Convert to STD_LOGIC_VECTOR
            i_in <= std_logic_vector(to_signed(integer(i_val), 16));
            q_in <= std_logic_vector(to_signed(integer(q_val), 16));
            
            -- Set valid signal
            sample_valid <= '1';
            wait for clk_period;
            sample_valid <= '0';
            wait for clk_period * 3;  -- Simulate processing time
            
            -- Update sample counter
            sample_count := (sample_count + 1) mod sps;
        end loop;
        
        -- End simulation
        wait for 5000 ns;
        report "Simulation completed" severity note;
        wait;
    end process;

    -- Process to monitor demodulator output
    monitor_process: process
        variable demod_bits : std_logic_vector(31 downto 0) := (others => '0');
        variable bit_idx : integer := 0;
    begin
        wait until rising_edge(clk);
        
        if bit_valid = '1' then
            demod_bits(bit_idx) := bit_out;
            bit_idx := (bit_idx + 1) mod 32;
            
            -- Every 32 bits, check the pattern
            if bit_idx = 0 then
                report "Demodulated 32 bits: " & to_hstring(demod_bits);
                if demod_bits = test_bit_sequence then
                    report "Pattern match successful!" severity note;
                else
                    report "Pattern mismatch!" severity warning;
                end if;
            end if;
        end if;
    end process;

end Behavioral;

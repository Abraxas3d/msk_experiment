library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity pluto_hodgart_msk_decimator is
    Port ( 
        -- Clock and reset
        clk         : in STD_LOGIC;
        reset_n     : in STD_LOGIC;
        
        -- Input IQ samples (from ADC)
        i_in        : in STD_LOGIC_VECTOR(15 downto 0);
        q_in        : in STD_LOGIC_VECTOR(15 downto 0);
        sample_valid_in : in STD_LOGIC;
        
        -- Output decimated IQ samples (to demodulator)
        i_out       : out STD_LOGIC_VECTOR(15 downto 0);
        q_out       : out STD_LOGIC_VECTOR(15 downto 0);
        sample_valid_out : out STD_LOGIC;
        
        -- Configuration
        enable      : in STD_LOGIC;  -- Enable decimation
        
        -- Status
        stage1_valid : out STD_LOGIC  -- For debug
    );
end pluto_hodgart_msk_decimator;

architecture Behavioral of pluto_hodgart_msk_decimator is
    -- Constants for decimation factors
    constant STAGE1_DECIM_FACTOR : integer := 8;
    constant STAGE2_DECIM_FACTOR : integer := 71;  -- Total decimation ≈ 568
    
    -- Stage 1 filter coefficients (33-tap lowpass FIR)
    type coeff_array_stage1_t is array (0 to 32) of signed(15 downto 0);
    constant STAGE1_COEFFS : coeff_array_stage1_t := (
        to_signed(-127,   16), to_signed(-189,   16), to_signed(-240,   16), to_signed(-256,   16),
        to_signed(-215,   16), to_signed(-102,   16), to_signed(84,     16), to_signed(335,    16),
        to_signed(632,    16), to_signed(951,    16), to_signed(1264,   16), to_signed(1546,   16),
        to_signed(1776,   16), to_signed(1935,   16), to_signed(2012,   16), to_signed(2001,   16),
        to_signed(1904,   16), -- Center tap (index 16)
        to_signed(2001,   16), to_signed(2012,   16), to_signed(1935,   16), to_signed(1776,   16),
        to_signed(1546,   16), to_signed(1264,   16), to_signed(951,    16), to_signed(632,    16),
        to_signed(335,    16), to_signed(84,     16), to_signed(-102,   16), to_signed(-215,   16),
        to_signed(-256,   16), to_signed(-240,   16), to_signed(-189,   16), to_signed(-127,   16)
    );
    
    -- Stage 2 filter coefficients (25-tap lowpass FIR)
    type coeff_array_stage2_t is array (0 to 24) of signed(15 downto 0);
    constant STAGE2_COEFFS : coeff_array_stage2_t := (
        to_signed(-107,   16), to_signed(-173,   16), to_signed(-167,   16), to_signed(-58,    16),
        to_signed(159,    16), to_signed(468,    16), to_signed(826,    16), to_signed(1182,   16),
        to_signed(1488,   16), to_signed(1703,   16), to_signed(1808,   16), to_signed(1792,   16),
        to_signed(1808,   16), -- Center tap (index 12)
        to_signed(1792,   16), to_signed(1808,   16), to_signed(1703,   16), to_signed(1488,   16),
        to_signed(1182,   16), to_signed(826,    16), to_signed(468,    16), to_signed(159,    16),
        to_signed(-58,    16), to_signed(-167,   16), to_signed(-173,   16), to_signed(-107,   16)
    );
    
    -- Stage 1 decimation signals
    type shift_reg_t is array (0 to 32) of signed(15 downto 0);
    signal i_shift_reg1 : shift_reg_t := (others => (others => '0'));
    signal q_shift_reg1 : shift_reg_t := (others => (others => '0'));
    signal stage1_count : integer range 0 to STAGE1_DECIM_FACTOR-1 := 0;
    signal i_stage1, q_stage1 : signed(15 downto 0) := (others => '0');
    signal valid_stage1 : std_logic := '0';
    signal i_acc1, q_acc1 : signed(32 downto 0) := (others => '0'); -- Extra bit for sign
    
    -- Stage 2 decimation signals
    type shift_reg2_t is array (0 to 24) of signed(15 downto 0);
    signal i_shift_reg2 : shift_reg2_t := (others => (others => '0'));
    signal q_shift_reg2 : shift_reg2_t := (others => (others => '0'));
    signal stage2_count : integer range 0 to STAGE2_DECIM_FACTOR-1 := 0;
    signal i_acc2, q_acc2 : signed(32 downto 0) := (others => '0'); -- Extra bit for sign
    signal i_out_reg, q_out_reg : signed(15 downto 0) := (others => '0');
    signal valid_out_reg : std_logic := '0';
    
    -- Function for symmetric FIR filtering (optimizes by folding symmetric coefficients)
    function symmetric_fir(data: shift_reg_t; coeffs: coeff_array_stage1_t) return signed is
        variable result : signed(32 downto 0) := (others => '0');
        variable product : signed(31 downto 0);
    begin
        -- Process center tap
        product := data(coeffs'length/2) * coeffs(coeffs'length/2);
        result := result + resize(product, 33);
        
        -- Process other taps in folded pairs
        for i in 0 to (coeffs'length/2)-1 loop
            product := (data(i) + data(coeffs'length-1-i)) * coeffs(i);
            result := result + resize(product, 33);
        end loop;
        
        return result;
    end function;
    
    -- Function for symmetric FIR filtering with different type
    function symmetric_fir2(data: shift_reg2_t; coeffs: coeff_array_stage2_t) return signed is
        variable result : signed(32 downto 0) := (others => '0');
        variable product : signed(31 downto 0);
    begin
        -- Process center tap
        product := data(coeffs'length/2) * coeffs(coeffs'length/2);
        result := result + resize(product, 33);
        
        -- Process other taps in folded pairs
        for i in 0 to (coeffs'length/2)-1 loop
            product := (data(i) + data(coeffs'length-1-i)) * coeffs(i);
            result := result + resize(product, 33);
        end loop;
        
        return result;
    end function;
    
begin
    -- Forward stage1_valid for debugging
    stage1_valid <= valid_stage1;
    
    -- Output assignment
    i_out <= std_logic_vector(i_out_reg);
    q_out <= std_logic_vector(q_out_reg);
    sample_valid_out <= valid_out_reg;
    
    -- Stage 1 FIR filter and decimation by STAGE1_DECIM_FACTOR
    process(clk, reset_n)
    begin
        if reset_n = '0' then
            i_shift_reg1 <= (others => (others => '0'));
            q_shift_reg1 <= (others => (others => '0'));
            stage1_count <= 0;
            i_stage1 <= (others => '0');
            q_stage1 <= (others => '0');
            valid_stage1 <= '0';
            i_acc1 <= (others => '0');
            q_acc1 <= (others => '0');
        elsif rising_edge(clk) then
            valid_stage1 <= '0';  -- Default state
            
            if enable = '1' and sample_valid_in = '1' then
                -- Shift in new sample
                i_shift_reg1 <= signed(i_in) & i_shift_reg1(0 to i_shift_reg1'length-2);
                q_shift_reg1 <= signed(q_in) & q_shift_reg1(0 to q_shift_reg1'length-2);
                
                -- Apply FIR filter with symmetric optimization
                i_acc1 <= symmetric_fir(i_shift_reg1, STAGE1_COEFFS);
                q_acc1 <= symmetric_fir(q_shift_reg1, STAGE1_COEFFS);
                
                -- Decimate by STAGE1_DECIM_FACTOR
                if stage1_count = STAGE1_DECIM_FACTOR-1 then
                    stage1_count <= 0;
                    -- Scale output to 16 bits (right shift by 15 for Q15 format)
                    i_stage1 <= resize(shift_right(i_acc1, 15), 16);
                    q_stage1 <= resize(shift_right(q_acc1, 15), 16);
                    valid_stage1 <= '1';
                else
                    stage1_count <= stage1_count + 1;
                end if;
            end if;
        end if;
    end process;
    
    -- Stage 2 FIR filter and decimation by STAGE2_DECIM_FACTOR
    process(clk, reset_n)
    begin
        if reset_n = '0' then
            i_shift_reg2 <= (others => (others => '0'));
            q_shift_reg2 <= (others => (others => '0'));
            stage2_count <= 0;
            i_out_reg <= (others => '0');
            q_out_reg <= (others => '0');
            valid_out_reg <= '0';
            i_acc2 <= (others => '0');
            q_acc2 <= (others => '0');
        elsif rising_edge(clk) then
            valid_out_reg <= '0';  -- Default state
            
            if enable = '1' and valid_stage1 = '1' then
                -- Shift in new sample from stage 1
                i_shift_reg2 <= i_stage1 & i_shift_reg2(0 to i_shift_reg2'length-2);
                q_shift_reg2 <= q_stage1 & q_shift_reg2(0 to q_shift_reg2'length-2);
                
                -- Apply FIR filter with symmetric optimization
                i_acc2 <= symmetric_fir2(i_shift_reg2, STAGE2_COEFFS);
                q_acc2 <= symmetric_fir2(q_shift_reg2, STAGE2_COEFFS);
                
                -- Decimate by STAGE2_DECIM_FACTOR
                if stage2_count = STAGE2_DECIM_FACTOR-1 then
                    stage2_count <= 0;
                    -- Scale output to 16 bits (right shift by 15 for Q15 format)
                    i_out_reg <= resize(shift_right(i_acc2, 15), 16);
                    q_out_reg <= resize(shift_right(q_acc2, 15), 16);
                    valid_out_reg <= '1';
                else
                    stage2_count <= stage2_count + 1;
                end if;
            end if;
        end if;
    end process;
    
end Behavioral;
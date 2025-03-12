-- Stage 1 Filter Coefficients (33-tap lowpass FIR)
-- Cutoff frequency: 3.07 MHz (optimal for 8x decimation from 61.44 MHz)
-- These coefficients are scaled for 16-bit signed representation
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

-- Stage 2 Filter Coefficients (25-tap lowpass FIR)
-- Cutoff frequency: 67.7 kHz (optimal for 71x decimation from 7.68 MHz)
-- These coefficients are scaled for 16-bit signed representation
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

-- Summary of decimation parameters:
-- Original sample rate: 61.44 MHz
-- Stage 1 decimation: 8x → 7.68 MHz
-- Stage 2 decimation: 71x → 108.17 kHz
-- Total decimation: 568x
-- Samples per symbol: 3.99 (for 27.1 kHz symbol rate)
-- MSK bandwidth: 40.65 kHz (1.5 × symbol rate)
-- Nyquist frequency: 54.08 kHz
-- Filter margin: 13.43 kHz

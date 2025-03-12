# Experimental MSK demodulator for ORI

Reading the papers in https://github.com/OpenResearchInstitute/pluto_msk/tree/main/docs/papers

- The Hodgart paper provided a practical implementation of a "robust MSK demodulator using dual Costas loops"
- The Pasupathy paper gave us an overview of MSK and its relationship to other modulation schemes.
- The Sun paper showed a practical DSP implementation and testing of the Hodgart demodulator.
- The Massey paper provided the theoretical foundation for the optimal demodulation approach. 

I used these papers to try and write a somewhat independent version so that it might help the main version of the code over in pluto_msk. I say "somewhat" because I've been working on the pluto_msk codebase for a while and am not unbiased. 

The priorities for this version are:

- The decision-switched Costas loops from Hodgart's design
- The two-symbol interval correlation for optimal detection from Massey's theory
- The practical considerations from Sun's implementation - Doppler and lock detection, things like that

This is essentially the same as pluto_msk and can hopefully be used to verify results. We've had challenges with getting stable Costas loops, which has lead to inconsistent receiver behavior. If a relatively independent implementation behaves the same, then we have a clue that we're still missing something fundamental. 

I focused on implementing the following functionality. 

- Decision-Switched Costas Loops: The demodulator uses two Costas loops, one for each MSK frequency, with the decision bit controlling which loop is active at any time. This approach allegedly provides "excellent frequency tracking and phase recovery" as described in Hodgart's paper.
- Two-Symbol Detection: Following Massey's theoretical foundation, the demodulator makes decisions based on correlation over two symbol intervals, which is described as "achieving optimal detection for MSK".
- Doppler Compensation: Based on Sun's paper, this implementation includes Doppler shift compensation in the top-level module to handle the large frequency shifts encountered in LEO satellite communications and HEO satellite close approaches. 
- Adaptive Loop Filtering: The PI controllers in the Costas loops use proportional and integral gains that can be adjusted based on signal conditions, enhancing tracking performance in noisy environments, or allowing a "big" loop for initial tracking and a "small" loop for maintaining tracking. 
- Lock Detection: A lock detection mechanism is in here providing status information that can be used by higher-level systems. Lock detection is a function that pluto_msk has and is considered important.

## Clock Planner

The plan:

RF Front End → Anti-aliasing Filter → Decimation by 568 → Frequency Correction → Costas Loop → Symbol Timing Recovery → Decision Device



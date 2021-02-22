# guitar_pedal

Work in progress 

Overview
1. [ ]Sample audio into STM32F4
2. [ ]Apply effects
3. [ ]Output graphic representation of audio and EQ to Nokia 5110 LCD
4. [ ]Output audio to Cirrus Logic Chip or to onboard DAC's

Detailed Descriptions:

#**1. Sample audio into STM32F4**  
  This is done using 2 ADC's(stereo) using interrupts to get as close as possible to 44.1kHz.
  That is one sample every 22.67 microseconds.
  STM32F4 has a maximum ADC sample time: 16.4uS (30MHz, 12-bit)
  ADC clock can run at 30 MHz, so 17 cycles per conversion = 566nS ->1.3uS for 2?

#**2. Apply effects**    
  Using an FFT to get the frequencies, then manipulate them as desired.  
  Ideas:  
    distortion  
    echo/reverb  
    chorus  
    
**3. Output graphic representation of audio and EQ to Nokia 5110 LCD**  
  Use FFT output to display EQ
  Use ADC values to display audio
  
**4. Output audio to Cirrus Logic Chip or to onboard DAC's**    
  Start by using the Cirrus Logic Chip following the example code provided by ST  
  
  **Current order of business:**  
    Get Nokia 5110 working so I can output data, etc.
  
  **Next:**  
    Sample Audio using ADC's.
  

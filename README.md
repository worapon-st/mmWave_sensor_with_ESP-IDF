# FMCW Sensing with ESP-IDF

## What's the "FMCW" ?
**Frequency-Modulated Continuous-Wave:FMCW** is a technique of radar by sends out signals with 
changing frequency over time. Which is help improve the old radar systems that send out same frequency
pulse, FMCW significant improve distance and speed of radar.

## My Experiment
I use is **BGT60TR13C** mmWave sensor from Infineon.
- Carrier frequency : 60GHz
- Bandwidth : 5GHz
- Number of TX : 1
- Number of RX : 3
- Antenna type : Antenna on Chip (AoC)
- Protocol : SPI 32 bit
  [31:25] Address bit
  [24:24] Read/Write bit
  [23:00] Data bit

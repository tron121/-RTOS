# RTOS
Final Project for Real Time Embedded Systems Course:
Advanced optimization of ADC I/O using Direct Memory Access

*Software: Code Composer Studio and TI-RTOS
*Hardware: TI EK-TM4C1294XL, TI BOOSTXL-EDUMKII and jumper wires
CCS Project settings and debug files not included

Joystick Select : play audio 
S1: increase pwm period
S2: Decrease pwm period

USER SW2: spectrum analyzer
user SW1: increase time scale

**Final Results:
*ADC Configuration, Sampling Rate, and CPU Load:
*Single-sample ISR	1 Msps	79.3%
*DMA	1 Msps	0.9%
*DMA	2 Msps	1.5%
*audio 60.4 %


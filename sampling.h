//


#ifndef SAMPLING_H_
#define SAMPLING_H_

#include <stdint.h>

#define PWM_FREQUENCY 20000 // PWM frequency = 20 kHz
#define ADC_BUFFER_SIZE 2048                             // size must be a power of 2
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1)) // index wrapping macro

// Step 4 ADC sample scaling
#define ADC_OFFSET 2048
#define PIXELS_PER_DIV 20
#define VIN_RANGE 3.3f
#define ADC_BITS 12

// variables
extern volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];   // circular buffer
extern volatile uint32_t gADCErrors;    // number of missed ADC deadlines
extern uint32_t gSystemClock;
extern volatile uint32_t period;
extern volatile int pwm_Period;

// function declarations
void ADC_init(void);
void signal_init(void);
int RisingTrigger(void);
int32_t getADCBufferIndex(void);
uint32_t cpu_load_count(void);
void frequencyCounter(void);

#endif /* SAMPLING_H_ */

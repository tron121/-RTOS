/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* Lab 1 Header Files*/
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/interrupt.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "Crystalfontz128x128_ST7735.h"
#include <stdio.h>
#include "buttons.h"
#include "driverlib/timer.h"
#include "sampling.h"
#include <math.h>
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "inc/hw_memmap.h"

/*kiss header files*/
#include "kiss_fft.h"
#include "_kiss_fft_guts.h"

#define PI 3.14159265358979f
#define NFFT 1024 // FFT length
#define KISS_FFT_CFG_SIZE (sizeof(struct kiss_fft_state)+sizeof(kiss_fft_cpx)*(NFFT-1))
volatile int fftOn = 1;
volatile int trigger;
bool triggerSlope;
const char * const gVoltageScaleStr[] = { "100 mV", "200 mV", "500 mV", " 1 V",
                                          " 2 V" };
const char * const gTimeScaleStr[] = { "100 ms", "50 ms", "20 ms", "10 ms",
                                       "5 ms", "2 ms", "1 ms", "500 us",
                                       "200 us", "100 us", "50 us", "20 us" };
static volatile int16_t sample[LCD_HORIZONTAL_MAX]; // local copy of ADC buffer
static volatile int16_t finalWave[LCD_HORIZONTAL_MAX];
static volatile int16_t fft_sample[NFFT]; // local copy of buffer fft
volatile float fScale; // scale
float fVoltsPerDiv[] = { 0.1, 0.2, 0.5, 1, 2 }; // voltage scale/div values

// cpu load variables
uint32_t count_unloaded = 0;
uint32_t count_loaded = 0;
float cpu_load = 0;

/*function declarations */
void display_init();
void signal_init();
int RisingTrigger();



/*
 *  ======== main ========
 */
int main(void)
{
    IntMasterDisable();
    // hardware initialization
    signal_init(); //pwm signal
    ADC_init();   // adc
    ButtonInit(); // buttons
    display_init(); //lcd initialization
    frequencyCounter(); // initiate timer

    // Timer for CPU Load  measurement -
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    TimerDisable(TIMER3_BASE, TIMER_BOTH);
    TimerConfigure(TIMER3_BASE, TIMER_CFG_ONE_SHOT);
    TimerLoadSet(TIMER3_BASE, TIMER_A, (gSystemClock / 50) - 1);
    count_unloaded = cpu_load_count();  // get unloaded CPU count

    /* Start BIOS */
    BIOS_start();
    return (0);
}

//acuires waveform
void waveformFunc(UArg arg1, UArg arg2)
{
    IntMasterEnable();

    while (true)
    {
        Semaphore_pend(waveSem, BIOS_WAIT_FOREVER);
        int i,j;
        if (fftOn)
        {
            for (i = 0; i < NFFT; i++)
            {
                fft_sample[i] =
                        gADCBuffer[ADC_BUFFER_WRAP(getADCBufferIndex() - i)];
            }
        }
        else
        {
            trigger = RisingTrigger();
            for (j = 0; j < LCD_HORIZONTAL_MAX; j++)
            {
                sample[j] = gADCBuffer[ADC_BUFFER_WRAP(
                        trigger- LCD_HORIZONTAL_MAX/2) + j]; //subtract half screen
            }
        }
        Semaphore_post(processSem);
    }
}

//processes waveform
void processFunc(UArg arg1, UArg arg2)
{
    static char kiss_fft_cfg_buffer[KISS_FFT_CFG_SIZE]; // Kiss FFT config memory
    size_t buffer_size = KISS_FFT_CFG_SIZE;
    kiss_fft_cfg cfg; // Kiss FFT config
    static kiss_fft_cpx in[NFFT], out[NFFT]; // complex waveform and spectrum buffers
    int i;
    cfg = kiss_fft_alloc(NFFT, 0, kiss_fft_cfg_buffer, &buffer_size); // init Kiss FFT

    //window to reduce spectral leakage
    static float w[NFFT]; // window function
    for (i = 0; i < NFFT; i++)
    {
        // Blackman window
        w[i] = 0.42f - 0.5f * cosf(2 * PI * i / (NFFT - 1))
                + 0.08f * cosf(4 * PI * i / (NFFT - 1));
    }

    while (true)
    {
        Semaphore_pend(processSem, BIOS_WAIT_FOREVER);
        if (fftOn)
        {
            for (i = 0; i < NFFT; i++)
            { // generate an input waveform
                in[i].r = (float) fft_sample[i] * w[i]; // real part of waveform
                in[i].i = 0; // imaginary part of waveform
            }
            kiss_fft(cfg, in, out); // compute FFT
            for (i = 0; i < LCD_HORIZONTAL_MAX; i++)
            {
                // convert first 128 bins of out[] to dB for display
                finalWave[i] =
                        (150
                                - roundf(
                                        10
                                        * log10f(
                                                out[i].r * out[i].r
                                                + out[i].i
                                                * out[i].i)));
            }
        }
        else
        {
            fScale = (VIN_RANGE * PIXELS_PER_DIV)
                            / ((1 << ADC_BITS) * fVoltsPerDiv[voltageScale]);
            for (i = 0; i < LCD_HORIZONTAL_MAX; i++)
            {
                finalWave[i] = LCD_VERTICAL_MAX / 2
                        - (int) roundf(fScale * ((int) sample[i] - ADC_OFFSET));
            }

        }

        Semaphore_post(displaySem);
        Semaphore_post(waveSem);
    }
}

//displays waveform
void displayFunc(UArg arg1, UArg arg2)
{
    tContext sContext;
    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8); // select font
    tRectangle rectFullScreen = { 0, 0, GrContextDpyWidthGet(&sContext) - 1,
    GrContextDpyHeightGet(&sContext) - 1 };

    //local variables
    int i;
    char str[50]; // string buffer
    int x;

    while (true)
    {
        Semaphore_pend(displaySem, BIOS_WAIT_FOREVER);


        //fill screen with black rectangle
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen);

        //Draw 6X6 grid
        GrContextForegroundSet(&sContext, ClrBlue);
        for (i = 1; i < LCD_HORIZONTAL_MAX; i += LCD_HORIZONTAL_MAX / 6)
        {
            GrLineDraw(&sContext, i, 0, i, 128);
            GrLineDraw(&sContext, 0, i, 128, i);
        }

        // compute CPU load
        count_loaded = cpu_load_count();
        cpu_load = 1.0f - (float) count_loaded / count_unloaded;

        // Display CPU load %
        GrContextForegroundSet(&sContext, ClrWhite); // white text
        snprintf(str, sizeof(str), "CPU load = %.1f%%", cpu_load * 100);
        GrStringDraw(&sContext, str, -1, 0, 120, false);

        // display period and frequency
        snprintf(str, sizeof(str), " f = %d Hz", gSystemClock / period);
        GrStringDraw(&sContext, str, -1, 0, 110, false);
        snprintf(str, sizeof(str), "T = %d", pwm_Period);
        GrStringDraw(&sContext, str, -1, 0, 100, false);

        // units for spectrum
        if (fftOn)
        {

            //draw units for spectrum

            GrStringDraw(&sContext, "20 dB", -1, 50, 0, false);
            GrContextBackgroundSet(&sContext, ClrGreen);
            GrStringDraw(&sContext, "20 kHz", -1, 0, 0, false);
            GrContextForegroundSet(&sContext, ClrDarkOrange);
        }
        else  // draw units for wave
        {
            // Display time scale
            snprintf(str, sizeof(str), gTimeScaleStr[timeScale]);
            GrStringDraw(&sContext, str, /*length*/-1, /*x*/0, /*y*/0, /*opaque*/
                         false);
            // Display trigger slope
            GrLineDrawV(&sContext, 115, 0, 8);
            if (triggerSlope)
            {   //rising trigger
                GrLineDrawH(&sContext, 115, 123, 0);
                GrLineDrawH(&sContext, 107, 115, 8);
                GrLineDraw(&sContext, 115, 2, 112, 5);
                GrLineDraw(&sContext, 115, 2, 118, 5);
            }
            else
            {   //falling edge
                GrLineDrawH(&sContext, 107, 115, 0);
                GrLineDrawH(&sContext, 115, 123, 8);
                GrLineDraw(&sContext, 115, 5, 112, 2);
                GrLineDraw(&sContext, 115, 5, 118, 2);
            }
            // Display voltage scale
            snprintf(str, sizeof(str), gVoltageScaleStr[voltageScale]);
            GrStringDraw(&sContext, str, -1, 50, 0, true);
            GrContextForegroundSet(&sContext, ClrYellow);
        }

        // finally draw wave/spectrum
        for (x = 0; x < LCD_HORIZONTAL_MAX; x++)
        {
            GrLineDraw(&sContext, x, finalWave[x], (x + 1), finalWave[x + 1]);
        }
        GrFlush(&sContext); // flush the frame buffer to the LCD

    }
}

// initializes lcd display
void display_init(void)
{
    Crystalfontz128x128_Init(); // Initialize the LCD display driver
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation
}


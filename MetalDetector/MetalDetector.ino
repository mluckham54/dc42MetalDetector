// Induction balance metal detector (Mirko Pavleski)
// https://www.hackster.io/mircemk/diy-sensitive-arduino-ib-metal-detector-with-discrimination-b55e9d
// https://www.youtube.com/watch?v=BolEUTYOES0

// from original code by dc42 David Crocker (2013)
// https://github.com/dc42/arduino/blob/master/MetalDetector
//   (his version has support for battery power measurement and adjustable threshold)

// This is a VLF (very low frequency) Induction Balance detector technology and contains two identical coils : transmitter and receiver coil in
// a double-D arrangement.  As with all induction balance detectors, coil balance is very critical. The potentiometer is used to zero the small
// 90 degrees out-of-phase component of the signal (the in-phase component is nulled by adjusting the relative placement (overlap) of the coils
// in typical IB-detector style).
//
// Does the 90 degree out-of-phase component arise from the current in the coil lagging the voltage?  The current in the capacitor leads the voltage
// and is also 90 degrees out-of-phase.  https://eepower.com/technical-articles/understanding-resonance-in-parallel-rlc-circuits/#



// ********************************************************************************************************************
    
#define ENABLE_SOUND       (1)

// control what is printed to the COM port

#define PRINT_TICKS (0)
#define PRINT_ISR_VARS (0)
#define PRINT_ADC_MINMAX (0)
#define PRINT_VOLTS (1)
#define PRINT_FFT (0)
#define PRINT_AMPLITUDE (1)
#define PRINT_PHASE (1)
#define PRINT_AVERAGES (0)
#define PRINT_READINGS (0)
#define PRINT_MATERIAL (0)
#define PRINT_BARS (0)

// control what is dislayed on the LCD

#define DISPLAY_VOLTS (1)
#define DISPLAY_AMPLITUDE (1)
#define DISPLAY_PHASE (1)
#define DISPLAY_MATERIAL (0)



// TRANSMIT & RECEIVE COILS
//
// Two coils of same size and number of winding are required.  A variety of sizes could be considered, probably the larger and more windings the
// better, to have a stronger field pulse for the transmit and more sensitivity for the receive coil.  The voltage and maximum current that can be
// delivered to the coils must be considered also.
//
// The inductance and resonant frequency of a coil, and the parallel-resonance circuit formed by coil and capacitor, can be estimated.  Note that the
// current circulating between coil and capacitor is many times greater than the supply current.
//
// For the coil only: https://www.teslascientific.com/products/coil-resonant-frequency-calculator/
//     wire 0.45mm^2, 15cm dia, 0.5cm height, 44 turns - 1570 kHz, 630.500 µH, 14.400 pF
//     wire 0.45mm^2, 15.5 dia, 0.5cm height, 55 turns - 1333.757 kHz, 956.938 µH, 14.880 pF   (* BUILT)
//     wire 0.45mm^2, 16cm dia, 0.5cm height, 55 turns - 1290 kHz, 989.876 µH, 15.360 pF
//       10⁶ ÷ ((2 × 3.142 ) × √(989.876 µH × 15.360 pF))
//       ≈ 10⁶ ÷ (6.283 × √15204.495) ≈ 10⁶ ÷ (6.283 × 123.307)
//       Resonant Frequency ≈ 1000000 ÷ 774.758 = 1290.726 kilocycles/sec
//       Conductor Length = 3.141 × 0.160 Metres × 55 Turns = 27.646 Metres
//
// For the coil-capacitor L-C circuit: https://www.redcrab-software.com/en/Calculator/Electrics/RCL-Parallel-Resonance-Circuit
//     15.5 cm coil Inductor L  956.938 µH Capacitor C 470 nF Resistor R 1000 Ω Voltage U 5.5 V Decimal places 2   
//                  Resonance Frequency f0 7.5 kHz Total current I0 5.5 mA Currents IL, IC 121.89 mA
//                  Impedance XL/XC 45.12 Ω Q factor 22.16 Damping d 0.05 Bandwidth b 338.63 Hz Upper cut-off fH 7.67 kHz Lower cut-off fL 7.34 kHz
//
// A plastic plate was 3D printed from https://www.thingiverse.com/thing:5966085 (DD Coil Modifiable), enlarged to 105%.  The plate is in two parts,
// one for transmit and the other for receive coil.  The plates can slide together to form an overlap of the D-coils and fixed in place.  When
// installed on the plates, each coil (now D-shaped) is about 20.5 x 13 cm.
//
// Each of the coils was wound on 15.5 cm round form using 55 turns of 0.45mm^2 enameled copper wire.  The estimated diameter across the coil 0.5 cm.
// Wrap insulating tape around each coil to bundle the wires together, and form each into a D shape.  Add insulated leads from the coil (head) to run
// back to the electronics enclosure.
//
// The coils as-built were measured using a cheap ESD L/C meter.  The coil w/BL+WH wires was 987.8 uH, w/BLU+YEL wires 1029 uH - agreeing pretty closely
// to the calculated value of 956 uH.  Measurement was done after the coils were wrapped in tape, making it impossible to adjust the coil's inductance by
// adding or removing turns - instead, the capacitor values can be adjusted for equal resonance.
//
// OPTIONALLY screen one or both of the coils using aluminum foil bound with tinned copper wire (taking care to leave a small gap of 1-2 inches along one
//            part of the coil so that the screen doesn’t behave like a shorted turn).  Ground the shield, and tie-wrap the coils onto a plastic plate.
//            Some sources say that for audio frequencies, screening the coils makes no difference.  Some screen the transmit coil, some the receive coil,
//            some both. I did not screen the coils - experiments can come later.

// DRIVING THE TRANSMIT COIL AT ITS RESONANT FREQUENCY
//
// The Timer0 square wave drive the transmit coil, at around 8 kHz.  To determine the Timer0 square wave frequency:
//
//  - measure the coil inductance, or calculate after measuring the coil diameter, cross-sectional height, and number of turns.
//  - an interesting way to obtain the resonant frequency uses an oscilloscope across the coil or capitor-coil (in free space, not connecting to any
//    other circuit).  Tap a 1.5 volt battery across the circuit and watch the decaying sine wave on the scope.  The frequency of the sine wave
//    seems to be the resonant frequency.
//
//      Test 1 using the receiver coil (two 0.1uF ceramic capacitors gives around 500nF) around 22 kHz
//      Test 2                          one 0.1                                          around 16 kHz
//      Test 3                          two 0.1 in parallel, around 0.2 uF               around 11 kHz
//      Test 4                          three 0.1 in parallel, around 0.3 uF             around 8.9 kHz
//      Test 5                          one 0.47 uF                                      around 7.2 kHz
//      Test 6                          one 0.33 uF                                      around 8.6 kHz
//
//  - calculate the parallel-resonance frequency of the capacitor-coil combination.  At resonance, current moves alternately from the coil
//    to the capacitor and back, multiplying the input current that was required to charge the system.  The resonance current is higher than
//    the input current, and while in resonance the input current falls to zero.  Do these two times correspond to the peak and valley of the
//    50% duty cycle?  The resonance frequency of the LC circuit is much lower than the coil's inherent resonant frequency - and is the frequency
//    required for the square wave to drive the coil.
//
//    ONCE THE COIL FREQUENCY IS CLOSE, TIMER1_TOP (255), connect Timer0 square wave output to the coil as per the circuit.  Connect the scope
//    probe across the coil-capacitor combo, observe a sin wave at the same frequency (7-8 kHz) and measure the voltage amplitude.  Change
//    TIMER1_TOP to produce different frequencies around the calculated resonance frequency - choose the value whose output frequency maximizes
//    the voltage.
//
//  - adjust TIMER1_TOP to achieve that frequency while observing the sine wave amplitude on the scope.  Observe the square wave output by the
//    digital output is a sine wave on the scope.  Adjust TIMER1_TOP to produce a frequency that maximizes the amplitude - it should be close to the
//    estimated value from the calculators above.
//  - note that the amplitude will be dependent on the square wave peak-to-peak voltage, either 3.3V or 5V depending on the Arduino power
//    configuration.
//
// Adjusting the square wave drive frequency
//  - digital outputs on the Arduino Uno can handle 40 mA current max.  Connect a suitable resistor to one of the coil-capacitor leads to limit the
//    current below 40 mA, at the digital output voltage (3.3 or 5V).  The resonant capacitor-coil circuit will have a resistive value as well, it
//    can be calculated.  Connect the resistor and capacitor-coil to output D5.
//
//    RESULTS (unshielded, BLU&YEL, 55 turns, 15.5cm, 0.45mm2 wire, 1Kohm current-limit resistor to pin D5 which can safely supply 40mA, 0.33 uF ceramic capacitor (actual measured .305 uF)
//    225 = 8.842kHz 2.07Vp2p 722mVrms
//    233 = 8.539kHz 2.43Vp2p 854mVrms
//    238 = 8.361kHz 2.55Vp2p 906mVrms
//    240 = 8.292kHz 2.60Vp2p 922mVrms
//    242 = 8.223kHz 2.60Vp2p 923mVrms (*)
//                                       https://www.omnicalculator.com/physics/resonant-frequency-lc the inductance of the coil at 8.223kHz is 1,228.2 uH
//                                       https://www.teslascientific.com/products/coil-resonant-frequency-calculator/ wire 0.45mm^2, 15.5 dia, 0.5cm height, 55 turns - estimated resonance at 1333.757 kHz, 956.938 µH, 14.880 pF
//                                       https://www.redcrab-software.com/en/calculator/electrics/RCL-parallel-resonance-circuit the frequency is 8.22kHz, bandwidth 521Hz, Q 15.76, impedance 63.5ohms, total current 4mA, LC current 63mA
//    243 = 8.189kHz 2.58Vp2p 912mVrms
//    245 = 8.123kHz 2.55Vp2p 907mVrms
//    250 = 7.961kHz 2.36Vp2p 827mVrms
//
//    RESULTS (unshielded coil, BLK&WH, 55 turns, 15.5cm, 0.45mm2 wire, 1Kohm current-limit resistor to pin D4 which can safely supply 40mA, 0.33 uF ceramic capacitor (actual measured .308 uF)
//    240 = 8.291kHz 2.45Vp2p 868mVrms
//    242 = 8.223kHz 2.53Vp2p 906mVrmw
//    243 = 8.189kHz 2.57Vp2p 920mVrms
//    245 = 8.123kHz 2.60Vp2p 935mVrms
//    249 = 7.993kHz 2.65Vp2p 945mVrms (*) THIS COIL HAS THE HIGHER OUTPUT BUT AT A LOWER FREQUENCY THAN THE OTHER COIL
//                                       https://www.omnicalculator.com/physics/resonant-frequency-lc the inductance of the coil at 7.993kHz is 1,287.3 uH
//                                       https://www.teslascientific.com/products/coil-resonant-frequency-calculator/ wire 0.45mm^2, 15.5 dia, 0.5cm height, 55 turns - estimated resonance at 1333.757 kHz, 956.938 µH, 14.880 pF
//                                       https://www.redcrab-software.com/en/calculator/electrics/RCL-parallel-resonance-circuit the frequency is 7.99kHz, bandwidth 517Hz, Q 15.47, impedance 64.6ohms, total current 4mA, LC current 62mA
//   voltage across the 1K resistor 5.7Vp2p 2.85Vrms (current 2.85/1000 = 2.8mA).  voltage across coil at 63.5ohm impedance (current .923V/63.5 = 0.0145 = 14 mA)
//   voltage D5 to GND 3.28Vrms, resistance 1000 + 63.5, current 3.28/1063.5 = 0.0031 = 3mA
//   
//    250 = 7.961kHz 2.65Vp2p 944mVrms
//    251 = 7.929kHz 2.63Vp2p 937mVrms
//    252 = 7.898kHz 2.60Vp2p 929mVrms

// TWEAKS
// ***   adding capacitance (10nF across the coil) lowers the RMS volts.  Use this to 'tune' the two coils to peak equally at the same frequency
// ***   change to 5V from 3.3V
// ***   adjust current-limit resistor.  If halved, voltage across the coil will go up - it's worth exploring how much current because the coil output should be improved
//                          (2) 1K resistors in parallel)  ... 1.6 Vrms across the coil vs 850mV-ish at 1K resistor
//                          recommended 100-470 ohms for current limiting, abs max 40mA but continuous 20mA limit recommended.  Total output pins current 200mA max.
//                          digital pins consume the most current when driven low


// HOW THE SQUARE WAVE IS GENERATED
//
//   The CPU clock is 16MHz and the ADC clock at 1MHz. ADC resolution is reduced to 8 bits at this speed.
//
//   Timer 1 is used to divide the system clock by about 256 to produce a 62.5kHz square wave (see the TIMER1_TOP which determines the frequency).
//
//   This is used to drive timer 0 and also to trigger ADC conversions.
//
//   Timer 0 is used to divide the output of Timer 1 by 8, giving (about) 7.8125kHz signal for driving the transmit coil.  This frequency should be the RLC
//   resonance frequency.
//
//   This allows time for 16 ADC clock cycles for each ADC conversion (it actually takes 13.5 cycles), and we take 8 samples per cycle of the coil drive voltage.
//   The ADC implements four phase-sensitive detectors at 45 degree intervals. Using 4 instead of just 2 allows us to cancel the third harmonic of the
//   coil frequency.
//      WHY MENTION THE 3RD HARMONIC?  7.8125, 15.625, 23.4375, 31.25kHz
//      https://powerquality.blog/2023/09/08/effect-of-harmonics-on-power-transformers-a-practical-demonstration-and-analysis/
//         2nd harmonic lowers the second half of each half-phase
//         3rd harmonic lowers the peak of each half-phase
//
//   Timer 2 is used to generate a tone for the earpiece or headset.

#define TIMER1_TOP  (249)        // adjust this to adjust the frequency to tune the LC to maximize voltage across the coil
                                 // initial target 62.5kHz on pin 9 -> pin 4 (Timer0 input), 7.8125 on pin 5 (Timer0 output -> coil)
                                 // final result (249) 7.999 kHz



// When using USB power, change analog reference to the 3.3V pin, because there is too much noise on the +5V rail to get good sensitivity.  When choosing the values
// for current limiting resistors (40mA max per pin, 200mA total), don't forget to take the selected voltage into account.

// Pin D5 transmitter coil - default current limit resistor 1K + trim pot 100 + coil impedance 55.   default current I=E/R      5/1150=4mA    3.3/1150=2mA
//                           max current limit R=E/I    40mA: R=5/0.04=125ohm   R=3.3/0.04=82.5ohm   20mA: R=5/0.02=250ohm   R=3.3/0.02=165ohm

// (0) results in a higher voltage received from the receive-coil (2.54V P2P)
#define USE_3V3_AREF  (0)        // set to 1 of running on an Arduino with USB power, 0 for an Atmega28p with no 3.3V supply available



// WIRING
//
//   Connect digital pin 4 (Timer 0 external clock) to digital pin 9
//   Connect digital pin 5 through current limiting resistor to primary coil and capacitor
//   Connect output from receive amplifier to analog pin 1 (receiverInputPin).  The output of receive amplifier should be biased to about half of the analog reference.

//   LCD Display
//     A 2-line 1602 LCD display with HD44780 controller and an I2C add-on PCB with PFC8574 is used.  SCL (A5) and SDA (A4) signals on the Arduino Uno are used.

/////////////////////////////////////////////////////////////////////////////////////////
// *** NOTES ABOUT WIRING ***
//
// 1. Arduino Uno uses D0 and D1 for Tx/Rx signalling with host computer for programming
//    if these signals are connected to an external board, you may not be able to upload
//    a new program to the board ("not in sync" error)


// Uno I/O PINS IN USE:
//   D0, D1 - Tx/Rx signalling with host computer for programming

// internal
//   D4 OUT jumpered to D9 IN - TIMER0

// outputs
//   D3 OUT - (signal OCR2B) tone generation to earpiece
//   D5 OUT = 8 KHz square wave to TX coil
//   5V output to audio transistor and LCD
//   3.3V output jumpered to AREF - to RX COIL OPAMP

// inputs
//   A1 IN - detected signal from RX coil
//   D2 IN - calibrate button (active LOW)

//   A4 OUT - I2C SDA to LCD/I2C board
//   A5 OUT - I2C SCL to LCD/I2C board
/////////////////////////////////////////////////////////////////


// LCD library - implements all the hd44780 library i/o methods to control an LCD based on the Hitachi HD44780 and compatible
//               chipset using I2C extension backpacks that use a simple I2C I/O expander chip. Currently the PCF8574 or the
//               MCP23008 are supported.

#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c LCD i/o class header

hd44780_I2Cexp lcd(0x27, 16, 2);   // I2C address, width # of characters, height # of lines


// Digital pin definitions for the metal detector

// Digital pin 0 not used, however if we are using the serial port for debugging then it's serial input
const int debugTxPin = 1;        // transmit pin reserved for debugging
const int blinkPin = 13;         // uses the on-board LED for debugging the ISR

const int T0OutputPin = 9;       // Timer0 high freq square wave
const int T0InputPin = 4;        // jumpered to TOOutputPin

const int encoderButtonPin = 2;  // encoder button, calibrate button, also IN0 for waking up from sleep mode
const int earpiecePin = 3;       // tone output to earpiece, aka OCR2B for tone generation
const int coilDrivePin = 5;      // output drive to the primary coil

#define max_ampAverage 200


// Analog pin definitions
const int receiverInputPin = A1;

// for future adjustments using an encoder
//const int encoderAPin = A1;
//const int encoderBpin = A2;


// Variables used only by the ISR
int16_t bins[4];                 // bins used to accumulate ADC readings, one for each of the 4 phases
uint16_t numSamples = 0;
volatile uint8_t lastctr;
volatile uint16_t misses = 0;    // this counts how many times the ISR has been executed too late. Should remain at zero if everything is working properly.

// (1) amplifies by integration
// (0) averages consecutive samples
#define USE_INTEGRATION (0)
// 100 works for the averaging technique .. phase agrees with Rigol scope phase measurement.  1000 for averaging produces incorrect phase
const uint16_t samplesToAccumulate = 100;     // amplification factor.  Larger numbers also slow down the rate of samples available to loop()
const uint16_t maxIntegrationAmount = 15000;     // amplification truncated above this point
const int16_t zeroValue = 126;       // usually 127 or 128 but need also to tune the 100K voltage divider feeding into the OPAMP

// Variables used by the ISR and outside it
volatile int16_t averages[4];    // when we've accumulated enough readings in the bins, the ISR copies them to here and starts again
volatile int16_t rawbins[8];     // ADC readings accumulated in the ISR

volatile int16_t readings[8];    // ADC readings that go into making averages
volatile int16_t readingsMin;    // minimum ADC value read during a cycle
volatile int16_t readingsMax;    // maximum ADC value read during a cycle .. use to determine the voltage midpoint (127 or 128?)
volatile int16_t readingsMisses;

volatile uint32_t ticks = 0;     // system tick counter for timekeeping
volatile bool sampleReady = false;  // indicates that the averages array has been updated
volatile uint32_t sampleReadyTicks = 0;   // system tick counter value when sampleReady was set to TRUE.  For measuring latency between ISR and loop()
volatile int16_t adcMin = 255;   // raw value of the ADC reading - minimum during a cycle
volatile int16_t adcMax = 0;     // raw value of the ADC reading - maximum during a cycle

// Variables used only outside the ISR
double volts = 0;
double calibratedVolts = 0;
double voltsAmplitude = 0;
double previousPhase = 0;
bool phaseStable = false;
double calibratedPhase = 0;
bool calibrated = false;


const double halfRoot2 = sqrt(0.5);
const double quarterPi = 3.1415927 / 4.0;
const double radiansToDegrees = 180.0 / 3.1415927;
const double voltsPerAdcCount = 3.3 / 256;   // ADC range 5 volts.  ADC is 10-bit (0-1023) but since only the ADCH register is used (0-255) resolution is 5/256=0.019 volts

// The ADC sample and hold occurs 2 ADC clocks (= 32 system clocks) after the timer 1 overflow flag is set.
// This introduces a slight phase error, which we adjust for in the calculations.

// why 45.0 ... is the RX phase natively 45 degrees different relative to the TX phase?  Under what test conditions?
const float phaseAdjust = (45.0 * 32.0) / (float)(TIMER1_TOP + 1);


// TODO: ADJUSTABLE THRESHOLD
// The user will be able to adjust this via a pot or rotary encoder.

float ampThreshold = 0.1;           // lower = greater sensitivity. 10 is barely usable with a well-balanced coil.
float posPhaseThreshold = 1.0;      // positive phase shift for ferrous
float negPhaseThreshold = -10.0;    // negative phase shift for non-ferrous



// OPAMP Gain
//   RX coil near the null point (minimum voltage point)      measured 25mV, opamp output to A1 was 643mV .. so OPAMP gain about 26.
//   A trimmer resistor might be needed to adjust the voltage divider feeding the OPAMP reference voltage




// ADC Converter
//
// The ATmega328P has one ADC, uses a multiplexor to select one of the analog input pins on the board,
// and acquires a 10-bit (or fewer bits) value representing the voltage level scaled to a reference voltage.
//
// The ADC clock is derived from the system clock running at 16 MHz, divided by a prescaler.  The default
// prescaler value of 128 sets the ADC clock rate to 125kHz.  Conversion requires 13.5 ADC clock cycles, but
// the first conversion is complete after 25 cycles due to analog circuit initialization.  Subsequent conversions
// are complete after 14 cycles.  Two 8-bit registers are read to retrieve the value, before the next conversion cycle
// can begin.
//
// If a lower number of ADC bits is adequate, a faster ADC clock rate can be used.  In this mode, only one of the
// 8-bit registers are read to retrieve the value (8-bit ADC vs the standard 10-bit ADC).  ADC conversions still require
// 14 cycles.
//
// To calculate the ADC clock:  CPU clock 16,000,000 / prescaler 128 = 125,000

/* benchmark readAnalog() without auto-triggering
     setup()
     {
       ADCSRA &= ~PS_128; // remove bits set by Arduino library, default ADC clock 125kHz

       // start with default prescaler
       ADCSRA |= PS_16;
   }

    loop()
    {
      const long Loops = 1000;
      const float CpuFreqUsec = 1000000.0;

      unsigned long value;
      unsigned long start_time;
      unsigned long stop_time;
      float average_usec;
      float samples_per_sec;
      float nyquist_hz;

      start_time = micros();    // micros() has 4 usec resolution

      int i;
      for ( i = 0; i < Loops; i++)
      {
        value = analogRead(0);
      }

      stop_time = micros();

      average_usec = ((float)stop_time - (float)start_time) / (float)Loops;
      samples_per_sec = (float)CpuFreqUsec / average_usec;
      nyquist_hz = samples_per_sec / 2;
    }

    // Benchmark results (note acquisition times range +/1 .01 usec)
    //  PS_0     3.08 usec, 324675.34 s/sec, Nyquist 162337.67 kHz
    //  PS_2     3.08 usec, 324675.34 s/sec, Nyquist 162337.67 kHz
    //  PS_4     4.65 usec, 214961.31 s/sec, Nyquist 107480.66 kHz
    //  PS_8     8.01 usec, 124875.12 s/sec, Nyquist  62437.56 kHz
    //  PS_16   15.05 usec,  66436.35 s/sec, Nyquist  33218.17 kHz
    //  PS_32   28.06 usec,  35643.00 s/sec, Nyquist  17821.50 kHz
    //  PS_64   56.00 usec,  17855.87 s/sec, Nyquist   8927.93 kHz
    //  PS_128 112.00 usec,   8928.25 s/sec, Nyquist   4464.13 kHz
  */
  
// ADCSRA control and status register  (0x7A) ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0
const unsigned char ADC_ENABLE = (1 << ADEN);

// ADC start/converting state
// write 1 to start conversion, or start the first conversion in free-running mode.
// When conversion is finished this bit is read as 0.
const unsigned char ADC_START = (1 << ADSC);

// ADC auto-trigger enable
const unsigned char ADC_AUTO_TRIGGER =  (1 << ADATE);

// ADC conversion-complete interrupt
const unsigned char COMPLETE_ISR = (1 << ADIF);

// ADC enable conversion-complete interrupt
const unsigned char INTERRUPT_ENABLE = (1 << ADIE);

// ADC clock prescalers
//   The successive approximation circuitry requires an input clock frequency between 50kHz and 200kHz to get maximum resolution (10 bits).
//   If a lower resolution than 10 bits is needed, the input clock frequency to the ADC can be higher than 200kHz to get a higher sample rate.
// 8-bit resolution ADLAR=1
const unsigned char PS_0 = 0;                                             // 000 (ADC clock 8MHz)
const unsigned char PS_2 = (1 << ADPS0);                                  // 001 (ADC clock 8MHz)
const unsigned char PS_4 = (1 << ADPS1);                                  // 010 (ADC clock 4MHz)
const unsigned char PS_8 = (1 << ADPS1) | (1 << ADPS0);                   // 011 (ADC clock 2MHz)
const unsigned char PS_16 = (1 << ADPS2);                                 // 100 (ADC clock 1MHz)
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);                  // 101 (ADC clock 500kHz)
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);                  // 110 (ADC clock 250kHz)
// 10-bit resolution ADLAR=0
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // 111 (ADC clock 125kHz)

// ADMUX register (0x7C) REFS1 REFS0 ADLAR – MUX3 MUX2 MUX1 MUX0
// ADMUX voltage reference REFS1..REFS0
const unsigned char AREF_EXT = 0;                                         // 00
const unsigned char AREF_VCC = (1 << REFS0);                              // 01
const unsigned char AREF_RESV = (1 << REFS1);                             // 10
const unsigned char AREF_1_1V = (1 << REFS1) | (1 << REFS0);              // 11


// ADMUX left adjust result in registers ADCH/ADCL
const unsigned char LEFT_ADJUST = (1 << ADLAR);                           // 1

// which ADC channel will be converted
// ADMUX Multiplexor channel selectors MUX3..MUX0
const unsigned char CH_0 = 0;                                             // 0000
const unsigned char CH_1 = (1 << MUX0);                                   // 0001
const unsigned char CH_2 = (1 << MUX1);                                   // 0010
const unsigned char CH_3 = (1 << MUX0) | (1 << MUX1);                     // 0011
const unsigned char CH_4 = (1 << MUX2);                                   // 0100
const unsigned char CH_5 = (1 << MUX2) | (1 << MUX0);                     // 0101
const unsigned char CH_6 = (1 << MUX2) | (1 << MUX1);                     // 0110
const unsigned char CH_7 = (1 << MUX2) | (1 << MUX1) | (1 << MUX0);       // 0111
const unsigned char CH_Temp = (1 << MUX3);                                // 1000
// reserved 1001-1101
const unsigned char CH_1_1V = (1 << MUX3) | (1 << MUX2) | (1 << MUX1);    // 1110
const unsigned char CH_GND = (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | 1; // 1111

// ADC data registers ADCL, ADCH
//   When ADCL is read, the ADC data register is not updated until ADCH is read. Consequently, if the result is left adjusted and
//   no more than 8-bit precision is required, it is sufficient to read ADCH. Otherwise, ADCL must be read first, then ADCH.
//
// ADLAR == 0   (0x79) – – – – – – ADC9 ADC8 ADCH
//              (0x78) ADC7 ADC6 ADC5 ADC4 ADC3 ADC2 ADC1 ADC0 ADCL
// ADLAR == 1   (0x79) ADC9 ADC8 ADC7 ADC6 ADC5 ADC4 ADC3 ADC2 ADCH
//              (0x78) ADC1 ADC0 – – – – – – ADCL


// ADCSRB Control and Status Register B (0x7B) – ACME – – – ADTS2 ADTS1 ADTS0
// enable analog comparator
const unsigned char ANALOG_COMPARATOR = (1 << ACME);

// ADC conversion trigger source
const unsigned char FREE_RUNNING = 0;                                      // 000
const unsigned char COMPARATOR_MATCH = (1 << ADTS0);                       // 001
const unsigned char EXTERNAL_REQUEST0 = (1 << ADTS1);                      // 010
const unsigned char TIMER_COUNTER0_MATCHA = (1 << ADTS1) | (1 << ADTS0);   // 011
const unsigned char TIMER_COUNTER0_OVERFLOW = (1 << ADTS2);                // 110
const unsigned char TIMER_COUNTER1_MATCHB = (1 << ADTS2) | (1 << ADTS0);   // 101
const unsigned char TIMER_COUNTER1_OVERFLOW = (1 << ADTS2) | (1 << ADTS1); // 110
const unsigned char TIMER_COUNTER1_CAPTURE = (1 << ADTS2) | (1 << ADTS1) | (1 << ADTS0);   // 111


// DIDR0 digital input disable register (0x7E) – – ADC5D ADC4D ADC3D ADC2D ADC1D ADC0D
const unsigned char DID_DISABLE_CH_0 = (1 << ADC0D);                       // 000001
const unsigned char DID_DISABLE_CH_1 = (1 << ADC1D);                       // 000010
const unsigned char DID_DISABLE_CH_2 = (1 << ADC2D);                       // 000100
const unsigned char DID_DISABLE_CH_3 = (1 << ADC3D);                       // 001000
const unsigned char DID_DISABLE_CH_4 = (1 << ADC4D);                       // 010000
const unsigned char DID_DISABLE_CH_5 = (1 << ADC5D);                       // 100000


// Timer1
// TCCCR1A Timer/Counter1 Control Register A (0x80) COM1A1 COM1A0 COM1B1 COM1B0 – – WGM11 WGM10
const unsigned char CLEAR_01CA_01CB_WHEN_UPCOUNTING_SET_WHEN_DOWNCOUNTING = (1 << COM1A1);       // 10
const unsigned char MODE2_PHASECORRECT_PWM_9BIT = (1 << WGM11);            // 010

// TCCR1B Timer/Counter1 Control Register B (0x81) ICNC1 ICES1 – WGM13 WGM12 CS12 CS11 CS10
const unsigned char CTC_MODE = (1 << WGM12) | (1 << WGM13);                // 11  clear timer on compare match
const unsigned char NO_TIMER_PRESCALING = (1 << CS10);

// TCCR1C Timer/Counter1 Control Register C (0x82) FOC1A FOC1B – – – – – – TCCR1C

// OCR1AH Output Compare Register 1 A (0x89) OCR1A[15:8]
// OCR1AL                             (0x88) OCR1A[7:0]

// TCNT1H Timer/Counter 1 (0x85) TCNT1[15:8]
// TCNT1L                 (0x84) TCNT1[7:0]

// TIFR0 Timer/Counter 0 Interrupt Register 0x15 (0x35) – – – – – OCF0B OCF0A TOV0
const unsigned char TIMER0_BMATCH_FLAG = (1 << OCF0B);
const unsigned char TIMER0_AMATCH_FLAG = (1 << OCF0A);
const unsigned char TIMER0_OVERFLOW_FLAG = (1 << TOV0);

// TIFR1 Timer Interrupt 1 Flag Register 0x16 (0x36) – – ICF1 – – OCF1B OCF1A TOV1
const unsigned char TIMER1_BMATCH_FLAG = (1 << OCF1B);
const unsigned char TIMER1_AMATCH_FLAG = (1 << OCF1A);
const unsigned char TIMER1_OVERFLOW_FLAG = (1 << TOV1);

// TIMSK1 Timer/Counter 1 Interrupt Mask Register (0x6F) – – ICIE1 – – OCIE1B OCIE1A TOIE1
// ICIE1: Timer/Counter1, Input Capture Interrupt Enable
// OCIE1B: Timer/Counter1, Output Compare B Match Interrupt Enable
// OCIE1A: Timer/Counter1, Output Compare A Match Interrupt Enable
const unsigned char TIMER1_OVERFLOW_INTERRUPT_ENABLE = (1 << TOIE1);

// TCCR0A Timer/Counter Control Register A 0x24 (0x44) COM0A1 COM0A0 COM0B1 COM0B0 – – WGM01 WGM00
// COM0A1:0: Compare Match Output A Mode
// COM0B1:0: Compare Match Output B Mode
// WGM01:0: Waveform Generation Mode
const unsigned char CLEAR_OC0B_ON_COMPARE_MATCH = (1 << COM0B1);
const unsigned char FAST_PWM_MODE = (1 << WGM01) | (1 << WGM00);

// TTCR0B Timer/Counter Control Register B 0x25 (0x45) FOC0A FOC0B – – WGM02 CS02 CS01 CS00
// WGM02: Waveform Generation Mode
// CS02:0: Clock Select
const unsigned char EXTERNAL_T0_PIN_RISING_EDGE = (1 << CS00) | (1 << CS01) | (1 << CS02);       // 111
const unsigned char CLEAR_0C0A_ON_COMPARE_MATCH = (1 << WGM02);

// OCR0A – Output Compare Register A - compare to TCNT0, interrupt or waveform on 0C0A pin

// OCR0B – Output Compare Register B - compare to TCNT0, interrupt or waveform on 0C0B pin

// Timer/Counter 0 8-bit counter TCNT0 0x26 (0x46) TCNT0[7:0]



void setup()
{
  // pin used for ISR debugging

  pinMode(blinkPin, OUTPUT);
  digitalWrite(blinkPin, LOW);   // OFF

  // initialize the LCD display
  
  int status = lcd.begin(16, 2);   // LCD 16x2
  if(status)
  {
    // begin() failed so blink error code using the onboard LED if possible
    hd44780::fatalError(status); // does not return
  }

  //lcd.backlight();
  lcd.display();


  // pins for the metal detector
  
  pinMode(encoderButtonPin, INPUT_PULLUP);
  pinMode(earpiecePin, OUTPUT);
  
  digitalWrite(T0OutputPin, LOW);
  pinMode(T0OutputPin, OUTPUT);       // pulse pin from timer 1 used to feed timer 0

  pinMode(T0InputPin, INPUT);
  
  digitalWrite(coilDrivePin, LOW);
  pinMode(coilDrivePin, OUTPUT);      // timer 0 output, square wave to drive transmit coil




    // By running the CPU at 16MHz, the ADC clock at 1MHz, reducing ADC resolution to 8 bits, and acquiring ADC
    // samples continuously using the timer interrupt, the time per ADC conversion can be much reduced.
    //
    // Timer 1 is used to divide the system clock by about 256 (TIMER1_TOP)
    //    - to produce a 62.5kHz square wave
    //    - drive Timer 0, and
    //    - to trigger ADC conversions
    //
    // Timer 0 is used to divide the output of timer 1 by 8, giving a 7.8125kHz signal for driving the transmit coil.
    //
    // This gives us 16 ADC clock cycles for each ADC conversion (it actually takes 13.5 cycles), and we take 8 samples per cycle of the coil drive voltage.
    // The ADC implements four phase-sensitive detectors at 45 degree intervals. Using 4 instead of just 2 allows us to cancel the third harmonic of the
    // coil frequency.

    // Interrupts off
    cli();

    // Stop timer 0 which was set up by the Arduino core for millis(), micros(), etc
    TCCR0B = 0;        // stop the timer
    TIMSK0 = 0;        // disable interrupt
    TIFR0 = TIMER0_BMATCH_FLAG | TIMER0_AMATCH_FLAG | TIMER0_OVERFLOW_FLAG;      // clear any pending interrupt


    // Select the ADC reference voltage
    // Read channel A1, turn off the digital input feature for A1
    // Trigger ADC conversions to occur on timer 1 overflow.
    // Set the ADC prescaler for a 1MHz clock.  Because the clock is greater than 200kHz, only 8-bit ADC values can be read so set ADLAR to 1.

    // use AREF pin (connected to 3.3V) as voltage reference, read pin A1, left-adjust result
    ADMUX = 0;
    ADMUX |= LEFT_ADJUST;      // only 8-bit conversion values will be read
    ADMUX |= CH_1;             // reading analog input A1
    
    #if USE_3V3_AREF
      ADMUX |= AREF_EXT;       // use AREF pin (connected to 3.3V) as voltage reference
    #else
      ADMUX |= AREF_VCC;       // use AVcc (5V) as voltage reference
    #endif

    ADCSRB = TIMER_COUNTER1_OVERFLOW;                             // auto-trigger ADC on timer/counter 1 overflow

    ADCSRA = ADC_ENABLE | ADC_START | ADC_AUTO_TRIGGER | PS_16;   // enable and start ADC, auto-trigger, prescaler = 16 (1MHz ADC clock)

    DIDR0 = 0;
    DIDR0 |= DID_DISABLE_CH_1;

    // Set up timer 1.
    // Prescaler = 1, phase correct PWM mode, TOP = ICR1A
    TCCR1A = CLEAR_01CA_01CB_WHEN_UPCOUNTING_SET_WHEN_DOWNCOUNTING | MODE2_PHASECORRECT_PWM_9BIT;
    TCCR1B = CTC_MODE | NO_TIMER_PRESCALING;    // CTC mode, prescaler = 1

    TCCR1C = 0;

    // Divide the system clock by about 256 to produce a 62.5kHz square wave - tune the TOP value as necessary to achieve
    // the desired coil-drive frequency driven by Timer 0
    OCR1AH = (TIMER1_TOP/2 >> 8);
    OCR1AL = (TIMER1_TOP/2 & 0xFF);
    ICR1H = (TIMER1_TOP >> 8);
    ICR1L = (TIMER1_TOP & 0xFF);
    
    TCNT1H = 0;
    TCNT1L = 0;

    TIFR1 = TIMER1_BMATCH_FLAG | TIMER1_AMATCH_FLAG | TIMER1_OVERFLOW_FLAG;      // clear any pending interrupt

    TIMSK1 = TIMER1_OVERFLOW_INTERRUPT_ENABLE;

    // Set up timer 0
    // Clock source = T0, fast PWM mode, TOP (OCR0A) = 7, PWM output on OC0B
    // Timer 0 is used to divide the output of timer 1 by 8, giving a 7.8125kHz signal for driving the transmit coil.
    TCCR0A = CLEAR_OC0B_ON_COMPARE_MATCH | FAST_PWM_MODE;
    
    TCCR0B = EXTERNAL_T0_PIN_RISING_EDGE | CLEAR_0C0A_ON_COMPARE_MATCH;

    // configure output compare registers.  The result of the compare (to TCNT0) generates output PWM on the output compare pins (OCOA - pin 6 pinchip 12 PD6, OCOB pin 5 coil)
    
    // In clear timer on compare or CTC mode (WGM02:0 = 2), the OCR0A register is used to manipulate the counter resolution. In
    //   CTC mode the counter is cleared to zero when the counter value (TCNT0) matches the OCR0A. The OCR0A defines the top
    //   value for the counter, hence also its resolution. This mode allows greater control of the compare match output frequency. It
    //   also simplifies the operation of counting external events.
    
    OCR0A = 7;    // 0C0A pin (D5-coil) when TCNT0 == 7    // does this provide every-8th timing?
    OCR0B = 3;    // 0C0B (pin D6 ?) when TCNT0 == 3

    // reset the Timer/Counter 8-bit counter
    TCNT0 = 0;


    // alternate tone generation copied from https://github.com/dc42/arduino/blob/master/MetalDetector/MetalDetector.ino
    // tone() function also copied

    // Set up timer 2 for tone generation
    TIMSK2 = 0;
    TCCR2A = (1 << COM2B0) | (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);   // set OC2B on compare match, clear OC2B at zero, fast PWM mode
    TCCR2B = (1 << WGM22) | (1 << CS22) | (1 << CS21);                      // prescaler 256, allows frequencies from 245Hz upwards
    OCR2A = 62;   // 1kHz tone
    OCR2B = 255;  // greater than OCR2A to disable the tone for now. Set OCR2B = half OCR2A to generate a tone.

    ticks = 0;
  
    // interrupts on
    sei();


  Serial.begin(115200);
//  Serial.println("Discarding first sample");

    while (!sampleReady) {}    // discard the first sample collected by the ISR

    sampleReady = false;
    sampleReadyTicks = 0;


  // calibration required
  lcd.print("Calibrate");
  calibrated = false;
}

// tone(freq in Hz)
void tone(unsigned int freq)
{
  if (freq == 0)
  {
    OCR2B = 255;
  }
  else
  {
    const uint8_t divisor = 62500u/constrain(freq, 245u, 6000);
    OCR2A = divisor;
    OCR2B = divisor/2;
  }
}



//   Timer 1 is used to divide the system clock by about 256 to produce a 62.5kHz square wave, which in turn is used to drive timer 0 and also to trigger ADC conversions.
//   Timer 0 is used to divide the output of Timer 1 by 8, giving (about) 8kHz square wave for driving the transmit coil (the TX coil LC network converts that to a sin wave).
//   If the coil inductance is known, the capacitor value can be chosen for resonance at the TIMER0 frequency, for maximum sensitivity of the detector.
//
// Timer 1 overflow interrupt. This serves 2 purposes:
// 1. It clears the timer 0 overflow flag. If we don't do this, the ADC will not see any more Timer 0 overflows and we will not get any more conversions.
// 2. It increments the tick counter, allowing is to do timekeeping. We get 62500 ticks/second, depending on the Timer1 frequency that was configured.
//
// We now read the ADC in the timer interrupt routine instead of having a separate conversion complete interrupt.  (?? is ADC-completion checked, or is
//   Timer1 running so slowly that the 14-cycle ADC conversion is always completed when the interrupt occurs ??)
//
// The ADC value is added to the corresponding phase bin, value is capped at +/-15000  (max signed integer value +/-32767)
//
//   Timer0 square wave is 1/8 the frequency of Timer1 interrupt rate.  So Timer1 interrupts 8 times during one complete Timer0 cycle (one 50% duty cycle square wave sent to the TX coil).
//   The TX cycle is divided into 8 intervals .. for half of them the output is HIGH and the second have is LOW.  The TX coil LC network response time converts the square wave to a sin
//   wave.  The RX coil LC network receives a reflected, weakened and phase-shifted version of the sin wave, modulated by the response of any metal in the magnetic field.  The phase
//   shifts negative for non-ferrous and no shift or a slight positive shift for ferrous metals.  The amplitude of the received signal is higher for large or near objects than it is
//   for objects smaller or farther away.
//
//   At the end of each of the 8 intervals the amplified signal from the RX coil is read and converted by the ADC.  The first four intervals represent the first half of the output square
//   wave duty cycle and the second four intervals the second half.  During the first cycle the amplitude is expected to be positive, during the second negative.  The maximum amplitude and
//   the the phase shift relative to the output square wave are determined in this ISR.
//
// If Timer0 has the value 7 (every 8th count?) and the number of samples acquired has reached the number-to-average amount, the phase bin values are copied
// to the averages array, bins are zeroed, and sampleReady is set to true.
//
// A race condition between the main code that reads the averages array, and the ISR, is managed using the sampleReady flag .. averages[] is not loaded unless
// the main program has set sampleReady to false.  The maximum rate sampleReady will be set true to deliver averages[] to the main loop() is about once/second
// 8192 ticks.
//
// The current value of Timer0 (which counts upward from 0 to 7 before overflowing) is compared to the predicted value.
ISR(TIMER1_OVF_vect)
{
  ++ticks;
  uint8_t ctr = TCNT0;                      // current value of Timer0 - the counter value is synchronized with the output PWM signal
  int16_t val = (int16_t)(uint16_t)ADCH;    // only need to read most significant 8 bits (0-255), when using 1 MHz ADC clock rate. ADCH contains the previous ADC conversion, the read starts the next ADC conversion cycle.

  if (ctr != ((lastctr + 1) & 7))           // checking whether Timer0 has incremented since the last interrupt, or whether a Timer0 count got missed
  {
    // if one of the counts (0-7) got missed then one or more of the bins won't get updated with ADC value, the ACD could be incomplete and return zero
    // ?? how could this happen?  TCNT0 counter rolled while interrupts were off? ISR taking too long? Aliasing between TIMER1 interrupt rate and the TIMER0 TCNT0 update rate?  power supply unstable?
    ++misses;
  }
  lastctr = ctr;                           // counter value 0,1,2,3,4,5,6,7
  
 
  // exploring symmetry:  min-max voltages peak-to-peak (x-y diff=), in the first half of the cycle (128-142 diff=14), in the second half (112-125 diff=13)
  // if the voltage divider feeding the OPAMP isn't exactly 3.3V/2 the neutral value from the ADC can be different than 127 or 128.  Set zeroValue to the actual neutral value.
  if (val < adcMin)    adcMin = val;
  if (val > adcMax)    adcMax = val;

  int16_t valZeroCorrected = (zeroValue - val);
  rawbins[ctr] = valZeroCorrected;                // zero-corrected ADC reading

  // pointers to value storage
  int16_t *p = &bins[ctr & 3];             // bitwise AND providing an index 0..3  - for the four 45-degree samples

/*
  if (ctr < 4)
  {                                        // 0,1,2,3  (eg: 128-142 diff=14)
                                           // this is the first half of the cycle when the PWM output is HIGH so ADC value is expected to be above the 'zero' midpoint (128)
    *p += (val - zeroValue;               // accumulate the ADC value (0-255) relative to 0V (128) into bin[ctr] with the result positive
    *b = (val - zeroValue);
    
    if (*p > 15000) *p = 15000;            // set a ceiling on the accumulated value
  }
  else
  {                                        // 4,5,6,7  (eg: 112-125 diff=13)
                                           // this is the last half of the cycle when the PWM output is LOW so ADC value is expected to be below the 'zero' midpoint (128)
    *p -= (zeroValue - val);               // accumulate the ADC value (0-255) relative to 0V (128) into bin[ctr] with the result negative.
    *b = (zeroValue - val);
    
    if (*p < -15000) *p = -15000;          // sets a minimum on the accumulated value
  }
*/

  // only collecting the second half of the cycle - ADC readings more closely reflect the waveform
  if (ctr >= 4)
  {
#if USE_INTEGRATION
    // integration technique (amplifies the signal by adding contribution from numerous samples)
    *p += valZeroCorrected;               // zero-corrected averaged ADC value
    
    if (*p > maxIntegrationAmount) *p = maxIntegrationAmount;            // set a ceiling on the accumulated value (15000)
#else
    // averaging technique
    *p = (*p + valZeroCorrected)/2;       // zero-corrected averaged ADC value
#endif
  }

  if (ctr == 7)
  {
    ++numSamples;
    if (numSamples == samplesToAccumulate)  // 1024 full-wave cycles have been averaged in the bins
    {
      // if loop() did not signal ready to receive another sample, this one will be discarded
      if (!sampleReady)
      {
        memcpy((void*)averages, bins, sizeof(averages));         // 4 values
        memcpy((void*)readings, rawbins, sizeof(readings));      // 8 values
        readingsMin = adcMin;
        readingsMax = adcMax;
        readingsMisses = misses;
        
        sampleReady = true;
        sampleReadyTicks = ticks;
      }
      
      numSamples = 0;
      memset(bins, 0, sizeof(bins));
    }

    adcMin = 255;
    adcMax = 0;    // ideal midpoint between 0-255 assuming the OPAMP value at 0V gives an ADC reading of 128 (or 127)
    misses = 0;
  }
}

// MAIN LOOP - TO AVOID CONFLICTS WITH THE INTERRUPT SERVICE ROUTINE, LOAD ALL ISR-MANAGED VARIABLES USED BY loop() IMMEDIATELY WHEN sampleReady == TRUE.  THEN SET
//             sampleReady = FALSE and continue with printing/etc.
void loop()
{
  digitalWrite(blinkPin, LOW);   // OFF

  // a new sample in averages[] will be available approximately once/second .. 8192 ticks
  // NOTE one sampleReady interval is already 1 second or about 8000 samples!
 
  while (!sampleReady) {}
  
  uint32_t oldTicks = ticks;
  uint32_t sampleReadyLatency = 0;
  
  if (digitalRead(encoderButtonPin) == LOW)
  {
    // Calibrate button pressed. Save the average coil voltage to subtract from future results.
    // This lets us use the detector if the coil is slightly off-balance.
    // Calibration can be repeated as often as needed.
    
    lcd.setCursor(0,0);
    lcd.print("Calibrating...  ");

    Serial.print("Calibrating");

    calibratedVolts = 0;
    calibratedPhase = 0;
    
    const int CalibrationCycles = 100;
    for( int cycles=0; cycles<CalibrationCycles; cycles++)
    {
      volts = (readingsMax-readingsMin) * voltsPerAdcCount;
      calibratedVolts = (calibratedVolts + volts)/2;
      
      while (!sampleReady) {}
      
      Serial.print('.');
    }

    calibrated = true;

    // erase "Calibrating...  "
    lcd.setCursor(0,0);
    lcd.print("                ");

    sampleReady = false;
    sampleReadyLatency = ticks - sampleReadyTicks;
    sampleReadyTicks = 0;

    Serial.print("done: ");

    Serial.println(calibratedVolts);

    return;
  }


  
  const double f = 8000.0;    // 200.0;

  // I think the averages index (0-3) represents the Timer0 output waveform phase since the ctr value in the ISR
  // determines which bin is filled with the ADC value at that moment.  The value itself represents the waveform
  // on the receiver coil.  As only one waveform (the received) is provided, phase shift is computed based on which
  // of the averages[] elements *WITH RESPECT TO THE TX WAVEFORM TIMES* is higher or lower than expected.

  // averages[] are provided to loop() at about a 1Hz rate (8192 ticks)

  // Massage the results to eliminate sensitivity to the 3rd harmonic (Timer0 freq x 3), and divide by 200
  // Why 200?  Why is the 3rd harmonic a concern?  How does this math desensitize?
  //     - 2nd harmonic tends to lower the sin peak at 90 and 270 deg
  //     - 3rd harmonic tends to lower the right half the peak between 90-180

  // averages[0..3] contains the net difference at a certain phase angle of the first half of a TX (transmitted) clock cycle, minus the same phase angle of the second half of the clock cycle.
  //   The RX (received) signal will track the TX signal at a lesser amplitude and with a phase shift.
  
  // Compute the amplitude and the amount of phase shift - using FOURIER ANALYSIS.
  // https://stats.stackexchange.com/questions/541277/how-can-i-get-the-phase-difference-between-two-frequencies
  // https://forum.arduino.cc/t/program-to-find-zero-crossing-of-a-sinusoidal-wave/443323/7
  // If you know the frequency, sample them - not sure how many samples you need - then do a simple fourier analysis. Use micros() to control when you take the samples.
  // For instance, if your sine waves are 50Hz, that's 20ms. Take samples at the 1ms mark gives you 20 samples per cycle. Pre compute sine and cosine of 2pi/20 * n.
  //   Take your samples. multiply them by the sines and cosines and average each. atan2 of those will give you the phase.
  //
  // https://www.linkedin.com/pulse/how-fft-algorithm-works-part-2-divide-and-conquer-mark-newman/
  
  // Complex number
  double f0real = (averages[0] + halfRoot2 * (averages[1] - averages[3]))/f;           // the Real part at 0deg
  double f0cmpl = (averages[1] + halfRoot2 * (averages[0] + averages[2]))/f;           // the Complex part at 45deg
  
  double f1real = (averages[2] + halfRoot2 * (averages[1] + averages[3]))/f;           // the Real part at 90deg
  double f1cmpl = (averages[3] + halfRoot2 * (averages[2] - averages[0]))/f;           // the Complex part at 135deg

  // compute magnitude (amplitudes) 
  double amp1 = sqrt((f0real * f0real) + (f1real * f1real));
  double amp2 = sqrt((f0cmpl * f0cmpl) + (f1cmpl * f1cmpl));
  double ampAverage = (amp1 + amp2)/2.0;

  // The ADC sample/hold takes place 2 clocks after the timer overflow
  //  ?? two Timer1 clocks ??

  // compute the phase angle - slopes to radians to degrees?
  double phase1 = atan2(f0real, f1real) * radiansToDegrees + 45.0;
  double phase2 = atan2(f0cmpl, f1cmpl) * radiansToDegrees;

  if (phase1 > phase2)
  {
    // swap the phase values
    double temp = phase1;
    phase1 = phase2;
    phase2 = temp;
  }

  double phaseAverage = ((phase1 + phase2)/2.0) - phaseAdjust;                  // (45.0 * 32.0) / (float)(TIMER1_TOP + 1);

  // change the sign of the phase, since the ISR only collected the second half of the PWM cycle whose amplitude is generally opposite of the first half
  // when compared to phase calculated by an oscilloscope, the values agree closely (although not exactly) prior to adjusting for phaseCalibration
  // on the Rigol 1052E, with Ch1 the PWM pulse and Ch2 the output from the OPAMP
  //   if the positive peak of the wave is nearest to the rising edge of the PWM output the phase is negative (around -90 degrees if right on the edge)
  //   if the positive peak is nearest to the falling edge of the output the phase is positive (around 90 degrees if right on the edge)
  //   if the positive peak is in the middle of the PWM positive pulse, the phase is zero
  // metal passing over the non-overlapping parts of the coils produces a negative phase shift, while in the neutral middle will show positive
  // best sensitivity and range is in the middle when the coil is adjusted for minimum voltage (60 mA or less)
  
  phaseAverage = -phaseAverage;

  phaseStable = abs(previousPhase - phaseAverage) <= 5.0;      // phase fluctuates wildly especially when the signal voltage is very low
  previousPhase = phaseAverage;

  // phase part of calibration
  if(calibrated && calibratedPhase == 0)
  {
    // displayed phase will become the phase relative to calibration
    // caution. near minimum coil voltage phase fluctuates wildly
    calibratedPhase = phaseAverage;
  }
  
  double displayPhase = phaseAverage - calibratedPhase;

  volts = (readingsMax-readingsMin) * voltsPerAdcCount;
  voltsAmplitude = (volts - calibratedVolts);

  if (readingsMisses > 0)
  {
    digitalWrite(blinkPin, HIGH);  // turn on the overflow LED ('L' on the Arduino board)
  }
  
  // ***** NO CALCULATIONS BELOW HERE - ONLY OUTPUTS *****


#if PRINT_TICKS    
  Serial.print(ticks);    // current ISR clock tick - theoretically 8192 but actually 8000-8039 per sample
  Serial.write(' ');
#endif

#if PRINT_ISR_VARS
  Serial.print(sampleReadyLatency);
  Serial.write(' ');
  Serial.print(readingsMisses);
  Serial.print('/');
  Serial.print(samplesToAccumulate);
  Serial.write(' ');
#endif

#if PRINT_ADC_MINMAX
  Serial.print(readingsMin);
  Serial.write(' ');
  Serial.print(readingsMax);
  Serial.write(' ');
#endif

#if PRINT_VOLTS
  Serial.print(volts, 3);
  Serial.write(' ');
#endif

#if PRINT_FFT
  // fourier series
  if (f0real >= 0.0) Serial.write(' ');  // prints a space where the minus sign would be
  Serial.print(f0real, 2);
  Serial.write(' ');
  
  if (f0cmpl >= 0.0) Serial.write(' ');
  Serial.print(f0cmpl, 2);
  Serial.write(' ');
  
  if (f1real >= 0.0) Serial.write(' ');
  Serial.print(f1real, 2);
  Serial.write(' ');
  
  if (f1cmpl >= 0.0) Serial.write(' ');
  Serial.print(f1cmpl, 2);
  Serial.write(' ');

  // fourier magnitudes
  Serial.print(amp1, 2);
  Serial.write(' ');
  
  Serial.print(amp2, 2);
  Serial.write(' ');

  if (phase1 >= 0.0) Serial.write(' ');
  Serial.print(phase1, 2);
  Serial.write(' ');
  
  if (phase2 >= 0.0) Serial.write(' ');
  Serial.print(phase2, 2);
  Serial.print("    ");
#endif

#if PRINT_AMPLITUDE
  // waveform amplitude after integration
  if (ampAverage >= 0.0) Serial.write(' ');
  Serial.print(ampAverage, 2);

  // volts amplitude
  if (voltsAmplitude >= 0.0) Serial.write(' ');
  Serial.print(voltsAmplitude, 2);
  Serial.write(' ');
#endif

#if PRINT_PHASE
  if (phaseAverage >= 0.0) Serial.write(' ');
  Serial.print(phaseAverage, 0);
  Serial.write(' ');

  Serial.print(displayPhase, 0);
  Serial.write(' ');

  if (!phaseStable)
  {
    Serial.print("unstable");
    Serial.write(' ');
  }
#endif

#if PRINT_AVERAGES
  Serial.println();  
  for (int i=0;i<4; i++)
  {
    Serial.println(averages[i]);   // one value per line for the Serial Plotter
  }
  Serial.write(' ');
#endif

#if PRINT_READINGS
  Serial.println();  
  for (int i=0;i<8; i++)
  {
    Serial.println(readings[i]);
  }
  Serial.write(' ');
#endif

  ticks = 0;

#if DISPLAY_VOLTS 
  // adjust D-coil overlap for minimum voltage - 40 mV (0.040) is achievable with care.  Then CALIBRATE.
  lcd.setCursor(0,1);
  lcd.print("V");
  lcd.print(volts, 2);
#endif

#if DISPLAY_AMPLITUDE
  lcd.setCursor(10,0);
  lcd.print("      ");
  
  lcd.setCursor(10,0);
  lcd.print("A");
  
  // amplitude using the ISR averaging method is extremely small or zero - use volts instead of ampAverage
  // if using integration in the ISR to amplify the signal - ampAverage can be used

  double amplitude = 0;

#if USE_INTEGRATION
  amplitude = ampAverage;
#else
  amplitude = voltsAmplitude * 100;
#endif
    
  if (amplitude >= 100)
  {
    lcd.print(" ");
  }
  else if (amplitude >= 10)
  {
    lcd.print("  ");
  }
  else
  {
    lcd.print("   ");
  }

  lcd.print(amplitude, 0);
#endif

#if DISPLAY_BARS
  //lbg.drawValue(ampAverage, max_ampAverage);      // bargraph
#endif

#if DISPLAY_PHASE
  lcd.setCursor(10,1);
  lcd.print("      ");

  lcd.setCursor(10,1);
  lcd.print("P");
  
  if (phaseStable)
  {
    if (displayPhase >= 0.0) lcd.print(' ');    // otherwise there's a minus sign
  
    // right-justify the number
    if (abs(displayPhase) < 100)
      lcd.print(' ');
    else if (abs(displayPhase) < 10)
      lcd.print(' ');
    else if (abs(displayPhase) < 1)
      lcd.print(' ');
    
    lcd.print(displayPhase, 0);
    lcd.print((char)223);        // degree symbol
  }
#endif

  // Decide what we have found and tell the user
  if (ampAverage >= ampThreshold)
  {
#if ENABLE_SOUND
    // detection tone generation copied from ds42
     if (calibrated)
     {
       tone(ampAverage * 10 + 245);
     }
#endif
       
    // When held in line with the centre of the coil:
    // - non-ferrous metals give a negative phase shift, e.g. -90deg for thick copper or aluminium, a copper olive, -30deg for thin alumimium.
    // Ferrous metals give zero phase shift or a small positive phase shift.
    // So we'll say that anything with a phase shift below -20deg is non-ferrous.
    
    if (phaseAverage <= negPhaseThreshold)
    {
#if PRINT_MATERIAL        
      Serial.print("Non-ferrous");
      Serial.write(' ');
#endif

#if DISPLAY_MATERIAL
      lcd.setCursor(0,0);
      lcd.print("nonFER");
#endif
      
    }
    else if (phaseAverage >= posPhaseThreshold)
    {
#if PRINT_MATERIAL        
      Serial.print("Ferrous");
      Serial.write(' ');
#endif

#if DISPLAY_MATERIAL
      lcd.setCursor(0,0);
      lcd.print("FER");
#endif
    }

#if PRINT_BARS
    // graphical display of amount over the threshold
    float temp = ampAverage;
    while (temp > ampThreshold)
    {
      Serial.write('!');
      temp -= (ampThreshold/2);
    }
#endif
  }

  else
  {
     tone(0);
  }

  Serial.println();

  sampleReady = false;          // we've finished reading the averages, so the ISR is free to overwrite them again
  sampleReadyTicks = 0;
  sampleReadyLatency = ticks - sampleReadyTicks;


  // ISR interrupts just over 8000 times per sample.  No point in running the loop() any faster.
  //while (ticks - oldTicks < 8000)
  //{
  //}
}

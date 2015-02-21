
/*
      This code was quick and dirty, based on a PCM audio example in the
      arduino playground: http://playground.arduino.cc/Code/PCMAudio

      It's been heavely modified for use with RC to generate something that's
      a bit like an engine sound. I've started work on making the program
      readable, still some to do though.
*/


#include "settings.h"
#include "idle.h"

// Mode settings - These could easily be 4 jumpers connected to spare pins, checked at startup to determine mode
boolean pwmThrottle = true;        // Takes a standard servo signal on pin 2 (UNO)

// Stuff not to play with!
#define SPEAKER 5                               // Seapker pin = 5
volatile uint16_t currentSmpleRate = BASE_RATE; // Current playback rate, this is adjusted depending on engine RPM
boolean audioRunning = false;                   // Audio state, used so we can toggle the sound system
uint16_t curVolume = 0;                         // Current digi pot volume, used for fade in/out
volatile uint16_t curEngineSample;              // Index of current loaded sample
uint8_t  lastSample;                            // Last loaded sample
int16_t  currentThrottle = 0;                   // 0 - 1000, a top value of 1023 is acceptable
uint8_t  throttleByte = 0;                      // Raw throttle position in SPI mode, gets mapped to currentThrottle
uint8_t  spiReturnByte = 0;                     // The current RPM mapped to a byte for SPI return
volatile int16_t pulseWidth = 0;                // Current pulse width when in PWM mode





void setup()
{
  // pwm in setup, for a standard servo pulse
  pinMode(2, INPUT); // We don't want INPUT_PULLUP as the 5v may damage some receivers!
  if(pwmThrottle){   // And we don't want the interrupt firing when not in pwm mode
    attachInterrupt(0, getPulsewidth, CHANGE);
  }
  // setup complete, so start making sounds
  startPlayback();
}




void loop()
{
	doPwmThrottle();
}




/* _____ _               _   _   _
  |_   _| |__  _ __ ___ | |_| |_| | ___  ___
    | | | '_ \| '__/ _ \| __| __| |/ _ \/ __|
    | | | | | | | | (_) | |_| |_| |  __/\__ \
    |_| |_| |_|_|  \___/ \__|\__|_|\___||___/ */

void doPwmThrottle(){
    if(pulseWidth > 800 && pulseWidth < 2200){ // check if the pulsewidth looks like a servo pulse
      if(pulseWidth < 1000) pulseWidth = 1000; // Constrain the value
      if(pulseWidth > 2000) pulseWidth = 2000;

      if(pulseWidth > 1520) currentThrottle = (pulseWidth - 1500) *2;  // make a throttle value from the pulsewidth 0 - 1000
      else if(pulseWidth < 1470) currentThrottle = abs( (pulseWidth - 1500) *2);
      else currentThrottle = 0;
      currentSmpleRate = F_CPU / (BASE_RATE + long(currentThrottle * TOP_SPEED_MULTIPLIER));
    }
}

/* ____   ____ __  __   ____       _
  |  _ \ / ___|  \/  | / ___|  ___| |_ _   _ _ __
  | |_) | |   | |\/| | \___ \ / _ \ __| | | | '_ \
  |  __/| |___| |  | |  ___) |  __/ |_| |_| | |_) |
  |_|    \____|_|  |_| |____/ \___|\__|\__,_| .__/
                                            |_|    */
void startPlayback()
{
  pinMode(SPEAKER, OUTPUT);
  audioRunning = true;

  // Set up Timer 3 to do pulse width modulation on the speaker pin.
  TCCR3B &= ~(_BV(CS32) | _BV(CS31));					// Internal clock, No prescaling
  TCCR3B |= _BV(CS30);									// Internal clock, No prescaling

  TCCR3A |= _BV(WGM31) | _BV(WGM30);                        // Set fast PWM mode  (p.157)
  TCCR3B &= ~_BV(WGM32);

  TCCR3A = (TCCR3A | _BV(COM3B1)) & ~_BV(COM3B0);           // Do non-inverting PWM on pin OC3A
  TCCR3A &= ~(_BV(COM3A1) | _BV(COM3A0));                   // On the Arduino Micro this is pin 5.
  TCCR3B &= ~(_BV(CS32) | _BV(CS31));						// No prescaler (p.158)
  TCCR3B |= _BV(CS30);

  OCR3B = pgm_read_byte(&idle_data[0]);                     // Set initial pulse width to the first sample.

  // Set up Timer 1 to send a sample every interrupt.
  cli();

	TCCR1A = (TCCR1A | _BV(COM1A1) & ~(_BV(COM1A0)));		// Set CTC & Clear OC1A at TOP
	TCCR1A |= _BV(WGM10) | _BV(WGM11);						// Set fast PWM, TOP = OCR1A
	TCCR1B |= _BV(WGM13) | _BV(WGM12);						// Set fast PWM, TOP = OCR1A
	
	TCCR1B &= ~(_BV(CS12) | _BV(CS11));					// Internal clock, No prescaling
	TCCR1B |= _BV(CS10);									// Internal clock, No prescaling

	OCR1A = F_CPU / BASE_RATE;								// Set the compare register (OCR1A).
															// OCR1A is a 16-bit register, so we have to do this with
															// interrupts disabled to be safe.

	TIMSK1 |= _BV(OCIE1A);									// Enable interrupt when TCNT1 == OCR1A (p.136)

  lastSample = pgm_read_byte(&idle_data[idle_len-1]);
  curEngineSample = 0;
  sei();
}


void stopPlayback()
{
  audioRunning = false;

  TIMSK1 &= ~_BV(OCIE1A); // Disable playback per-sample interrupt.
  TCCR1B &= ~_BV(CS10);   // Disable the per-sample timer completely.
  TCCR3B &= ~_BV(CS30);   // Disable the PWM timer.

  digitalWrite(SPEAKER, LOW);
}






/* ___       _                             _
  |_ _|_ __ | |_ ___ _ __ _ __ _   _ _ __ | |_ ___
   | || '_ \| __/ _ \ '__| '__| | | | '_ \| __/ __|
   | || | | | ||  __/ |  | |  | |_| | |_) | |_\__ \
  |___|_| |_|\__\___|_|  |_|   \__,_| .__/ \__|___/
                                    |_|            */

// Uses a pin change interrupt and micros() to get the pulsewidth at pin 2
void getPulsewidth(){
  unsigned long currentMicros = micros();
  boolean currentState = digitalRead(2);

  static unsigned long prevMicros = 0;
  static boolean lastState = LOW;

  if(lastState == LOW && currentState == HIGH){      // Rising edge
    prevMicros = currentMicros;
    lastState = currentState;
  }
  else if(lastState == HIGH && currentState == LOW){ // Falling edge
    pulseWidth = currentMicros - prevMicros;
    lastState = currentState;
  }
}

// This is the main playback interrupt, keep this nice and tight!!
ISR(TIMER1_COMPA_vect) {
  OCR1A = currentSmpleRate;

  if (curEngineSample >= idle_len) { // Loop the sample
    curEngineSample = 0;
  }

  OCR3B = pgm_read_byte(&idle_data[curEngineSample]);

  ++curEngineSample;

}

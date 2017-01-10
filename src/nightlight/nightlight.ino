#include <stdlib.h>
#include <avr/io.h>         
#include <avr/sleep.h>
#include <avr/interrupt.h>

// Routines to set and claer bits (used in the sleep code)
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


//constants
const int solar = 1; //analog 1 solar input pin 0-3v
const int led = 1;  //led panel 
const float alpha = 0.5; //exp average constant
const int thres = 10;    //darkness threshold
const byte brightness = 127; //led brightness
volatile boolean f_wdt = 1;  //watchdog trigger flag

//variables
float solar_level = 2 * thres; //0-1024 --> 0-2.56v

void setup()  { 
  setup_watchdog(9); // approximately 8 second sleep
  analogReference(INTERNAL2V56); //use interval since vcc can range from 0-4.5v
  pinMode(led,OUTPUT);
} 

void loop()  { 
  if (f_wdt==1) {  // wait for timed out watchdog / flag is set when a watchdog timeout occurs
    f_wdt=0;       // reset flag
    solar_level = analogRead(solar) * alpha + solar_level*(1-alpha); //running average from solar panel
    if(solar_level < thres){
      analogWrite(led,brightness); //turn on
      system_sleep(false);  // Send the unit to sleep
    }
    else{
      analogWrite(led,0); //turn off
      system_sleep(true);  // Send the unit to deep sleep
    }
  }
}


// set system into the sleep state 
// system wakes up when wtchdog is timed out
void system_sleep(boolean deep_sleep) {
  cbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter OFF

  // sleep mode is set here
  if(deep_sleep){
     set_sleep_mode(SLEEP_MODE_PWR_DOWN);    //no pwm 
  }
  else{
    set_sleep_mode(SLEEP_MODE_IDLE);     //pwm
  }
  
  sleep_enable();
  sleep_mode();                        // System actually sleeps here
  sleep_disable();                     // System continues execution here when watchdog timed out 
  sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON
  
}

// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) {

  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;

  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCR = bb;
  WDTCR |= _BV(WDIE);
}
  
// Watchdog Interrupt Service / is executed when watchdog timed out
ISR(WDT_vect) {
  f_wdt=1;  // set global flag
}


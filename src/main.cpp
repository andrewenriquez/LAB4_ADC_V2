// Date:
// Assignment:
//
// Description: This file contains a programmatic overall description of the
// program. It should never contain assignments to special function registers
// for the exception key one-line code such as checking the state of the pin.
//
// Requirements:
//----------------------------------------------------------------------//

#include <avr/io.h>
#include <Arduino.h>
#include "PWM.h"
#include "timer.h" 
#include "adc.h"
#include "switch.h"

/*
 * Define a set of states that can be used in the state machine using an enum
 */
typedef enum stateType_enum {
  wait_press, debounce_press, wait_release, debounce_release, count_sec
} stateType;

/* setting state as volatile and initializing it to wait_press.
 * setting count as volatile and initializing it to 0.
 * setting myDelay as volatile and initializing it to 100 milliseconds.
*/
volatile stateType state = count_sec;
volatile unsigned int count = 0;
volatile unsigned int myDelay = 100;


volatile unsigned int result=0;

int main(){



initPWMTimer4();
initTimer1();
initTimer0();
//delayUs(2000);
initADC();
initSwitchPB3();

Serial.begin(9600);

sei();


  while(1){
    //turnON();
    
    switch(state) {

      case count_sec:
        delayMs(1000);
        Serial.println("cs");
        if (((float)result/1024.00 * 5.00 > 2.50) & (count < 5)) {
          count += 1;
          Serial.println(result);
          Serial.println(count);
          delayMs(1000);
        }

        else if (count == 5) {
          count = 0;
          state = wait_press;
        }

        else {
          count = 0;
        }
        break;
      /* wait_press is the default state the arduino goes into. This state will 
       * turn on the leds using the count variable. wait_press will increment count
       * only till count = 16 since we only want the LEDs to display 0 to 16 in 
       * binary. delayMS() is called in each iteration to cause a delay of myDelay 
       * which will either be equal to 100ms or 200ms. wait_press moves onto 
       * debounce_press when the switch is pressed using a Pin Change Interrupt.
      */
      case wait_press:
      Serial.println("wp");
      turnON();
      //delayMs(10);
      PWMChangeFrequency();
      //OCR4A += 100;

      //delayMs(10);
      //delayMs(1000);
      //turnOFF();
      
      //delayMs(10);
      
        break;
      /* 1 millisecond delay is used to ignore any other bouncing caused
       * when pressing the switch. Once the delay is over, the Arduino moves
       * onto wait_release.
      */
      case debounce_press:
      Serial.println("dp");
        delayMs(1);
        state = wait_release;
        break;
      /* wait_release is the state the Arduino moves into when the switch button
       * is pressed. This state will continue the same functionality from wait_press
       * in order to keep the LEDs and count running with a new delay that is changed in 
       * the ISR.
      */
      case wait_release:
      Serial.println("wr");
      //turnOnLEDWithChar(count);
      //count = 0;
      turnOFF();
      delayMs(1);
      state = debounce_release;
      //ADCSRA |= (1 << ADSC);
      //count++;
        break;
      /* debounce_release is given a 1 millisecond delay and then enters the default 
       * wait_press state. 
      */
      case debounce_release:
      Serial.println("dr");
        delayMs(1);
        ADCSRA |= (1 << ADSC);
        count = 0;
        state = count_sec; //1st state
        
        break;
    }

   //delayUs(10000);
   //delayMs(1000);
   //Serial.println("V");
  }

  return 0;
}

/* Implement an Pin Change Interrupt which handles the switch being
* pressed ant released. When the switch is pressed an released, the LEDs
* change at twice the original rate. If the LEDs are already changing at twice
* the original rate, it goes back to the original rate.
*/
ISR(PCINT0_vect) {
  if(state == wait_press){
    state = debounce_press;
  }
  else if(state == wait_release){

    state = debounce_release;
    count = 0;
    
  }
}


ISR(ADC_vect){
  
  result=ADCL;
   result+=((unsigned int)ADCH) << 8;
   //Serial.println((float)result/1024.00 *5.00);
  //turnON();
  ADCSRA |= (1 << ADSC);
}
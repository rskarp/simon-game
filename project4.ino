/* Riley Karp
 *  CS342 - Project4: Simon Game
 *  3 December 2018
 */

#include <avr/io.h>     // for register names
#include "USART.h" // USART (serial bus) communication library

// Infrared communication libraries
#include <IRLibDecodeBase.h>
#include <IRLib_P01_NEC.h>
#include <IRLibRecv.h>

// The Mini IR Remote's button codes
#define BUTTON_RIGHT  0xfd50af 
#define BUTTON_LEFT   0xfd10ef 
#define BUTTON_SELECT 0xfd906f 
#define BUTTON_UP     0xfda05f 
#define BUTTON_DOWN   0xfdb04f
#define BUTTON_0      0xfd30cf
#define BUTTON_1      0xfd08f7
#define BUTTON_2      0xfd8877
#define BUTTON_3      0xfd48b7
#define BUTTON_4      0xfd28d7
#define BUTTON_5      0xfda857
#define BUTTON_6      0xfd6897
#define BUTTON_7      0xfd18e7
#define BUTTON_8      0xfd9867
#define BUTTON_9      0xfd58a7

IRdecodeNEC nec;          // an NEC decoder object
IRrecv rx( 2 );           // the TSOP38238 IR receiver is connected to pin 2
uint32_t valuePrevious;   // handles NEC repeat codes

/* Global variables for game */
const int maxLength = 10; // maximum length of the pattern
int pattern[maxLength]; // holds the randomly generated pattern of LEDs to be displayed
int idx = 0; // current index in the pattern array
int numItems = 0; // current number of items in the pattern
int ms = 0; // for note duration
bool on = false; // boolean to tell if any lights are on
int level = 1; // level corresponding to # of new notes to add to the end of the pattern
bool repeat = false; // boolean to tell if a button was repeated by being held down 
bool won = false; // boolean set to true if player wins
bool lost = false; // boolean set to true if the player loses
int colorCount = 0; // used to determine pattern of lights to display when the game ends

/* Arrays to hold LED locations in pad D and corresponding note frequencies for each color:
 * Index 0: red, 1: blue, 2: yellow, 3: green */
int location[4] = { 0x40, 0x20, 0x10, 0x08 };
float freq[4] = { 880.00, 659.25, 554.37, 329.63 };
int color = -1; // integer corresponding to the color of the LED that will light up when a button is pressed. initialized to -1 because no button is pressed

/* Global variables use for ADC reading */
int nSamples = 0; // number of ADC measurements taken
int N = 10; // sliding window width
float omean = 0.0; // online mean ADC value
bool complete = false; // tells the main loop if ADC reading is complete
uint16_t sensedADC; // most recent ADC measurement

/* Restart Game: reinitialize global variables. Create new pattern & display the pattern */
void restart() {
  idx = 0;
  numItems = 0;
  won = false;
  lost = false;
  for( int i = 0; i<level; i++ ) { // number of colors to add is based on the level
    newColor();
  }
  showPattern();
}

/* Generate random number to add to pattern of LEDs */
void newColor() {
  pattern[numItems] = rand()%4; // random int between 0 and 3
  numItems += 1;
}

/* Display a single LED & play the corresponding note */
void showColor(int color) {
    ms = 0;
    on = true;
    PIND &= 0b10000111; // turn off all LEDs
    PIND |= location[color]; // turn on the correct LED
    // set correct sound to output through piezo
    int counts = ( (16*pow(10,6))/(2*freq[color]) ) - 1;
    OCR1AH = ( counts & 0xFF00 ) >> 8; // set high byte
    OCR1AL = ( counts & 0x00FF ) ; // set low byte
}

/* Display the whole pattern of LEDs and sounds */
void showPattern() {
  char t[16];
  sprintf(t, "New Pattern length: %d\n", numItems);
  printString(t);
  for( int i = 0; i<numItems; i++ ) {
    showColor( pattern[i] );
    sprintf(t, "Show Item: %d\n", i+1);
    printString(t);
    while(TCCR1B & 0x01){;}
  }
  printString("Done\n");
}

/* Read ADC value every 1ms when enabled */
ISR( ADC_vect ) {
  nSamples++;
  sensedADC = ADCL; // must read low byte first
  sensedADC |= (ADCH & 0x03) << 8; // 10-bit precision
  omean = ( omean * float(N-1)/float(N) ) + ( float(sensedADC)/float(N) ); // online mean ADC value
  
  if( nSamples >= N ) {
    ADCSRA &= 0x77;   // disable ADC
    complete = true; // tell the main loop that ADC reading is done
    nSamples = 0;
  }
}

/* Interrupt triggered every 1ms. Increment counter that controls note duration.
turn off sound and lights after 500ms. */
ISR( TIMER0_COMPA_vect ) {
  ms++;
  if( on ) {
    TCCR1B |= 0b00000001; // set TC1's prescale to 1, turns on sound
  }
  if( (ms >= 500) & (!lost) ) {
    on = false;
    TCCR1B &= 0b11111000; // set TC1's prescale to 0, turns off sound
    PORTD &= 0b10000111; // turn off all LEDs
  }
}

/* Runs the Simon Game */
int main() {

  /* Configure GPIO:
    - PD2: input, IR sensor
    - PD3: output, green LED
    - PD4: output, yellow LED
    - PD5: output, blue LED
    - PD6: output, red LED
    - PB1: output, piezo
    - PC0: input, potentiometer */
   DDRD = 0b01111000;   //set PD[6:3] to outputs for LEDs
   DDRB = 0x02; // set PB1 to output for piezo
  
  /* Configure Timer/Counter 0 :  1 ms interrupts to control note duration */
  TCCR0A = 0b00000010; // set mode to CTC
  TCCR0B = 0b00000011; // prescale by 64
  OCR0A = 0b11111001; // count up to 250
  TIMSK0 = 0b00000010; // enable output compare match A

  /* Configre TC1 for piezo */
  TCCR1A = 0b01000000; // toggle on compare match
  TCCR1B = 0b00001000; // set timer to CTC mode, no prescale

  /* ADC Configuration :  collect potentiometer readings on pad A0 */
  ADMUX  = 0x40;  // right-adjusted ADC on PC[0] w/ reference to 5V (Vcc)
  ADCSRA = 0x77;  // ADC not yet enabled, but set up for automatic conversions and prescale = 128
  ADCSRB = 0x03;  // when enabled, ADC conversions will be auto-triggered by TIMER0_COMPA events
  DIDR0  = 0x3F;  // disable digital input on PC[5:0] to save power

  /* Global interrupt enable */
  SREG |= 0x80;
  
  initUSART();

  rx.enableIRIn();  // start the receiver

  // start game by generating and displaying the first note(s) of the pattern
  restart();
  
  while( true ){  
    repeat = false; // reinitialize repeated button boolean to false
    color = -1; // reinitialize color to -1 (not a real color value)
    if( rx.getResults() ) {             // wait for a button press
       printByte( nec.decode() );    // decode the pulse train 
       printString( "\t0x" );
       printHexByte( (nec.value & 0xFF000000) >> 24 );
       printHexByte( (nec.value & 0x00FF0000) >> 16 );
       printHexByte( (nec.value & 0x0000FF00) >> 8 );
       printHexByte(  nec.value & 0x000000FF );
       printString( "\t" );
       if( nec.value == 0xFFFFFFFF )    // check to see if it's a repeat code
       { 
          nec.value = valuePrevious;    // if it's a repeat code, keep the previous value
          repeat = true; // tell the main loop that the button is being held down, so don't take any action
       }
       switch( nec.value ) { // respond to the button press: choose a behavior based on the value!
          case BUTTON_LEFT:
            printString("Button L\n");
            break;
            
          case BUTTON_RIGHT:
            printString("Button R\n");
            break;
            
          case BUTTON_SELECT: // Turn on ADC reading to change level
            ADCSRA |= 0x88; // enable ADC measurements & the ADC's ISR
            printString("Button S: Change Level\n");
            break;
            
          case BUTTON_UP:
            printString("Button U\n");
            break;
            
          case BUTTON_DOWN:
            printString("Button D\n");
            break;
            
          case BUTTON_0: // restart game
            restart();
            printString("Button 0: RESTART\n");
            break;
            
          case BUTTON_1: // green LED, E4
            showColor(3);
            color = 3;
            printString("Button 1: GREEN\n"); 
            break;
            
          case BUTTON_2: // red LED A5
            showColor(0);
            color = 0;
            printString("Button 2: RED\n"); 
            break;
            
          case BUTTON_3:
            printString("Button 3\n");
            break;
            
          case BUTTON_4: // yellow LED, C#5
            showColor(2);
            color = 2;
            printString("Button 4: YELLOW\n");
            break;
            
          case BUTTON_5: // blue LED, E5
            showColor(1);
            color = 1;
            printString("Button 5: BLUE\n"); 
            break;
            
          case BUTTON_6:
            printString("Button 6\n"); 
            break;
            
          case BUTTON_7:
            printString("Button 7\n"); 
            break;
            
          case BUTTON_8:
            printString("Button 8\n"); 
            break;
            
          case BUTTON_9:
            printString("Button 9\n"); 
            break;
            
          default: 
            printString("Button ?\n"); 
            break;
       }

       // Change level if ADC reading is done
       if(complete) {
         float angle = (-0.27)*omean + 219; // equation based on sensor characterization
         level = int( angle*5/180 ); // set level to one of 5 possible levels based on angle
         if(level < 1) { // bounds checking
            level = 1;
         }
         else if(level > 5) { // bounds checking
            level = 5;
         }
         char t[16];
         sprintf(t,"New level: %d of 5\n", level );
         printString(t); // print level
         complete = false; // reinitialize complete boolean to false
       }

       // Handle a color being pressed during gameplay
       if( (color > -1) & (!repeat) & (!won) & (!lost) ) {
         if( pattern[idx] != color ) { // incorrect button was pressed
            lost = true; // set lost to true
            printString("Oh no, wrong color! You lost :( Press button 0 to replay!\n");
         }
         else if ( pattern[idx] == color ) { // correct button was pressed, but the end of the pattern hasn't been reached
            idx += 1; // go to next color in pattern
         }
         if( idx >= (numItems) ) { // the end of the pattern was reached. (all items were correct)
            if( numItems >= maxLength ) { //pattern array is full
              won = true;
              printString("Congratulations! You Won! :) Press button 0 to replay!\n");
            }
            else { // pattern array is not full yet
              // add new items
              for( int i = 0; i < level; i++ ) {
                if( numItems < maxLength ) {
                  newColor();
                }
             }
             idx = 0; // reinitialize tracking index to start at the beginning
             printString("Correct!\n");
             showPattern(); // display new pattern
           }
         }
       }
       valuePrevious = nec.value;
       rx.enableIRIn();
    }

    // Display losing pattern
    if( lost & ms%500==0) { // alternate all lights & sound on and off every 500ms
      on = false; // gameplay is off. pattern plays until restart (button 0) is pressed
      if( (colorCount%2) == 1 ) { // turn off all LEDs and sounds every other half second
        PORTD &= 0b10000111; // turn off all LEDs
        TCCR1B &= 0b11111000; // set TC1's prescale to 0, turns off sound
        printString(""); // need to print empty string to give the timer enough time to change its prescale before continuing
      }
      else { // turn on all LEDs & sounds
        PORTD |= 0b01111000; // turn on all LEDs
        TCCR1B |= 0b00000001; // set TC1's prescale to 1, turns on sound
        printString(""); // need to print empty string to give the timer enough time to change its prescale before continuing
      }
      colorCount++; // increment counter
    }

    // Display winning pattern
    if( won & (ms%500==0) ) { // alternate color of LED & corresponding sound every 500ms to play a song
      showColor(colorCount%4); // display one color
      colorCount++; // increment counter
    }
  }
  return 0;
}

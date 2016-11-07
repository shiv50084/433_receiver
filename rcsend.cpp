/*

 'codesend' hacked from 'send' by @justy
 
 - The provided rc_switch 'send' command uses the form systemCode, unitCode, command
   which is not suitable for our purposes.  Instead, we call 
   send(code, length); // where length is always 24 and code is simply the code
   we find using the RF_sniffer.ino Arduino sketch.

 Usage: ./codesend decimalcode
 (Use RF_Sniffer.ino to check that RF signals are being produced by the RPi's transmitter)
 */

#include <stdlib.h>
#include <stdio.h>
#include <wiringPi.h>
#include <stdint.h>

#define NULL 0
#define CHANGE 1


#define HIGH_LEVEL HIGH
#define LOW_LEVEL LOW

#define BUTTON_PIN1  5
#define BUTTON_PIN2  8
#define RF_DATA_PIN 1

#define SHORT_DELAY 373
#define LONG_DELAY  (3*SHORT_DELAY)
#define TOTAL_DELAY (SHORT_DELAY + LONG_DELAY)
#define SYNC_DELAY  (6*SHORT_DELAY)
#define EXTRALONG_DELAY (3*LONG_DELAY)     

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)

unsigned long signal;
unsigned long signalA1 = 0b110101100100001101110000;
unsigned long signalA0 = 0b110111001010000100010000;

unsigned long signalB1 = 0b110101100100001101110100;
unsigned long signalB0 = 0b110111001010000100010100;
unsigned long signalC1 = 0b110101100100001101111100;
unsigned long signalC0 = 0b110111001010000100011100;
unsigned long signalD1 = 0b110111001010000100010010;
unsigned long signalD0 = 0b110101100100001101110010;
unsigned long signalM1 = 0b110111001010000100011010;
unsigned long signalM0 = 0b110101100100001101111010;


/* CO/TECH remote */
unsigned long signalRA1 = 0b101000010111010010101100;
unsigned long signalRA0 = 0b101010101111011110111100;

unsigned long signalRB1 = 0b101001110101001011110101;
unsigned long signalRB0 = 0b101001000100010111010101;

unsigned long signalRC1 = 0b101011000000111110001110;
unsigned long signalRC0 = 0b101010111001110101101110;

unsigned long signalRD1 = 0b101001000100010111010111;
unsigned long signalRD0 = 0b101001110101001011110111;


unsigned long signals[] = {
	// A1
	0b101000010111010010101100,
        // A0
	0b101010101111011110111100,
	// unsigned long signalRB1
	0b101001110101001011110101,
	// unsigned long signalRB0
	0b101001000100010111010101,
	// unsigned long signalRC1
	0b101011000000111110001110,
	// unsigned long signalRC0
	0b101010111001110101101110,

	// unsigned long signalRD1
	0b101001000100010111010111,
	// unsigned long signalRD0
	0b101001110101001011110111};




void send_short_seq()
{
    //send the short 0 signal in the 1st sequence and the pause between repeats
    unsigned long d = (SHORT_DELAY);
    //digitalWrite(13, HIGH_LEVEL);
    digitalWrite(RF_DATA_PIN, HIGH_LEVEL);
    delayMicroseconds(d);
    digitalWrite(RF_DATA_PIN, LOW_LEVEL);
    //digitalWrite(13, LOW_LEVEL);   
    delayMicroseconds(SYNC_DELAY); 
}

int send_plug(int on, int sw)
{
    signal = signals[sw * 2 + !on];
    /* Send short seq first */
    send_short_seq();
    for (unsigned char j=0; j<1; j++) {
    for (unsigned char i=0; i<5; i++) { // repeat 1st signal sequence 3 times
      for (unsigned char k=0; k<24; k++) { //as 24 long and short signals, this loop sends each one and if it is long, it takes it away from total delay so that there is a short between it and the next signal and viceversa
        unsigned long d = ((bitRead(signal, 23-k)) == 1 ? LONG_DELAY : SHORT_DELAY);
        //digitalWrite(13, HIGH_LEVEL);
        digitalWrite(RF_DATA_PIN, HIGH_LEVEL);
        delayMicroseconds(d);
        digitalWrite(RF_DATA_PIN, LOW_LEVEL);
        //digitalWrite(13, LOW_LEVEL);   
        delayMicroseconds(TOTAL_DELAY - d);
        }
      send_short_seq();
#if 0
      //send the short 0 signal in the 1st sequence and the pause between repeats
      unsigned long d = (SHORT_DELAY);
      //digitalWrite(13, HIGH_LEVEL);
      digitalWrite(RF_DATA_PIN, HIGH_LEVEL);
      delayMicroseconds(d);
      digitalWrite(RF_DATA_PIN, LOW_LEVEL);
      //digitalWrite(13, LOW_LEVEL);   
      delayMicroseconds(SYNC_DELAY);
#endif
    }
    for (unsigned char i=0; i<5; i++) { // repeat 2nd signal sequence 3 times
      for (unsigned char k=0; k<24; k++) {
        unsigned long d = ((bitRead(signal, 23-k)) == 1 ? LONG_DELAY : SHORT_DELAY);
        //digitalWrite(13, HIGH_LEVEL);
        digitalWrite(RF_DATA_PIN, HIGH_LEVEL);
        delayMicroseconds(d);
        digitalWrite(RF_DATA_PIN, LOW_LEVEL);
        //digitalWrite(13, LOW_LEVEL);   
        delayMicroseconds(TOTAL_DELAY - d);
        }
      // send the extra long 1 signal in the 2nd sequence and long pause between repeats
      unsigned long d = EXTRALONG_DELAY;
      //digitalWrite(13, HIGH_LEVEL);
      digitalWrite(RF_DATA_PIN, HIGH_LEVEL);
      delayMicroseconds(EXTRALONG_DELAY);
      digitalWrite(RF_DATA_PIN, LOW_LEVEL);
      //digitalWrite(13, LOW_LEVEL);   
      delayMicroseconds(EXTRALONG_DELAY);
   
      delayMicroseconds(SYNC_DELAY);
    }
    digitalWrite(RF_DATA_PIN, HIGH_LEVEL);
    delayMicroseconds(SYNC_DELAY);
    }
    digitalWrite(RF_DATA_PIN, HIGH_LEVEL);
}

int main(int argc, char *argv[])
{
    
	// Parse the firt parameter to this command as an integer
	int code = 1, sw = 0;
	
	if (argc > 1)
		code  = atoi(argv[1]);
	if (argc > 2)
		sw = atoi(argv[2]);
    
    	if (wiringPiSetup () == -1) return 1;
	pinMode(RF_DATA_PIN, OUTPUT);
	printf("sending code %d to %d\n", code, sw);
	send_plug(code, sw);
	pinMode(RF_DATA_PIN, INPUT);
	return 0;
}

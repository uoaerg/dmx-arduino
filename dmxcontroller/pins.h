/*-
 * Copyright (c) 2017 Tom Jones, UoA tom@erg.abdn.ac.uk
 * Copyright (c) 2023 Gorry Fairhurst, UoA gorry@erg.abdn.ac.uk
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/* Rewrite completed by Aug 2023 */


/* NOTE USE INCLUDED LIBRARYIES ONLY,
A SERIAL CONFLICT exists in the on-line library,
https://github.com/mathertel/DMXSerial/tree/master
The Arduino MEGA 2560 boards use the serial port 0 on pins 0 an 1.
The vector numbers differ because you use the Mega board and not the UNO.
On Mega you have the option to use the port 1 for DMX and the Serial for printing to the Monitor.
see https://github.com/mathertel/DMXSerial/blob/master/src/DMXSerial_avr.h 
for details to enable DMX_USE_PORT1 and change your wiring.
The hack here is to reload DMX after each pixel load.
Aug 2023 GF */

#include <DMXSerial.h>
#include <Servo.h>

#include <Adafruit_NeoPixel.h>

extern "C" {
	#include "dmxboard.h"
	#include "pins.h"
	#include "dip.h"
}

#define DMXADDRESSMAX 513 	/* maximum number of channels read in a frame */

#define DMXTIMEOUT 1000		/* One Second timeout to take receiver off-line */

void controllermain(void);
void digitalmain(void);
void pwmmain(void);
void servomain(void);
void neopixelmain(void);
void steppermain(void);
void defaultprogram(void);
void neopixellong(void);
void printdipmode(void);

void readDMXChannels(int *, uint16_t );
void readStartDMXChannels(int *, uint16_t, uint16_t startaddr );

void stepmotor(int *motorpins, int *motorsteps, int direction, int step);
void stopmotor(int *motorpins);

int OutputDisabled;		/* Disable output signal */
uint8_t readRed(uint8_t);
uint8_t readGreen(uint8_t);
uint8_t readBlue(uint8_t);

void (*program)(void);
Servo servos[SERVOPINS_MAX];
int16_t indexposition = 0;

/* Driver for the WS2812 3-colour LED strip */
/* each pixel is programmed in a chain with 3x 8-bit colour values */
Adafruit_NeoPixel strip = Adafruit_NeoPixel
(
	LONGLEDCOUNT,		/* Maximum number of LEDs on the strip - use the longest size */
	STRIP_PIN,		/* Arduino output pin used for control */
	NEO_GRB + NEO_KHZ800 	/* Colour Byte order and Strip Speed */
);

/* This is the menu for the switch configuration */
#define NUMPROGRAMS 8
void (*programs[NUMPROGRAMS])(void) = {
	controllermain, 	/* Mode 0 */
	digitalmain,	/* Mode 1 */
	pwmmain,	/* Mode 2 */
	servomain,	/* Mode 3 */
	neopixelmain,	/* Mode 4 */
	steppermain,	/* Mode 5 */
	defaultprogram,	/* Mode 6 */
	neopixellong,	/* Mode 7 */
};

/* This is the menu as a list of programs */
/* Note lsb and msb versions of the mode are provided in the comments ! */
char *program_name[]= {
	"DMX Controller  {Prog 0}", /* Mode 0  000 msb: 000 */
	"Digital Control {Prog 4}",	/* Mode 1  001 msb: 100 */
	"PWM Control     {Prog 2}",	/* Mode 2  010 msb: 010*/
	"Servo Control   {Prog 6}",	/* Mode 3  011 msb: 110 */
	"Pixel Control   {Prog 1}",	/* Mode 4  100 msb: 001 */
	"Stepper Control {Prog 5}",	/* Mode 5  101 msb: 101*/
	" --- None ---   {Prog 3}",	/* Mode 6  110 msb: 011 */
	"Long Pixel Cont.{Prog 7}",	/* Mode 7  111 msb: 111 */
};

/* -------------------------------------------------------------------------- */

void 
setup()
{

	/* Configure pins */
	pinMode(GREEN_LED, OUTPUT);
	pinMode(YELLOW_LED, OUTPUT);

	/* RED status LED on board
	 * RED On indicates no DMX data; RED flashing indicates good DMX frames received. */
	pinMode(RED_LED, OUTPUT);
	pinMode(STRIP_PIN, OUTPUT);
	pinMode(POT_PIN, INPUT);

	/* DIP switch input of address and mode */
 	for(int i = 0; i < DIPMAX; i++) {
		pinMode(dipSwitches[i], INPUT);
		digitalWrite(dipSwitches[i], HIGH);
	}

	/* Line driver has two halves, pins are pulled high with a LOW */
	digitalWrite(DEENABLE1, LOW);
	digitalWrite(DEENABLE2, LOW);

	for(int i = 0;i < DEPINS_MAX; i++) {
		pinMode(depins[i],OUTPUT);
		digitalWrite(depins[i], HIGH);
	}
	for(int i = 0;i < PWMPINS_MAX; i++) {
		pinMode(pwmpins[i], OUTPUT);
	}
	for(int i = 0;i < SERVOPINS_MAX; i++) {
		pinMode(servopins[i], OUTPUT);
	}
	for(int i = 0;i < STEPPERPINS_MAX; i++) {
		pinMode(stepperpins[i], OUTPUT);
	}

	Serial.begin(9600);

	Serial.println("---------------------------------------------------------");
	Serial.println("Build name:  " GIT);
	Serial.println("Board rev :  " DMXBOARDREV);
	
	Serial.print("Servo pin: " );
  Serial.print(SERVOPIN);
  Serial.print(", Pot pin: ");
	Serial.print( POT_PIN);
	Serial.print(", PWM pin: " );
  Serial.println( PWM1PIN);

#ifdef NEOPIXELDISPLAY
	/* Test routine for NEOPIXEL strip when enabled */
  Serial.print(LONGLEDCOUNT);
	Serial.print(" NeoPixels using pin: ");
	Serial.print(STRIP_PIN);

	strip.begin();	/* Initialize the WS2812 LED strip */
	strip.show(); 	/* Set all pixels in strip to 'off' */

	// Panel test executes for address > 502
	if (dipReadAddress() > 502) {
		Serial.print(" Running Neopixel test for ");
    Serial.print(LONGLEDCOUNT);
    Serial.println (" pixels.");
		strip.setPixelColor(0, strip.Color(255,255,255));
		for(int i = 1; i < LONGLEDCOUNT; i++) {
			strip.setPixelColor(i,
				strip.Color(
					random(5,255),
					random(5,255),
					random(5,255)
				)
			);
			strip.show();
			delay(100);
    }
  } else {
    Serial.println(" (No test - enabled when address > 502 )");
  }

  // Anyway clear the strip (there might be previosu garbage displayed)
	for(int i = 0; i < LONGLEDCOUNT; i++)
		strip.setPixelColor(i, strip.Color(0, 0, 0) );
	strip.show();
  
#endif //NEOPIXELDISPLAY
	Serial.println("---------------------------------------------------------");

	DMXSerial.init(DMXReceiver);	/* initialise the DMX receive code */

	strip.begin();	/* Initialize the WS2812 LED strip */
	strip.show(); 	/* Set all pixels in strip to 'off' */
}

void 
loop()
{
	static int led = 0;              /* for red LED */
	static int lastflash = millis(); /* for red LED */

	dmxAddress = dipReadAddress();
	dmxMode = dipReadMode();

	if(dmxMode >= 0 && dmxMode < NUMPROGRAMS) {
		void (*newprogram)(void) = programs[dmxMode];

		if(newprogram != program) {		/* If the program changed */

      printdipmode();

			digitalWrite(GREEN_LED, HIGH);	/* Green board status LED */
			digitalWrite(YELLOW_LED, LOW);	/* Yellow board status LED */

			/* Pull the line drivers low, which is logical HIGH */	
			for(int i = 0;i < DEPINS_MAX; i++)
				digitalWrite(depins[i], HIGH);

			/* Detach any attached servos when changing from servomain */
			if(program == servomain && newprogram != servomain) 
				for(int i = 0;i < SERVOPINS_MAX; i++) 
					if(servos[i].attached())
						servos[i].detach();

			/* If we have started the control program, enable DMX control */
			/* This sets the line driver to transmit and starts sending */
			if(newprogram == controllermain) {
				DMXSerial.init(DMXController);
				/* 0 each slot to make DMXSerial always send a full frame */
				for (int i = 0; i < 512; i++)
					DMXSerial.write(i+1, 0x00);
			} else if(program == controllermain) {
				DMXSerial.init(DMXReceiver);
			}

			program = newprogram;

			/* Add some delay when we change programs */
			delay(200);
			digitalWrite(GREEN_LED, LOW);
		}
	}

	/* Call main function */
	if(program != NULL)
		program();

	/* Clear the red status LED if in controller mode
   * otherwise the blink the red status LED when there is DMX data within our timeout.
 	 * If no data, set the status red LED high, this is how commercial hardware works.
 	 */
  if(program == controllermain) {
    digitalWrite(RED_LED, LOW);
  } else {
    if(DMXSerial.noDataSince() < DMXTIMEOUT) {
      // Serial.println(DMXSerial.noDataSince()); // Enbale to trace idle time
      OutputDisabled = false;
      digitalWrite(RED_LED, led);
      int now = millis();
      if ( now - lastflash > 200) {
        led = !led;
        lastflash = now;
      }
    } else {
      /* Outputs ought to be disabled here */
      /* Currently no action taken */
      digitalWrite(RED_LED, HIGH);
      OutputDisabled = true;
    }
  }
}

/* Read and print DMX address and mode from the DIP switches */
void printdipmode()
{
  int address;
  int mode;

  address = dipReadAddress();
  Serial.print("DMX Address: ");
		Serial.print(address, DEC);
    Serial.print(" (b");
  	Serial.print(address, BIN);

  Serial.print(") ");

  mode = dipReadMode();
  Serial.print("Board mode: ");
 	Serial.print(mode, DEC);
	Serial.print(" (b");
  Serial.print(mode, BIN);
  Serial.print(") ");
  Serial.println( program_name[mode] );
}

/* ************************************************* */
/* Program to execute when no valid mode is selected :
 * The green LED flashes */
void
defaultprogram()
{
	static int led = 0;
	static unsigned long lasttoggle = micros();
	unsigned long now = millis();

	if (now - lasttoggle > 200) {
		led = !led;
		lasttoggle = now;
	}
	digitalWrite(GREEN_LED, led); 
}

/* ************************************************* */
/* Program to execute in controller mode             */
/* Outputs low order DIP switches output to D9 socket*/
/* Control waveform sent on DMX output               */
/* 1. frame is static if low order DIP switch is 0   */
/* 2. frame sets slot for switch with value from pot */
/* Yellow LED indicates if pot > 128                 */

void 
controllermain()
{
	uint16_t value = 1;
	uint8_t reading = 0;

	static int lastAddress;

  /*  RED LED is set off in main, because no DMX Input in this mode */

  /* If the address has changed, then clear slots at the last address */
	if(dmxAddress != lastAddress) {
		if(lastAddress == 0x00) {
      /* Overwrite the static frame contents when address was previously zero */
			DMXSerial.write(1, 0x00);
			DMXSerial.write(2, 0x00);
			DMXSerial.write(3, 0x00);
			DMXSerial.write(4, 0x00);
		} else {
      /* Reset last slot at last addresss to zero */
			DMXSerial.write(lastAddress, 0x00);
		}
	}
	/* Read the pot value and set the yellow status LED only when > 128 */
	reading = (uint16_t)analogRead(POT_PIN) / 4; 
	if(reading > 127)
		digitalWrite(YELLOW_LED, HIGH);
	else
		digitalWrite(YELLOW_LED, LOW);

  /* Update the requires lots with new data  */
	/* If address is zero, send a recognisable pattern of 4 slots & set green LED off */
  /* Else if not zero, set the slot value at the slected address & set green LED on */
	if(dmxAddress == 0) {
		DMXSerial.write(1, 0x55); /* Slot 1 */
		DMXSerial.write(2, 0xFF); /* Slot 2 */
		DMXSerial.write(3, 0x0F); /* Slot 3 */
		DMXSerial.write(4, 0xF0); /* Slot 4 */
    digitalWrite(GREEN_LED, LOW);
	} else {
		DMXSerial.write(dmxAddress, reading);
    digitalWrite(GREEN_LED, HIGH);
	}
	lastAddress = dmxAddress;

  /* For debugging set the output port */
	/* Output a copy of the DIP switch pattern on the output ports */
	for(int i = 0;i < DEPINS_MAX; i++) {
		int status = !(dmxAddress & value);
		digitalWrite(depins[i], status);	
		value = value << 1;
	}

	/* For debugging set the green LED from the last bit of address */
	// value = digitalRead(DIP9) << 8;
	// if(value > 0)
	//	digitalWrite(GREEN_LED, HIGH);	
	//else
	//	digitalWrite(GREEN_LED, LOW);	
}

/* ************************************************* */
/* Program to execute when digital mode selected     */
/* Reads 8 channels of the DMX frame                 */
/* outputs a logic 1 on main port when channel > 128 */
/* Yellow LED indicates value of channel 1           */
void 
digitalmain()
{
	int values[DEPINS_MAX];
	readDMXChannels(values,DEPINS_MAX);

	if(values[0] > 0)
		digitalWrite(YELLOW_LED, HIGH);	
	else
		digitalWrite(YELLOW_LED, LOW);	

	for(int i = 0;i < DEPINS_MAX; i++)
		if(values[i] > 128)
			digitalWrite(depins[i], DEHIGH);
		else
			digitalWrite(depins[i], DELOW);
}

/* ************************************************* */
/* Program to execute when PWM mode selected         */
/* Reads PWMPINS_MAX channels of the DMX frame       */
/* outputs PWM for channel value on analogue output  */
/* Yellow LED indicates if slot 1 > 127              */
/* Yellow LED indicates if slot 2 > 127              */

void 
pwmmain()
{
	int values[PWMPINS_MAX];

	readDMXChannels(values,PWMPINS_MAX);

	if(values[0] > 127)
		digitalWrite(YELLOW_LED, HIGH);	
	else
		digitalWrite(YELLOW_LED, LOW);

  if(values[1] > 127)
		digitalWrite(GREEN_LED, HIGH);	
	else
		digitalWrite(GREEN_LED, LOW);	
	
	for(int i = 0;i < PWMPINS_MAX; i++)
		analogWrite(pwmpins[i], values[i]);
}

/* ************************************************* */
/* Program to execute when servo mode selected       */
void 
servomain()
{
	int values[SERVOPINS_MAX];

	for(int i = 0;i < SERVOPINS_MAX; i++)
		if(!servos[i].attached())
			servos[i].attach(servopins[i]);

	readDMXChannels(values,SERVOPINS_MAX);

	if(values[0] > 0)
		digitalWrite(YELLOW_LED, HIGH);	
	else
		digitalWrite(YELLOW_LED, LOW);	

	for(int i = 0; i < SERVOPINS_MAX; i++) {
		 /* Convert dmx value to degrees */
		int value = values[i]/255.0f * 180;	
		servos[i].write(value);
	}
}

/* ************************************************* */
/* Program to execute when WS2812 mode selected       */
/* outputs TDM waveform for each channel value onto a*/
/* serial WS2812 port, one channel per LED           */
/* channel value is mapped via a colour lookup table */
/* to three 8-bit colour values for each pixel       */
/* Yellow LED indicates if channel 1 > 128           */
void
neopixelmain()
{
	int values[LEDCOUNT];

	// GF: Awkwardly, the Neopixel library doesn't coexist with the DMX ISR
	// A solution calls DMXSerial.init(DMXReceiver) here each time!
	DMXSerial.init(DMXReceiver);

	// We might need a pause here .. for the initialisation and to grab a DMX frame //
	delay(75); // 100 ms ... depends on refersh rate

	readDMXChannels(values,LEDCOUNT);	
	if(values[0] > 127)
		digitalWrite(YELLOW_LED, HIGH);	
	else
		digitalWrite(YELLOW_LED, LOW);	

	for(int i = 0;i < LEDCOUNT; i++) {
		strip.setPixelColor(i,
			strip.Color(
				readRed(values[i]),
				readGreen(values[i]),
				readBlue(values[i])
			)
		);
	}
	strip.show();
}

/* ************************************************* */
/* neopixellong                                      */
/* Varient  to execute when WS2812 mode selected     */
/* outputs TDM waveform for each channel value onto a*/
/* serial WS2812 port, one channel per LED           */
/* channel value is mapped via a colour lookup table */
/* to three 8-bit colour values for each pixel       */
/* Yellow LED indicates if channel 1 > 128           */
void
neopixellong()
{
	int values[LONGLEDCOUNT];
	// GF: Awkwardly, the Neopixel library doesn't coexist with the DMX ISR
	// A solution calls DMXSerial.init(DMXReceiver) here each time!
	DMXSerial.init(DMXReceiver);
	// We might need a pause here .. for the initialisation and to grab a DMX frame //
	delay(75); // 100 ms ... depends on refersh rate

	readDMXChannels(values,LONGLEDCOUNT);	
	if(values[0] > 127)
		digitalWrite(YELLOW_LED, HIGH);	
	else
		digitalWrite(YELLOW_LED, LOW);	
 
	for(int i = 0;i < LONGLEDCOUNT; i++) {
		strip.setPixelColor(i,
			strip.Color(
				readRed(values[i]),
				readGreen(values[i]),
				readBlue(values[i])
			)
		);
	}
	strip.show();
}

/* ************************************************* *
 * steppermain                                       *
 * Main stepper motor function                       *
 *   The red status LED monitors DMX (flashes)       *
 *   The yellow status LED set when rotation enabled *
 *   The green status LED set when indexing is used  *
 *                                                   *
 * The stepper motor is controlled from 4 slots      *
 * Slot 1: Rotation speed, this rotates when > 0     *
 * Slot 2: Direction: forward <=128, reverse > 128   *
 * Slot 3: Index mode disabled = 0                   *
 *         Indexing enabled = 50-200                 *
 *         Set index reference = 255                 *
 * Slot 4: Set the index angle (0 to 360 degrees)    */


void
steppermain()
{
	uint16_t stepdelay = 25600;
	int clockwise = 1;

	int values[STEPPERCHANNELS];	/* Read all slots needed */
	readDMXChannels(values, STEPPERCHANNELS);

	int16_t stepperindex = 0; // Target stepper position 

  /* Setup index referencing, or disable indexed use if requested. */
  /* Note: This happens even if the motor is not moving
  /* A slot value of 255 records a new index reference position */
  /* See if the indexing is being used (green LED on for indexed) */
	if (values[STEPPER_INDEX_MODE] < 50) {
    // Not in indexed mode
    digitalWrite(GREEN_LED, LOW);
  }
  else {
   // indexed mode
    digitalWrite(GREEN_LED, HIGH);
    if (values[STEPPER_INDEX_MODE] == 255) {
			// We are  at the reference position
      indexposition = values[STEPPER_INDEX_ROTATION] << 1; 
		}
  }

  /* If the motor is not allowed to rotate (yellow LED off) */
  if (values[STEPPER_ROTATION_SPEED] == 0 || values[STEPPER_ROTATION_DIRECTION] == 0) {
		stopmotor(stepperpins);
    digitalWrite(YELLOW_LED, LOW);
		return;
	}
  
  // Rotation was enabled, continue processing the stepper motor action ...
  /* Set yellow status LED on when rotation is enabled */
	digitalWrite(YELLOW_LED, HIGH);
  
  /* First see is indexing mode is enabled? */
	if (values[STEPPER_INDEX_MODE] < 50 ) {
    /* This stepper motor is not using indexing */
    /* Now rotate the stepper motor at the required speed */

		clockwise = values[STEPPER_ROTATION_DIRECTION] > 131 ? 1 : 0;
		stepdelay = ((255 - values[STEPPER_ROTATION_SPEED]) + 1)  * 128;

		/* TODO: missing direction-based step speed shift */
		stepmotor(stepperpins, motorsteps, clockwise, stepdelay);
	} else {
    /* Index mode is being used for this stepper motor */
    /* So, check if index movement is enabled, there are null values
     * 200...254 before the value that triggers indexing */
		if (values[STEPPER_INDEX_MODE] >= 200
				&& values[STEPPER_INDEX_MODE] <= 254)
			return; //paused

    /* Now get the relative index to the reference, and see if we need to move */
		stepperindex = values[STEPPER_INDEX_ROTATION] << 1;

    /* Check if motor is already at the indexed position */
		if (indexposition == stepperindex) {
			/*
			 * Leave the motor free when we reach the index. If there is a load
			 * on the motor (something heavy that will turn it) you may want to
			 * hold the motor at the index instead.
       * The green LED should already be on.
			 */
			stopmotor(stepperpins);
			return;
		}

    /* Otherwise the motor now needs to move towards the required offset to the index */
		clockwise = indexposition > stepperindex? 0 : 1;
		stepdelay = ((255 - values[STEPPER_ROTATION_SPEED]) + 1)  * 128;
		stepmotor(stepperpins, motorsteps, clockwise, stepdelay);
	}
}

/* Function to move the stepper motor */
void
stepmotor(int *pins, int *steps, int direction, int stepdelay)
{
	static unsigned long laststep = micros();
	static int step = 0;
	unsigned long now = micros();

	stepdelay = max(stepdelay, 1100);

	if ((now - laststep) > stepdelay) {
		step += direction ?  1 : -1;

		if (step < 0) {
			step = 7;
			indexposition--;
		}
		if (step > 7) {
			step = 0;
			indexposition++;
		}
		if (indexposition < 0)
			indexposition == 511;
		if (indexposition > 511)
			indexposition == 0;

		digitalWrite(pins[0], steps[step] & MOTORPIN1_MASK);
		digitalWrite(pins[1], steps[step] & MOTORPIN2_MASK);
		digitalWrite(pins[2], steps[step] & MOTORPIN3_MASK);
		digitalWrite(pins[3], steps[step] & MOTORPIN4_MASK);

		laststep = now;
	}
}

/* Function to move the stepper motor */
void stopmotor(int *pins)
{

		digitalWrite(pins[0], LOW);
		digitalWrite(pins[1], LOW);
		digitalWrite(pins[2], LOW);
		digitalWrite(pins[3], LOW);
}

/* Reads a set of DMX channels from the DMX frames *
 * starting at the configured DMX base address 
 * DMX base zero could in future be used for RDM support */
void
readDMXChannels(int *dmxvalues, uint16_t dmxchannels)
{
	int max = dmxAddress+dmxchannels;
	/* There are only ever 512 slots in a frame */
	if (max>511) max = 511;

	for(uint16_t i = dmxAddress, j = 0; i < max; i++, j++)
		dmxvalues[j] = DMXSerial.read(i);
}

/* ------------------------ RGB Routines ------------------------ */ 

/* Interpret a channel value as a RED channel for WS2812 *
 * The red value is stored in the top three bits of value.
 * Mask out the the bottom bits.
 * Set the bottom bits high
 */
uint8_t 
readRed(uint8_t value) 
{
	uint8_t red = value & 0xE0;

	if(red == 0x00)
		return 0x00;
	else 
		return red | 0x1F;
}

/* Interpret a channel value as a BLUE channel for WS2812 *
 * The blue value is stored in the bottom two bits
 * Shift these bits up, then set the lower bits high
 * Note only 2 bits are used to represent Blue.
 */
uint8_t
readBlue(uint8_t value)
{ 
	uint8_t blue = value << 6;

	if(blue == 0x00) 
		return 0x00;
	else
		return blue | 0x3F;
}

/* Interpret a channel value as a GREEN channel for WS2812 *
 * The Green value is stored in bits 3, 4, and 5
 * Shift these bits up. Mask the lower bits
 * Then set the lower bits high
 */
uint8_t 
readGreen(uint8_t value)
{
	uint8_t green = value << 3;
	green &= 0xE0;

	if(green == 0x00)
		return 0x00;
	else
		return green | 0x1F;
}



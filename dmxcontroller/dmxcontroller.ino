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

/* Issues list to be completed by Aug 2023 */
/* 	0. Fix serial conflict. (see below and Tom's Libaries)
	1. DMX Timeout
		1a. Check code for DMXTIMEOUT (1 sec)
  		1b. Check board.
  		2b. Update Lab Handbook - neopixel
    	5. Check both board revisions
     		5a Update firmware in production boards !!!
*/

/* SERIAL CONFLICT to resolve -- GF
https://github.com/mathertel/DMXSerial/tree/master
The Arduino MEGA 2560 boards use the serial port 0 on pins 0 an 1.
The vector numbers differ because you use the Mega board and not the UNO.
On Mega you have the option to use the port 1 for DMX and the Serial for printing to the Monitor.
see https://github.com/mathertel/DMXSerial/blob/master/src/DMXSerial_avr.h for details to enable DMX_USE_PORT1 and change your wiring.
*/

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
	LEDCOUNT,		/* Maximum number of LEDs on the strip */
	STRIP_PIN,		/* Arduino output pin used for control */
	NEO_GRB + NEO_KHZ800 	/* Colour Byte order and Strip Speed */
);

/* This is the menu man for the board */
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
	Serial.println("Build name: " GIT);
	Serial.println("Board rev : " DMXBOARDREV);
	
	Serial.print("DMX Address: b");
	Serial.print(dipReadAddress(), BIN);
	Serial.print(" d");
	Serial.println(dipReadAddress(), DEC);

	Serial.print("Board mode:  b");
	Serial.print(dipReadMode(), BIN);
	Serial.print(" d");
	Serial.println(dipReadMode(), DEC);

	Serial.println("Red LED blinks when DMX data is received.");

	/* Test routine for NEOPIXEL strip when enabled */
#ifdef NEOPIXELDISPLAY
	Serial.print("Board configured for NEOPIXEL on pin ");
	Serial.println(STRIP_PIN);

	strip.begin();	/* Initialize the WS2812 LED strip */
	strip.show(); 	/* Set all pixels in strip to 'off' */

	// Panel test executes for full (>511)
	if(dipReadAddress() > 502) {
		Serial.println("Running panel test");

		strip.setPixelColor(0, strip.Color(255,255,255));
		for(int i = 1; i < LEDCOUNT; i++) {
			strip.setPixelColor(i,
				strip.Color(
					random(1,255),
					random(1,255),
					random(1,255)
				)
			);
			strip.show();
			//delay(5);
		}

		for(int i = 0; i < LEDCOUNT; i++)
			strip.setPixelColor(i,
				strip.Color(0, 0, 0)
			);
		strip.show();
	}
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

			/* If we have started the test program, enable DMX control */
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

	/* Blink the red status LED if there is DMX data within our timeout.
 	 * If not, hold the status red LED high, this is how commercial hardware works
 	 */

	if(DMXSerial.noDataSince() < DMXTIMEOUT) {
		OutputDisabled = FALSE;
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
		OutputDisabled = TRUE;
	}
}

/* ************************************************* */
/* Program to execute when no valid mode is selected :
 * Green LED flashes */
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
/* - frame is static if low order DIP switch is 0    */
/* = frame sets slot for switch with value from pot  */
/* Yellow LED indicates if pot > 128                 */
void 
controllermain()
{
	uint16_t value = 1;
	uint8_t reading = 0;

	static int lastAddress;

	/* read pot value and set the yellow status LED when > 128 */
	reading = (uint16_t)analogRead(POT_PIN) / 4; 
	if(reading > 128)
		digitalWrite(YELLOW_LED, HIGH);
	else
		digitalWrite(YELLOW_LED, LOW);

	/* If the address has changed, clear the last address */
	if(dmxAddress != lastAddress) {
		if(lastAddress == 0x00) {
			DMXSerial.write(1, 0x00);
			DMXSerial.write(2, 0x00);
			DMXSerial.write(3, 0x00);
			DMXSerial.write(4, 0x00);
		} else {
			DMXSerial.write(lastAddress, 0x00);
		}
	}

	/* If address is zero, send a recognisable pattern of 4 slots */
	if(dmxAddress == 0) {
		DMXSerial.write(1, 0x55); /* Slot 1 */
		DMXSerial.write(2, 0xFF); /* Slot 2 */
		DMXSerial.write(3, 0x0F); /* Slot 3 */
		DMXSerial.write(4, 0xF0); /* Slot 4 */
	} else {
		DMXSerial.write(dmxAddress, reading);
	}
	lastAddress = dmxAddress;

	/* Output a copy of the DIP switch pattern on the output ports */
	for(int i = 0;i < DEPINS_MAX; i++) {
		int status = !(dmxAddress & value);
		digitalWrite(depins[i], status);	
		value = value << 1;
	}

	/*	
	 *  The following code checks the bit max, then sets accordingly. For
	 *  some reason the bit mask does not work. I think gcc might be reducing the
	 *  size of the int.
	 *	value = (1 << 8);
	 *	digitalWrite(GREEN_LED, (dmxAddress & value) );	
	 */
	value = digitalRead(DIP9) << 8;
	if(value > 0)
		digitalWrite(GREEN_LED, HIGH);	
	else
		digitalWrite(GREEN_LED, LOW);	
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
/* Yellow LED indicates if channel 1 > 128           */

void 
pwmmain()
{
	int values[PWMPINS_MAX];

	readDMXChannels(values,PWMPINS_MAX);

	if(values[0] > 0)
		digitalWrite(YELLOW_LED, HIGH);	
	else
		digitalWrite(YELLOW_LED, LOW);	
	
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
	delay(100); // 100 ms

	readDMXChannels(values,LEDCOUNT);	
	if(values[0] > 0)
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

/* neopixellong */
/* ************************************************* */
/* Varient of program to execute when WS2812 mode selected       */
/* outputs TDM waveform for each channel value onto a*/
/* serial WS2812 port, one channel per LED           */
/* channel value is mapped via a colour lookup table */
/* to three 8-bit colour values for each pixel       */
/* Yellow LED indicates if channel 1 > 128           */
void
neopixellong()
{
#define LONGLEDCOUNT 128
	int values[LONGLEDCOUNT];
	// GF: Awkwardly, the Neopixel library doesn't coexist with the DMX ISR
	// A solution calls DMXSerial.init(DMXReceiver) here each time!
	DMXSerial.init(DMXReceiver);
	// We might need a pause here .. for the initialisation and to grab a DMX frame //
	delay(100); // 100 ms

	readDMXChannels(values,LONG LEDCOUNT);	
	if(values[0] > 0)
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

/* Main stepper motor function */
void
steppermain()
{
	uint16_t stepdelay = 25600;
	int clockwise = 1;

	int values[STEPPERCHANNELS];	/* Read all slots needed */
	readDMXChannels(values, STEPPERCHANNELS);

	int validstepperindex = 0;
	int16_t stepperindex = 0;

	if(values[0] > 0)
		digitalWrite(YELLOW_LED, HIGH);
	else
		digitalWrite(YELLOW_LED, LOW);

	if (values[STEPPER_INDEX_MODE] == 0 ) {
		if (values[STEPPER_ROTATION_SPEED] == 0 ||
			values[STEPPER_ROTATION_DIRECTION] == 0) {
			stopmotor(stepperpins);
			return;
		}

		clockwise = values[STEPPER_ROTATION_DIRECTION] > 131 ? 1 : 0;
		stepdelay = ((255 - values[STEPPER_ROTATION_SPEED]) + 1)  * 128;

		/* TODO: missing direction-based step speed shift */
		stepmotor(stepperpins, motorsteps, clockwise, stepdelay);
	} else {
		digitalWrite(GREEN_LED, LOW);
		if ((values[STEPPER_INDEX_MODE] > 0 && values[STEPPER_INDEX_MODE] <= 50)
			|| (values[STEPPER_INDEX_MODE] >= 200
				&& values[STEPPER_INDEX_MODE] <= 254))
			return;

		if (values[STEPPER_INDEX_MODE] == 255) {
			indexposition = 0;
			return;
		}

		if ((values[STEPPER_INDEX_MODE] >= 50 &&
			values[STEPPER_INDEX_MODE] <= 200 )) {

			stepperindex = values[STEPPER_INDEX_ROTATION] << 1;
			digitalWrite(GREEN_LED, HIGH);
		}

		if (indexposition == stepperindex) {
			/*
			 * Leave the motor free when we reach the index. If there is a load
			 * on the motor (something heavy that will turn it) you may want to
			 * hold the motor at the index instead.
			 */
			stopmotor(stepperpins);
			return;
		}
		if (values[STEPPER_ROTATION_SPEED] == 0) {
			stopmotor(stepperpins);
			return;
		}

		clockwise = indexposition > stepperindex? 0 : 1;
		stepdelay = ((255 - values[STEPPER_ROTATION_SPEED]) + 1)  * 128;
		stepmotor(stepperpins, motorsteps, clockwise, stepdelay);
	}
}

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



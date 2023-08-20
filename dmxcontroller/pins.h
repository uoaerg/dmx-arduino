/* - Pins.h
 * Copyright (c) 2017 Tom Jones tom@erg.abdn.ac.uk
 * Copyright (c) 2023 Gorry Fairhurst gorry@erg.abdn.ac.uk

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

#ifndef PINS_H
#define PINS_H

#include "dmxboard.h"

/* Board status LEDs */
#define GREEN_LED  A0
#define YELLOW_LED A1
#define RED_LED    A2

/* Potentiometer input for input of slot value in test mode */
#define POT_PIN    A3

#define NEOPIXELDISPLAY     /* Uncomment to enable NEOPIXEL display */
/* WS2812 TDM strip configuration */
#ifdef NEOPIXELDISPLAY
  // No of LEDs is a strip - Long strips have timing implications
  #define LEDCOUNT 16 
  #define LONGLEDCOUNT 128 
  // #define STRIP_PIN 22
  #define STRIP_PIN 9
#endif

#define TRIGGER_PIN 8

/* Pin to use for Servo output */
#define SERVOPIN 13

/* SERVO mode output configuration */
#define SERVOPINS_MAX 2
#define SERVO1PIN 12 
#define SERVO2PIN 13

static int servopins[SERVOPINS_MAX] =
{
	SERVO1PIN,
	SERVO2PIN
};

/* PWM mode output configuration */
#define PWMPINS_MAX 2
#define PWM1PIN 10
#define PWM2PIN 11

static int pwmpins[PWMPINS_MAX] =
{
	PWM1PIN,
	PWM2PIN
};


/* D 9 Output Socket on the mega shield */
#define DEPINS_MAX 8

#define DEENABLE1 3
#define DEENABLE2 4

#if REV == REV_II

	#define DE1PIN1 26
	#define DE1PIN2 24
	#define DE1PIN3 25
	#define DE1PIN4 23
	#define DE1PIN5 22
	#define DE1PIN6 27
	#define DE1PIN7 28
	#define DE1PIN8 29

	/* REV_II DEPINS are routed through an inverting buffer */
	#define DEHIGH LOW
	#define DELOW HIGH

#elif REV == REV_IIIA

	#define DE1PIN1 24
	#define DE1PIN2 22
	#define DE1PIN3 27
	#define DE1PIN4 30
	#define DE1PIN5 31
	#define DE1PIN6 26
	#define DE1PIN7 29
	#define DE1PIN8 28
	
	#define DEHIGH HIGH
	#define DELOW LOW

#endif

static int de1pins[DEPINS_MAX] = 
{
	DE1PIN1,
	DE1PIN2, 
	DE1PIN3, 
	DE1PIN4, 
	DE1PIN5, 
	DE1PIN6, 
	DE1PIN7, 
	DE1PIN8,
};

#define depins de1pins

/* DIP Switch mapping to I/O pins */
#define DIPMAX 12

#if REV == REV_II

	#define DIP1 37
	#define DIP2 36
	#define DIP3 35
	#define DIP4 34
	#define DIP5 33
	#define DIP6 32
	#define DIP7 31
	#define DIP8 30 
	#define DIP9 38
	#define DIP10 39 
	#define DIP11 40 
	#define DIP12 41

#elif REV == REV_IIIA

	#define DIP1 49
	#define DIP2 48
	#define DIP3 47
	#define DIP4 43
	#define DIP5 42
	#define DIP6 41
	#define DIP7 40
	#define DIP8 39
	#define DIP9 38
	#define DIP10 37
	#define DIP11 36
	#define DIP12 34

#else
	#error "please set board revision"
#endif // REV switch

static int dipSwitches[DIPMAX] = 
{
	DIP1, 
	DIP2, 
	DIP3, 
	DIP4, 
	DIP5, 
	DIP6, 
	DIP7, 
	DIP8, 
	DIP9, 
	DIP10, 
	DIP11, 
	DIP12 
};

#define STEPPERPIN1 50
#define STEPPERPIN2 51
#define STEPPERPIN3 52
#define STEPPERPIN4 53
#define STEPPERPINS_MAX 4

#define STEPPER_ROTATION_SPEED 0
#define STEPPER_ROTATION_DIRECTION 1
#define STEPPER_INDEX_MODE 2
#define STEPPER_INDEX_ROTATION 3


#define STEPPERCHANNELS 4

static int stepperpins[STEPPERPINS_MAX] =
{
	STEPPERPIN1,
	STEPPERPIN2,
	STEPPERPIN3,
	STEPPERPIN4
};

#define MOTORPIN1_MASK 0x01
#define MOTORPIN2_MASK 0x02
#define MOTORPIN3_MASK 0x04
#define MOTORPIN4_MASK 0x08

#define MAXSTEPS 8

static int motorsteps[MAXSTEPS] =
{
	B01000,
	B01100,
	B00100,
	B00110,
	B00010,
	B00011,
	B00001,
	B01001
};

/* Global mode variables read from DIP switches */
uint8_t dmxMode;
uint16_t dmxAddress;

#endif	//PINS_H

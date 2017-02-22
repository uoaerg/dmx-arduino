/*-
 * Copyright (c) 2017 Tom Jones tom@erg.abdn.ac.uk
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

#ifndef TEST_H
#define TEST_H

/* Board status LEDs */
#define GREEN_LED A0
#define YELLOW_LED A1
#define RED_LED A2

/* Potentiometer input for input of slot value in test mode */
#define POT_PIN A3

//#define NEOPIXELDISPLAY     /* Uncomment to enable NEOPIXEL display */
/* WS2812 TDM strip configuration */
#ifdef NEOPIXELDISPLAY

#define LEDCOUNT 256
#define STRIP_PIN 22

#pragma message "USING CONFIG FOR NEOPIXEL GRID DISPLAY"

#else NEOPIXELDISPLAY

#define LEDCOUNT 128 
#define STRIP_PIN 9

#endif //NEOPIXELDISPLAY

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

#define DE1PIN1 26
#define DE1PIN2 24
#define DE1PIN3 25
#define DE1PIN4 23
#define DE1PIN5 22
#define DE1PIN6 27
#define DE1PIN7 28
#define DE1PIN8 29

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


/* Global mode variables read from DIP switches */
uint8_t dmxMode;
uint16_t dmxAddress;

#endif	//TEST_H

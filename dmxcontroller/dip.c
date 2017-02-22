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

#include <Arduino.h>

#include "dip.h"
#include "pins.h"

/*
 * DMX controller has 8 modes, encoded in the top 3(DIP 1,2,3) bits of
 * the dip switches. 
 */
uint8_t
dipReadMode()
{
	uint8_t value = 0;	

	value |= digitalRead(DIP12) << 2; 
	value |= digitalRead(DIP11) << 1; 
	value |= digitalRead(DIP10); 

	return value;
}

/*
 * DMX Controller has a 9 bit address encoded in the 9 lower bits of the
 * dip switches.
 */
uint16_t
dipReadAddress()
{
	uint16_t value = 0;	

	value |= digitalRead(DIP9) << 8; 
	value |= digitalRead(DIP8) << 7; 
	value |= digitalRead(DIP7) << 6; 
	value |= digitalRead(DIP6) << 5; 
	value |= digitalRead(DIP5) << 4; 
	value |= digitalRead(DIP4) << 3; 
	value |= digitalRead(DIP3) << 2; 
	value |= digitalRead(DIP2) << 1; 
	value |= digitalRead(DIP1); 

	return value;
}

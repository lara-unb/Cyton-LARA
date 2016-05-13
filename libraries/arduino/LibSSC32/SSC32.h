/**
*	@file SSC32.h
*	@brief Control Lynxmotion's SSC-32 with Arduino.
*	@author Martin Peris (http://www.martinperis.com)
*	@date 5/2/2011
*
*	@change 27.10.2011 by Marco Schwarz <marco.schwarz@cioppino.net>
*/

/*
  SSC32.h - Control Lynxmotion's SSC-32 V2 with Arduino
  Copyright (c) 2011 Martin Peris (http://www.martinperis.com).
  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef SSC32_H
#define SSC32_H

/*
* @author Marco Schwarz
* @descr for Arduino >= 1.0rc2 
*/ 
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"	// for digitalRead, digitalWrite, etc
#else
#include "WProgram.h"
#endif

#define SSC32_MIN_CH 0
#define SSC32_MAX_CH 31

#define SSC32_MIN_PW 500
#define SSC32_MAX_PW 2500

#define SSC32_MIN_TIME 1
#define SSC32_MAX_TIME 65535

#define SSC32_MIN_OFFSET -100
#define SSC32_MAX_OFFSET 100

#define SSC32_CMDGRP_TYPE_NONE 0
#define SSC32_CMDGRP_TYPE_SERVO_MOVEMENT 1
#define SSC32_CMDGRP_TYPE_PULSE_OFFSET 2
#define SSC32_CMDGRP_TYPE_DISCRETE_OUTPUT 3

#define SSC32_BANK_0 0
#define SSC32_BANK_1 1
#define SSC32_BANK_2 2
#define SSC32_BANK_3 3

#define SSC32_DIGITAL_INPUT_A 0
#define SSC32_DIGITAL_INPUT_B 1
#define SSC32_DIGITAL_INPUT_C 2
#define SSC32_DIGITAL_INPUT_D 3

#define SSC32_DIGITAL_INPUT_AL 4
#define SSC32_DIGITAL_INPUT_BL 5
#define SSC32_DIGITAL_INPUT_CL 6
#define SSC32_DIGITAL_INPUT_DL 7

#define SSC32_ANALOG_INPUT_VA 8
#define SSC32_ANALOG_INPUT_VB 9
#define SSC32_ANALOG_INPUT_VC 10
#define SSC32_ANALOG_INPUT_VD 11

class SSC32
{
	private:
		int _ttcm;
		int _commandType;

	public:

		SSC32();
		
		void begin(int bauds);

		boolean beginGroupCommand(int type);
		boolean abortGroupCommand();
		boolean endGroupCommand();

		boolean servoMove(int channel, int position);
		boolean servoMove(int channel, int position, int speed);
		boolean servoMoveTime(int channel, int position, int ttcm);

		boolean pulseOffset(int channel, int offset);
		boolean discreteOutput(int channel, boolean level);
		boolean byteOutput(int bank, int value);

		boolean isMoving();
		int queryPulseWidth(int channel);
		int readDigitalInput(int input);
		int readAnalogInput(int input);
		

};

#endif

/**
*	@file SSC32.h
*	@brief Control Lynxmotion's SSC-32 with Arduino.
*	@author Martin Peris (http://www.martinperis.com)
*	@date 5/2/2011
*
*	@change 27.10.2011 by Marco Schwarz <marco.schwarz@cioppino.net>
*/

/*
  SSC32.cpp - Control Lynxmotion's SSC-32 V2 with Arduino
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

/*
* @author Marco Schwarz
* @descr for Arduino >= 1.0rc2 
*/ 
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"	// for digitalRead, digitalWrite, etc
#else
#include "WProgram.h"
#endif

#include "SSC32.h"

/**
*	Create an SSC32 object and initialize SoftwareSerial1.
*/
SSC32::SSC32() 
{
	_ttcm = -1;
	_commandType = SSC32_CMDGRP_TYPE_NONE;
}

/**
*	Begin the Serial1 port communications. Should be called inside the setup() function of the Arduino.
*	@param bauds	The speed in bauds for the Serial1 port
*/
void SSC32::begin(int bauds)
{
	Serial1.begin(bauds);
}


/**
*	Start a new group of commands
*	@param type	The type of group command. Should be one of the following : SSC32_CMDGRP_TYPE_SERVO_MOVEMENT, SSC32_CMDGRP_TYPE_PULSE_OFFSET, SSC32_CMDGRP_TYPE_DISCRETE_OUTPUT
*	@return It will return false if a previous call to beginGroupCommand has not been finished yet by calling endGroupCommand. True otherwise.
*/
boolean SSC32::beginGroupCommand(int type)
{

	if (type < SSC32_CMDGRP_TYPE_NONE || type > SSC32_CMDGRP_TYPE_DISCRETE_OUTPUT)
	{
		//The command type is not valid
		return false;
	}


	if ( _commandType != SSC32_CMDGRP_TYPE_NONE)
	{
		//Can not start another command group while in the middle of another command group
		return false;
	}

	_commandType = type;
	return true;

}

/**
*	Abort a group of commands
*	@return It will return false if a group command has not been started by calling beginGroupCommand. True otherwise.
*/
boolean SSC32::abortGroupCommand()
{

	if (_commandType == SSC32_CMDGRP_TYPE_NONE)
	{
		//Can not abort, we are not in a group of commands
		return false;
	}

	//According to the manual I should write the ascii character for <esc>
	
	Serial1.write(27);
	_commandType = SSC32_CMDGRP_TYPE_NONE;
	_ttcm = -1;

	return true;
}

/**
*	End a group of commands
*	@return It will return false if a group command has not been started by calling beginGroupCommand or it has been aborted by abortGroupCommand. True otherwise.
*/
boolean SSC32::endGroupCommand()
{

	if (_commandType == SSC32_CMDGRP_TYPE_NONE)
	{
		//Can not end, we are not in a group of commands
		return false;
	}

	if (_ttcm != -1)
	{
		//Set the time to complete movement
		Serial1.print(" T");
		Serial1.print(_ttcm);
	}

	Serial1.println();
	_commandType = SSC32_CMDGRP_TYPE_NONE;
	_ttcm = -1;
	return true;

}

/**
*	Move the servo at #channel to "position".
*	If this function is called from outside a pair of beginGroupCommand/endGroupCommand	then the servo will move right away.
*
*	But if this function is called inside a gruop of commands, then the servo will not move	until you call endGroupCommand
*	@param	channel	The servo to move
*	@param	position	The position where to move the servo
*	@return False if the channel or position is not valid or if this function is called while inside a command group other than SSC32_CMDGRP_TYPE_SERVO_MOVEMENT. True otherwise.
*/
boolean SSC32::servoMove(int channel, int position)
{
	if (channel < SSC32_MIN_CH || channel > SSC32_MAX_CH)
	{
		//Channel not valid
		return false;
	}

	if (position < SSC32_MIN_PW || position > SSC32_MAX_PW)
	{
		//Position not valid
		return false;
	}

	if (_commandType != SSC32_CMDGRP_TYPE_NONE && _commandType != SSC32_CMDGRP_TYPE_SERVO_MOVEMENT)
	{
		//This can only be called as a single command or inside a group of 
		//commands of type SSC32_CMDGRP_TYPE_SERVO_MOVEMENT
		return false;	
	}

	//We are good to go
	Serial1.print("#");
	Serial1.print(channel);
	Serial1.print(" P");
	Serial1.print(position);
	Serial1.print(" ");

	if (_commandType == SSC32_CMDGRP_TYPE_NONE)
	{
		//This is a single command so execute it
		Serial1.println();
	}

	return true;

}

/**
*	Move the servo at #channel to "position" with speed "speed".
*	If this function is called from outside a pair of beginGroupCommand/endGroupCommand	then the servo will move right away.
*
*	But if this function is called inside a gruop of commands, then the servo will not move	until you call endGroupCommand.
*	@param channel The servo to move
*	@param position	The position where to move the servo
*	@param speed	The speed for the movement
*	@return False if the channel or position or speed is invalid or if this function is called while inside a command group other than SSC32_CMDGRP_TYPE_SERVO_MOVEMENT. True otherwise.
*/
boolean SSC32::servoMove(int channel, int position, int speed)
{
	if (channel < SSC32_MIN_CH || channel > SSC32_MAX_CH)
	{
		//Channel not valid
		return false;
	}

	if (position < SSC32_MIN_PW || position > SSC32_MAX_PW)
	{
		//Position not valid
		return false;
	}

	if (speed < 0)
	{
		//Speed not valid
		return false;
	}

	if (_commandType != SSC32_CMDGRP_TYPE_NONE && _commandType != SSC32_CMDGRP_TYPE_SERVO_MOVEMENT)
	{
		//This can only be called as a single command or inside a group of 
		//commands of type SSC32_CMDGRP_TYPE_SERVO_MOVEMENT
		return false;	
	}

	//We are good to go
	Serial1.print("#");
	Serial1.print(channel);
	Serial1.print(" P");
	Serial1.print(position);
	Serial1.print(" S");
	Serial1.print(speed);
	Serial1.print(" ");

	if (_commandType == SSC32_CMDGRP_TYPE_NONE)
	{
		//This is a single command so execute it
		Serial1.println();
	}

	return true;

}

/**
*	Move the servo at #channel to "position" taking "time" milliseconds.
*	If this function is called from outside a pair of beginGroupCommand/endGroupCommand	then the servo will move right away.
*
*	But if this function is called inside a gruop of commands, then the servo will not move	until you call endGroupCommand
*	@param channel The servo to move
*	@param position	The position where to move the servo
*	@param ttcm	Time to complete the movement
*	@return False if the channel or position or time is invalid or if this function is called while inside a command group other than SSC32_CMDGRP_TYPE_SERVO_MOVEMENT. True otherwise.
*/
boolean SSC32::servoMoveTime(int channel, int position, int ttcm)
{
	if (channel < SSC32_MIN_CH || channel > SSC32_MAX_CH)
	{
		//Channel not valid
		return false;
	}

	if (position < SSC32_MIN_PW || position > SSC32_MAX_PW)
	{
		//Position not valid
		return false;
	}

	if (ttcm < SSC32_MIN_TIME || ttcm > SSC32_MAX_TIME )
	{
		//time not valid
		return false;
	}

	if (_commandType != SSC32_CMDGRP_TYPE_NONE && _commandType != SSC32_CMDGRP_TYPE_SERVO_MOVEMENT)
	{
		//This can only be called as a single command or inside a group of 
		//commands of type SSC32_CMDGRP_TYPE_SERVO_MOVEMENT
		return false;	
	}

	//We are good to go
	Serial1.print("#");
	Serial1.print(channel);
	Serial1.print(" P");
	Serial1.print(position);

	if (_commandType == SSC32_CMDGRP_TYPE_NONE)
	{
		//This is a single command so execute it
		Serial1.print(" T");
		Serial1.print(ttcm);
		Serial1.print(" ");
		Serial1.println();
	}else{
		//This is a command group. Store the "time to complete movement" for later
		_ttcm = ttcm;
	}

	return true;

}

/**
*	The servo channel will be offset by the amount indicated in the offset value
*	@param channel The servo to move
*	@param offset	The offset to apply
*	@return False if the channel or offset is invalid or if this function is called while inside a command group other than SSC32_CMDGRP_TYPE_PULSE_OFFSET. True otherwise.
*/

boolean SSC32::pulseOffset(int channel, int offset)
{

	if (channel < SSC32_MIN_CH || channel > SSC32_MAX_CH)
	{
		//Channel not valid
		return false;
	}

	if (offset < SSC32_MIN_OFFSET || offset > SSC32_MAX_OFFSET)
	{
		//Offset not valid
		return false;
	}

	if (_commandType != SSC32_CMDGRP_TYPE_NONE && _commandType != SSC32_CMDGRP_TYPE_PULSE_OFFSET)
	{
		//This can only be called as a single command or inside a group of 
		//commands of type SSC32_CMDGRP_TYPE_PULSE_OFFSET
		return false;	
	}
	
	//We are good to go
	Serial1.print("#");
	Serial1.print(channel);
	Serial1.print(" PO");
	Serial1.print(offset);
	Serial1.print(" ");

	if (_commandType == SSC32_CMDGRP_TYPE_NONE)
	{
		//This is a single command so execute it
		Serial1.println();
	}

	return true;

}

/**
*	The channel will go to the indicated level
*	@param channel The output channel
*	@param level	The level to write (HIGH or LOW)
*	@return	It will return false if channel is invalid or if this function is called while inside a command group other than SSC32_CMDGRP_TYPE_DISCRETE_OUTPUT. True otherwise.
*/
boolean SSC32::discreteOutput(int channel, boolean level)
{
	if (channel < SSC32_MIN_CH || channel > SSC32_MAX_CH)
	{
		//Channel not valid
		return false;
	}

	if (_commandType != SSC32_CMDGRP_TYPE_NONE && _commandType != SSC32_CMDGRP_TYPE_DISCRETE_OUTPUT)
	{
		//This can only be called as a single command or inside a group of 
		//commands of type SSC32_CMDGRP_TYPE_PULSE_OFFSET
		return false;	
	}

	//We are good to go
	Serial1.print("#");
	Serial1.print(channel);

	if (level == HIGH)
		Serial1.print("H");
	else
		Serial1.print("L");

	if (_commandType == SSC32_CMDGRP_TYPE_NONE)
	{
		//This is a single command so execute it
		Serial1.println();
	}

	return true;
}

/**
*	This command allows 8 bits of binary data to be written at once.
*	@param bank The output bank of bits. Value should be one of these : SSC32_BANK_0, SSC32_BANK_1, SSC32_BANK_2, SSC32_BANK_3
*	@param value The value to write. Should be between 0 and 255.
*	@return It will return false if the bank or the value is not valid. True otherwise.  
*/
boolean SSC32::byteOutput(int bank, int value)
{

	if (bank < SSC32_BANK_0 || bank > SSC32_BANK_3)
	{
		//Bank is not valid
		return false;
	}

	if (value < 0 || value > 255)
	{
		//Value is not valid
		return false;
	}

	//We are good to go
	Serial1.print("#");
	Serial1.print(bank);
	Serial1.print(":");
	Serial1.print(value);
	Serial1.println();
	

	return true;
}

/**
*	Check whether there is a movement in progress or not.
*	@return This will return false if the previous movement is complete and true if it is still in progress.
*/
boolean SSC32::isMoving()
{
	char c = '.';
	Serial1.println("Q");
	Serial1.println();
	
	//delay(50);
	if (Serial1.available())
	{

		c = Serial1.read();
	
	}
	

	if ( c == '+' )
		return true;
	
	return false;

}

/**
*	This will return a value indicating the pulse width of the selected servo with a resolution	of 10uS. For example, if the pulse width is 1500uS, the returned value would be 150.
*	@param channel The channel to query
*	@return Will return -1 if the channel is not valid or the function is called inside a group of commands. Will return the pusle width of the desired channel otherwise.
*/
int SSC32::queryPulseWidth(int channel)
{

	if (channel < SSC32_MIN_CH || channel > SSC32_MAX_CH)
	{
		//Channel not valid
		return -1;
	}

	if (_commandType != SSC32_CMDGRP_TYPE_NONE)
	{
		//This can only be executed from outside a group of commands. That is, as a single command.
		return -1;
	}

	char c;
	Serial1.print("QP");
	Serial1.print(channel);
	Serial1.println();

	delay(50);
	if (Serial1.available())
	{	
		c = Serial1.read();
		return int(c);
	}else{
		return -1;
	}
}

/**
*	This function reads the digital inputs A, B, C, D, AL, BL, CL and DL according to the SSC-32 user manual
*	@param input The digital input to read. Should be one of these : SSC32_DIGITAL_INPUT_A, SSC32_DIGITAL_INPUT_B, SSC32_DIGITAL_INPUT_C, SSC32_DIGITAL_INPUT_D, SSC32_DIGITAL_INPUT_AL, SSC32_DIGITAL_INPUT_BL, SSC32_DIGITAL_INPUT_CL, SSC32_DIGITAL_INPUT_DL
*	@return It will return the state of the desired input (0 or 1). Or -1 if any error happens.  
*/
int SSC32::readDigitalInput(int input)
{

	if (_commandType != SSC32_CMDGRP_TYPE_NONE)
	{
		//This can only be executed from outside a group of commands. That is, as a single command.
		return -1;
	}

	switch(input)
	{
		case SSC32_DIGITAL_INPUT_A:
			//Read digital input A
			Serial1.print("A");
			break;
		case SSC32_DIGITAL_INPUT_B:
			//Read digital input B
			Serial1.print("B");
			break;
		case SSC32_DIGITAL_INPUT_C:
			//Read digital input C
			Serial1.print("C");
			break;
		case SSC32_DIGITAL_INPUT_D:
			//Read digital input D
			Serial1.print("D");
			break;
		case SSC32_DIGITAL_INPUT_AL:
			//Read digital input AL
			Serial1.print("AL");
			break;
		case SSC32_DIGITAL_INPUT_BL:
			//Read digital input BL
			Serial1.print("BL");
			break;
		case SSC32_DIGITAL_INPUT_CL:
			//Read digital input CL
			Serial1.print("CL");
			break;
		case SSC32_DIGITAL_INPUT_DL:
			//Read digital input DL
			Serial1.print("DL");
			break;
		default:
			//Input not valid
			return -1;
	}

	char c;
	delay(50);
	if (Serial1.available())
	{
		c = Serial1.read();
		return int(c);
	}else{
		return -1;
	}

}

/**
*	This function reads the analog inputs VA, VB, VC, VD according to the SSC-32 user manual
*	@param input The analog input to read. Should be one of these : SSC32_DIGITAL_INPUT_VA, SSC32_DIGITAL_INPUT_VB, SSC32_DIGITAL_INPUT_VC, SSC32_DIGITAL_INPUT_VD
*	@return It will return the state of the desired input (Between 0 and 255). Or -1 if any error happens.  
*/
int SSC32::readAnalogInput(int input)
{

	if (_commandType != SSC32_CMDGRP_TYPE_NONE)
	{
		//This can only be executed from outside a group of commands. That is, as a single command.
		return -1;
	}

	switch(input)
	{
		case SSC32_ANALOG_INPUT_VA:
			//Read digital input A
			Serial1.print("VA");
			break;
		case SSC32_ANALOG_INPUT_VB:
			//Read digital input B
			Serial1.print("VB");
			break;
		case SSC32_ANALOG_INPUT_VC:
			//Read digital input C
			Serial1.print("VC");
			break;
		case SSC32_ANALOG_INPUT_VD:
			//Read digital input D
			Serial1.print("D");
			break;
		
		default:
			//Input not valid
			return -1;
	}

	char c;
	delay(50);
	if (Serial1.available())
	{
		c = Serial1.read();
		return int(c);
	}else{
		return -1;
	}

}



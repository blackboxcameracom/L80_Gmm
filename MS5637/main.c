/***************************************************************************
* Copyright (C) 2018 The BlackBoxCmaera Company Limited  GPS-PIE.COM
*
* File : MS5637/main.c
*
* Date : 2018/08/28
*
* Revision : 1.0
*
* Usage: main.c file for the MS5637 pressure / temperature sensor interface functions 
*
****************************************************************************
* section License
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
**************************************************************************/

#include<stdio.h>
#include<wiringPi.h>
#include "ms5637_funcs.h"

#define BUTTON1		21	// Wiring pin 21, physical pin 29, connection for button 1
#define STATUS_LED	25	// Wiring pin 25, physical pin 37, connected to status LED

void button1_ISR( void );		// Interrupt service routine for button 1
void toggleStatus( void );	// Toggle the state of the status LED

volatile char button1;	// Flag set in the button interrupt service rountine

int main()
{
	// Open the i2c device and connect to the ms5637
	if( ms5637_init() == false)
	{
		return(0);	// Exit if the device is not opened correctly
	}

	ms5637_reset();	// Reset the ms5637 prior to operation


//--- GPIO pin set up with WiringPi library functions

	wiringPiSetup();
	
	pinMode( STATUS_LED, OUTPUT );		// Set up the status LED pin
	digitalWrite( STATUS_LED, LOW );	// Set initial state to low, LED off	

	pinMode( BUTTON1, INPUT );			// Set up the button pins as inputs
	pullUpDnControl( BUTTON1, PUD_UP );	// with pull ups

//--- Set up ISR for button 1
	wiringPiISR(BUTTON1, INT_EDGE_FALLING, &button1_ISR);
	button1 = 0;	

//---			
	
	/** To calculate the height above sea level the pressure at sea level must be set here. 
	 * For the UK the National Physical Laboratry provides an online bariograph
	 * giving a sea level pressure reference for London.
	 * See resource.npl.co.uk/pressure/pressure.html 
	 * For other areas see weatheronline.co.uk
	 * The defined SEA_LEVEL_PRESSURE_MBAR = 1012.3 is the average pressure at 
	 * sea level. */
	 setBasePressure( 1003.3 );	// Sea level equivalent for London, UK
	// setBasePressure( SEA_LEVEL_PRESSURE_MBAR );	// Average pressure at sea level 


	/** As an alternative to the sea level realtive calculation above,
	 * to calculate relative altitude from a current altitude, the current 
	 * pressure must be read, then set as the base pressure.  Comment out the 
	 * setBasePressure( ) above and un-comment the two lines below. */
	/**/
	//read_MS5637();	// Read the temperature & pressure from the MS5637, calculate altitude
	
	//setCurrentPressure();	// Set the pressure read as the base pressure, altitude read from zero
	
	
		
	while(1)
	{
		read_MS5637();	// Read the temperature & pressure from the MS5637, calculate altitude
	
		printf("Pressure %5.1f mbar   Temperature %5.1f celcius \n", pressure, temperature);
	
		printf("Altitude %5.1fm  %5.1ff\n", altitude, ( altitude * 3.28084 ) );
		
		if( button1 )
		{
			button1 = 0;
			
			read_MS5637();	// Read the temperature & pressure from the MS5637, calculate altitude
	
			setCurrentPressure();	// Set the pressure read as the base pressure, altitude read from zero
			
			toggleStatus();
		}
		
		
		delay( 1000 );
		
	}	
	
	return(0);
}

// ISR for the button1 interrupt
void button1_ISR( void )
{
	button1 = 1;	// Set the flag toshow data waiting to be read
	
}	


// Routine to toggle the status LED
void toggleStatus( void )
{
	static char toggleState = 0;
	
	if( toggleState )
	{
		toggleState = 0;
		digitalWrite( STATUS_LED, LOW );	// Set LED off
	}
	else
	{
		toggleState = 1;
		digitalWrite( STATUS_LED, HIGH );	// Set LED on
	}
	
	
	return;
}

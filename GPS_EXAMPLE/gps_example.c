/***************************************************************************
* Copyright (C) 2018 The BlackBoxCmaera Company Limited  GPS-PIE.COM
*
* File : gpsExample/gps_example.c
*
* Date : 2018/08/28
*
* Revision : 1.0
*
* Usage: GPS Example  gps_example.c showing non-threaded usage of GPS interface functions 
*
****************************************************************************
* \section License
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
#include<unistd.h>
#include<string.h>
#include<time.h>
#include<wiringPi.h>
#include "gps_functions.h"

#define BUTTON1		21	// Wiring pin 21, physical pin 29, connection for button 1


char filename[20];	// String for the filename
void makeFilename(void);	// Function that makes a filename from the time
void button1_ISR( void );		// Interrupt service routine for button 1	

volatile char button1;	// Flag set in the button interrupt service rountine

int main(int argc, char *argv[])
{
	FILE *gps_log_file;

//--- GPIO pin set up with WiringPi library functions

	wiringPiSetup();
	
	pinMode( BUTTON1, INPUT );			// Set up the button pin as an input
	pullUpDnControl( BUTTON1, PUD_UP );	// with pull ups

	if(!digitalRead(BUTTON1) )
	{
		exit(0);	// On start up if button 1 is down exit the program
		
	}

//--- Set up ISR for button 1
	wiringPiISR(BUTTON1, INT_EDGE_FALLING, &button1_ISR);
	button1 = 0;	

	
	/* Open the UART and connect to the GPS receiver. Note that short intervals cannot be set with low
	 * baud rates.  Incompatible settings will be ignored by the receiver.
	 * If you are experimenting and temporarily change from the default, replace default with the current
	 * baud rate.
	 * See description in gps_functions.c
	 */
	if( openGPSUART(L80, 9600, 9600, 1000 ) <0 )
//	if( openGPSUART(PA6B, 9600, 9600, 1000 ) <0 )	
//	if( openGPSUART(GMM, 38400, 38400, 1000 ) <0 )	
	{	// Abort if UART open fails								
        return -1;
	}
 
	startSerialThread();	// 	Start the thread that reads the GPS NMEA data from the serial port
	
	printf( "Thread started\n");
	
	makeFilename();		// Create a unique filename from the current time 

	gps_log_file = fopen( filename, "wb" );	 // "gps_data.log"
	
	setTimerAlarmInterval( 180  );	// Set a timer interval for number of minutes reqd. 
   
	// Loop waiting for characters to arrive from the GPS receiver
	do 
	{
			
		if( rmc_recvd )	// Flag is set by the serial thread to indicate data received
		{
			printf("%s", rmcBuffer ); 	
			
			fwrite( rmcBuffer, 1, strlen( rmcBuffer ), gps_log_file );			
			
			rmc_recvd = FALSE;	
		}
		
		if( gga_recvd )
		{
			printf("%s", ggaBuffer );
			
			fwrite( ggaBuffer, 1, strlen( ggaBuffer ), gps_log_file );		
			
			gga_recvd = FALSE;	
		}
		
		if( nmea_recvd )
		{
			printf("%s", nmeaBuffer );
			
			fwrite( nmeaBuffer, 1, strlen( nmeaBuffer ), gps_log_file );		
			
			nmea_recvd = FALSE;	
		}
		
		usleep(100000);	// 100ms for max 10Hz data rate	

		if( button1 )	// button1 is set when the button is pressed
		{
			fclose( gps_log_file );
   
			closeGPSUART();
			
			system("shutdown now");	// Safely exit the program and shutdown the Raspberry Pi
		}
					
	}
	while( !timesUp );	// Timed loop set by period of alarm interrupt  
	// while( 1 );    // Uncomment for endless loop 
	
	fclose( gps_log_file );
   
	closeGPSUART();
	
	return 0;
}


// Function that makes a filename from the time
void makeFilename(void)
{
	struct tm *local;
	time_t t;
	
	t = time(NULL);
	local = localtime(&t);
	
	sprintf( filename, "%02d%02d%02d%02d%02d%02d.log", local->tm_hour, local->tm_min, local->tm_sec, local->tm_mday, local->tm_mon + 1, local->tm_year - 100 );
	
	//printf("%s", filename);
	
	return;
}

// ISR for the button1 interrupt
void button1_ISR( void )
{
	button1 = 1;	// Set the flag to show data waiting to be read

	
}	

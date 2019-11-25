/***************************************************************************
* Copyright (C) 2019 The BlackBoxCmaera Company Limited  GPS-PIE.COM
*
* File : COMBINED_EXAMPLE/main.c
*
* Date : 2019/11/25
*
* Revision : 1.01
*
* Usage: main.c file for Gmm1 GPS, BNO055, MS5673 interface example 
* This file is specifically for the Gmm-u1 GPS receiver.  The reading 
* of the MS5673 must take place after the last GPS data sentence whiich
* is RMC for the Gmm-u1 versus GGA for the L80
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
#include "../GPS_EXAMPLE/gps_functions.h"	// Defines data types and functions for accessing the GPS Receiver
#include "../MS5637/ms5637_funcs.h"	// Defines data types and functions for accessing the MS5637 sensor
#include "../BNO055/bno055_funcs.h"	// Defines data types and functions for accessing the BNO055 sensor

#define BUTTON1		21	// Wiring pin 21, physical pin 29, connection for button 1
#define BUTTON2		22	// Wiring pin 22, physical pin 31, connection for button 2

char filename[20];	// String for the filename
void makeFilename(void);	// Function that makes a filename from the time
void button1_ISR( void );		// Interrupt service routine for button 1	
void button2_ISR( void );		// Interrupt service routine for button 2	

volatile char button1;	// Flag set in the button 1 interrupt service rountine
volatile char button2;	// Flag set in the button 2 interrupt service rountine

// The program can store GPS and sensor data either as plain text or a KML point, or both 
char	useKML = TRUE;		// Set true to store data as a KML point
char	useText = FALSE;	// Set true to store data as plain text	

BNO055_NDOF_Data_t NDOF_data;		// Data structure used to return data from the BNO055
BNO055_QuaternionToEuler_Data_t QtoEuler;	// Data structure used to return Quaternion data converted to Euler angles from the BNO055

int main(int argc, char *argv[])
{
	FILE *gps_log_file;
	
	// Open the i2c device and connect to the BNO055
	if( !i2cBusSetUp() )
	{
		return(0);	//Exit the program if no BNO055 is detected at start up		
	}
	

//--- GPIO pin set up with WiringPi library functions.  Library is initialised by call to i2cBusSetUp() routine above.

	pinMode( BUTTON1, INPUT );			// Set up the button 1 pin as an input
	pullUpDnControl( BUTTON1, PUD_UP );	// with pull ups

	if(!digitalRead(BUTTON1) )
	{
		exit(0);	// On start up if button 1 is down exit the program
		
	}

//--- Set up ISR for button 1
	wiringPiISR(BUTTON1, INT_EDGE_FALLING, &button1_ISR);
	button1 = 0;	

	pinMode( BUTTON2, INPUT );			// Set up the button 2 pin as an input
	pullUpDnControl( BUTTON2, PUD_UP );	// with pull ups

//--- Set up ISR for button 2
	wiringPiISR(BUTTON2, INT_EDGE_FALLING, &button2_ISR);
	button2 = 0;	

//---	
	
	configureBNO055();

	setUpMotionInterrupt();		

	setUPGPIOInterrupt();

//---

	// Open the i2c device and connect to the ms5637
	if( ms5637_init() == false)
	{
		return(0);	// Exit if the device is not opened correctly
	}

	ms5637_reset();	// Reset the ms5637 prior to operation

	/** To calculate the altitude the pressure at sea level must be set here. 
	 * For the UK the National Physical Laboratry provides an online bariograph
	 * giving a sea level pressure reference for London.
	 * See resource.npl.co.uk/pressure/pressure.html 
	 * For other areas see weatheronline.co.uk
	 * The defined SEA_LEVEL_PRESSURE_MBAR = 1012.3 is the average pressure at 
	 * sea level. */
	setBasePressure( 1010.8 );	// Sea level equivalent for London, UK
	
//---
	
	// Open the UART and connect to the GPS receiver.
	if( openGPSUART(GMMU1, 38400, 38400, 200 ) <0 )	
	{	// Abort if UART open fails								
        return -1;
	}
 
	startSerialThread();	// 	Start the thread that reads the GPS NMEA data from the serial port
	
	printf( "Thread started\n");

//---	

	makeFilename();		// Create a unique filename from the current time 

	gps_log_file = fopen( filename, "wb" );	 // "gps_data.log"
	if( useKML )
	{// KML header
		fprintf(gps_log_file, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n" );
		fprintf(gps_log_file, "<kml xmlns=\"http://earth.google.com/kml/2.0\">\n" );
		fprintf(gps_log_file, "<Document>\n\n" );

		fprintf(gps_log_file, "<Style id=\"My_Icon_Style\">\n" );
		fprintf(gps_log_file, "\t<IconStyle> <Icon> <href>cpoint.png</href> </Icon> </IconStyle>\n" );
		fprintf(gps_log_file, "</Style>\n" );
	}

	setTimerAlarmInterval( 150 );	// Set a timer interval in minutes 
   
	// Loop waiting for characters to arrive from the GPS receiver
	do 
	{
		if( IntDataWaiting )	// Check the bit indicating interrupt data waiting
		{
			printf(" Int!\n");	// This is the BNO055 motion interrupt

			resetGPIOInterrupt();

			toggleStatus();
			
		}	




		if( rmc_recvd )
		{
			rmc_recvd = FALSE;	
			
			printf("%s %s\n", gprmc.time, gprmc.date );
			
			if( gprmc.dataValid )
			{
				printf("Lat %lf Lon %lf Speed %3.2lf knots heading %3.2lf degrees\n", gprmc.latitude, gprmc.longitude, gprmc.speed, gprmc.course);
		
				if( useText )
				{
					fprintf(gps_log_file, "%s %s\n", gprmc.time, gprmc.date );
					fprintf(gps_log_file, "Lat %lf Lon %lf Speed %3.2lf knots heading %3.2lf degrees\n", gprmc.latitude, gprmc.longitude, gprmc.speed, gprmc.course);
				}
			}
			
			//Synchronise altitude display with RMC data
			// Read NDOF data from the BNO055.
			readNDOFData( &NDOF_data);
			
			// Read Quaternion data from the BNO055, convert to Euler angles
			QuatToEuler( &QtoEuler );	

			/*	Write BNO055 quaternion derived Euler data output in degrees to lthe terminal window */
			printf("Quat.   h %5.2f r %5.2f p %5.2f\n", QtoEuler.QuatToEulerYaw, QtoEuler.QuatToEulerRoll, QtoEuler.QuatToEulerPitch);

			// This function writes the NDOF data to the terminal window
			DisplayNDOFData( &NDOF_data);
			
			if( useText )
			{
				/*	Write BNO055 quaternion derived Euler data output in degrees to log file */
				fprintf(gps_log_file, "Quat.   h %5.2f r %5.2f p %5.2f\n", QtoEuler.QuatToEulerYaw, QtoEuler.QuatToEulerRoll, QtoEuler.QuatToEulerPitch);
				
				/*	Write BNO055 Euler data output in degrees to log file */
				fprintf(gps_log_file, "euler   h %5.2f r %5.2f p %5.2f\n", NDOF_data.f_euler_data_h, NDOF_data.f_euler_data_r, NDOF_data.f_euler_data_p);
	
	
				/*	Write Linear acceleration data output in m/s2 to log file */
				fprintf(gps_log_file, "linear  x %5.2fm/s2 y %5.2fm/s2 z %5.2fm/s2\n", NDOF_data.f_linear_accel_datax, NDOF_data.f_linear_accel_datay, NDOF_data.f_linear_accel_dataz);
	
	
				/*	Write Gravity sensor data output in m/s2 to log file */
				fprintf(gps_log_file, "gravity x %5.2fm/s2 y %5.2fm/s2 z %5.2fm/s2\n", NDOF_data.f_gravity_data_x, NDOF_data.f_gravity_data_y, NDOF_data.f_gravity_data_z);
	
				/* Write sensor temperature data in celcius to log file */
				fprintf(gps_log_file, "Temp. %5.2f \n", NDOF_data.temperature);
			}

			
			// MS5637
			read_MS5637();	// Read the temperature & pressure from the MS5637, calculate altitude
		
			printf("Pressure %5.1f mbar   Temperature %5.1f celcius \n", pressure, temperature);

			printf("Altitude %5.1fm  %5.1ff\n", altitude, ( altitude * 3.28084 ) );	

			if( useText )
			{
				fprintf(gps_log_file, "Pressure %5.1f mbar   Temperature %5.1f celcius \n", pressure, temperature);
	
				fprintf(gps_log_file, "Altitude %5.1fm  %5.1ff\n", altitude, ( altitude * 3.28084 ) );				
			}

			
			if( gpgga.dataValid && gprmc.dataValid )
			{
				if( useKML )
				{
					fprintf(gps_log_file, "<Placemark>\n" );
		
					fprintf(gps_log_file, "\t<name>%3.2lf</name>\n", gprmc.speed * 1.15078 );
					
					fprintf(gps_log_file, "\t<description>eh %5.2f  gh %3.2lf %3.2lf mph  x %5.2fm/s2 y %5.2fm/s2</description>\n", NDOF_data.f_euler_data_h, gprmc.course, gprmc.speed * 1.15078, NDOF_data.f_linear_accel_datax, NDOF_data.f_linear_accel_datay );

					fprintf(gps_log_file, "\t<styleUrl>#My_Icon_Style</styleUrl>\n" );
					fprintf(gps_log_file, "\t<altitudeMode>clampedToGround</altitudeMode>\n" );
					fprintf(gps_log_file, "\t<TimeStamp>\n" );
		
					fprintf(gps_log_file, "\t\t<when>%s %sZ</when> \n", gprmc.date, gprmc.time );
							
					fprintf(gps_log_file, "\t</TimeStamp>\n" );
					fprintf(gps_log_file, "\t<Point>\n" );

					fprintf(gps_log_file, "\t\t<coordinates>%2.6f, %3.6f, %5.1f</coordinates>\n", gprmc.longitude, gprmc.latitude, altitude );
					
		
					fprintf(gps_log_file, "\t</Point>\n" );
					fprintf(gps_log_file, "</Placemark>\n" );
				}	
				
				
			}
 
			
			fflush( gps_log_file );	// Write the file buffer to disk
			
		}
		
		if( gga_recvd ) //Synchronise altitude display with GGA data
		{
			gga_recvd = FALSE;	
			
			if( gpgga.dataValid )
			{
				printf("GPS Altitude %5.2lfm\n", gpgga.altitude);
				
				if( useText )
				{
					fprintf(gps_log_file, "GPS Altitude %5.2lfm\n", gpgga.altitude);
				}
			}

			

		}
		
		
			
		
		
		if( button1 )	// button1 is set when the button is pressed
		{
			
			if( useKML )
			{// Footer
				fprintf(gps_log_file, "</Document>\n" );
				fprintf(gps_log_file, "</kml>\n" );
			}			
			
			fclose( gps_log_file );
   
			closeGPSUART();
			
			//exit(0);
			
			system("shutdown now");	// Safely exit the program and shutdown the Raspberry Pi
		}
		
		if( button2 )	// Button 2 resets the base pressure to the current reading to reset the altitude
		{
			button2 = 0;
			
			read_MS5637();	// Read the temperature & pressure from the MS5637, calculate altitude
	
			setCurrentPressure();	// Set the pressure read as the base pressure, altitude read from zero
			
		}
		
	}
	while( !timesUp );	// Uncomment for timed loop set by period of alarm interrupt  
	// while( 1 );    // Uncomment for endless loop 
   
	if( useKML )
	{// Footer
		fprintf(gps_log_file, "</Document>\n" );
		fprintf(gps_log_file, "</kml>\n" );
	}			
			   
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
	button1 = 1;	// Set the flag to show button has been pressed
	
}	

// ISR for the button2 interrupt
void button2_ISR( void )
{
	button2 = 1;	// Set the flag to show button has been pressed
	
}	

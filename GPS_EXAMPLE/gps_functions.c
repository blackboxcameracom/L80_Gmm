/***************************************************************************
* Copyright (C) 2018 The BlackBoxCmaera Company Limited  GPS-PIE.COM
*
* File : gpsExample/gps_functions.c
*
* Date : 2018/08/28
*
* Revision : 1.0
*
* Usage: GPS interface functions source file
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
#include<fcntl.h>
#include<unistd.h>
#include<termios.h>
#include<string.h>
#include<signal.h>
#include<wiringPi.h>

#include<pthread.h>
#include<assert.h>
#include<stdlib.h>
#include<math.h>

#include "gps_functions.h"




//Chipset defines.  Used in the gpsReceiverSetup() routine below.
#define MT3339 0	// Define for the chipset for the L80 and PA6H.
#define MT3329 1	// Define for the chipset of Gmm-u1 and PA6B.


void signal_handler_IO(int status);   			// serial signal handler */
void sigalrm_handler(int sig);					// alarm signal interrupt handler
void sendMTKCommand(char *commandString);		// Routine to send MTK commands to the GPS receiver
void setGPSBaudRate( int selectedBaudRate );	// Routine to set the GPS receiver baud rate
void setUARTBaudRate( void );					// Routine to set the UART baud rate

void setDefaultBaudFlag( int baudRate );		// Routine to set the serial port baud rate flags	
void setDesiredBaudFlag( int baudRate );

void setSerialIOSignal( void );		// Set the serial IO signal & handler
void *serialReadThread( void *arguments ); // Serial read thread


volatile  char timesUp;	// Timer alarm signal flag

struct sigaction saio;		// serial signal structure
volatile char serialSignal;	// Serial data signal flag

int file;				// File for the UART connection
struct termios options;	// Structure which holds the attributes of the opened UART

int chipset; 			// Set to either MT3339 or MT3329 based on receiver type
int defaultBaudFlag; 	// This is the default start up baudrate of the gps receiver set as required
int desiredBaudFlag; 	// This is the baudrate we want the gps receiver to operate at
int desiredInterval;	// Data output interval in milliseconds.  Set 100 for 10Hz, 200 for 5Hz & 1000 for 1Hz
						// Used in gpsReceiverSetup() routine below.
	
char sentenceStartFlag = FALSE;	

// Data structures from Walter Dal Mut's GPS library.  Defined in nmea.h
gpgga_t gpgga;
gprmc_t gprmc;

// Flags set when data has been loaded into the data structures
char rmc_recvd = FALSE;	
char gga_recvd = FALSE;
char nmea_recvd = FALSE;	 
char rmcBuffer[255];	// buffer for GPRMC data sentence
char ggaBuffer[255];	// buffer for GPGGA data sentence
char nmeaBuffer[255];	// buffer for all other nmea data sentences	

/*void milliWaitTime( DWORD interval )
{
	time
	
	return;
}*/

/* This routine is the serial read thread */
void *serialReadThread( void *arguments ) 
{
	int index = *( (int*)arguments );
	
	serialSignal = FALSE;
	
	while(1)
	{
 		if( serialSignal )
		{
			serialSignal = FALSE;
			checkGPSData();
		}

		
		usleep(1000);
    }
}

/* This routine starts the serial read thread */
void startSerialThread( void )
{

	static pthread_t threads[1];
	static int thread_args[1];
	static int result_code;
	
	threads[0] = 0;
	thread_args[0] = 0;
	
	result_code = pthread_create(&threads[0], NULL, serialReadThread, &thread_args[0] );
	assert( !result_code );
	
	return;
}


/* This routine is used to check the UART for incoming characters.  When a 
 * character is received it is put into a buffer.  The buffer is tested for
 * the NMEA data sentence end of line condition.  When this is detected the 
 * data sentence is then parsed and the data stored into two data structures 
 * position, speed and altitude.  The routine sets flags to show which sentence 
 * has been received so it can be used. */
int checkGPSData( void )
{
	static int serialBufferIndex;		// index counter for the serial buffer
	unsigned char RX_data;				// char value for incoming serial characters
	static char serialInBuffer[255];	// buffer for incoming characters from the UART
	
	while(read(file,&RX_data,1)>0)		// if(read(file,&RX_data,1)>0)
	{
		
		if( RX_data == '$' )
		{
			sentenceStartFlag = TRUE;	// Start of the NMEA data sentence
				
			serialBufferIndex = 0;		// Reset the buffer index counter
						
		}
		
		if( sentenceStartFlag == TRUE )
		{
  
			serialInBuffer[serialBufferIndex] = RX_data;	// Store each incoming character from the NMEA data sentence
  
			serialBufferIndex++;	// Increment the buffer index counter
  
			// The 0x0D character signals the end of the NMEA sentence data
			if( RX_data == 0x0D || serialBufferIndex == 128 )	// Receiving 128 character would indicate a buffer overrun.
			{
				serialInBuffer[ serialBufferIndex ] = 0x0A;		// Insert linefeed character into the buffer rather than waiting for it to arrive
				
				serialBufferIndex++;

				serialInBuffer[ serialBufferIndex ] = 0x00;		// Terminate the string with a null

				
				/* To parse each NMEA data sentence we need to recognise which one is in the buffer.
				 * All NMEA data sentence begin $GPXXX where XXX indentifies the sentence.  Therefore
				 * we test character 3, 4 & 5 in the buffer to determine sentence has been received.
				 * Each data sentence would then be parsed according to the comma separated data it 
				 * contains */					
				if( serialInBuffer[3] == 'R' & serialInBuffer[4] == 'M' & serialInBuffer[5] == 'C' )
				{// Handle the GPRMC data sentence here
					
					memcpy( rmcBuffer, serialInBuffer, serialBufferIndex + 1 );	// Copy the data sentence from the serial in buffer
					
					/* Data parsing routines from Walter Dal Mut's GPS library.  Defined in nmea.h*/
					nmea_parse_gprmc(rmcBuffer, &gprmc);
					
					gps_convert_deg_to_dec(&(gprmc.latitude), gprmc.lat, &(gprmc.longitude), gprmc.lon);
										
					rmc_recvd = TRUE;			
				}
				else if( serialInBuffer[3] == 'G' & serialInBuffer[4] == 'G' & serialInBuffer[5] == 'A' )
				{//Handle the GPGGA data sentence here
					
					memcpy( ggaBuffer, serialInBuffer, serialBufferIndex + 1 );	// Copy the data sentence from the serial in buffer
					
					nmea_parse_gpgga(ggaBuffer, &gpgga);	// Parse the GGA data sentebce for the altitude

					gga_recvd = TRUE;										
				}	
				else	
				{//Handle all other NMEA data sentences here
					
					memcpy( nmeaBuffer, serialInBuffer, serialBufferIndex + 1 );	// Copy the data sentence from the serial in buffer

					nmea_recvd = TRUE;		
				}
				
				sentenceStartFlag = FALSE;	// Reset for the next sentence
				
				serialBufferIndex = 0;
	
			}// end of RX_data == 0x0D || serialBufferIndex == 128 )
  
		}// end of sentenceStartFlag == TRUE
			
	} // End of if(read(file,&RX_data,1)>0)

	// Return true if data sentence has been parsed so mainloop can use the data
	if( rmc_recvd || gga_recvd)
	{
		return 1;
	}
	else
	{
		return 0;
	}
				
}



/* Use the timer alarm signal to set an interval period.  The alarm signal interrupt
 * handler sets timeUp. */
void setTimerAlarmInterval( int timerInterval )
{
	// Set timer interval of 60 * X seconds, timesUp will be set to TRUE by the interrupt handler
	timesUp = FALSE;
	alarm( 60 * timerInterval );
   
	return;
}	




/* This routine opens the UART and sets baud rate to match the GPS receiver.
 * Please note that as this code is specifically written for a
 * GPS board so there is no check if a GPS is attached.
 * The MTK GPS receivers on the GPS-PIE boards only have their settings 
 * stored in RAM backed up by a super capacitor.  This means that after a 
 * few hours without power the GPS receiver will revert to the default settings
 * for baud rate, data sentences and navigation parameters.  
 * GPS receivers have a default baud rate which may or may not be desired for a particular
 * application.  The data is sent at an output frequency interval set in milliseconds.
 * Set interval to 100 for 10Hz, 200 for 5Hz & 1000 for 1Hz
 * If you want to increase the output frequency from 1Hz you will have change the 
 * baud rate to accomodate the higher data rate of NMEA data sentences.  Change the input parameters  
 * to match your receiver and application.  If your changes don't work check the baud rate is 
 * high enough as the receiver will ignore commands to set a 5Hz output rate at 4800 baud or vice versa. 
 * Receiver Type L80 or PA6H or PA6B or GMMU1
 * L80 default baud rate 9600
 * PA6H default baud rate 9600
 * PA6B default baud rate 9600 
 * Gmm-u1 default baud rate 38400 
 * Desired baud rate can be 115200, 57600, 38400, 19200, 9600 or 4800 
 * If you are experimenting and temporarily change from the default, replace default with the current
 * baud rate */
int openGPSUART( int receiverType, int defaultBaud, int desiredBaud, int interval )
{
	unsigned char RX_data;	// char value for incoming serial characters
	int	timedOut = FALSE;
	
	// Select chip set based on receiverType
	if( ( receiverType == L80 ) || ( receiverType == PA6H ) )
	{
			chipset = MT3339;
	}
	else
	{		// PA6B & GMMU1 receiver
			chipset = MT3329;
	}
	
	// Set the serial port baud rate flags	
	setDefaultBaudFlag( defaultBaud );
	setDesiredBaudFlag( desiredBaud );
	
	if( ( interval >= 100 ) && ( interval <= 1000 ) )
	{
		desiredInterval = interval;
	}
	else
	{
		desiredInterval = 1000;	// Default to 1Hz
		printf( "\nInvalid interval set.  Defaulting to 1Hz\n");
	}
	

	//Register an timer alarm interrupt handler.  Used for baud rate detection delay
	signal( SIGALRM, &sigalrm_handler );
   
	// Open the UART as a file on /dev/ttyAMA0
	if ((file = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY))<0)
	{								
      perror("UART: Failed to open the device.\n");
      return -1;
	}

	// Set up the serial IO signal
	setSerialIOSignal();
   
	// Set the attribtues of the UART connection
	tcgetattr(file, &options);
	options.c_cflag = defaultBaudFlag | CS8 | CREAD | CLOCAL;
	options.c_iflag = IGNPAR;	
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcsetattr(file, TCSANOW, &options);
	tcflush(file, TCIOFLUSH);
	fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);  // make reads non-blocking   

	// If we do not want to operate the GPS at its default baud rate we test the current baud rate. 
	// If the receiver is sending at the default rate we need to reset it to the desired rate
	if( desiredBaud != defaultBaud )
	{
		// Set timer interval of 3 seconds, timesUp will be set to TRUE by the signal handler
		timesUp = FALSE;
		alarm( 3 );
	
		// During 3 second interval test for '$' at default baud rate
		while( timesUp == FALSE )
		{
			if(read(file,&RX_data,1)>0)	// read incoming characters from the UART
			{
				if( RX_data == '$' )
				{	
					/* If we get the '$' character from the GPS at the defualt baud rate 
					 * we need to reset the GPS to the desired baud rate by sending it a series
					 * of MTK commands at the default baud rate.	*/
					setGPSBaudRate( desiredBaud );
													
					printf( "\nGPS default baud rate reset to desired\n");

					break;	// Leave the timer wait loop.				
				}
			}	
		}
		
		 
		if( timesUp )	// 
		{
			printf( "\nTimed out. Desired baud rate already set\n");	   
		}
		
		/* The GPS receiver is at the desired baud rate, now set the UART
		 * baud rate from the default to match the desired */
		setUARTBaudRate();	
		
		gpsReceiverSetup();	// Send the MTK commands to configure the GPS receiver
			
	}// end if( desiredBaud != defaultBaud )
	else
	{	// If we are operating the GPS receiver at its default baud we just configure it
		gpsReceiverSetup();	// Send the MTK commands to configure the GPS receiver
	}


	return 1;
}

// Close the UART file
void closeGPSUART( void )
{
	close(file);
	
	return;
}


 /* This is the timer alarm signal handler. */  
void sigalrm_handler(int sig)
{
	timesUp = TRUE;
}


/* This is the serial port signal handler*/
void signal_handler_IO(int status)
{
	serialSignal = TRUE;
	
}


/* This routine sets up the serial IO signal used to detect incoming 
 * characters on the serial line  */
void setSerialIOSignal( void )
{
	saio.sa_handler = signal_handler_IO;
	saio.sa_flags = 0;
	saio.sa_restorer = NULL; 
	sigaction(SIGIO,&saio,NULL); 	

	// Set the attribtues of the UART file
	fcntl(file, F_SETFL, FNDELAY);
	fcntl(file, F_SETOWN, getpid());
    fcntl(file, F_SETFL,  O_ASYNC ); 	
	
	return;
}


/* This routine is used to send MTK command via the UART to the GPS
 * receiver.  See the receiver documentation for the full list of 
 * command and the general format of each.
 * Usage example: sendMTKCommand( "$PMTK251,115200*" );
 * The commandString must include the full command syntax and end with *.
 * The checksum and terminating carriage return linefeed pair are
 * generated and added by this routine.  
 * */
void sendMTKCommand(char *commandString)
{
	char commandBuffer[64];
	char cr = 0x0D;	// Carriage return character
	char lf = 0x0A;  // Linefeed character
	unsigned char checksum = 0;
	int i = 1;
	

	
	/* Calculate the checksum of the commandSting by exclusive ORing 
	 *  characters betweeen the $ and * characters */
	while( ( commandString[ i ] != '*' ) && ( i < strlen( commandString ) ) )
	{
		checksum ^= commandString[ i ];
		
		i++;
	}	
	
	// Create the complete MTK command by adding the checksum to the commandString
	sprintf( commandBuffer, "%s%02X%c%c", commandString, checksum, cr, lf);
	
	printf( "%s", commandBuffer);	// Uncomment for debugging
	
	// Send the MTK command to the GPS receiver	via the UART			
	write(file, commandBuffer, strlen( commandBuffer ) );		

	usleep(100000);	// Delay 100 ms for the command to be applied by the GPS receiver
	//delay( 100 );
	
	return;
}	


/* Routine to set the baud rate of the GPS receiver.  
 * Once called the routine to set the baud rate of the UART
 * must also be called  
 * Please note that this code works for the MTK receivers where 
 * you cannot modify the firmware or permanent change the baudrate
 * setting in flash.  For the L80 you could replace the MTK command
 * $PMTK251,<baudrate>* with $PQBAUD,W,<baudrate>* which will change 
 * and set the new baudrate as the new default. 
 * Refer to the receiver documentation for full details */ 
void setGPSBaudRate( int selectedBaudRate )
{
	long counter;
	
	tcflush(file, TCIOFLUSH);
	
	switch( selectedBaudRate )
	{
		case 115200:
			sendMTKCommand( "$PMTK251,115200*" );
		break;

		case 57600:
			sendMTKCommand( "$PMTK251,57600*" );
		break;
		
		case 38400:
			sendMTKCommand( "$PMTK251,38400*" );
		break;
			
		case 19200:
			sendMTKCommand( "$PMTK251,19200*" );
		break;
		
		// Although supported by some receivers, 14400 baud is not by the Pi
		
		case 9600:
			sendMTKCommand( "$PMTK251,9600*" );
		break;

		case 4800:
			/* For 4800 baud the data rate will be too high with GSV & GSA data sentences
			 * turned on so these are turned off prior to selecting this baud rate */
			sendMTKCommand( "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*" );
			
			/*For 4800 baud the output interval must be 1Hz otherwise the call to change
			 * the baud rate will be ignored by the receiver */
			sendMTKCommand( "$PMTK220,1000*" );
			
			sendMTKCommand( "$PMTK251,4800*" );
		break;
		
		default:
			printf( "\n Invalid gps rate selection. No baud rate change\n");

	}
	

		
	/* This delay ensures that the MTK baud rate command is fully sent 
	 * before the Raspberry Pi UART baud rate is changed.  Otherwise
	 * the MTK command to change the receiver baud rate is lost.
	 * sleep, delay etc did not work effectively. 
	 */	
	for( counter = 1; counter < 0x000FFFFE; counter++ )
	{
		sqrt( (double)counter );

	}
	

	return;
}   



/* Routine to set the Navigation mode of the L80 GPS receiver  
 * using command 886
 * See the L80 PMTK manual page 44 - 45 */
void setNavMode( int navMode )
{

	switch( navMode )
	{
		case NORMAL_NAV_MODE:
			sendMTKCommand( "$PMTK886,0*" );
		break;

		case FITNESS_NAV_MODE:
			sendMTKCommand( "$PMTK886,1*" );
		break;
		
		case AVIATION_NAV_MODE:
			sendMTKCommand( "$PMTK886,2*" );
		break;
			
		case BALLOON_NAV_MODE:
			sendMTKCommand( "$PMTK886,4*" );
		break;

		default:
			printf( "\n Invalid Navigation mode selection. No change made.\n");

	}
	
	return;
}   



/* This routine takes a baud rate value and sets the serial port baud flag 
 * for the default receiver rate */
void setDefaultBaudFlag( int baudRate )
{										
	switch( baudRate )
	{
		case 115200:
			defaultBaudFlag = B115200;
		break;

		case 57600:
			defaultBaudFlag = B57600;
		break;
		
		case 38400:
			defaultBaudFlag = B38400;
		break;
			
		case 19200:
			defaultBaudFlag = B19200;
		break;
		
		// Although supported by some receivers, 14400 baud is not by the Pi
	
		case 9600:
			defaultBaudFlag = B9600;
		break;

		case 4800:
			defaultBaudFlag = B4800;
		break;
		
		default:
			printf( "\n Invalid selection. Defaulting to 9600 baud \n");
			defaultBaudFlag = B9600;
	}		
	
	return;
}	


/* This routine takes a baud rate value and sets the serial port baud flag 
 * to the desired baud rate we want the receiver to operate at. */
void setDesiredBaudFlag( int baudRate )
{										
	switch( baudRate )
	{
		case 115200:
			desiredBaudFlag = B115200;
		break;

		case 57600:
			desiredBaudFlag = B57600;
		break;
		
		case 38400:
			desiredBaudFlag = B38400;
		break;
			
		case 19200:
			desiredBaudFlag = B19200;
		break;
		
		// Although supported by some receivers, 14400 baud is not by the Pi
	
		case 9600:
			desiredBaudFlag = B9600;
		break;

		case 4800:
			desiredBaudFlag = B4800;
		break;
		
		default:
			printf( "\n Invalid selection. Defaulting to 9600 baud \n");
			desiredBaudFlag = B9600;
	}		
	
	return;
}	


/* This routine sets the baud rate of the already open UART file */
void setUARTBaudRate( void )
{										

	// Set the UART file options to change the baud rate to match the new
	// rate set for the receiver.
	options.c_cflag = desiredBaudFlag | CS8 | CREAD | CLOCAL;
	tcsetattr(file, TCSANOW, &options);		
	tcflush(file, TCIOFLUSH);	
	
	return;
}	


/* Routine that sends the MTK commands to configure the GPS receiver as desired.
 * Note conditional for either L80 MT3339 chipset or the 
 * Gmm-u1 and PA6B based on the MT3329 chipset.
 * Please refer to the MTK command set documentation for your receiver.  
 * */
void gpsReceiverSetup( void )
{	
	char commandBuffer[64];
	
	/* For most applications the GSV & GSA data sentences which contain satellite
	 * visibilty information are not required.  Therefore we turn off the output
	 * of these and all others except the RMC & GGA data sentences. 
	 * For update rates greater than 1Hz these should be turned off to reduce
	 * the data rate or the desired baud rate must be high enough.
	 */
	sendMTKCommand( "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*" );

	
	/* Set the position fix interval.
	 * This sets the GPS receiver's data sentence output rate.  The value 
	 * is in milliseconds with a range from 100 to 10000.  Set  100 for 10Hz,  
	 * 200 for 5Hz and 1000 for 1Hz etc.  Please note that the desired baudrate 
	 * MUST to be set to accomodate the higher data rate.  
	 * */
	 // Create the complete MTK command by adding the defined DESIRED_INTERVAL to the commandString
	sprintf( commandBuffer, "$PMTK220,%d*", desiredInterval);
	sendMTKCommand( commandBuffer );
	
	if( chipset == MT3339 )
	{
		/* Set the static navigation threshold.  L80 with MT3339 chipset.
		 * This sets the speed threshold in m/s below which the position is fixed and 
		 * the speed output from the gps receiver is fixed to zero.  Range is 0 to 2.0 m/s.
		 * 2.0 m/s = 7.2 km/h = 4.5 mph.  Set to 0.0 to turn this off.  
		 * Effectively this acts as a filter to the random walk of positions due 
		 * to the errors in the GPS system which will be seen when the receiver is 
		 * stationary.  For pedestian or very low speed marine survey applications 
		 * set to 0.0 and work up.  
		 * For road or airborne vehicles set to 1.3 for a 3.0 mph ~ 4.6 kph cut off.    
		 * */
		sendMTKCommand( "$PMTK386,1.3*" );	// MT3339 chipset command
		
		/* Set the HDOP threshold.  Not supported by Gmm-u1 or PA6B MT3329 chipset
		 * HDOP or Horizontal Dilution Of Precision is a measure of the quality of a
		 * GPS fix based on the relative positions of the satellites used in the
		 * calculation.  When HDOP is above the threshold no valid fix will be reported.
		 * For this example the threshold is set to 2.0.  1.0 is considered ideal, but 
		 * setting the threshold lower increases the time to first fix.  
		 * */
		sendMTKCommand( "$PMTK356,2.0*" );
	
		/* Turn off the $GPTXT antenna status command.  
		 * See the L80 SDK documentation.  
		 * */
		sendMTKCommand( "$PQTXT,W,0,0*" );  
		
		setNavMode( NORMAL_NAV_MODE );
	}	
	else
	{
		/* Set the static navigation threshold.  MT3329 chipset
		 * For Gmm-u1 and PA6B receivers with MT3329 chipsets only values of 
		 * 0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.5, 2.0 can be used as parameters.
		 * For road or airborne vehicles set to 1.5 for a 3.3 mph ~ 5.4 kph cut off.
		 * */ 
		sendMTKCommand( "$PMTK397,0*" );	// MT3329 chipset command
	   
		/* Enable WAAS or equivalent SBAS satellite.
		 * See the documentation.  The L80 has this enabled by default, other
		 * receivers may need to have it turned on. 
		 * */
		sendMTKCommand( "$PMTK313,1*" );
		
		/* Set the source of DGPS correction data.  
		 * See the documentation.  This message tells the receiver to use the 
		 * SBAS sattelite for DGPS. The L80 has this enabled by default, other
		 * receivers may need to have it turned on. 
		 * */
		sendMTKCommand( "$PMTK301,2*" );
	}

	return;
}

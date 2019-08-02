/***************************************************************************
* Copyright (C) 2018 The BlackBoxCmaera Company Limited  GPS-PIE.COM
*
* File : gpsExample/gps_functions.h
*
* Date : 2018/08/28
*
* Revision : 1.0
*
* Usage: GPS interface functions header file
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


#include "nmea.h"

//#define TRUE 1		//Defined in WiringPi library,  Uncomment if not using WiringPi
//#define FALSE 0

// GPS receiverType defines. 
#define L80	0		// Define to select L80 specific setup
#define PA6H 1		// Define to select PA6H specific setup
#define PA6B 2		// Define to select PA6B specific setup
#define GMMU1 3		// Define to select GMMU1 specific setup

/* The L80 navigation mode definitions for command 886.
 * See the L80 PMTK manual page 44 - 45 */
#define NORMAL_NAV_MODE		0
#define FITNESS_NAV_MODE	1
#define AVIATION_NAV_MODE	2
#define BALLOON_NAV_MODE	3


extern int openGPSUART( int receiverType, int defaultBaud, int desiredBaud, int interval );	// Opens the UART and sets baud rate to match GPS receiver
extern void closeGPSUART( void );			// Close the UART
extern void gpsReceiverSetup( void );		// Routine that sends the MTK commands to configure the GPS receiver
extern void setNavMode( int navMode );		// Routine to set the L80 Navigation mode
extern int checkGPSData( void );			// Routine that checks UART for data sentence
extern void setTimerAlarmInterval( int timerInterval );	// Set timer alarm interval

extern void startSerialThread( void );		// This routine starts the serial read thread

extern volatile  char timesUp;		// Timer signal flag
extern volatile char serialSignal;	// Serial data signal flag

// Flags set when data has been loaded into the data structures
extern char rmc_recvd;	
extern char gga_recvd;
extern char nmea_recvd;

extern char rmcBuffer[255];	// buffer for GPRMC data sentence
extern char ggaBuffer[255];	// buffer for GPGGA data sentence	
extern char nmeaBuffer[255];	// buffer for all other nmea data sentences	

extern gpgga_t gpgga;
extern gprmc_t gprmc;

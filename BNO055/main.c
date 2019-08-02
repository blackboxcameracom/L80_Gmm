/***************************************************************************
* Copyright (C) 2018 The BlackBoxCmaera Company Limited  GPS-PIE.COM
*
* File : BNO055/main.c
*
* Date : 2018/08/28
*
* Revision : 1.0
*
* Usage: main.c file for the BNO055 sensor example
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
#include "bno055_funcs.h"	// Defines data types and functions for accessing the BNO055 sensor


#define NDOF	// Define BNO055 NDOF mode else simple accelerometer

BNO055_NDOF_Data_t NDOF_data;

BNO055_ACCL_Data_t Accl_data;


int main()
{


	if( !i2cBusSetUp() )
	{
		return(0);	//Exit the program if no BNO055 is detected at start up		
	}
	
	#ifdef NDOF 
	configureBNO055();	// Either compile for 9DOF operating mode...
	#else
	configureBNO055_ACCL();	// or simple accelerometer
	#endif

	setUpMotionInterrupt();		

	setUPGPIOInterrupt();
	
	
	while( 1 )
	{

		#ifdef NDOF 
		readNDOFData( &NDOF_data);

		DisplayNDOFData( &NDOF_data);
		#else
		readACCLData( &Accl_data);
		
		DisplayACCLData( &Accl_data);
		#endif	
		

		if( IntDataWaiting )	// Check the bit indicating data waiting to be read
		{
			printf(" Int!\n");

			resetGPIOInterrupt();

			toggleStatus();
			
		}	

			
		delay( 1000 );
	}

//--
	deinitBNO055_Interface();	// End by setting the BNO055 into low power mode

	return(0);
}

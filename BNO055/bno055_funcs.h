/***************************************************************************
* Copyright (C) 2018 The BlackBoxCmaera Company Limited  GPS-PIE.COM
*
* File : BNO055/bno055_funcs.h
*
* Date : 2018/08/28
*
* Revision : 1.0
*
* Usage: Interface functions header file for the BNO055 sensor example
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


#include<wiringPi.h>
#include "bno055.h"	//This include file defines all the function and data types for the BNO055 in bno055.c

extern volatile char IntDataWaiting;	// Flag variable set in the interrupt service routine

// This structure contains the variables for the BNO055 NDOF data
struct BNO055_NDOF_Data {
		
	/* variable used to read the accel x data output as m/s2 or mg */
	float f_accel_datax;
	/* variable used to read the accel y data output as m/s2 or mg */
	float f_accel_datay;
	/* variable used to read the accel z data output as m/s2 or mg */
	float f_accel_dataz;	

	/*******************read euler converted data*******************/
	/* variable used to read the euler h data output as degree or radians*/
	float f_euler_data_h;
	/* variable used to read the euler r data output as degree or radians*/
	float f_euler_data_r;
	/* variable used to read the euler p data output as degree or radians*/
	float f_euler_data_p;

	/*********read linear acceleration converted data**********/
	/* variable used to read the linear accel x data output as m/s2*/
	float f_linear_accel_datax;
	/* variable used to read the linear accel y data output as m/s2*/
	float f_linear_accel_datay;
	/* variable used to read the linear accel z data output as m/s2*/
	float f_linear_accel_dataz;

	/********************Gravity converted data**********************/	
	/* variable used to read the gravity sensor x data output as m/s2*/
	float f_gravity_data_x;
	/* variable used to read the gravity sensor y data output as m/s2*/
	float f_gravity_data_y;
	/* variable used to read the gravity sensor z data output as m/s2*/
	float f_gravity_data_z;	
	
	/* variable used to read the temperature data output */
	float temperature;

};
typedef struct BNO055_NDOF_Data BNO055_NDOF_Data_t;


// This structure contains the variables for the BNO055 Accelerometer data
// in BNO055_OPERATION_MODE_ACCONLY
struct BNO055_ACCL_Data {
		
	/* variable used to read the accel x data output as m/s2 or mg */
	float f_accel_datax;
	/* variable used to read the accel y data output as m/s2 or mg */
	float f_accel_datay;
	/* variable used to read the accel z data output as m/s2 or mg */
	float f_accel_dataz;	
};
typedef struct BNO055_ACCL_Data BNO055_ACCL_Data_t;

// This structure contains the variables that hold the pitch, roll and yaw
// angles calculated from the quaternion data from the BNO055 NDOF data
struct BNO055_QuaternionToEuler_Data {
		
	/* variable used to hold the euler pitch angle calculated from quaternion data */
	float QuatToEulerPitch;
	/* variable used to hold the euler roll angle calculated from quaternion data */
	float QuatToEulerRoll;
	/* variable used to hold the euler yaw angle calculated from quaternion data */
	float QuatToEulerYaw;	
};
typedef struct BNO055_QuaternionToEuler_Data BNO055_QuaternionToEuler_Data_t;


extern void initBNO055_Interface( void );	// Init the i2c interface to the BNO055 

extern void deinitBNO055_Interface( void );	// Set the BNO055 to lower power 

extern int i2cBusSetUp( void );				// Set up the WiringPi i2c bus, check for the BNO055, return true if present.

extern void writeConfigToFile( void );		// Write the BNO055 9DOF configuration to the bno055_data.cfg file.

extern void configureBNO055( void );		// Set up the BNO055 in 9DOF mode.

extern void toggleStatus( void );			// Routine to toggle the status LED

extern void setUpMotionInterrupt( void );	// Routine to set up the any motion interrupt

extern void setUPGPIOInterrupt( void );		// Set up the WiringPi GPIO interrupt on the BNO055 interrupt pin

extern void resetGPIOInterrupt( void );		// Reset the GPIO interrupt

extern void readNDOFData( BNO055_NDOF_Data_t *NDOF_data );		// Read the NDOF data from the BNO055 

extern void DisplayNDOFData( BNO055_NDOF_Data_t *NDOF_data );	// Display the NDOF data read from the BNO055 

extern void configureBNO055_ACCL( void );	// Configure the BNO055 for simply accelerometer mode

extern void readACCLData( BNO055_ACCL_Data_t *Accl_data );	// Read the accelerometer data

extern void DisplayACCLData( BNO055_ACCL_Data_t *Accl_data );	// Display the accelerometer data

extern void QuatToEuler( BNO055_QuaternionToEuler_Data_t *QtoEuler_data );		// Calculate the Euler angles from BNO055 Quaternion data.

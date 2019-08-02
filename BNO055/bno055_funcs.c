/***************************************************************************
* Copyright (C) 2018 The BlackBoxCmaera Company Limited GPS-PIE.COM
*
* File : BNO055/bno055_funcs.c
*
* Date : 2018/08/28
*
* Revision : 1.0
*
* Usage: Interface functions file for the BNO055 sensor example
*
****************************************************************************
***************************************************************************
* Original Copyright 
* Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
*
* bno055_support.c
* Date: 2016/03/14
* Revision: 1.0.4 $
*
* Usage: Sensor Driver support file for BNO055 sensor
*
****************************************************************************
* License:
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
/*---------------------------------------------------------------------------*
 Includes
*---------------------------------------------------------------------------*/
#include "bno055_funcs.h"	//Defines data types and functions in this file
#include<wiringPiI2C.h>
#include<stdio.h>
#include<stdint.h>
#include<string.h>

#include<fcntl.h>

#include<math.h>

/*----------------------------------------------------------------------------*
 *  The following APIs are used for reading and writing of
 *	sensor data using I2C communication. From bno055_support.c
 * 
*----------------------------------------------------------------------------*/
#define BNO055_API
//----------------------------------------------------------------------------
#ifdef	BNO055_API
#define	BNO055_I2C_BUS_WRITE_ARRAY_INDEX	((u8)1)

/*	\Brief: The API is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *   will data is going to be read
 *	\param reg_data : This data read from the sensor,
 *   which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

/*	\Brief: The API is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *   will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *	will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);


/*
 * \Brief: I2C init routine
*/
s8 I2C_routine(void);


/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BNO055_delay_msek(u32 msek);

#endif
/********************End of I2C APIs declarations***********************/

/* Stub ISR routine
 */
void int1_ISR( void );


/*----------------------------------------------------------------------------*
 *  struct bno055_t parameters can be accessed by using BNO055
 *	BNO055_t having the following parameters
 *	Bus write function pointer: BNO055_WR_FUNC_PTR
 *	Bus read function pointer: BNO055_RD_FUNC_PTR
 *	Burst read function pointer: BNO055_BRD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
*---------------------------------------------------------------------------*/
struct bno055_t bno055;

//======================================================================

	/* variable used to set the power mode of the sensor*/
	u8 power_mode = BNO055_INIT_VALUE;

//======================================================================

#define GPIO_INT	2	// Wiring pin 2, physical pin 13, connected to int pin

#define STATUS_LED	25	// Wiring pin 25 connected to status LED

#define BUTTON1		21	// Wiring pin 21 connection for button 1
#define BUTTON2		22	// Wiring pin 22 connection for button 2

//======================================================================
int fd;	// file descriptor for the i2c device used by WiringPi i2c routines
volatile char IntDataWaiting;	// Flag variable set in the interrupt service routine

// Structures which hold the sensor configuration data
static struct bno055_gyro_offset_t 	gyro_config_data;
static struct bno055_accel_offset_t 	accel_config_data; 
static struct bno055_mag_offset_t		mag_config_data;
FILE *config_file;
int bytesToRead;
//======================================================================


/* Set up the BNO055 in ACCL mode. */
void configureBNO055_ACCL( void )
{

	/* Select raw accelerometer data mode.  See pg 22 */	
	bno055_set_operation_mode(BNO055_OPERATION_MODE_ACCONLY);	
	
	
	return;
}


/* This routine reads the raw accelerometer data in mg from the BNO055 
 * and converts the values to floats */
void readACCLData( BNO055_ACCL_Data_t *Accl_data )
{
	
	// Convert raw accelerometer data to floating point values
	bno055_convert_float_accel_x_mg(&Accl_data->f_accel_datax);
	bno055_convert_float_accel_y_mg(&Accl_data->f_accel_datay);
	bno055_convert_float_accel_z_mg(&Accl_data->f_accel_dataz);
	
	Accl_data->f_accel_datax /= 1000;	//Convert mg to g
	Accl_data->f_accel_datay /= 1000;
	Accl_data->f_accel_dataz /= 1000;
	
	return;
}	


/* This routine displays the raw accelerometer data read from the BNO055 */
void DisplayACCLData( BNO055_ACCL_Data_t *Accl_data )
{
	printf("x %3.1fg y %3.1fg z %3.1fg\n", Accl_data->f_accel_datax, Accl_data->f_accel_datay, Accl_data->f_accel_dataz);	
	
	return;
}	


/* Calculate the Euler angles from BNO055 Quaternion data. Based on the example source code from
 * https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_Code_2 
 * For reference see also http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/
 * The following changes were made from the example code:  The calculations for roll and pitch were swapped
 * so the results matched the axes of the Euler output of the BNO055.  
 * The sign of the roll and pitch values are reversed to match the Euler output of the BNO055.  
 * */
void QuatToEuler( BNO055_QuaternionToEuler_Data_t *QtoEuler_data )	
{
	/* variable used to read the quaternion w data */
	s16 quaternion_data_w = 0;
	/* variable used to read the quaternion x data */
	s16 quaternion_data_x = 0;
	/* variable used to read the quaternion y data */
	s16 quaternion_data_y = 0;
	/* variable used to read the quaternion z data */
	s16 quaternion_data_z = 0;

	double quaternion_double_w;	// Holds the raw data converted for calculation
	double quaternion_double_x;
	double quaternion_double_y;
	double quaternion_double_z;
	
	
	double roll;	// Used in the calculations below
	double pitch;
	double yaw;


	// Read the raw quaternion data from the BNO055
	bno055_read_quaternion_w(&quaternion_data_w);
	bno055_read_quaternion_x(&quaternion_data_x);
	bno055_read_quaternion_y(&quaternion_data_y);
	bno055_read_quaternion_z(&quaternion_data_z);
	
	/* Convert the raw quaternion 16 bit data from the BNO055 to floating point numbers.	
	 * From these values you can use any algorithm you like to produce a 3D spatial rotation.
	 */ 
	quaternion_double_w = (float)quaternion_data_w / 16384.0;
	quaternion_double_x = (float)quaternion_data_x / 16384.0;
	quaternion_double_y = (float)quaternion_data_y / 16384.0;
	quaternion_double_z = (float)quaternion_data_z / 16384.0;
	
	
	

	/* Create Roll Pitch Yaw Angles from Quaternions */	
	
	// pitch (x-axis rotation) was roll in original example code
	double sinr_cosp = +2.0 * (quaternion_double_w * quaternion_double_x + quaternion_double_y * quaternion_double_z);
	double cosr_cosp = +1.0 - 2.0 * (quaternion_double_x * quaternion_double_x + quaternion_double_y * quaternion_double_y);
	pitch = atan2(sinr_cosp, cosr_cosp);	// was roll

	// roll (y-axis rotation) was pitch in original example code
	double sinp = +2.0 * (quaternion_double_w * quaternion_double_y - quaternion_double_z * quaternion_double_x);
	if (fabs(sinp) >= 1)
		roll = copysign(M_PI / 2, sinp); // use 90 degrees if out of range // was pitch
	else
		roll = asin(sinp);	// was pitch

	// yaw (z-axis rotation)
	double siny_cosp = +2.0 * (quaternion_double_w * quaternion_double_z + quaternion_double_x * quaternion_double_y);
	double cosy_cosp = +1.0 - 2.0 * (quaternion_double_y * quaternion_double_y + quaternion_double_z * quaternion_double_z);  
	yaw = atan2(siny_cosp, cosy_cosp);


	/*  Convert Radians to Degrees, pitch and roll signs are reversed to match BNO055 output */
	QtoEuler_data->QuatToEulerRoll  = (float)roll * 57.2958;
	QtoEuler_data->QuatToEulerPitch = (float)pitch * 57.2958;
	QtoEuler_data->QuatToEulerYaw   = (float)yaw * 57.2958;	
	
	// Convert +180 to -180 range given by the calculation above to 0 to 360 range of the BNO055 
	if( QtoEuler_data->QuatToEulerYaw < 0 )
	{
		QtoEuler_data->QuatToEulerYaw = -1 * QtoEuler_data->QuatToEulerYaw;
	}
	else
	{
		QtoEuler_data->QuatToEulerYaw = 360 - QtoEuler_data->QuatToEulerYaw;
	}
	
	
	
	return;
}



/* This routine reads the NDOF data from the BNO055 */
void readNDOFData( BNO055_NDOF_Data_t *NDOF_data )
{
	/*	API used to read Euler data output as float  - degree */
	bno055_convert_float_euler_h_deg(&NDOF_data->f_euler_data_h);
	bno055_convert_float_euler_r_deg(&NDOF_data->f_euler_data_r);
	bno055_convert_float_euler_p_deg(&NDOF_data->f_euler_data_p);


	/*	API used to read Linear acceleration data output as m/s2 */
	bno055_convert_float_linear_accel_x_msq(&NDOF_data->f_linear_accel_datax);
	bno055_convert_float_linear_accel_y_msq(&NDOF_data->f_linear_accel_datay);
	bno055_convert_float_linear_accel_z_msq(&NDOF_data->f_linear_accel_dataz);

	/*	API used to read Gravity sensor data output as m/s2 */
	bno055_convert_gravity_float_x_msq(&NDOF_data->f_gravity_data_x);
	bno055_convert_gravity_float_y_msq(&NDOF_data->f_gravity_data_y);
	bno055_convert_gravity_float_z_msq(&NDOF_data->f_gravity_data_z);

	// Read sensor temperature data in celcius
	bno055_convert_float_temp_celsius(&NDOF_data->temperature);

	return;
}


/* This routine displays the NDOF data read from the BNO055 */
void DisplayNDOFData( BNO055_NDOF_Data_t *NDOF_data )
{
	/*	Display Euler data output in degrees */
	printf("euler   h %5.2f r %5.2f p %5.2f\n", NDOF_data->f_euler_data_h, NDOF_data->f_euler_data_r, NDOF_data->f_euler_data_p);


	/*	Display Linear acceleration data output in m/s2 */
	printf("linear  x %5.2fm/s2 y %5.2fm/s2 z %5.2fm/s2\n", NDOF_data->f_linear_accel_datax, NDOF_data->f_linear_accel_datay, NDOF_data->f_linear_accel_dataz);


	/*	Display Gravity sensor data output in m/s2 */
	printf("gravity x %5.2fm/s2 y %5.2fm/s2 z %5.2fm/s2\n", NDOF_data->f_gravity_data_x, NDOF_data->f_gravity_data_y, NDOF_data->f_gravity_data_z);

	// Display sensor temperature data in celcius
	printf("Temp. %5.2f \n", NDOF_data->temperature);
	

	return;
}

/* Write the BNO055 9DOF configuration to the bno055_data.cfg file. */
void writeConfigToFile( void )
{
	config_file = fopen( "bno055_data.cfg", "wb" );		// Open the config file to overwrite previous data
	
	if( config_file != NULL )
	{
		// BNO055 registers should be read in CONFIG_MODE 
		bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);	
		delay( 500 );	//Delay to allow sensor to process the command
		
		// Read the sensor config data from the registers
		bno055_read_accel_offset( &accel_config_data );
		bno055_read_gyro_offset( &gyro_config_data );
		bno055_read_mag_offset( &mag_config_data);
	
		bytesToRead = sizeof( accel_config_data );
		fwrite( &accel_config_data, 1, bytesToRead, config_file );
	
		bytesToRead = sizeof( gyro_config_data );
		fwrite( &gyro_config_data, 1, bytesToRead, config_file );
		
		bytesToRead = sizeof( mag_config_data );
		fwrite( &mag_config_data, 1, bytesToRead, config_file );		
		
		fclose( config_file );
		
		printf("Data written to file\n");
		
		bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);	
	}
	else
	{
		printf("No file to write to\n"); // 	
	}
	
	return;
}



/* Set up the BNO055 in 9DOF mode.  Previous configuration is read from a file. */
void configureBNO055( void )
{
	
	unsigned char sys_reg, acc_reg, mag_reg, gyro_reg;
	
	// BNO055 starts in CONFIG_MODE only after power on reset so set here in case not power on
	bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);	
	delay( 500 );	//Delay to allow sensor to process the command

	/* The configuration of the sensor is stored in a binary file that is 
	 * written to the disk after each successful self configuration.  This
	 * file is opened here and the contents read into the BNO055 variables.	
	 * */	 
	config_file = fopen( "bno055_data.cfg", "rb" );
	
	if( config_file != NULL )	// If the file was opened successfully read in the data.
	{
		bytesToRead = sizeof( accel_config_data );
		fread( &accel_config_data, 1, bytesToRead, config_file );

		bytesToRead = sizeof( gyro_config_data );
		fread( &gyro_config_data, 1, bytesToRead, config_file );
		
		bytesToRead = sizeof( mag_config_data );
		fread( &mag_config_data, 1, bytesToRead, config_file );		
		
		fclose( config_file );
		
		printf("Data read from file\n");
	}
	else
	{
		// If there is no config file set config data to all zeros.
		bytesToRead = sizeof( accel_config_data );
		memset( &accel_config_data, 0, bytesToRead );

		bytesToRead = sizeof( gyro_config_data );
		memset( &gyro_config_data, 0, bytesToRead );
		
		bytesToRead = sizeof( mag_config_data );
		memset( &mag_config_data, 0, bytesToRead );		
		
		printf("No file to read from\n"); // 	
	}

	// Initialise with data read from file or default 0x00
	bno055_write_accel_offset( &accel_config_data );
	bno055_write_gyro_offset( &gyro_config_data );
	bno055_write_mag_offset( &mag_config_data);

	/* Select fusion mode with 9 degrees of freedom.  See pg 23 */	
	bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);	
	delay( 2000 );	//Delay 2 seconds to allow sensor to start auto calibration

	sys_reg = 0;	// Clear the sys_reg value 
	acc_reg = 0;	// Clear the acc_reg value 
	mag_reg = 0;	// Clear the mag_reg value 
	gyro_reg = 0;	// Clear the gyro_reg value 

	/* This loop reads the calibration state registers until they show the BNO055 
	 * auto calibated the magnetometer, accelerometer and gyro.  When the process
	 * is complete the sys_reg will be 3.  Please see pg 48 for a description 
	 * of the process and how the sensor needs to be physically moved. 
	 * 
	 * You can get a quicker start up if you only look at the system staus register, but
	 * the magnetometer may not be calibrated.
	 * while( sys_reg != 0x03 )
	 * */
	
	while( (sys_reg != 0x03) || (acc_reg != 0x03) || (mag_reg != 0x03) || (gyro_reg != 0x03) )	
	{
		bno055_get_sys_calib_stat(&sys_reg);
		bno055_get_accel_calib_stat(&acc_reg);
		bno055_get_mag_calib_stat(&mag_reg);
		bno055_get_gyro_calib_stat(&gyro_reg);
		printf("sys %02X   acc %02X   mag %02X   gyro %02X\n", sys_reg, acc_reg, mag_reg, gyro_reg);

		/*if( sys_reg == 0x03 )
		{
			break;
			printf("Timed out");
		}*/
	
		toggleStatus();	// Flash the status LED while the sensor calibrates
		delay( 500 );
	}

	digitalWrite( STATUS_LED, LOW );	// Set status LED off

	/* When the sensors in the BNO055 have been auto calibrated we write 
	 * the calibration data to a file so the data can be used to give us 
	 * a faster start up next time. 
	 * */
	 writeConfigToFile();	// Write the BNO055 9DOF configuration to the bno055_data.cfg file.
	 

	//---
	
	return;
}	



/* Set up the WiringPi i2c bus, check for the BNO055, return true if present. */
int i2cBusSetUp( void )
{	
	const char *i2cDevice;
	unsigned char id_reg;	// Holds the id reg. value read from the BNO055
	unsigned short sw_rev;	// Holds the software revision read from the BNO055

	//--- GPIO pin set up
	wiringPiSetup();

	pinMode( GPIO_INT, INPUT );				// Interrupt pins are set as inputs
	pullUpDnControl( GPIO_INT, PUD_DOWN );	// with pull downs
	
	pinMode( STATUS_LED, OUTPUT );		// Set up the status LED pin
	digitalWrite( STATUS_LED, LOW );	// Set initial state to low, LED off

		
	i2cDevice = "/dev/i2c-3";

	fd = wiringPiI2CSetupInterface(i2cDevice,0x28);	// Initialise the i2c interface
	//--- 

	initBNO055_Interface();	// Init the BNO055 API i2c interface.  This must come before all calls to API functions. 

	bno055_read_chip_id( &id_reg );	// This should return the BNO055 chip id value 0xA0 
		

	if( id_reg == 0xA0 )
	{
		bno055_read_sw_rev_id( &sw_rev );	// This will return the BNO055 sofware revision
		printf("BNO055 detected.  ID register value is %02X.  Software revision %04X\n", id_reg, sw_rev);
		return(1); 	
	}
	else
	{
		printf("No BNO055 detected\n"); 
		return(0);	// return false if no BNO055 is detected at start up		
	}
	
	// Initialise the BNO055 by performing a system reset
	bno055_set_sys_rst(BNO055_BIT_ENABLE);	// System reset
	delay( 2000 );	// Ensure that the BNO055 has time to reset
	
	bno055_set_clk_src(BNO055_BIT_ENABLE);	// Set external oscillator	
	

}

/* This routine is an example of how to set up the BNO055 any motion interrupt */
void setUpMotionInterrupt( void )
{	
	// Enable the any motion interrupt on X and Y axis.  ISR is set up and linked below after sensor config.
	bno055_set_accel_any_motion_no_motion_axis_enable(BNO055_ACCEL_ANY_MOTION_NO_MOTION_X_AXIS, BNO055_BIT_ENABLE);
	bno055_set_accel_any_motion_no_motion_axis_enable(BNO055_ACCEL_ANY_MOTION_NO_MOTION_Y_AXIS, BNO055_BIT_ENABLE);

	// Set the duration that the acceleration has to be above the threshold
	bno055_set_accel_any_motion_durn(0x01);
	
	// The motion threshold
	bno055_set_accel_any_motion_thres(0x01);
	
	// Set the any motion interrupt enable
	bno055_set_intr_accel_any_motion(BNO055_BIT_ENABLE);
	
	// Set the any motion interrupt mask.  This connects the hardware pin
	bno055_set_intr_mask_accel_any_motion(BNO055_BIT_ENABLE);

	return;
}


/* This routine sets up the interrupt service routine on physical pin 13 
 * connected to the BNO055's interrupt pin */
void setUPGPIOInterrupt( void )
{
	/* Set up the ISR for the any motion interrupt set up above.
	 * This attaches the ISR to the pin so the BNO055 can signal
	 * the program */
	wiringPiISR(GPIO_INT, INT_EDGE_RISING, &int1_ISR);
	
	resetGPIOInterrupt();	// Clear the flag set in the interrupt service routine
							// Reset the interrupt by setting SYS_TRIGGER reg bit 6
			
	return;
}	



/* Reset the GPIO interrupt */
void resetGPIOInterrupt( void )
{
	IntDataWaiting = 0;
	
	bno055_set_intr_rst(BNO055_BIT_ENABLE);	// Reset the interrupt by setting SYS_TRIGGER reg bit 6
	
	return;
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


// ISR for the interrupt
void int1_ISR( void )
{
	IntDataWaiting = 1;	// Set the flag to show data waiting to be read
	
}	


//======================================================================
// This routine was taken out of the s32 bno055_data_readout_template(void)
// routine below.  It initialises the BNO055 API code with the i2c routine 
// pointers	
void initBNO055_Interface( void )
{	
/*---------------------------------------------------------------------------*
 *********************** START INITIALIZATION ************************
 *--------------------------------------------------------------------------*/
 #ifdef	BNO055_API
/*	Based on the user need configure I2C interface.
 *	It is example code to explain how to use the bno055 API*/
	I2C_routine();
 #endif
/*--------------------------------------------------------------------------*
 *  This API used to assign the value/reference of
 *	the following parameters
 *	I2C address
 *	Bus Write
 *	Bus read
 *	Chip id
 *	Page id
 *	Accel revision id
 *	Mag revision id
 *	Gyro revision id
 *	Boot loader revision id
 *	Software revision id
 *-------------------------------------------------------------------------*/
	bno055_init(&bno055);

/*	For initializing the BNO sensor it is required to the operation mode
	of the sensor as NORMAL
	Normal mode can set from the register
	Page - page0
	register - 0x3E
	bit positions - 0 and 1*/
	power_mode = BNO055_POWER_MODE_NORMAL;
	/* set the power mode as NORMAL*/
	bno055_set_power_mode(power_mode);
/*----------------------------------------------------------------*
************************* END INITIALIZATION *************************
*-----------------------------------------------------------------*/
	return;
}


// This routine was taken out of the s32 bno055_data_readout_template(void)
// routine below.  It deinitialises the BNO055 by setting it into 
// suspend mode
void deinitBNO055_Interface( void )
{
/************************* START DE-INITIALIZATION ***********************
*-------------------------------------------------------------------------*/
/*	For de - initializing the BNO sensor it is required
	to the operation mode of the sensor as SUSPEND
	Suspend mode can set from the register
	Page - page0
	register - 0x3E
	bit positions - 0 and 1*/
	power_mode = BNO055_POWER_MODE_SUSPEND;
	/* set the power mode as SUSPEND*/
	bno055_set_power_mode(power_mode);

/*---------------------------------------------------------------------*
************************* END DE-INITIALIZATION **********************
*---------------------------------------------------------------------*/
	return;
}	


#ifdef	BNO055_API
/*--------------------------------------------------------------------------*
*	The following API is used to map the I2C bus read, write, delay and
*	device address with global structure bno055_t
*-------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------*
 *  By using bno055 the following structure parameter can be accessed
 *	Bus write function pointer: BNO055_WR_FUNC_PTR
 *	Bus read function pointer: BNO055_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *--------------------------------------------------------------------------*/
s8 I2C_routine(void)
{
	bno055.bus_write = BNO055_I2C_bus_write;
	bno055.bus_read = BNO055_I2C_bus_read;
	bno055.delay_msec = BNO055_delay_msek;
	bno055.dev_addr = BNO055_I2C_ADDR1;

	return BNO055_INIT_VALUE;
}

/************** I2C buffer length******/

#define	I2C_BUFFER_LEN 8
#define I2C0 5
/*-------------------------------------------------------------------*
*
*	This is a sample code for read and write the data by using I2C
*	Use either I2C  based on your need
*	The device address defined in the bno055.h file
*
*--------------------------------------------------------------------*/

/*	\Brief: The API is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *   will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 BNO055_iERROR = BNO055_INIT_VALUE;
	u8 array[I2C_BUFFER_LEN];
	u8 stringpos = BNO055_INIT_VALUE;
	
	u8 i2c_addr;	//added for the index in the loop below

//	array[BNO055_INIT_VALUE] = reg_addr;  //Not sure why?
/*	for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
	{	array[stringpos + BNO055_I2C_BUS_WRITE_ARRAY_INDEX] =
			*(reg_data + stringpos);
	}*/
	
	for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
	{	
		array[stringpos] = *(reg_data + stringpos);	// BNO055_I2C_BUS_WRITE_ARRAY_INDEX
	}
	
	
	BNO055_iERROR = BNO055_SUCCESS;
	
	i2c_addr = reg_addr;
	
	for(stringpos = 0; stringpos < cnt; stringpos++)
	{
		wiringPiI2CWriteReg8(fd, i2c_addr, array[ stringpos ] );	// Write the  register
		i2c_addr++;
	}
	
	
	/*
	* Please take the below APIs as your reference for
	* write the data using I2C communication
	* "BNO055_iERROR = I2C_WRITE_STRING(DEV_ADDR, ARRAY, CNT+1)"
	* add your I2C write APIs here
	* BNO055_iERROR is an return value of I2C read API
	* Please select your valid return value
	* In the driver BNO055_SUCCESS defined as 0
    * and FAILURE defined as -1
	* Note :
	* This is a full duplex operation,
	* The first read data is discarded, for that extra write operation
	* have to be initiated. For that cnt+1 operation done
	* in the I2C write string function
	* For more information please refer data sheet SPI communication:
	*/
	return (s8)BNO055_iERROR;
}

 /*	\Brief: The API is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *  will data is going to be read
 *	\param reg_data : This data read from the sensor,
 *   which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 BNO055_iERROR = BNO055_INIT_VALUE;
	u8 array[I2C_BUFFER_LEN] = {BNO055_INIT_VALUE};
	u8 stringpos = BNO055_INIT_VALUE;

	u8 i2c_addr;	//added for the index in the loop below

	array[BNO055_INIT_VALUE] = reg_addr;

	/* Please take the below API as your reference
	 * for read the data using I2C communication
	 * add your I2C read API here.
	 * "BNO055_iERROR = I2C_WRITE_READ_STRING(DEV_ADDR,
	 * ARRAY, ARRAY, 1, CNT)"
	 * BNO055_iERROR is an return value of SPI write API
	 * Please select your valid return value
     * In the driver BNO055_SUCCESS defined as 0
     * and FAILURE defined as -1
	 */
	
	i2c_addr = reg_addr;
	
	for(stringpos = 0; stringpos < cnt; stringpos++)
	{
		
		array[ stringpos ] = wiringPiI2CReadReg8(fd, i2c_addr);	// Read the  register
		i2c_addr++;
	}
		
	 
	BNO055_iERROR = BNO055_SUCCESS;
	 
	for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
		*(reg_data + stringpos) = array[stringpos];
	return (s8)BNO055_iERROR;
}

/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BNO055_delay_msek(u32 msek)
{
	/*Here you can write your own delay routine*/
	delay( msek );
}

#endif

/***************************************************************************
* Raspberry Pi Interface Copyright (C) 2018 The BlackBoxCmaera Company Limited  GPS-PIE.COM
*
* File : MS5637/ms5637_funcs.c
*
* Date : 2018/08/28
*
* Revision : 1.0
*
* Usage: Interface Functions for the MS5637 pressure / temperature sensor
*
****************************************************************************
* Original Copyright notice
* file ms5637.c
*
* brief MS5637 Temperature sensor driver source file
*
* Copyright (c) 2016 Measurement Specialties. All rights reserved.
*
* For details on programming, refer to ms5637 datasheet :
* http://www.meas-spec.com/downloads/MS5637-02BA03.pdf
*
*
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
#include<stdint.h>
#include<stdbool.h>
#include<math.h>

#include<wiringPi.h>
#include<wiringPiI2C.h>

#include<unistd.h>
#include<fcntl.h>
#include<sys/ioctl.h>
#include<linux/i2c.h>
#include<linux/i2c-dev.h>
#include "ms5637_funcs.h"


#ifdef __cplusplus
extern "C" {
#endif

// Constants

// MS5637 device address
#define MS5637_ADDR		0x76 //0b1110110 write address

// MS5637 device commands
#define MS5637_RESET_COMMAND					0x1E
#define MS5637_START_PRESSURE_ADC_CONVERSION			0x40
#define MS5637_START_TEMPERATURE_ADC_CONVERSION			0x50
#define MS5637_READ_ADC						0x00

#define MS5637_CONVERSION_OSR_MASK				0x0F

/* The following constants define the conversion delays for each
 * resolution.  See page 3 of the datasheet for the actual times.
 * In testing it was found that the 8192 delay was marginally not 
 * long enough.  If you find that you are getting the message
 * "write command failed" IO error then extend the delay period 
 * for the resolution you are using.  The 
 */
#define MS5637_CONVERSION_TIME_OSR_256								1000
#define MS5637_CONVERSION_TIME_OSR_512								2000
#define MS5637_CONVERSION_TIME_OSR_1024								3000
#define MS5637_CONVERSION_TIME_OSR_2048								5000
#define MS5637_CONVERSION_TIME_OSR_4096								9000
#define MS5637_CONVERSION_TIME_OSR_8192								20000	// 17000

// MS5637 commands
#define MS5637_PROM_ADDRESS_READ_ADDRESS_0							0xA0
#define MS5637_PROM_ADDRESS_READ_ADDRESS_1							0xA2
#define MS5637_PROM_ADDRESS_READ_ADDRESS_2							0xA4
#define MS5637_PROM_ADDRESS_READ_ADDRESS_3							0xA6
#define MS5637_PROM_ADDRESS_READ_ADDRESS_4							0xA8
#define MS5637_PROM_ADDRESS_READ_ADDRESS_5							0xAA
#define MS5637_PROM_ADDRESS_READ_ADDRESS_6							0xAC
#define MS5637_PROM_ADDRESS_READ_ADDRESS_7							0xAE

// Coefficients indexes for temperature and pressure computation
#define MS5637_CRC_INDEX											7
#define MS5637_PRESSURE_SENSITIVITY_INDEX							1 
#define MS5637_PRESSURE_OFFSET_INDEX								2
#define MS5637_TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX				3
#define MS5637_TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX					4
#define MS5637_REFERENCE_TEMPERATURE_INDEX							5
#define MS5637_TEMP_COEFF_OF_TEMPERATURE_INDEX						6
#define MS5637_COEFFICIENT_NUMBERS									8   

#define I2C_DEVICE	"/dev/i2c-3"	// The linux device file for the i2c 

// Static functions
static enum ms5637_status ms5637_write_command(uint8_t);
static enum ms5637_status ms5637_read_eeprom_coeff(uint8_t, uint16_t*);
static enum ms5637_status ms5637_read_eeprom(void);
static enum ms5637_status ms5637_conversion_and_read_adc( uint8_t, uint32_t *);
static bool ms5637_crc_check (uint16_t *n_prom, uint8_t crc);

enum ms5637_resolution_osr ms5637_resolution_osr;
static uint16_t eeprom_coeff[MS5637_COEFFICIENT_NUMBERS];
static uint32_t conversion_time[6] = {	MS5637_CONVERSION_TIME_OSR_256,
										MS5637_CONVERSION_TIME_OSR_512,
										MS5637_CONVERSION_TIME_OSR_1024,
										MS5637_CONVERSION_TIME_OSR_2048,
										MS5637_CONVERSION_TIME_OSR_4096,
										MS5637_CONVERSION_TIME_OSR_8192};

// Default value to ensure coefficients are read before converting temperature
bool ms5637_coeff_read = false;

int ms5637_file;	// file for the MS5637 device i2c


float temperature;
float pressure;
float altitude;	
float base_pressure;
double alt_coeff = 1 / 5.255;	// Used to convert pressure to altitude in metres
	

/* This routine allows the setting of the base pressure which is used to 
 * calculate the altitude in metres */
void setBasePressure( float basePressureSetting )
{
	base_pressure = basePressureSetting;
	
	return;
}


/* This routine sets the base pressure to the current pressure.
 * This sets the altitude to zero at the current pressure & altitude. */
void setCurrentPressure( void )
{
	base_pressure = pressure;
	
	return;
}



/* This routine is called to read the pressure and temperature from the
 * MS5637 sensor.  The altitude is then calculated from the pressure
 * measurement and the base pressure setting. */
void read_MS5637( void )
{

	ms5637_read_temperature_and_pressure( &temperature, &pressure);
		
	altitude = 44330 * ( 1 - pow( (double)(pressure / base_pressure), alt_coeff)  );	
	
	return;
}


/**
 * \brief Configures the I2C master to be used with the MS5637 device.
 * Note we do not use WiringPi for this sensor as we need multi-byte reads
 */
bool ms5637_init(void)
{
	const char *i2cDevice;

	/* Set conversion resolution.  8192 is the highest resolution 
	 * and therefore takes the longest conversion time.
	 * 256 is the lowest resolution and takes the shortest conversion time
	 * See page 4 of the datasheet for the resolution in
	 * temp. and pressure for the settingss.  
	 * Page 3 give conversion times.
	 */
	 ms5637_resolution_osr = ms5637_resolution_osr_8192;  // Approximately .0.1m resolution
	// ms5637_resolution_osr = ms5637_resolution_osr_256;	// Approximately 0.3m resolution
 
   /* Initialize and enable device with config. */  
   if((ms5637_file=open(I2C_DEVICE, O_RDWR)) < 0){
      perror("failed to open the ms5637 i2c bus\n");
      return( false );
   }
  
   if(ioctl(ms5637_file, I2C_SLAVE, MS5637_ADDR) < 0){
      perror("Failed to connect to the ms5637 sensor\n");
      return( false );
   }
   
   ms5637_reset();
	
	return( true );
}


	
/**
 * \brief Reset the MS5637 device
 *
 * \return ms5637_status : status of MS5637
 *       - ms5637_status_ok : I2C transfer completed successfully
 *       - ms5637_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms5637_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum ms5637_status  ms5637_reset(void)
{
	return ms5637_write_command(MS5637_RESET_COMMAND);
}

/**
 * \brief Set  ADC resolution.
 *
 * \param[in] ms5637_resolution_osr : Resolution requested
 *
 */
void ms5637_set_resolution(enum ms5637_resolution_osr res)
{
	ms5637_resolution_osr = res;
	return;
}

/**
 * \brief Writes the MS5637 8-bits command with the value passed
 *
 * \param[in] uint8_t : Command value to be written.
 *
 * \return ms5637_status : status of MS5637
 *       - ms5637_status_ok : I2C transfer completed successfully
 *       - ms5637_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms5637_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum ms5637_status ms5637_write_command( uint8_t cmd)
{
	uint8_t data[1];
		
	data[0] = cmd;
		
	/* Do the transfer */
	if( write( ms5637_file, data, 1) != 1 )
	{
		perror("write command failed\n");	// Debug comment
		return ms5637_status_i2c_transfer_error;
	} 


	return ms5637_status_ok;
}

/**
 * \brief Reads the ms5637 EEPROM coefficient stored at address provided.
 *
 * \param[in] uint8_t : Address of coefficient in EEPROM
 * \param[out] uint16_t* : Value read in EEPROM
 *
 * \return ms5637_status : status of MS5637
 *       - ms5637_status_ok : I2C transfer completed successfully
 *       - ms5637_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms5637_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - ms5637_status_crc_error : CRC check error on the coefficients
 */
enum ms5637_status ms5637_read_eeprom_coeff(uint8_t command, uint16_t *coeff)
{
	enum ms5637_status status;
	uint8_t buffer[2];
	int i;
	
	buffer[0] = 0;
	buffer[1] = 0;	

	/* Read data */

	// Send the PROM read command
	status = ms5637_write_command(command);
	if(status != ms5637_status_ok)
		return status;

	if( read( ms5637_file, buffer, 2 ) != 2 )
	{
		perror("EEPROM buffer read failed\n");
		return ms5637_status_i2c_transfer_error;
	}
	   
	
	*coeff = (buffer[0] << 8) | buffer[1];
    
    if (*coeff == 0)
    {
        return ms5637_status_i2c_transfer_error;
	}
	
	return ms5637_status_ok;	
}

/**
 * \brief Reads the ms5637 EEPROM coefficients to store them for computation.
 *
 * \return ms5637_status : status of MS5637
 *       - ms5637_status_ok : I2C transfer completed successfully
 *       - ms5637_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms5637_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - ms5637_status_crc_error : CRC check error on the coefficients
 */
enum ms5637_status ms5637_read_eeprom(void)
{
	enum ms5637_status status;
	uint8_t i;
	
	for( i=0 ; i< MS5637_COEFFICIENT_NUMBERS - 1; i++)
	{
		status = ms5637_read_eeprom_coeff( MS5637_PROM_ADDRESS_READ_ADDRESS_0 + i*2, eeprom_coeff+i);
		if(status != ms5637_status_ok)
			return status;
	}

	if( !ms5637_crc_check( eeprom_coeff, eeprom_coeff[MS5637_CRC_INDEX] & 0x000F ) )
		return ms5637_status_crc_error;
	
	ms5637_coeff_read = true;
	
	return ms5637_status_ok;
}


/**
 * \brief CRC check
 *
 * \param[in] uint16_t *: List of EEPROM coefficients
 * \param[in] uint8_t : crc to compare with
 *
 * \return bool : TRUE if CRC is OK, FALSE if KO
 */
bool ms5637_crc_check (uint16_t *n_prom, uint8_t crc)
{
    uint8_t cnt, n_bit; 
    uint16_t n_rem; 
    uint16_t crc_read;

    n_rem = 0x00;
    crc_read = n_prom[7]; 
    n_prom[7] = (0xFF00 & (n_prom[7])); 
    for (cnt = 0; cnt < 16; cnt++) 
    {
        if (cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
        else n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);
        for (n_bit = 8; n_bit > 0; n_bit--)
        {
            if (n_rem & (0x8000))
                n_rem = (n_rem << 1) ^ 0x3000;
            else
                n_rem = (n_rem << 1);
        }
    }
    n_rem = (0x000F & (n_rem >> 12)); 
    n_prom[7] = crc_read;
    n_rem ^= 0x00;
        
	return  ( n_rem == crc );
}


/**
 * \brief Triggers conversion and read ADC value
 *
 * \param[in] uint8_t : Command used for conversion (will determine Temperature vs Pressure and osr)
 * \param[out] uint32_t* : ADC value.
 *
 * \return ms5637_status : status of MS5637
 *       - ms5637_status_ok : I2C transfer completed successfully
 *       - ms5637_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms5637_status_no_i2c_acknowledge : I2C did not acknowledge
 */
static enum ms5637_status ms5637_conversion_and_read_adc(uint8_t cmd, uint32_t *adc)
{
	enum ms5637_status status;
	uint8_t buffer[3];
	
	buffer[0] = 0;
	buffer[1] = 0;
	buffer[2] = 0;

	/* Read data */
	status = ms5637_write_command(cmd);
	if( status != ms5637_status_ok)
	{
		perror("conversion cmd failed\n"); 
		return status;
	}
		
	/* delay conversion depending on resolution
	 * The conversion will fail is the delay is not long enough.
	 * See the constants defined above the actual times in ms.  
	 * If the conversion read here fails then it is likely that the delay 
	 * is not long enough so increase the constant value for the resolution 
	 * you are using.   
	 */ 
	delay( conversion_time[ (cmd & MS5637_CONVERSION_OSR_MASK)/2 ]/1000 );		
	

	// Send the read command
	status = ms5637_write_command(MS5637_READ_ADC);
	if( status != ms5637_status_ok)
	{
		perror("conversion read failed\n"); 
		return status;
	}
	
	
	if( read( ms5637_file, buffer, 3 ) != 3 )
	{
		perror("ADC buffer read failed\n");
		return ms5637_status_i2c_transfer_error;
	}


	*adc = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
	
	return status;
}

/**
 * \brief Reads the temperature and pressure ADC value and compute the compensated values.
 *
 * \param[out] float* : Celsius Degree temperature value
 * \param[out] float* : mbar pressure value
 *
 * \return ms5637_status : status of MS5637
 *       - ms5637_status_ok : I2C transfer completed successfully
 *       - ms5637_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms5637_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - ms5637_status_crc_error : CRC check error on the coefficients
 */
enum ms5637_status ms5637_read_temperature_and_pressure( float *temperature, float *pressure)
{
	enum ms5637_status status = ms5637_status_ok;
	uint32_t adc_temperature, adc_pressure;
	int32_t dT, TEMP;
	int64_t OFF, SENS, P, T2, OFF2, SENS2;
	uint8_t cmd;
	
	
	// If first time adc is requested, get EEPROM coefficients
	if( ms5637_coeff_read == false )
		status = ms5637_read_eeprom();
	if( status != ms5637_status_ok)
	{
		perror("crc failed\n"); 
		return status;
	}
	
	/* First read temperature
	 * For most practical application you may not need temperature
	 * so this read can be commented out or deleted.  Each read
	 * takes 20ms at highest resolution.
	 */ 
	cmd = ms5637_resolution_osr*2;
	cmd |= MS5637_START_TEMPERATURE_ADC_CONVERSION;
	status = ms5637_conversion_and_read_adc( cmd, &adc_temperature);
	if( status != ms5637_status_ok)
	{
		perror("Temp. read failed\n");
		//ms5637_reset();
		return status;
	}

	// Now read pressure
	cmd = ms5637_resolution_osr*2;
	cmd |= MS5637_START_PRESSURE_ADC_CONVERSION;
	status = ms5637_conversion_and_read_adc( cmd, &adc_pressure);
	if( status != ms5637_status_ok)
	{
		perror("Pressure read failed\n");
		//ms5637_reset();
		return status;
	}
  

//---
	/** This is a check on the calculations performed below.
	 * These figures come from pg 7 of the MS5637 datasheet
	 * which provides an example calculation.  Uncommment to
	 * use these figures, rather than the real time data read 
	 * from the sensor, to provide a known result.	*/ 
/* 	adc_pressure = 6465444;
	adc_temperature = 8077636;
	
    eeprom_coeff[ 1 ] = 46372;
  	eeprom_coeff[ 2 ] = 43981;  	
   	eeprom_coeff[ 3 ] = 29059;  	
  	eeprom_coeff[ 4 ] = 27842;  	
 	eeprom_coeff[ 5 ] = 31553;  	
  	eeprom_coeff[ 6 ] = 28165;  	
*/  	
//---

  		
    if (adc_temperature == 0 || adc_pressure == 0)
    {
        return ms5637_status_i2c_transfer_error;
	}

	// Difference between actual and reference temperature = D2 - Tref
	dT = (int32_t)adc_temperature - ((int32_t)eeprom_coeff[MS5637_REFERENCE_TEMPERATURE_INDEX] * 256 );

 
	// Actual temperature = 2000 + dT * TEMPSENS
	TEMP = 2000 + ((int64_t)dT * (int64_t)eeprom_coeff[MS5637_TEMP_COEFF_OF_TEMPERATURE_INDEX] / 8388608) ;

  
	
	// Second order temperature compensation
	if( TEMP < 2000 )
	{
		T2 = ( 3 * ( (int64_t)dT  * (int64_t)dT  ) ) / 8589934592;	//
		OFF2 = 61 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 16 ;
		SENS2 = 29 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 16 ;
		
		if( TEMP < -1500 )
		{
			OFF2 += 17 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) ;
			SENS2 += 9 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) ;
		}
	}
	else
	{
		T2 = ( 5 * ( (int64_t)dT  * (int64_t)dT  ) ) / 274877906944;
		OFF2 = 0 ;
		SENS2 = 0 ;
	}
	
	// OFF = OFF_T1 + TCO * dT
	OFF = ( (int64_t)eeprom_coeff[MS5637_PRESSURE_OFFSET_INDEX] * 131072 ) + ( ( (int64_t)eeprom_coeff[MS5637_TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX] * dT ) / 64 ) ;
	OFF -= OFF2 ;	
	
 	
	// Sensitivity at actual temperature = SENS_T1 + TCS * dT
	SENS = ( (int64_t)eeprom_coeff[MS5637_PRESSURE_SENSITIVITY_INDEX] * 65536 ) + ( ((int64_t)eeprom_coeff[MS5637_TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX] * dT) / 128 ) ;
	SENS -= SENS2 ;	

	
	// Temperature compensated pressure = D1 * SENS - OFF
	P = ( ( adc_pressure * SENS / 2097152 ) - OFF ) / 32768 ;
	
	*temperature = ( (float)TEMP - T2 ) / 100;
	*pressure = (float)P / 100;
	
	return status;
}



#ifdef __cplusplus
}
#endif

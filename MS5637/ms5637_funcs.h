/***************************************************************************
* Raspberry Pi Interface Copyright (C) 2018 The BlackBoxCmaera Company Limited  GPS-PIE.COM
*
* File : MS5637/ms5637_funcs.h
*
* Date : 2018/08/28
*
* Revision : 1.0
*
* Usage: Interface Functions header file for the MS5637 pressure / temperature sensor
*
****************************************************************************
* Original Copyright notice
* file ms5637.h
*
* brief MS5637 Temperature sensor driver header file
*
* Copyright (c) 2016 Measurement Specialties. All rights reserved.
*
* asf_license_start
*
* page License
*
*
* asf_license_stop
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

#ifndef MS5637_H_INCLUDED
#define MS5637_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define SEA_LEVEL_PRESSURE_MBAR 1013.25	// air pressure at sea level in mbar

enum ms5637_resolution_osr {
	ms5637_resolution_osr_256 = 0,
	ms5637_resolution_osr_512,
	ms5637_resolution_osr_1024,
	ms5637_resolution_osr_2048,
	ms5637_resolution_osr_4096,
	ms5637_resolution_osr_8192
};

enum ms5637_status {
	ms5637_status_ok,
	ms5637_status_no_i2c_acknowledge,
	ms5637_status_i2c_transfer_error,
	ms5637_status_crc_error
};
	
// Functions

/**
 * \brief Configures the I2C master to be used with the ms5637 device.
 */
extern bool ms5637_init(void);


/**
 * \brief Reset the MS5637 device
 *
 * \return ms5637_status : status of MS5637
 *       - ms5637_status_ok : I2C transfer completed successfully
 *       - ms5637_status_i2c_transfer_error : Problem with i2c transfer
 *       - ms5637_status_no_i2c_acknowledge : I2C did not acknowledge
 */
extern enum ms5637_status ms5637_reset(void);


/**
 * \brief Set  ADC resolution.
 *
 * \param[in] ms5637_resolution_osr : Resolution requested
 *
 */
extern void ms5637_set_resolution(enum ms5637_resolution_osr );

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
 *       - ms5637_status_crc_error : CRC check error on on the PROM coefficients
 */
extern enum ms5637_status ms5637_read_temperature_and_pressure(float *, float *);


/* This routine sets the base pressure to the current pressure.
 * This sets the altitude to zero at the current pressure & altitude. */
extern void setCurrentPressure( void );


/* This routine allows the setting of the base pressure which is used to 
 * calculate the altitude in metres */
extern void setBasePressure( float basePressureSetting );


/* This routine is called to read the pressure and temperature from the
 * MS5637 sensor.  The altitude is then calculated from the pressure
 * measurement and the base pressure setting. */
extern void read_MS5637( void );


extern float temperature;
extern float pressure;
extern float altitude;	


#endif /* MS5637_H_INCLUDED */

/**
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * 
 * @file	bsec.h
 * @date	27 May 2022
 * @version	1.4.1492
 *  
 */

#ifndef BSEC_CLASS_H
#define BSEC_CLASS_H

/* Includes */
#include <stdbool.h>
#include "main.h"

#include "inc/bsec_datatypes.h"
#include "inc/bsec_interface.h"
#include "bme68x/bme68x.h"

typedef void TwoWire;
typedef void SPIClass;
#define millis() HAL_GetTick()
#define delay(x) HAL_Delay(x)

#define BME68X_ERROR            INT8_C(-1)
#define BME68X_WARNING          INT8_C(1)


	/* Public variables */
	extern bsec_version_t version;		// Stores the version of the BSEC algorithm
	extern int64_t nextCall;			// Stores the time when the algorithm has to be called next in ms
	extern int8_t bme68xStatus;		// Placeholder for the BME68x driver's error codes
	extern bsec_library_return_t bsecStatus;
	extern float iaq, rawTemperature, pressure, rawHumidity, gasResistance, stabStatus, runInStatus, temperature, humidity,
	      staticIaq, co2Equivalent, breathVocEquivalent, compGasValue, gasPercentage;
	extern uint8_t iaqAccuracy, staticIaqAccuracy, co2Accuracy, breathVocAccuracy, compGasAccuracy, gasPercentageAccuracy;
	extern int64_t outputTimestamp;	// Timestamp in ms of the output
//	static TwoWire *wireObj;
//	static SPIClass *spiObj;
	extern struct bme68x_conf conf;
	extern struct bme68x_heatr_conf heatrConf;


	/* Public APIs */
	/**
	 * @brief Constructor
	 */
	void Bsec_Bsec(void);

	/**
	 * @brief Function to initialize the BSEC library and the BME68x sensor
	 * @param intf		: BME68X_SPI_INTF or BME68X_I2C_INTF interface
	 * @param read     	: Read callback
	 * @param write    	: Write callback
     * @param idleTask 	: Delay or Idle function
     * @param intfPtr 	: Pointer to the interface descriptor
	 */
	void Bsec_begin(enum bme68x_intf intf, bme68x_read_fptr_t read, bme68x_write_fptr_t write, bme68x_delay_us_fptr_t idleTask, void *intfPtr);

	/**
	 * @brief Function to initialize the BSEC library and the BME68x sensor
	 * @param i2cAddr	: I2C address
	 * @param i2c		: Pointer to the TwoWire object
	 */
	void Bsec_begin_i2c(uint8_t i2cAddr, TwoWire *i2c);
	
	/**
	 * @brief Function to initialize the BSEC library and the BME68x sensor
	 * @param chipSelect	: SPI chip select
	 * @param spi			: Pointer to the SPIClass object
	 */
	void Bsec_begin_spi(uint8_t chipSelect, SPIClass *spi);

	/**
	 * @brief Function that sets the desired sensors and the sample rates
	 * @param sensorList	: The list of output sensors
	 * @param nSensors		: Number of outputs requested
	 * @param sampleRate	: The sample rate of requested sensors
	 */
	void Bsec_updateSubscription(bsec_virtual_sensor_t sensorList[], uint8_t nSensors, float sampleRate);

	/**
	 * @brief Callback from the user to trigger reading of data from the BME68x, process and store outputs
	 * @return true if there are new outputs. false otherwise
	 */
	bool Bsec_run(void);

	/**
	 * @brief Function to get the state of the algorithm to save to non-volatile memory
	 * @param state			: Pointer to a memory location that contains the state
	 */
	void Bsec_getState(uint8_t *state);

	/**
	 * @brief Function to set the state of the algorithm from non-volatile memory
	 * @param state			: Pointer to a memory location that contains the state
	 */
	void Bsec_setState(uint8_t *state);

	/**
	 * @brief Function to set the configuration of the algorithm from memory
	 * @param state			: Pointer to a memory location that contains the configuration
	 */
	void Bsec_setConfig(const uint8_t *config);

	/**
	 * @brief Function to set the temperature offset
	 * @param tempOffset	: Temperature offset in degree Celsius
	 */
	void Bsec_setTemperatureOffset(float tempOffset);
	
		
	/**
	 * @brief Function to calculate an int64_t timestamp in milliseconds
	 */
	int64_t getTimeMs(void);
	
	/**
	* @brief Task that delays for a ms period of time
	* @param period	: Period of time in us
	* @param intfPtr: Pointer to the interface descriptor
	*/
	void delay_us(uint32_t period, void *intfPtr);

	/**
	* @brief Callback function for reading registers over I2C
	* @param regAddr : Register address of the sensor
	* @param regData : Pointer to the data to be written to the sensor
	* @param length   : Length of the transfer
	* @param intfPtr : Pointer to the interface descriptor
	* @return	Zero for success, non-zero otherwise
	*/
	int8_t Bsec_i2cRead(uint8_t regAddr, uint8_t *regData, uint32_t length, void *intfPtr);

	/**
	* @brief Callback function for writing registers over I2C
	* @param regAddr : Register address of the sensor
	* @param regData : Pointer to the data to be written to the sensor
	* @param length   : Length of the transfer
	* @param intfPtr : Pointer to the interface descriptor
	* @return	Zero for success, non-zero otherwise
	*/
	int8_t Bsec_i2cWrite(uint8_t regAddr, const uint8_t *regData, uint32_t length, void *intfPtr);

	/**
	 * @brief Function that implements the default SPI read transaction
	 * @param regAddr : Register address of the sensor
	 * @param regData : Pointer to the data to be read from the sensor
	 * @param length   : Length of the transfer
	 * @param intfPtr : Pointer to the interface descriptor
	 * @return 0 if successful, non-zero otherwise
	 */
	int8_t Bsec_spiRead(uint8_t regAddr, uint8_t *regData, uint32_t length, void *intfPtr);

	/**
	 * @brief Function that implements the default SPI write transaction
	 * @param regAddr : Register address of the sensor
	 * @param regData : Pointer to the data to be written to the sensor
	 * @param length   : Length of the transfer
	 * @param intfPtr : Pointer to the interface descriptor
	 * @return 0 if successful, non-zero otherwise
	 */
	int8_t Bsec_spiWrite(uint8_t regAddr, const uint8_t *regData, uint32_t length, void *intfPtr);

	/**
     * @brief Function to set the Temperature, Pressure and Humidity over-sampling.
     *        Passing no arguments sets the defaults.
     * @param osTemp : BME68X_OS_NONE to BME68X_OS_16X
     * @param osPres : BME68X_OS_NONE to BME68X_OS_16X
     * @param osHum  : BME68X_OS_NONE to BME68X_OS_16X
     */
    void Bsec_setTPH(uint8_t osTemp, uint8_t osPres, uint8_t osHum);
	






#endif

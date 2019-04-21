/*
 * @filename 	: sensors.h
 * @description	: contains headers to support si7021.c
 * @author		: Puneet Bansal
 */

/*
 * @description
 * Populate the I2C_TransferSeq structure with slave address, the type of transfer (No hold master mode).
 * Call I2c write function, wait for atlest 3.8 ms before reading the value from slave using i2c_read();
 * Calculated the temperature in degree celcius from the ADC value received from sensor.
 *
 */
#include <stdint.h>

/**
 * @brief Sets the slave addresss, mode to No Hold Master Humid and writes it via i2c_write(). Then reads the raw humidity value
 * using i2c_read() and performs calculations to convert it into relative humidity.
 *
 * @return 32 bit relative humidity value
 */
uint32_t humid_get();

/**
 *  @brief Configures the CCS811 sensor. The CCS811 sensor starts up in boot mode and needs to be configured to application mode
 *  before obtaining any valueable data from it. So , this function reads the status register to check if valid firmware is present.
 *  If it is so, then a write is made to the APP_START. After this the Status register is checked again to confirm the mode
 *  changed to application mode.
 */
void sensorConfig();

/**
 * @brief Function to change the mode of CCS811 sensor to mode1. Which takes up reading from the sensor in every 1 second.
 */
void changeMode();


/**
 * @brief Reads the status register to see if valid data is present. If valid data is present, then ppm(eC02) value is read
 * from ALG_RES register and converted into 16 bit variable.
 *
 * @return 16 bit CO2 concentration value in ppm.
 */
uint16_t ppmGet();


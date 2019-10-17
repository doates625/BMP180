/**
 * @file BMP180.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "BMP180.h"
#include <math.h>

/**
 * @brief Constructs BMP180 interface
 * @param i2c Platform-specific I2C bus interface
 * @param p0 Sea-level pressure [kPa]
 */
BMP180::BMP180(I2CDEVICE_I2C_CLASS* i2c, float p0)
{
	this->i2c = I2CDevice(i2c, i2c_addr, I2CDevice::msb_first);
	this->pressure0 = p0;
}

/**
 * @brief Initializes BMP180
 * @return True if I2C communication succeeded
 */
bool BMP180::init()
{
	// Check ID register
	if (i2c.read_uint8(reg_id_addr) != reg_id_val)
	{
		return false;
	}

	// Read calibration params
	i2c.read_sequence(reg_cal_addr, 22);
	ac1 = i2c.read_int16();
	ac2 = i2c.read_int16();
	ac3 = i2c.read_int16();
	ac4 = i2c.read_uint16();
	ac5 = i2c.read_uint16();
	ac6 = i2c.read_uint16();
	b1 = i2c.read_int16();
	b2 = i2c.read_int16();
	mb = i2c.read_int16();
	mc = i2c.read_int16();
	md = i2c.read_int16();

	// Set sampling to 1x
	set_sampling(samples_1x);

	// Everything succeeded
	return true;
}

/**
 * @brief Sets sampling setting of device
 * @param sampling Sampling setting
 * 
 * Sampling options:
 * - samples_1x = No oversampling
 * - samples_2x = 2x oversampling
 * - samples_4x = 4x oversampling
 * - samples_8x = 8x oversampling
 */
void BMP180::set_sampling(sampling_t sampling)
{
	this->sampling = sampling;
	switch (sampling)
	{
		case samples_1x:
			reg_select_oss = reg_select_oss1;
			comp_time_us = 4500;
			oss_shift = 0;
			break;
		case samples_2x:
			reg_select_oss = reg_select_oss2;
			comp_time_us = 7500;
			oss_shift = 1;
			break;
		case samples_4x:
			reg_select_oss = reg_select_oss4;
			comp_time_us = 13500;
			oss_shift = 2;
			break;
		case samples_8x:
			reg_select_oss = reg_select_oss8;
			comp_time_us = 25500;
			oss_shift = 3;
			break;
	}
}

/**
 * @brief Updates temperature, pressure, and altitude readings
 */
void BMP180::update()
{
	// Read uncompensated temperature
	i2c.write_uint8(reg_select_addr, reg_select_temp);
	Platform::wait_us(4500);
	int32_t UT = i2c.read_int16(reg_data_addr);

	// Read uncompensated pressure
	i2c.write_uint8(reg_select_addr, reg_select_oss);
	Platform::wait_us(comp_time_us);
	i2c.read_sequence(reg_data_addr, 3);
	int32_t msb = i2c.read_uint8();
	int32_t lsb = i2c.read_uint8();
	int32_t xlsb = i2c.read_uint8();
	int32_t UP = ((msb << 16) + (lsb << 8) + xlsb) >> (8 - oss_shift);

	// Calculate temperature
	int32_t x1, x2, b5, T;
	x1 = (UT - (int32_t)ac6) * (int32_t)ac5 / 32768;
	x2 = (int32_t)mc * 2048 / (x1 + md);
	b5 = x1 + x2;
	T = (b5 + 8) / 16;

	// Calculate pressure
	int32_t b6, x3, b3, p;
	uint32_t b4, b7;
	b6 = b5 - 4000;
	x1 = ((int32_t)b2 * (b6 * b6 / 4096)) / 2048;
	x2 = (int32_t)ac2 * b6 / 2048;
	x3 = x1 + x2;
	b3 = ((((int32_t)ac1 * 4 + x3) << oss_shift) + 2) / 4;
	x1 = (int32_t)ac3 * b6 / 8192;
	x2 = ((int32_t)b1 * (b6 * b6 / 4096)) / 65536;
	x3 = ((x1 + x2) + 2) / 4;
	b4 = (uint32_t)ac4 * (uint32_t)(x3 + 32768) / 32768;
	b7 = ((uint32_t)UP - b3) * (uint32_t)(50000 >> oss_shift);
	if (b7 < 0x80000000) { p = (b7 * 2) / b4; }
	else { p = (b7 / b4) * 2; }
	x1 = (p / 256) * (p / 256);
	x1 = (x1 * 3038) / 65536;
	x2 = (-7357 * p) / 65536;
	p = p + (x1 + x2 + (int32_t)3791) / 16;

	// Calculate final values
	temp = T * 0.1f;
	pressure = p * 0.001f;
	altitude = 44330.0f * (1.0f - powf(pressure / pressure0, 1.0f / 5.255f));
}

/**
 * @brief Returns temperature [deg C]
 */
float BMP180::get_temp()
{
	return temp;
}

/**
 * @brief Returns pressure [kPa]
 */
float BMP180::get_pressure()
{
	return pressure;
}

/**
 * @brief Returns altitude above sea-level [m]
 */
float BMP180::get_altitude()
{
	return altitude - altitude_offset;
}

/**
 * @brief Sets current altitude to zero position
 */
void BMP180::zero_altitude()
{
	update();
	altitude_offset = altitude;
}
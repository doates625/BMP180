/**
 * @file BMP180.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "BMP180.h"
#include <math.h>

/**
 * @brief Constructs BMP180 interface
 * @param i2c Platform-specific I2C bus interface
 */
BMP180::BMP180(I2CDEVICE_I2C_CLASS* i2c)
{
	this->i2c = I2CDevice(i2c, i2c_addr, I2CDevice::msb_first);
	this->alt_zero = 0.0f;
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
 * @brief Updates temperature and pressure readings
 */
void BMP180::update()
{
	update_temp();
	update_pres();
}

/**
 * @brief Updates temperature reading
 */
void BMP180::update_temp()
{
	// Read uncompensated temperature
	i2c.write_uint8(reg_select_addr, reg_select_temp);
	Platform::wait_us(4500);
	int32_t UT = i2c.read_int16(reg_data_addr);

	// Calibration compensation
	int32_t x1, x2, T;
	x1 = ((UT - ac6) * ac5) >> 15;
	x2 = (mc << 11) / (x1 + md);
	b5 = x1 + x2;
	T = (b5 + 8) >> 4;

	// Calculate temperature
	temp = T * 0.1f;
}

/**
 * @brief Updates pressure reading
 * 
 * Uses temperature from last call to update() or update_temp().
 */
void BMP180::update_pres()
{
	// Read uncompensated pressure
	i2c.write_uint8(reg_select_addr, reg_select_oss);
	Platform::wait_us(comp_time_us);
	i2c.read_sequence(reg_data_addr, 3);
	uint32_t msb = i2c.read_uint8();
	uint32_t lsb = i2c.read_uint8();
	uint32_t xlsb = i2c.read_uint8();
	uint32_t UP = ((msb << 16) + (lsb << 8) + xlsb) >> (8 - oss_shift);

	// Calibration compensation
	int32_t b6, x1, x2 ,x3, b3, p;
	uint32_t b4, b7;
	b6 = b5 - 4000;
	x1 = (b2 * ((b6 * b6) >> 12)) >> 11;
	x2 = (ac2 * b6) >> 11;
	x3 = x1 + x2;
	b3 = ((((ac1 << 2) + x3) << oss_shift) + 2) >> 2;
	x1 = (ac3 * b6) >> 13;
	x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (ac4 * (uint32_t)(x3 + 32768)) >> 15;
	b7 = (UP - b3) * (uint32_t)(50000 >> oss_shift);
	if (b7 < 0x80000000) { p = (b7 << 1) / b4; }
	else { p = (b7 / b4) << 1; }
	x1 = p >> 8;
	x1 = x1 * x1;
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	p = p + ((x1 + x2 + 3791) >> 4);

	// Calculate pressure
	pres = p * 0.001f;
}

/**
 * @brief Returns temperature [deg C]
 * 
 * Uses temperature from last call to update() or update_temp().
 */
float BMP180::get_temp()
{
	return temp;
}

/**
 * @brief Returns pressure [kPa]
 * 
 * Uses pressure from last call to update() or update_pres().
 */
float BMP180::get_pres()
{
	return pres;
}

/**
 * @brief Returns altitude above sea-level [m]
 * @param sea_level_p Sea-level pressure [kPa]
 * 
 * Uses pressure from last call to update() or update_pres().
 */
float BMP180::get_alt(float sea_level_p)
{
	float alt = 44330.0f * (1.0f - powf(pres / sea_level_p, 0.190295f));
	return alt - alt_zero;
}

/**
 * @brief Sets current altitude to zero position
 * @param sea_level_p Sea-level pressure [kPa]
 */
void BMP180::zero_alt(float sea_level_p)
{
	update();
	alt_zero = get_alt(sea_level_p);
}
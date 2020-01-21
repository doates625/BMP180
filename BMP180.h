/**
 * @file BMP180.h
 * @brief Class for interfacing with BMP180 I2C pressure sensor
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once
#include <I2CDevice.h>

/**
 * Minimum I2C Buffer Size
 */
#if I2CDEVICE_BUFFER_SIZE < 22
	#error BMP180 requires I2CDEVICE_BUFFER_SIZE >= 22
#endif

/**
 * Class Declaration
 */
class BMP180
{
public:

	// Oversampling
	typedef enum
	{
		samples_1x,	// No oversampling
		samples_2x,	// 2x oversampling
		samples_4x,	// 4x oversampling
		samples_8x,	// 8x oversampling
	}
	sampling_t;

	// Constructor and basics
	BMP180(I2CDevice::i2c_t* i2c);
	bool init();
	void set_sampling(sampling_t sampling);

	// Measurements
	void update();
	void update_temp();
	void update_pres();
	float get_temp();
	float get_pres();
	float get_alt(float sea_level_p = 101.325f);
	
	// Altitude calibration
	void zero_alt(float sea_level_p = 101.325f);

protected:

	// I2C Communication
	static const uint8_t i2c_addr = 0x77;
	I2CDevice i2c;

	// I2C Registers
	static const uint8_t reg_cal_addr = 0xAA;
	static const uint8_t reg_id_addr = 0xD0;
	static const uint8_t reg_id_val = 0x55;
	static const uint8_t reg_select_addr = 0xF4;
	static const uint8_t reg_select_temp = 0x2E;
	static const uint8_t reg_select_oss1 = 0x34;
	static const uint8_t reg_select_oss2 = 0x74;
	static const uint8_t reg_select_oss4 = 0xB4;
	static const uint8_t reg_select_oss8 = 0xF4;
	static const uint8_t reg_data_addr = 0xF6;

	// Oversampling Parameters
	uint8_t reg_select_oss;
	uint32_t comp_time_us;
	uint8_t oss_shift;
	sampling_t sampling;

	// Calibration Parameters
	int32_t ac1, ac2, ac3;
	uint32_t ac4, ac5, ac6;
	int32_t b1, b2;
	int32_t mb, mc, md;

	// State data
	int32_t b5;
	float temp, pres, alt_zero;
};

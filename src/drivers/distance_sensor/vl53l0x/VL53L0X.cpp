/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "VL53L0X.hpp"

/* VL53L0X Registers addresses */
#define VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HW_REG            0x89
#define MSRC_CONFIG_CONTROL_REG                         0x60
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_REG 0x44
#define SYSTEM_SEQUENCE_CONFIG_REG                      0x01
#define DYNAMIC_SPAD_REF_EN_START_OFFSET_REG            0x4F
#define DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD_REG         0x4E
#define GLOBAL_CONFIG_REF_EN_START_SELECT_REG           0xB6
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0_REG            0xB0
#define SYSTEM_INTERRUPT_CONFIG_GPIO_REG                0x0A
#define SYSTEM_SEQUENCE_CONFIG_REG                      0x01
#define SYSRANGE_START_REG                              0x00
#define RESULT_INTERRUPT_STATUS_REG                     0x13
#define SYSTEM_INTERRUPT_CLEAR_REG                      0x0B
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0_REG            0xB0
#define GPIO_HV_MUX_ACTIVE_HIGH_REG                     0x84
#define SYSTEM_INTERRUPT_CLEAR_REG                      0x0B
#define RESULT_RANGE_STATUS_REG                         0x14
#define VL53L0X_RA_IDENTIFICATION_MODEL_ID              0xC0
#define VL53L0X_IDENTIFICATION_MODEL_ID                 0xEEAA

#define VL53L0X_US                                      1000    // 1ms
#define VL53L0X_SAMPLE_RATE                             50000   // 50ms   // Callback 실행주기

#define VL53L0X_BUS_CLOCK                               400000 // 400kHz bus clock speed

#define MODULE_NAME "vl53l0x"
VL53L0X *VL53L0X::_instance = nullptr;





VL53L0X::VL53L0X() :
    WorkItem(MODULE_NAME, px4::wq_configurations::hp_default) // 부모 클래스 생성자 호출
{
    PX4_INFO("VL53L0X default constructor called");
	// VL53L0X typical range 0-2 meters with 25 degree field of view
	// _px4_rangefinder.set_min_distance(0.f);
	// _px4_rangefinder.set_max_distance(2.f);
	// _px4_rangefinder.set_fov(math::radians(25.f));
	// Allow retries as the device typically misses the first measure attempts.
	// I2C::_retries = 1;
	// _px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_VL53L0X);
}

VL53L0X::~VL53L0X()
{
	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

// int VL53L0X::init()
// {
// 	int ret = I2C::init();

// 	if (ret != PX4_OK) {
// 		return ret;
// 	}

// 	ScheduleNow();
// 	return PX4_OK;
// }

int VL53L0X::collect()
{
// 	// Read from the sensor.
// 	uint8_t val[2] {};
// 	perf_begin(_sample_perf);

// 	_collect_phase = false;

// 	const hrt_abstime timestamp_sample = hrt_absolute_time();

// 	if (transfer(nullptr, 0, &val[0], 2) != PX4_OK) {
// 		perf_count(_comms_errors);
// 		perf_end(_sample_perf);
// 		return PX4_ERROR;
// 	}

// 	perf_end(_sample_perf);

// 	uint16_t distance_mm = (val[0] << 8) | val[1];
// 	float distance_m = distance_mm / 1000.f;

// 	_px4_rangefinder.update(timestamp_sample, distance_m);

// 	return PX4_OK;
// }

// int VL53L0X::measure()
// {
// 	uint8_t wait_for_measurement = 0;
// 	uint8_t system_start = 0;

// 	// Send the command to begin a measurement.
// 	const uint8_t cmd = RESULT_RANGE_STATUS_REG + 10;

// 	if (_new_measurement) {

// 		_new_measurement = false;

// 		writeRegister(0x80, 0x01);
// 		writeRegister(0xFF, 0x01);
// 		writeRegister(0x00, 0x00);
// 		writeRegister(0x91, _stop_variable);
// 		writeRegister(0x00, 0x01);
// 		writeRegister(0xFF, 0x00);
// 		writeRegister(0x80, 0x00);

// 		writeRegister(SYSRANGE_START_REG, 0x01);

// 		readRegister(SYSRANGE_START_REG, system_start);

// 		if ((system_start & 0x01) == 1) {
// 			ScheduleDelayed(VL53L0X_US);
// 			return PX4_OK;

// 		} else {
// 			_measurement_started = true;
// 		}
// 	}

// 	if (!_collect_phase && !_measurement_started) {

// 		readRegister(SYSRANGE_START_REG, system_start);

// 		if ((system_start & 0x01) == 1) {
// 			ScheduleDelayed(VL53L0X_US);
// 			return PX4_OK;

// 		} else {
// 			_measurement_started = true;
// 		}
// 	}

// 	readRegister(RESULT_INTERRUPT_STATUS_REG, wait_for_measurement);

// 	if ((wait_for_measurement & 0x07) == 0) {
// 		ScheduleDelayed(VL53L0X_US); // reschedule every 1 ms until measurement is ready
// 		return PX4_OK;
// 	}

// 	_collect_phase = true;

// 	int ret = transfer(&cmd, sizeof(cmd), nullptr, 0);

// 	if (ret != PX4_OK) {
// 		perf_count(_comms_errors);
// 		DEVICE_LOG("i2c::transfer returned %d", ret);
// 		return ret;
// 	}

	return PX4_OK;
}

int VL53L0X::module_stop()
{
    if (_instance != nullptr) {
        _instance->ScheduleClear(); // 작업 정지
        delete _instance;
        _instance = nullptr;
    }
    PX4_INFO("VL53L0X stopped");
    return PX4_OK;
}

void VL53L0X::print_status()
{ // SEUK
  // status 명령어 쓰면 나오는것들 설정. 아래에 module_status랑 연동되어있음
    PX4_INFO("Dynamixel driver status:");
    PX4_INFO("Dynamixel driver status:");
//	I2CSPIDriverBase::print_status();
//	perf_print_counter(_comms_errors);
//	perf_print_counter(_sample_perf);
}

int VL53L0X::probe()
{ 	
// 	if (sensorInit() == PX4_OK) {
// 		return PX4_OK;
// 	}

// 	// Device not found on any address.
// 	return -EIO;
// }

// int VL53L0X::readRegister(const uint8_t reg_address, uint8_t &value)
// {
// 	// Write register address to the sensor.
// 	int ret = transfer(&reg_address, sizeof(reg_address), nullptr, 0);

// 	if (ret != PX4_OK) {
// 		perf_count(_comms_errors);
// 		return ret;
// 	}

// 	// Read from the sensor.
// 	ret = transfer(nullptr, 0, &value, 1);

// 	if (ret != PX4_OK) {
// 		perf_count(_comms_errors);
// 		return ret;
// 	}

// 	return PX4_OK;
// }

// int VL53L0X::readRegisterMulti(const uint8_t reg_address, uint8_t *value, const uint8_t length)
// {
// 	// Write register address to the sensor.
// 	int ret = transfer(&reg_address, 1, nullptr, 0);

// 	if (ret != PX4_OK) {
// 		perf_count(_comms_errors);
// 		return ret;
// 	}

// 	// Read from the sensor.
// 	ret = transfer(nullptr, 0, &value[0], length);

// 	if (ret != PX4_OK) {
// 		perf_count(_comms_errors);
// 		return ret;
// 	}

	return PX4_OK;
}



void VL53L0X::Run()
{
	// measure();

	// if (_collect_phase) {

	// 	_collect_phase = false;
	// 	_new_measurement = true;

	// 	collect();

	// 	ScheduleDelayed(VL53L0X_SAMPLE_RATE);
	// }

	//SEUK
	count ++;
	if (count == 0) 
	{
		tx_function();
	}
	else if (count == 1) 
	{
		rx_function();
		count = 0;
	}

    // ScheduleDelayed(VL53L0X_SAMPLE_RATE); // 일정 주기로 다시 호출
}

// int VL53L0X::spadCalculations()
// {
// 	uint8_t val = 0;
// 	uint8_t spad_count = 0;
// 	uint8_t ref_spad_map[6] = {};

// 	bool spad_type_is_aperture = false;

// 	writeRegister(0x80, 0x01);
// 	writeRegister(0xFF, 0x01);
// 	writeRegister(0x00, 0x00);
// 	writeRegister(0xFF, 0x06);

// 	readRegister(0x83, val);
// 	writeRegister(0x83, val | 0x04);

// 	writeRegister(0xFF, 0x07);
// 	writeRegister(0x81, 0x01);
// 	writeRegister(0x80, 0x01);
// 	writeRegister(0x94, 0x6b);
// 	writeRegister(0x83, 0x00);

// 	readRegister(0x83, val);

// 	while (val == 0x00) {
// 		readRegister(0x83, val);
// 	}

// 	writeRegister(0x83, 0x01);
// 	readRegister(0x92, val);

// 	spad_count = val & 0x7f;
// 	spad_type_is_aperture = (val >> 7) & 0x01;

// 	writeRegister(0x81, 0x00);
// 	writeRegister(0xFF, 0x06);

// 	readRegister(0x83, val);
// 	writeRegister(0x83, val  & ~0x04);

// 	writeRegister(0xFF, 0x01);
// 	writeRegister(0x00, 0x01);
// 	writeRegister(0xFF, 0x00);
// 	writeRegister(0x80, 0x00);

// 	readRegisterMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0_REG, &ref_spad_map[0], 6);

// 	writeRegister(0xFF, 0x01);
// 	writeRegister(DYNAMIC_SPAD_REF_EN_START_OFFSET_REG, 0x00);
// 	writeRegister(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD_REG, 0x2C);
// 	writeRegister(0xFF, 0x00);
// 	writeRegister(GLOBAL_CONFIG_REF_EN_START_SELECT_REG, 0xB4);

// 	uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0;
// 	uint8_t spads_enabled = 0;

// 	for (uint8_t i = 0; i < 48; i++) {
// 		if (i < first_spad_to_enable || spads_enabled == spad_count) {
// 			ref_spad_map[i / 8] &= ~(1 << (i % 8));

// 		} else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1) {
// 			spads_enabled++;
// 		}
// 	}

// 	writeRegisterMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0_REG, &ref_spad_map[0], 6);

// 	sensorTuning();

// 	writeRegister(SYSTEM_INTERRUPT_CONFIG_GPIO_REG, 4);		// 4: GPIO interrupt on new data.
// 	readRegister(GPIO_HV_MUX_ACTIVE_HIGH_REG, val);

// 	writeRegister(GPIO_HV_MUX_ACTIVE_HIGH_REG, val & ~0x10);	// Active low.
// 	writeRegister(SYSTEM_INTERRUPT_CLEAR_REG, 0x01);
// 	writeRegister(SYSTEM_SEQUENCE_CONFIG_REG, 0xE8);
// 	writeRegister(SYSTEM_SEQUENCE_CONFIG_REG, 0x01);

// 	singleRefCalibration(0x40);

// 	writeRegister(SYSTEM_SEQUENCE_CONFIG_REG, 0x02);

// 	singleRefCalibration(0x00);

// 	writeRegister(SYSTEM_SEQUENCE_CONFIG_REG, 0xE8);		// Restore config.

// 	return OK;
// }

int VL53L0X::sensorInit()
{
	// uint8_t val = 0;

	// // I2C at 2.8V on sensor side of level shifter
	// int ret = PX4_OK;
	// ret |= readRegister(VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HW_REG, val);

	// if (ret != PX4_OK) {
	// 	return PX4_ERROR;
	// }

	// ret |= writeRegister(VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HW_REG, val | 0x01);

	// // set I2C to standard mode
	// ret |= writeRegister(0x88, 0x00);
	// ret |= writeRegister(0x80, 0x01);
	// ret |= writeRegister(0xFF, 0x01);
	// ret |= writeRegister(0x00, 0x00);
	// ret |= readRegister(0x91, val);
	// ret |= writeRegister(0x00, 0x01);
	// ret |= writeRegister(0xFF, 0x00);
	// ret |= writeRegister(0x80, 0x00);

	// if (ret != PX4_OK) {
	// 	return PX4_ERROR;
	// }

	// _stop_variable = val;

	// // Disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
	// readRegister(MSRC_CONFIG_CONTROL_REG, val);
	// writeRegister(MSRC_CONFIG_CONTROL_REG, val | 0x12);

	// // Set signal rate limit to 0.1
	// float rate_limit = 0.1 * 65536;
	// uint8_t rate_limit_split[2] = {};

	// rate_limit_split[0] = (((uint16_t)rate_limit) >> 8);
	// rate_limit_split[1] = (uint16_t)rate_limit;

	// writeRegisterMulti(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_REG, &rate_limit_split[0], 2);
	// writeRegister(SYSTEM_SEQUENCE_CONFIG_REG, 0xFF);

	// spadCalculations();


	// SEUK
	// TODO: Initialize Dynamixel info
	// ID, model_name, etc..



	return PX4_OK;
}

// int VL53L0X::sensorTuning()
// {
// 	// Magic register settings taken from the ST Micro API.
// 	writeRegister(0xFF, 0x01);
// 	writeRegister(0x00, 0x00);
// 	writeRegister(0xFF, 0x00);
// 	writeRegister(0x09, 0x00);
// 	writeRegister(0x10, 0x00);
// 	writeRegister(0x11, 0x00);
// 	writeRegister(0x24, 0x01);
// 	writeRegister(0x25, 0xFF);
// 	writeRegister(0x75, 0x00);
// 	writeRegister(0xFF, 0x01);
// 	writeRegister(0x4E, 0x2C);
// 	writeRegister(0x48, 0x00);
// 	writeRegister(0x30, 0x20);
// 	writeRegister(0xFF, 0x00);
// 	writeRegister(0x30, 0x09);
// 	writeRegister(0x54, 0x00);
// 	writeRegister(0x31, 0x04);
// 	writeRegister(0x32, 0x03);
// 	writeRegister(0x40, 0x83);
// 	writeRegister(0x46, 0x25);
// 	writeRegister(0x60, 0x00);
// 	writeRegister(0x27, 0x00);
// 	writeRegister(0x50, 0x06);
// 	writeRegister(0x51, 0x00);
// 	writeRegister(0x52, 0x96);
// 	writeRegister(0x56, 0x08);
// 	writeRegister(0x57, 0x30);
// 	writeRegister(0x61, 0x00);
// 	writeRegister(0x62, 0x00);
// 	writeRegister(0x64, 0x00);
// 	writeRegister(0x65, 0x00);
// 	writeRegister(0x66, 0xA0);
// 	writeRegister(0xFF, 0x01);
// 	writeRegister(0x22, 0x32);
// 	writeRegister(0x47, 0x14);
// 	writeRegister(0x49, 0xFF);
// 	writeRegister(0x4A, 0x00);
// 	writeRegister(0xFF, 0x00);
// 	writeRegister(0x7A, 0x0A);
// 	writeRegister(0x7B, 0x00);
// 	writeRegister(0x78, 0x21);
// 	writeRegister(0xFF, 0x01);
// 	writeRegister(0x23, 0x34);
// 	writeRegister(0x42, 0x00);
// 	writeRegister(0x44, 0xFF);
// 	writeRegister(0x45, 0x26);
// 	writeRegister(0x46, 0x05);
// 	writeRegister(0x40, 0x40);
// 	writeRegister(0x0E, 0x06);
// 	writeRegister(0x20, 0x1A);
// 	writeRegister(0x43, 0x40);
// 	writeRegister(0xFF, 0x00);
// 	writeRegister(0x34, 0x03);
// 	writeRegister(0x35, 0x44);
// 	writeRegister(0xFF, 0x01);
// 	writeRegister(0x31, 0x04);
// 	writeRegister(0x4B, 0x09);
// 	writeRegister(0x4C, 0x05);
// 	writeRegister(0x4D, 0x04);
// 	writeRegister(0xFF, 0x00);
// 	writeRegister(0x44, 0x00);
// 	writeRegister(0x45, 0x20);
// 	writeRegister(0x47, 0x08);
// 	writeRegister(0x48, 0x28);
// 	writeRegister(0x67, 0x00);
// 	writeRegister(0x70, 0x04);
// 	writeRegister(0x71, 0x01);
// 	writeRegister(0x72, 0xFE);
// 	writeRegister(0x76, 0x00);
// 	writeRegister(0x77, 0x00);
// 	writeRegister(0xFF, 0x01);
// 	writeRegister(0x0D, 0x01);
// 	writeRegister(0xFF, 0x00);
// 	writeRegister(0x80, 0x01);
// 	writeRegister(0x01, 0xF8);
// 	writeRegister(0xFF, 0x01);
// 	writeRegister(0x8E, 0x01);
// 	writeRegister(0x00, 0x01);
// 	writeRegister(0xFF, 0x00);
// 	writeRegister(0x80, 0x00);

// 	return PX4_OK;
// }

// int VL53L0X::singleRefCalibration(const uint8_t byte)
// {
// 	uint8_t val = 0;

// 	writeRegister(SYSRANGE_START_REG, byte | 0x01);         // VL53L0X_REG_SYSRANGE_MODE_START_STOP
// 	readRegister(RESULT_INTERRUPT_STATUS_REG, val);

// 	while ((val & 0x07) == 0) {
// 		readRegister(RESULT_INTERRUPT_STATUS_REG, val);
// 	}

// 	writeRegister(SYSTEM_INTERRUPT_CLEAR_REG, 0x01);
// 	writeRegister(SYSRANGE_START_REG, 0x00);

// 	return PX4_OK;
// }

// int VL53L0X::writeRegister(const uint8_t reg_address, const uint8_t value)
// {
// 	uint8_t cmd[2] {reg_address, value};
// 	int ret = transfer(&cmd[0], 2, nullptr, 0);

// 	if (ret != PX4_OK) {
// 		perf_count(_comms_errors);
// 		return ret;
// 	}

// 	return PX4_OK;
// }

// int VL53L0X::writeRegisterMulti(const uint8_t reg_address, const uint8_t *value, const uint8_t length)
// {
// 	if (length > 6 || length < 1) {
// 		DEVICE_LOG("VL53L0X::writeRegisterMulti length out of range");
// 		return PX4_ERROR;
// 	}

// 	/* be careful: for uint16_t to send first higher byte */
// 	uint8_t cmd[7] {};
// 	cmd[0] = reg_address;

// 	memcpy(&cmd[1], &value[0], length);

// 	int ret = transfer(&cmd[0], length + 1, nullptr, 0);

// 	if (ret != PX4_OK) {
// 		perf_count(_comms_errors);
// 		return ret;
// 	}

// 	return PX4_OK;
// }

void VL53L0X::print_usage()
{
	PRINT_MODULE_USAGE_NAME("vl53l0x", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x29);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 0, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}


int VL53L0X::init()
{
    PX4_INFO("Initializing VL53L0X...");
    ScheduleNow(); // 즉시 작업 시작
    return PX4_OK;
}

void VL53L0X::tx_function() {
    PX4_INFO("Sending data to Dynamixel...");
}

void VL53L0X::rx_function()
{

}





extern "C" __EXPORT int vl53l0x_main(int argc, char *argv[])
{
// 	int ch;
// 	using ThisDriver = VL53L0X;


// 	BusCLIArguments cli{true, false};
// 	cli.default_i2c_frequency = 400000;
// 	cli.rotation = (Rotation)distance_sensor_s::ROTATION_DOWNWARD_FACING;
// 	cli.i2c_address = VL53L0X_BASEADDR;

// 	while ((ch = cli.getOpt(argc, argv, "R:")) != EOF) {
// 		switch (ch) {
// 		case 'R':
// 			cli.rotation = (Rotation)atoi(cli.optArg());
// 			break;
// 		}
// 	}

// 	const char *verb = argv[1];

// // //SEUK
// 	if (!verb) { //잘못된명령어
// 		ThisDriver::print_usage();
// 		return -1;
// 	}

// 	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIST_DEVTYPE_VL53L0X);

// 	if (!strcmp(verb, "start")) {
// 		//TODO PX_INFO????
// 		return ThisDriver::module_start(cli, iterator);
// 		PX4_INFO("START!");
// 	}

// 	if (!strcmp(verb, "stop")) {
// 		return ThisDriver::module_stop(iterator);
// 	}

// 	if (!strcmp(verb, "status")) {
// 		return ThisDriver::module_status(iterator);
// 	}
// 	ThisDriver::print_usage();
// 	return -1;



    const char *verb = argv[1];

    if (!strcmp(verb, "start")) {
        PX4_INFO("Starting Dynamixel driver...");
        // UART 초기화 및 드라이버 실행
        if (VL53L0X::_instance == nullptr) {
            VL53L0X::_instance = new VL53L0X();
            VL53L0X::_instance->init();
        }
        return 0;
    }

    if (!strcmp(verb, "stop")) {
        PX4_INFO("Stopping Dynamixel driver...");
        if (VL53L0X::_instance != nullptr) {
            delete VL53L0X::_instance;
            VL53L0X::_instance = nullptr;
        }
        return 0;
    }

    if (!strcmp(verb, "status")) {
        PX4_INFO("Fetching Dynamixel driver status...");
        if (VL53L0X::_instance != nullptr) {
            VL53L0X::_instance->print_status();
        } else {
            PX4_WARN("Driver not running");
        }
        return 0;
    }

    PX4_WARN("Unknown command");
    return -1;







}

// TODO
// 
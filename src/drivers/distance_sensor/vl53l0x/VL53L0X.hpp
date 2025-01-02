/****************************************************************************
 *
 *   Copyright (c) 2018-2020 PX4 Development Team. All rights reserved.
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

/**
 * @file VL53L0X.hpp
 *
 * Driver for the ST VL53L0X ToF Sensor connected via I2C.
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp> // px4::WorkItem 포함
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
#include <termios.h>

/* Configuration Constants */
#define VL53L0X_BASEADDR                                0x29

class VL53L0X : public px4::ScheduledWorkItem
{
public:
    VL53L0X();
	~VL53L0X() override;

	static void print_usage();


    static VL53L0X *_instance; // 정적 멤버 변수 선언

    void start_function();
    void stop_function();
    void status_function();



	int init();


	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_status();


	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 */
    void Run() override;
	void send_tx(const char *value);
	void send_rx();

    char buffer[64] = {};


private:

    int setup_uart();

    void read_uart_data();







	int probe();

	int collect();

	// SEUK
	void tx_function();
	bool rx_function();
	/**
	 * Sends an i2c measure command to the sensor.
	 */
	int measure();

	int readRegister(const uint8_t reg_address, uint8_t &value);
	int readRegisterMulti(const uint8_t reg_address, uint8_t *value, const uint8_t length);


	bool validate_packet(const uint8_t *packet, ssize_t length);
	// int writeRegister(const uint8_t reg_address, const uint8_t value);
	// int writeRegisterMulti(const uint8_t reg_address, const uint8_t *value, const uint8_t length);



	int sensorInit();
	// int sensorTuning();
	// int singleRefCalibration(const uint8_t byte);
	// int spadCalculations();

	bool _collect_phase{false};
	bool _measurement_started{false};
	bool _new_measurement{true};

	uint8_t _stop_variable{0};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};

    int _uart_fd{-1}; // UART file descriptor, -1 for uninitialized state
    bool tx_pending = false;   // 송신 대기 상태
    uint8_t response_packet[64]; // 수신 데이터 버퍼
    ssize_t response_length = 0; // 수신된 데이터 길이
	uint8_t request_packet[8] = {0xFF, 0xFF, 0x01, 0x04, 0x02, 0x2A, 0x01, 0xD4};
};
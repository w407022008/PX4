/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file tfmini_s.cpp
 *
 * Driver for the downward facing TFmini-S connected via I2C.
 *
 * @author Ze WANG
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
#include <drivers/device/i2c.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>
#include <drivers/rangefinder/PX4Rangefinder.hpp>

static constexpr uint8_t facing_table[] = {
	distance_sensor_s::ROTATION_UPWARD_FACING,
	distance_sensor_s::ROTATION_FORWARD_FACING,
	distance_sensor_s::ROTATION_RIGHT_FACING,
	distance_sensor_s::ROTATION_BACKWARD_FACING,
	distance_sensor_s::ROTATION_LEFT_FACING,
	distance_sensor_s::ROTATION_DOWNWARD_FACING
};

using namespace time_literals;

/*
 * TFmini-S internal constants and data structures.
 */
#define TFMINI_S_BASE_ADDR                 0x10

/* The datasheet gives 500Hz maximum measurement rate, but it's not true according to tech support from Benewake*/
#define TFMINI_S_CONVERSION_INTERVAL	2_ms /* microseconds */

/*
 * Resolution & Limits according to datasheet
 */
#define TFMINI_S_MAX_DISTANCE_M               (12.00f)
#define TFMINI_S_MIN_DISTANCE_M               (0.01f)
#define TFMINI_S_FOV_DEG                      (1.0f)

/* Hardware definitions */


class tfmini_s : public device::I2C, public I2CSPIDriver<tfmini_s>
{
public:
	tfmini_s(const I2CSPIDriverConfig &config);
	~tfmini_s() override;

	static void print_usage();

	int init() override;
	void print_status() override;

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 */
	void RunImpl();

protected:

	virtual int probe() override;

private:
	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void start();

	/**
	 * Issue a measurement command.
	 *
	 * @return      OK if the measurement command was successful.
	 */
	int measure(uint8_t id);

	/**
	 * Collect the result of the most recent measurement.
	 */
	int collect(uint8_t id);

	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address The I2C bus address to probe.
	* @return True if the device is present.
	*/
	int probe_address(const uint8_t address);

	PX4Rangefinder _px4_rangefinder;

	int list[6] {-1,-1,-1,-1,-1,-1};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": comm_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
};

static uint8_t crc8(uint8_t *p, uint8_t len)
{
	uint16_t i;
	uint16_t crc = 0x0;

        for(i=0;i<len;i++)
            crc += p[i];

	return crc & 0xFF;
}

tfmini_s::tfmini_s(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_px4_rangefinder(10, config.rotation)
{
	// Allow retries as the device typically misses the first measure attempts.
	I2C::_retries = 3;

	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_TFMINI_S);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_LASER);
}

tfmini_s::~tfmini_s()
{
	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int tfmini_s::init()
{
	int32_t hw_model = 0;
	param_get(param_find("SENS_EN_TFMINI_S"), &hw_model);

	switch (hw_model) {
	case 0: // Disabled
		PX4_WARN("SENS_EN_TFMINI_S set as 0");
		return PX4_ERROR;

	case 1: // Enable
	{
		int id=0;
		for(int i=0;i<6;i++)
		{
			set_device_address(TFMINI_S_BASE_ADDR+i);
			if (I2C::init() == PX4_OK && measure(i) == PX4_OK) {
				PX4_DEBUG("Enabled the sensor at 0x%02x",TFMINI_S_BASE_ADDR+i);
				list[id++] = i;
			}
		}
		if(list[0] == -1){
			return PX4_ERROR;
		}else{
			_px4_rangefinder.set_fov(TFMINI_S_FOV_DEG);
			_px4_rangefinder.set_max_distance(TFMINI_S_MAX_DISTANCE_M);
			_px4_rangefinder.set_min_distance(TFMINI_S_MIN_DISTANCE_M);
			start();
			return PX4_OK;
		}

		break;
	}
	default:
		PX4_ERR("invalid HW model %" PRId32 ".", hw_model);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void tfmini_s::start()
{
	// Schedule the driver to run on a set interval
	ScheduleOnInterval(TFMINI_S_CONVERSION_INTERVAL);
}


int tfmini_s::probe()
{
	return PX4_OK;
}

void tfmini_s::RunImpl()
{
	for(int i=0;i<6;i++){
		if(list[i]==-1) break;
		if (OK != measure(list[i])){
			PX4_DEBUG("measure error");
			return;
		}
		if (OK != collect(list[i])){
			PX4_DEBUG("collect error");
			return;
		}
	}
}

int tfmini_s::measure(uint8_t id)
{
	/*
	 * Send the command to begin a measurement.
	 */
	set_device_address(TFMINI_S_BASE_ADDR+id);
	const uint8_t cmd[] = {0x5A,0x05,0x00,0x01,0x60};//  output unit: cm
	//const uint8_t cmd[] = {0x5A,0x05,0x00,0x06,0x65};//  output unit: mm
	int ret_val = transfer(&cmd[0], 5, nullptr, 0);

	if (ret_val != PX4_OK) {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret_val);
		return ret_val;
	}

	return PX4_OK;
}

int tfmini_s::collect(uint8_t id)
{
	perf_begin(_sample_perf);

	/* get measurements from the device */
	uint8_t val[9] {};
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	set_device_address(TFMINI_S_BASE_ADDR+id);
	int ret_val = transfer(nullptr, 0, &val[0], 9);

	if (ret_val < 0) {
		PX4_ERR("error reading from sensor: 0x%02x", TFMINI_S_BASE_ADDR+id);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret_val;
	}

	/*
	 * Check if value makes sense according to the FSR and Resolution of
	 * this sensor, discarding outliers, until
	 */
	while (val[0] != 0x59 || val[1] != 0x59){
		PX4_ERR("error reading from sensor: 0x%02X 0x%02X", val[0], val[1]);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return measure(id);
	}


	/* swap data */
        uint16_t distance_cm = (val[3] << 8) | val[2];
	float distance_m = static_cast<float>(distance_cm) * 1e-2f;
        // static float last_dist_m = distance_m;
	int strength = (val[5] << 8) | val[4];
	//uint16_t temperature = ((val[7] << 8) | val[6]) / 8 - 256;

	// Final data quality evaluation. This is based on the datasheet and simple heuristics retrieved from experiments
	// Step 1: Normalize signal strength to 0...100 percent using the absolute signal peak strength.
	uint8_t signal_quality = 1;//100 * strength / 65535.0f;

	// Step 2: Filter physically impossible measurements, which removes some crazy outliers that appear on LL40LS.
        if (distance_cm == 65535 || (strength < 10))
		signal_quality = 0;
        // last_dist_m = distance_m;

	if (crc8(val, 8) == val[8]) {
		if(param_find("TFMINIS_DOW_FIX"))
		{
			_px4_rangefinder.set_device_id(uint32_t(10+5));
			_px4_rangefinder.set_orientation(distance_sensor_s::ROTATION_DOWNWARD_FACING);
		}else
			_px4_rangefinder.set_orientation(facing_table[id]);
		_px4_rangefinder.update(timestamp_sample, distance_m, signal_quality);
	}

	perf_count(_sample_perf);
	perf_end(_sample_perf);

	return ret_val;
}


void tfmini_s::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	for(int i=0;i<6;i++){
		PX4_INFO("list[%d]:%d",i,list[i]);
		PX4_INFO("facing[%d]:%d",i,facing_table[i]);
	}
	PX4_INFO("Dis_min:%f",(double)TFMINI_S_MIN_DISTANCE_M);
	PX4_INFO("Dis_max:%f",(double)TFMINI_S_MAX_DISTANCE_M);
}

void tfmini_s::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

I2C bus driver for TFmini-s rangefinders.

The sensor/driver must be enabled using the parameter SENS_EN_TFMINI_S.

Setup/usage information: https://docs.px4.io/master/en/sensor/tfmini.html
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("tfmini_s", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 0, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int tfmini_s_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = tfmini_s;
	BusCLIArguments cli{true, false};
	cli.rotation = (Rotation)distance_sensor_s::ROTATION_DOWNWARD_FACING;
	cli.default_i2c_frequency = 400000;
	cli.i2c_address = TFMINI_S_BASE_ADDR;

	while ((ch = cli.getOpt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			cli.rotation = (Rotation)atoi(cli.optArg());
			break;
		}
	}

	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIST_DEVTYPE_TFMINI_S);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}

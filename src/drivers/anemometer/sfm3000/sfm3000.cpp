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


#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

#include <drivers/device/i2c.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/drivers/anemometer/PX4Anemometer.hpp>
#include <lib/perf/perf_counter.h>

#include <lib/parameters/param.h>

using namespace time_literals;

/* Configuration Constants */
#define TCA_ADDR                       0x70
#define SFM_BASEADDR                   0x40 // 7-bit address.

/* Device limits */
#define SFM_OFFSET                     (32000.0f)
#define SFM_SCALE                      (140.0f)
#define SFM_SLM2MS                     (0.0546808f)
#define SFM_MEASUREMENT_INTERVAL       2_ms // us

#define ANEMOMETER_MAX_SENSORS         (4)   // Maximum number of sensors on bus
#define TCA9578A_MAX_CHANAL            (8)   // Maximum number of chanals of multiplexer TCA9578A


class SFM3000 : public device::I2C, public I2CSPIDriver<SFM3000>
{
public:
	SFM3000(const I2CSPIDriverConfig &config);
	~SFM3000() override;

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
	int measure();

	/**
	 * Collect the result of the most recent measurement.
	 */
	int collect();

    /**
    * Test whether the device supported by the driver is present at a
    * specific address.
    *
    * @param address The I2C bus address to probe.
    * @return True if the device is present.
    */
    int probe_address(const uint8_t address);

    PX4Anemometer _px4_anemometer;

    bool enable_TCA9578A = true;

    /**
     * Gets the current sensor rotation value.
     */
    int get_sensor_rotation(const size_t index);

    size_t	_sensor_count{0};
    uint8_t _sensor_chanal[ANEMOMETER_MAX_SENSORS];
    uint8_t _sensor_rotations[ANEMOMETER_MAX_SENSORS];

    perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": comm_err")};
    perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};

};

SFM3000::SFM3000(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_px4_anemometer(get_device_id(), config.rotation)
{
	// Allow retries as the device typically misses the first measure attempts.
	I2C::_retries = 3;

	// _px4_anemometer.set_device_type(DRV_ANEMO_DEVTYPE_SFM3000);
}

SFM3000::~SFM3000()
{
	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int SFM3000::get_sensor_rotation(const size_t index)
{
    int32_t _q_sensor_f; // x - forward
    int32_t _q_sensor_r; // y - right
    int32_t _q_sensor_d; // z - down
    param_get(param_find("SFM_FOW_CH"),&_q_sensor_f);
    param_get(param_find("SFM_RIT_CH"),&_q_sensor_r);
    param_get(param_find("SFM_DOW_CH"),&_q_sensor_d); // which channel is connected by the facing-ward sensor
    switch (index) {
    case 0: // channel 1
        return (_q_sensor_f == 1)?windspeed_s::ROTATION_FORWARD_FACING:((_q_sensor_r == 1)?windspeed_s::ROTATION_RIGHT_FACING:((_q_sensor_d == 1)?windspeed_s::ROTATION_DOWNWARD_FACING:128));

    case 1: // channel 2
        return (_q_sensor_f == 2)?windspeed_s::ROTATION_FORWARD_FACING:((_q_sensor_r == 2)?windspeed_s::ROTATION_RIGHT_FACING:((_q_sensor_d == 2)?windspeed_s::ROTATION_DOWNWARD_FACING:128));

    case 2: //channel 3
        return (_q_sensor_f == 3)?windspeed_s::ROTATION_FORWARD_FACING:((_q_sensor_r == 3)?windspeed_s::ROTATION_RIGHT_FACING:((_q_sensor_d == 3)?windspeed_s::ROTATION_DOWNWARD_FACING:128));

    default: return windspeed_s::ROTATION_DOWNWARD_FACING_SECOND;
    }
}

int SFM3000::init()
{
    int32_t hw_model = 0;
    param_get(param_find("SENS_EN_SFM3000"), &hw_model);

    switch (hw_model) {
    case 0: // Disabled
        PX4_WARN("Disabled");
        return PX4_ERROR;

    case 1: // Enable
        set_device_address(TCA_ADDR);

        if (I2C::init() != OK) {
            set_device_address(SFM_BASEADDR);

            if (I2C::init() != OK) {
                PX4_DEBUG(" initialisation failed");
                return PX4_ERROR;

            } else {
                enable_TCA9578A = false;
            }
        }

        if(enable_TCA9578A){
            // Check for connected anemometer on each i2c port,
            // starting from the base address 0x40 and incrementing
            int j=0;
            int i=0;
            while (i < TCA9578A_MAX_CHANAL) {
                // set TCA9578A chanal
                set_device_address(TCA_ADDR);
                uint8_t cmd = 1 << i; // for port #index in tca9548a
                transfer(&cmd, 1, nullptr, 0);

                // Check if a sensor is present.
                set_device_address(SFM_BASEADDR);
                if (probe() != PX4_OK) {
                    PX4_DEBUG("At tca9578a chanal #0%d: There is not any sensor connected", i);
                    i++;
                    continue;
                }

                // Store I2C address
                _sensor_chanal[j] = i;
                _sensor_rotations[j] = get_sensor_rotation(j);
                _sensor_count++;
                j++;
                i++;
            }
        }else{
            // Check if a sensor is present.
            set_device_address(SFM_BASEADDR);
            if (probe() != PX4_OK) {
                PX4_DEBUG("There is not any sensor connected");
            }
            _sensor_count=1;
            _sensor_rotations[0]=windspeed_s::ROTATION_FORWARD_FACING;
            _sensor_chanal[0]=128;
        }
        start();
        break;

    default:
	PX4_ERR("invalid HW model %" PRId32 ".", hw_model);
        return PX4_ERROR;
    }

    return PX4_OK;
}

void SFM3000::start()
{
    // Schedule the driver to run on a set interval
    ScheduleOnInterval(SFM_MEASUREMENT_INTERVAL);
}

int SFM3000::probe()
{
	return PX4_OK;
}

void SFM3000::RunImpl()
{
    // Perform data collection.
    collect();
}

int SFM3000::collect()
{
    uint8_t val[2] {};
    const hrt_abstime timestamp_sample = hrt_absolute_time();
    float measurement[4];
    float confidence[4];
    uint8_t orientation = 128;

    perf_begin(_sample_perf);

    // Increment the sensor index, (limited to the number of sensors connected).
    for (size_t index = 0; index < _sensor_count; index++) {

        if(enable_TCA9578A){
            set_device_address(TCA_ADDR);
            uint8_t cmd = 1 << _sensor_chanal[index]; // for port #_sensor_chanal[index] in tca9548a
            transfer(&cmd, 1, nullptr, 0);
        }
        // Set address of the current sensor to collect data from.
        set_device_address(SFM_BASEADDR);
        // Transfer data from the bus.
        //if (PX4_OK != measure()) return 0;
        int ret_val = transfer(nullptr, 0, &val[0], 2);

        if (ret_val != PX4_OK) {
            PX4_DEBUG("sensor #0%i reading failed, at chanal: #0%d (#0128: disable TCA)", index, _sensor_chanal[index]);
            perf_count(_comms_errors);
            perf_end(_sample_perf);
            return ret_val;
        }

        uint16_t speed = uint16_t(val[0]) << 8 | val[1];
        float speed_m_s = static_cast<float>(speed - SFM_OFFSET) / SFM_SCALE * SFM_SLM2MS;
        orientation = _sensor_rotations[index];

        switch (orientation) {
		    case windspeed_s::ROTATION_FORWARD_FACING:{
		        measurement[0] = speed_m_s;
		        confidence[0] = 1;
		        break;
		        }

		    case windspeed_s::ROTATION_RIGHT_FACING:{
		        measurement[1] = speed_m_s;
		        confidence[1] = 1;
		        break;
		        }

		    case windspeed_s::ROTATION_DOWNWARD_FACING:{
		        measurement[2] = speed_m_s;
		        confidence[2] = 1;
		        break;
		        }

		    case windspeed_s::ROTATION_DOWNWARD_FACING_SECOND:{
		        measurement[3] = speed_m_s;
		        confidence[3] = 1;
		        break;
		        }
        }
    }

    _px4_anemometer.update(timestamp_sample, measurement, confidence, orientation);


    perf_count(_sample_perf);
    perf_end(_sample_perf);

    return PX4_OK;
}

int SFM3000::measure()
{
    // Send the command to begin a measurement.
    uint8_t cmd[2] = {0x10,0x00};
    int ret_val = transfer(&cmd[0], 2, nullptr, 0);

    if (ret_val != PX4_OK) {
        perf_count(_comms_errors);
        PX4_DEBUG("i2c::transfer returned %d", ret_val);
        return ret_val;
    }

    return PX4_OK;
}

void SFM3000::print_status()
{
    for(size_t i=0;i<_sensor_count;i++){
        PX4_INFO("Sensor on address 0x%02x at tca9578a chanal #0%d: Connected", SFM_BASEADDR, _sensor_chanal[i]);
    }

    I2CSPIDriverBase::print_status();
    perf_print_counter(_sample_perf);
    perf_print_counter(_comms_errors);
}

void SFM3000::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

I2C bus driver for SENSIRION SFM3000 Low Pressure DropDigital Flow Meter (i2c).

The sensor/driver must be enabled using the parameter SENS_EN_SFM3000.

Setup/usage information: https://docs.px4.io/master/en/sensor/tfmini.html
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("sfm3000", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("anemometer");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 1, 25, "Sensor rotation - forward facing by default", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int sfm3000_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = SFM3000;
    BusCLIArguments cli{true, false};// I2C ? SPI
    cli.rotation = (Rotation)windspeed_s::ROTATION_FORWARD_FACING;
    cli.default_i2c_frequency = 400000;
	cli.i2c_address = TCA_ADDR;

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

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_ANEMO_DEVTYPE_SFM3000);

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

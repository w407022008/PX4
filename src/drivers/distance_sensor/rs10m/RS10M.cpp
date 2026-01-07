#include "RS10M.hpp"

#include <lib/parameters/param.h>
#include <lib/drivers/device/Device.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <math.h>
#include <cctype>

RS10M::RS10M(const char *port, uint8_t rotation) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_px4_rangefinder(0, rotation)
{
	// store port name
	strncpy(_port, port, sizeof(_port) - 1);

	// enforce null termination
	_port[sizeof(_port) - 1] = '\0';

	device::Device::DeviceId device_id;
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_RS10M;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_LASER);
	_px4_rangefinder.set_min_distance(0.0f);
	_px4_rangefinder.set_max_distance(10.0f); // 10m max range
	_px4_rangefinder.set_fov(math::radians(3.0f)); // Small FOV for laser sensor
}

RS10M::~RS10M()
{
	// make sure we are truly inactive
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
RS10M::init()
{
	// status
	int ret = 0;

	// open fd
	_fd = ::open(_port, O_RDWR | O_NOCTTY);

	if (_fd < 0) {
		PX4_ERR("Error opening fd %s", _port);
		return -1;
	}

	// baudrate 115200, 8 bits, no parity, 1 stop bit
	unsigned speed = B115200;
	termios uart_config{};
	int termios_state{};

	tcgetattr(_fd, &uart_config);

	// clear ONLCR flag (which appends a CR for every LF)
	uart_config.c_oflag &= ~ONLCR;

	// set baud rate
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		PX4_ERR("CFG: %d ISPD", termios_state);
		ret = -1;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		PX4_ERR("CFG: %d OSPD\n", termios_state);
		ret = -1;
	}

	if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
		PX4_ERR("baud %d ATTR", termios_state);
		ret = -1;
	}

	uart_config.c_cflag |= (CLOCAL | CREAD);	// ignore modem controls
	uart_config.c_cflag &= ~CSIZE;
	uart_config.c_cflag |= CS8;			// 8-bit characters
	uart_config.c_cflag &= ~PARENB;			// no parity bit
	uart_config.c_cflag &= ~CSTOPB;			// only need 1 stop bit
	uart_config.c_cflag &= ~CRTSCTS;		// no hardware flowcontrol

	// setup for non-canonical mode
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	uart_config.c_oflag &= ~OPOST;

	// fetch bytes as they become available
	uart_config.c_cc[VMIN] = 1;
	uart_config.c_cc[VTIME] = 1;

	// close the fd
	::close(_fd);
	_fd = -1;

	if (ret == PX4_OK) {
		start();
	}

	return ret;
}

int
RS10M::collect()
{
	perf_begin(_sample_perf);

	// clear buffer if last read was too long ago
	int64_t read_elapsed = hrt_elapsed_time(&_last_read);

	// Temporary buffer to hold incoming data
	char readbuf[256] {};

	// Check the number of bytes available in the buffer
	int bytes_available = 0;
	::ioctl(_fd, FIONREAD, (unsigned long)&bytes_available);

	if (!bytes_available) {
		perf_end(_sample_perf);
		return 0;
	}

	// Read all available data at once
	int ret = ::read(_fd, &readbuf[0], sizeof(readbuf) - 1);

	if (ret < 0) {
		PX4_ERR("read err: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);

		// only throw an error if we time out
		if (read_elapsed > (kCONVERSIONINTERVAL * 2)) {
			/* flush anything in RX buffer */
			tcflush(_fd, TCIFLUSH);
			return ret;
		} else {
			return -EAGAIN;
		}
	}

	_last_read = hrt_absolute_time();

	// Parser state for the frame end sequence " mm\r\n", using switch-based approach
	// This follows the project specification for handling multi-frame data with only last frame extraction
	enum ParseState {
		UNSYNCED = 0,      // Not synchronized with frame
		IN_NUMBER,         // Reading digits of the distance value
		GOT_SPACE,         // Got ' ' (space) - first char of end sequence
		GOT_FIRST_M,       // Got first 'm'
		GOT_SECOND_M,      // Got second 'm'
		GOT_CR,            // Got '\r' (carriage return)
		GOT_LF             // Got '\n' (line feed)
	};

	// Process buffer to find the last complete frame, as per project specification
	// The parser needs to handle scenarios where multiple frames may be in the buffer
	// and extract only the last complete one
	ParseState state = UNSYNCED;
	int num_start = -1;  // Start index of the distance number
	int num_end = -1;    // End index of the distance number
	bool found_frame = false;
	float distance_m = -1.0f;

	for (int i = 0; i < ret; i++) {
		char c = readbuf[i];

		switch (state) {
		case UNSYNCED:
			if (isdigit(c)) {
				// Start of a potential number
				num_start = i;
				state = IN_NUMBER;
			}
			// We don't handle other characters when unsynced to avoid confusion
			break;

		case IN_NUMBER:
			if (isdigit(c)) {
				// Continue reading the number
			} else if (c == ' ') {
				// Number ended, start of frame end sequence
				num_end = i - 1; // Last digit of the number
				// Extract and validate the distance value
				if (num_start >= 0 && num_end >= num_start) {
					// Create a temporary string for the number
					char num_str[16] = {};
					int num_len = num_end - num_start + 1;

					if (num_len > 0 && num_len < 15) {
						memcpy(num_str, &readbuf[num_start], num_len);
						num_str[num_len] = '\0';

						char *endptr;
						float dist_value = strtof(num_str, &endptr);

						// Validate conversion and range (0-10m = 0-10000mm)
						// As per project specification: all sensor data must be validated in 0-10m range
						if ((endptr != num_str) && (dist_value >= 0.0f) && (dist_value <= 10000.0f)) {
							distance_m = dist_value / 1000.0f; // Convert mm to m
							found_frame = true;
						}
					}
				}
				state = GOT_SPACE;
			} else {
				// If we get anything other than a space after the number, reset
				num_start = -1;
				num_end = -1;
				found_frame = false;
				state = UNSYNCED;
			}
			break;

		case GOT_SPACE:
			if (c == 'm') {
				state = GOT_FIRST_M;
			} else {
				// Unexpected, reset to look for number
				num_start = -1;
				num_end = -1;
				found_frame = false;
				// If this character is a digit, start a new number
				if (isdigit(c)) {
					num_start = i;
					state = IN_NUMBER;
				} else {
					state = UNSYNCED;
				}
			}
			break;

		case GOT_FIRST_M:
			if (c == 'm') {
				state = GOT_SECOND_M;
			} else {
				// Unexpected, reset
				num_start = -1;
				num_end = -1;
				found_frame = false;
				// If this character is a digit, start a new number
				if (isdigit(c)) {
					num_start = i;
					state = IN_NUMBER;
				} else {
					state = UNSYNCED;
				}
			}
			break;

		case GOT_SECOND_M:
			if (c == '\r') {
				state = GOT_CR;
			} else if (c == 'm') {
				// If we get another 'm', stay in this state
				state = GOT_SECOND_M;
			} else {
				// Unexpected, reset
				num_start = -1;
				num_end = -1;
				found_frame = false;
				// If this character is a digit, start a new number
				if (isdigit(c)) {
					num_start = i;
					state = IN_NUMBER;
				} else {
					state = UNSYNCED;
				}
			}
			break;

		case GOT_CR:
			if (c == '\n') {
				state = GOT_LF;
			} else {
				// Unexpected, reset
				num_start = -1;
				num_end = -1;
				found_frame = false;
				// If this character is a digit, start a new number
				if (isdigit(c)) {
					num_start = i;
					state = IN_NUMBER;
				} else {
					state = UNSYNCED;
				}
			}
			break;

		case GOT_LF:
			// Frame complete, look for next number
			if (isdigit(c)) {
				// Start of a potential number
				num_start = i;
				num_end = -1;
				state = IN_NUMBER;
			} else {
				// Stay unsynchronized
				state = UNSYNCED;
			}
			break;
		}
	}
	dst = distance_m;
	// If no valid frame was found or the distance is invalid, return error
	if (!found_frame || distance_m < 0.0f) {
		perf_end(_sample_perf);
		return -EAGAIN;
	}

	// Publish the latest valid measurement
	_px4_rangefinder.update(hrt_absolute_time(), distance_m);

	perf_end(_sample_perf);

	return PX4_OK;
}

void
RS10M::start()
{
	// schedule a cycle to start things (the sensor sends at 50Hz, but we run a bit faster to avoid missing data)
	ScheduleOnInterval(15_ms);
}

void
RS10M::stop()
{
	ScheduleClear();
}

void
RS10M::Run()
{
	// fds initialized?
	if (_fd < 0) {
		// open fd
		_fd = ::open(_port, O_RDWR | O_NOCTTY);
	}

	// perform collection
	if (collect() == -EAGAIN) {
		// reschedule to grab the missing bits, time to transmit 9 bytes @ 115200 bps
		ScheduleClear();
		ScheduleOnInterval(15_ms, 87 * 9);
		return;
	}
}

void
RS10M::print_info()
{
	printf("Using port '%s'\n", _port);
	printf("dst: %f", dst);
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */


#include <px4_platform_common/module.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/windspeed.h>

__EXPORT int anemometer_test_main(int argc, char *argv[]);

int anemometer_test_main(int argc, char *argv[])
{
    PX4_INFO("Hello Sky!");

    /* subscribe to sensor_combined topic */
    int _anemometer_sub = orb_subscribe(ORB_ID(windspeed));
    /* limit the update rate to 1000 Hz */
    orb_set_interval(_anemometer_sub, 1);

    hrt_abstime now = hrt_absolute_time();
    int index_updated=0;
    while (hrt_absolute_time()-now<1000000) {
            bool updated;
            orb_check(_anemometer_sub, &updated);
            if(updated){
                // obtained data for the first file descriptor
                struct windspeed_s raw;
                // copy sensors raw data into local buffer
                orb_copy(ORB_ID(windspeed), _anemometer_sub, &raw);
                PX4_INFO("anemometer measurement:\t (%8.4f, %8.4f, %8.4f, %8.4f)",
			(double)raw.measurement_windspeed_x_m_s,
			(double)raw.measurement_windspeed_y_m_s,
			(double)raw.measurement_windspeed_z_m_s,
			(double)raw.measurement_windspeed_zs_m_s);
		index_updated++;
            } else{
                PX4_INFO("not updated!");
            }

    }
    PX4_INFO("with PX4_INFO, total updates %d times in 1 second",index_updated);
    index_updated=0;
    now = hrt_absolute_time();
    while (hrt_absolute_time()-now<1000000) {
            bool updated;
            orb_check(_anemometer_sub, &updated);
            if(updated){
                // obtained data for the first file descriptor
                struct windspeed_s raw;
                // copy sensors raw data into local buffer
                orb_copy(ORB_ID(windspeed), _anemometer_sub, &raw);
                index_updated++;
            }
    }
    PX4_INFO("without PX4_INFO, total updates %d times in 1 second",index_updated);

    return 0;
}

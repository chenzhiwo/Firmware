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

#include <cfloat>
#include <termios.h>

#include <px4_posix.h>
#include <px4_log.h>
#include <px4_getopt.h>
#include <px4_module.h>
#include <px4_module_params.h>

#include <matrix/math.hpp>

#include <drivers/drv_hrt.h>
#include <tunes/tune_definition.h>

#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_trajectory_waypoint.h>
#include <uORB/topics/tune_control.h>
#include <uORB/topics/debug_key_value.h>

#include "ranger_hub.h"

#define BAUD_RATE B57600

static const uint8_t CRC8_TABLE[] = {
        0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31,
        0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
        0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9,
        0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
        0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1,
        0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
        0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe,
        0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
        0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16,
        0x03, 0x04, 0x0d, 0x0a, 0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
        0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80,
        0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
        0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8,
        0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
        0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10,
        0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
        0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f,
        0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
        0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 0xae, 0xa9, 0xa0, 0xa7,
        0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
        0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,
        0xfa, 0xfd, 0xf4, 0xf3
}   ;

static uint8_t crc8(const uint8_t *data, uint8_t len)
{
    uint16_t reg = 0x0;
    uint16_t i;

    while (len--) {
        i = (reg ^ *data++) & 0xff;
        reg = (CRC8_TABLE[i] ^ (reg << 8)) & 0xff;
    }

    return reg & 0xff;
}

class RangerHub : public ModuleBase<RangerHub>, public ModuleParams
{
public:
    RangerHub(const char * serial_port)
    : ModuleParams(nullptr),
      _fd(-1)
    {
        strncpy(_port, serial_port, ARR_LEN(_port));
    }

    virtual ~RangerHub() = default;

    static int task_spawn(int argc, char *argv[])
    {
        _task_id = px4_task_spawn_cmd("ranger_hub",
                                      SCHED_DEFAULT,
                                      SCHED_PRIORITY_DEFAULT,
                                      1536,
                                      (px4_main_t)&run_trampoline,
                                      (char *const *)argv);

        if (_task_id < 0) {
            _task_id = -1;
            return -errno;
        }

        return PX4_OK;
    }

    static RangerHub *instantiate(int argc, char *argv[]){
        bool error_flag = false;
        int ch = 0, opt_idx = 1;
        const char * opt_arg = nullptr;
        const char * serial_port = nullptr;
        RangerHub * instance = nullptr;

        while((ch = px4_getopt(argc, argv, "d:", &opt_idx, &opt_arg)) != EOF)
        {
            switch (ch)
            {
                case 'd':
                    serial_port = opt_arg;
                    break;

                default:
                    error_flag = true;
                    break;
            }
        }

        if(serial_port == nullptr)
        {
            error_flag = true;
        }

        if(error_flag)
        {
            print_usage();
            return nullptr;
        }

        instance = new RangerHub(serial_port);
        return instance;
    }

    void run() override
    {
        bool updated;
        float min_dist;
        enum RangerDirections direction;
        float avoid_direction;
        matrix::Vector3f avoid_dist;

        struct debug_key_value_s dbg;
        strcpy(dbg.key, "min_dist");
        orb_advert_t dbg_pub = {};

        if(!_open_serial())
        {
            return;
        }

        dbg_pub = orb_advertise(ORB_ID(debug_key_value), &dbg);

        _parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
        _trajectory_waypoint_pub = orb_advertise(ORB_ID(vehicle_trajectory_waypoint), &_trajectory_waypoint);
        _tune_control_pub = orb_advertise(ORB_ID(tune_control), &_tune_control);
        _local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));

        parameters_update(_parameter_update_sub, true);

        // Mark vel and acc as unused.
        _trajectory_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity[0] = NAN;
        _trajectory_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity[1] = NAN;
        _trajectory_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity[2] = NAN;

        _trajectory_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_0].acceleration[0] = NAN;
        _trajectory_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_0].acceleration[1] = NAN;
        _trajectory_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_0].acceleration[2] = NAN;

        _trajectory_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_0].yaw_speed = NAN;

        _trajectory_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_0].point_valid = true;
        _trajectory_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_1].point_valid = false;
        _trajectory_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_2].point_valid = false;
        _trajectory_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_3].point_valid = false;
        _trajectory_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_4].point_valid = false;

        _tune_control.timestamp = hrt_absolute_time();
        _tune_control.tune_id = static_cast<int>(TuneID::SINGLE_BEEP);
        _tune_control.strength = tune_control_s::STRENGTH_MAX;
        _tune_control.tune_override = 0;

        orb_publish(ORB_ID(tune_control), _tune_control_pub, &_tune_control);

        while(!should_exit())
        {
            parameters_update(_parameter_update_sub, false);

            // Wait for a new report.
            _read_report();

            // Calculate distances.
            for(size_t i = 0; i < ARR_LEN(_report.distance); i++)
            {
                _range[i] = static_cast<float>(_report.distance[i]) / 1000.0f;
            }

            min_dist = _safety_dist.get();
            direction = RANGER_NONE;

            // Check minimal valid range
            for(int i = RANGER_FRONT; i < RANGER_NONE; i++)
            {
                // Greater than 0.0 and less than minimal range (SAFE_RANGE).
                if((FLT_EPSILON < _range[i]) && (_range[i] < min_dist))
                {
                    min_dist = _range[i];
                    direction = (RangerDirections)i;
                }
            }

            // We are safe.
            if(direction == RANGER_NONE)
            {
                continue;
            }

            // Where do we go?
            switch (direction)
            {
                case RANGER_FRONT:
                    PX4_WARN("FRONT");
                    avoid_direction = M_PI_F;
                    break;

                case RANGER_LEFT:
                    PX4_WARN("LEFT");
                    avoid_direction = M_PI_2_F + M_PI_F;
                    break;

                case RANGER_BACK:
                    PX4_WARN("BACK");
                    avoid_direction = M_PI_F + M_PI_F;
                    break;

                case RANGER_RIGHT:
                    PX4_WARN("RIGHT");
                    avoid_direction = -M_PI_2_F + M_PI_F;
                    break;

                default:
                    continue;
            }

            // Update local position
            orb_check(_local_position_sub, &updated);

            if (updated) {
                orb_copy(ORB_ID(vehicle_local_position), _local_position_sub, &_local_position);
            }

            // Transform to NED frame.
            avoid_dist = matrix::Dcmf(matrix::Eulerf(0.0f, 0.0f, _local_position.yaw + avoid_direction))
                        * matrix::Vector3f(2.0f, 0.0f, 0.0f);

            _trajectory_waypoint.timestamp = hrt_absolute_time();

            _trajectory_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_0].timestamp = _trajectory_waypoint.timestamp;

            _trajectory_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_0].position[0] = _local_position.x + avoid_dist(0);
            _trajectory_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_0].position[1] = _local_position.y + avoid_dist(1);
            _trajectory_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_0].position[2] = _local_position.z;

            _trajectory_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_0].yaw = _local_position.yaw;

            _tune_control.timestamp = _trajectory_waypoint.timestamp;

//            orb_publish(ORB_ID(vehicle_trajectory_waypoint), _trajectory_waypoint_pub, &_trajectory_waypoint);

            orb_publish(ORB_ID(tune_control), _tune_control_pub, &_tune_control);

            dbg.timestamp = _trajectory_waypoint.timestamp;
            dbg.value = min_dist;
            orb_publish(ORB_ID(debug_key_value), dbg_pub, &dbg);
        }

        orb_unsubscribe(_parameter_update_sub);
        orb_unsubscribe(_local_position_sub);
        orb_unadvertise(_trajectory_waypoint_pub);
        orb_unadvertise(_tune_control_pub);

        _close_serial();
    }

    static int custom_command(int argc, char *argv[]){
        PX4_INFO("Custom command WIP");
        return PX4_OK;
    }

    static int print_usage(const char *reason = nullptr){
        PX4_INFO("Usage WIP");
        return PX4_OK;
    }

    int print_status() override
    {
        PX4_INFO("Status WIP");
        return PX4_OK;
    }

private:
    char _port[16];
    int _fd;
    Report _report;
    float _range[8];
    orb_advert_t _tune_control_pub;
    orb_advert_t _trajectory_waypoint_pub;
    int _local_position_sub;
    int _parameter_update_sub;

    vehicle_trajectory_waypoint_s _trajectory_waypoint;
    struct vehicle_local_position_s _local_position;
    struct tune_control_s _tune_control;

    DEFINE_PARAMETERS(
            (ParamFloat<px4::params::SENS_RH_DIST>) _safety_dist
    )

    void parameters_update(int parameter_update_sub, bool force)
    {
        bool updated;
        struct parameter_update_s param_upd = {};

        orb_check(parameter_update_sub, &updated);

        if (updated) {
            orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_upd);
        }

        if (force || updated) {
            updateParams();
        }
    }

    bool _open_serial()
    {
        int fd;
        int ret;
        struct termios params = {};

        fd = ::open(_port, O_RDWR | O_NOCTTY);
        if(fd == -1)
        {
            PX4_ERR("Open %s error.", _port);
            return false;
        }

        ret = isatty(fd);
        if (ret == 0)
        {
            PX4_ERR("%s is not a tty.", _port);
            ::close(fd);
            return false;
        }

        // Get current parameters.
        ret = tcgetattr(fd, &params);
        if (ret == -1)
        {
            PX4_ERR("Get params of %s error.", _port);
            ::close(fd);
            return false;
        }

        // No input processing.
        params.c_iflag = 0;
        // No output processing.
        params.c_oflag = 0;
        // Clear input mode.
        params.c_cflag = 0;
        // Ignore modem control lines.
        params.c_cflag |= CLOCAL;
        // Enable receiver.
        params.c_cflag |= CREAD;
        // Char size 8bit.
        params.c_cflag |= CS8;
        // No line processing.
        params.c_lflag = 0;

        // Timeout 0.1s.
        params.c_cc[VMIN] = 0;
        params.c_cc[VTIME] = 1;

        // Set baud rate.
        cfsetispeed(&params, BAUD_RATE);
        cfsetospeed(&params, BAUD_RATE);

        // Set params.
        ret = tcsetattr(fd, TCSAFLUSH, &params);
        if (ret == -1)
        {
            PX4_ERR("Set params of %s error.", _port);
            ::close(fd);
            return false;
        }

        _fd = fd;
        return true;
    }

    bool _close_serial()
    {
        if(_fd != -1)
        {
            ::close(_fd);
            return true;
        }
        return false;
    }

    void _read_report()
    {
        ssize_t size;

        for(;;) {
            // Check start sign.
            size = ::read(_fd, &_report.T, sizeof(_report.T));
            if((size == 0) || (_report.T != 'T'))
            {
                continue;
            }

            size = 0;
            while(size < static_cast<ssize_t>(sizeof(_report) - sizeof(_report.T)))
            {
                size += ::read(_fd, (uint8_t *)&_report.H + size, sizeof(_report) - sizeof(_report.T) - size);
            }

            // Check CRC
            if(crc8((uint8_t *)&_report, sizeof(_report) - sizeof(_report.crc)) != _report.crc)
            {
                continue;
            }

            // Finished.
            return;
        }
    }
};

int ranger_hub_main(int argc, char *argv[])
{
    return RangerHub::main(argc, argv);
}

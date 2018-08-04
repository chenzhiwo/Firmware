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

#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <matrix/math.hpp>

#include "ranger_hub.h"

#define BAUD_RATE B57600

#define SAFE_RANGE 2.0f
static const char RESPONSE_BYTE = 0xAA;

class RangerHub : public ModuleBase<RangerHub>, public ModuleParams{
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
                                      1024,
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
        float min_range;
        enum RangerDirections direction;
        float avoid_direction;
        matrix::Vector3f avoid_vec;

        if(!_open_serial())
        {
            return;
        }

        _safety_local_position_setpoint_pub = orb_advertise(ORB_ID(vehicle_safety_local_position_setpoint), &_safety_local_position_sp);
        _local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));

        // Fill all fields with NaN
        _safety_local_position_sp.x = NAN;
        _safety_local_position_sp.y = NAN;
        _safety_local_position_sp.z = NAN;
        _safety_local_position_sp.yaw = NAN;
        _safety_local_position_sp.yawspeed = NAN;
        _safety_local_position_sp.vx = NAN;
        _safety_local_position_sp.vy = NAN;
        _safety_local_position_sp.vz = NAN;
        _safety_local_position_sp.acc_x = NAN;
        _safety_local_position_sp.acc_y = NAN;
        _safety_local_position_sp.acc_z = NAN;
        _safety_local_position_sp.thrust[0] = NAN;
        _safety_local_position_sp.thrust[1] = NAN;
        _safety_local_position_sp.thrust[2] = NAN;

        while(!should_exit())
        {
            // Wait for a new report.
            _read_report();

            // Calculate distances.
            for(size_t i = 0; i < _report.header.size; i++)
            {
                // ranger hub can capture up to 100000 per seconds.
                _range[i] = _report.value[i] * (340.0f / 2.0f / 100000.0f);
            }

            min_range = SAFE_RANGE;
            direction = RANGER_NONE;

            // Check minimal valid range
            for(int i = RANGER_FRONT; i < RANGER_NONE; i++)
            {
                // Greater than 0.0 and less than minimal range (SAFE_RANGE).
                if((FLT_EPSILON < _range[i]) && (_range[i] < min_range))
                {
                    min_range = _range[i];
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
            avoid_vec = matrix::Dcmf(matrix::Eulerf(0.0f, 0.0f, _local_position.yaw + avoid_direction))
                        * matrix::Vector3f(10.0f, 0.0f, 0.0f);

            // Obstacle avoidance based on current position (NED frame).

            _safety_local_position_sp.timestamp = hrt_absolute_time();
            _safety_local_position_sp.yaw = _local_position.yaw;
            _safety_local_position_sp.vx = avoid_vec(0);
            _safety_local_position_sp.vy = avoid_vec(1);
            _safety_local_position_sp.vz = avoid_vec(2);

            orb_publish(ORB_ID(vehicle_safety_local_position_setpoint),
                        _safety_local_position_setpoint_pub,
                        &_safety_local_position_sp
            );

            // Sending response
            if(::write(_fd, &RESPONSE_BYTE, 1) == -1)
            {
                PX4_ERR("Sending response error", strerror(errno));
            }
        }

        orb_unsubscribe(_local_position_sub);
        orb_unadvertise(_safety_local_position_setpoint_pub);

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
    ReportBuffer _report;
    float _range[16];
    orb_advert_t _safety_local_position_setpoint_pub;
    int _local_position_sub;

    struct vehicle_local_position_setpoint_s _safety_local_position_sp;
    struct vehicle_local_position_s _local_position;
    matrix::Vector3f _pos;

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

        // Never timeout.
        params.c_cc[VMIN] = 1;
        params.c_cc[VTIME] = 0;

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
            size = ::read(_fd, &_report.header.start, sizeof(_report.header.start));
            if ((_report.header.start != START_SIGN) || (size == -1)) {
                continue;
            }

            // Read and check header.
            size = ::read(_fd, &_report.header.start + sizeof(_report.header.start),
                          sizeof(_report.header) - sizeof(_report.header.start));
            if ((size == -1) ||
                (_report.header.checksum != (_report.header.start + _report.header.type + _report.header.size))) {
                continue;
            }

            // Read values.
            size = ::read(_fd, &_report.value, sizeof(_report.value[0]) * _report.header.size);
            if (size == -1) {
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

#ifndef RL_MASTER_ROBOT_IO_H
#define RL_MASTER_ROBOT_IO_H

#include "robot_types.h"

class RobotIO
{
public:
    virtual ~RobotIO() = default;

    virtual void connect() = 0;

    virtual bool read_state(rl_master::RobotStateData &state) = 0;
    virtual bool read_control_command(rl_master::TeleopCommand &command) = 0;
    virtual int read_mode_command(int fallback_mode) = 0;

    virtual bool write_command(const rl_master::RobotCommandData &command) = 0;

    virtual void estop() = 0;
};

#endif // RL_MASTER_ROBOT_IO_H

#include "fake_dynamixel_handle.hpp"

FakeDynamixelHandle::FakeDynamixelHandle(uint8_t id, uint8_t mode, shared_ptr<dynamixel::PortHandler> port, shared_ptr<dynamixel::PacketHandler> packet)
    : port_handler_(port), packet_handler_(packet), id_(id), mode_(mode)
{
    // Fake Dynamixel :
    //  Use the default constructor + setup instead
}

FakeDynamixelHandle::~FakeDynamixelHandle()
{
}

void FakeDynamixelHandle::simStep(double gain)
{

    std::chrono::duration<double> time_step = std::chrono::system_clock::now() - timer_;
    timer_ = std::chrono::system_clock::now();
    
    // Cast the step duration to millis (uint32_t)
    uint32_t millisec = std::chrono::duration_cast<std::chrono::milliseconds>(time_step).count();

    pos_sim_ = (pos_sim_ + (int)(vel_sim_ * millisec * gain)/10) % 4095;       // RungeKutta Integrator
}

void FakeDynamixelHandle::setup(uint8_t id, uint8_t mode, shared_ptr<dynamixel::PortHandler> port, shared_ptr<dynamixel::PacketHandler> packet)
{
    port_handler_ = port;
    packet_handler_ = packet;
    id_   = id;
    mode_ = mode;
}

string FakeDynamixelHandle::logHeader(uint8_t state)
{
    string status = "[fake_handle ID:" + to_string(id_) + "] ";
    if (state == LOG_SUCCESS)
    {
        return status + "SUCCESS: ";
    }
    else if (state == LOG_ERROR)
    {
        return status + "ERROR: ";
    }
    else if (state == LOG_INFO)
    {
        return status + "INFO: ";
    }

    return status + "DEBUG: ";
}

string FakeDynamixelHandle::init(int32_t init_pos)
{
    pos_sim_ = init_pos;
    return logHeader(LOG_SUCCESS) + "Initialization, Mode: " + (mode_ == POSITION_CONTROL ? "Position" : "Velocity") + " control";
}

string FakeDynamixelHandle::toggleLED()
{
    led_ = !led_;
    return logHeader(LOG_SUCCESS) + "LED toggle: " + (led_ ? "ON" : "OFF");
}

string FakeDynamixelHandle::activate()
{
    torque_ = 1;
    return logHeader(LOG_SUCCESS) + "Torque ON";
}

string FakeDynamixelHandle::deactivate()
{
    torque_ = 0;
    return logHeader(LOG_SUCCESS) + "Torque OFF";
}

string FakeDynamixelHandle::readEncoderPos()
{
    pos_read_ = pos_sim_;
    return logHeader(LOG_INFO) + "Encoder position: " + to_string(pos_read_);
}

string FakeDynamixelHandle::readPosLimit()
{
    return logHeader(LOG_INFO) + "Position limit max: " + to_string(pos_limit_max_) + ", min: " + to_string(pos_limit_min_);
}

float FakeDynamixelHandle::getPosDegree()
{
    readEncoderPos();
    return ((float)(pos_read_ - pos_offset_)) / 4095 * 360;
}

string FakeDynamixelHandle::setPosOffset(uint16_t offset)
{
    pos_offset_ = offset;
    return logHeader(LOG_SUCCESS) + "Position offset set: " + to_string(pos_offset_);
}

string FakeDynamixelHandle::setPosRaw(uint16_t goal)
{
    readPosLimit();
    if (torque_ == 1)
    {
        if (mode_ == POSITION_CONTROL)
        {
            if (goal >= pos_limit_min_ && goal <= pos_limit_max_)
            {
                pos_goal_ = goal;
                pos_sim_ = pos_goal_;

                return logHeader(LOG_SUCCESS) + "Position set, new position: " + to_string(pos_goal_);
            }
            return logHeader(LOG_ERROR) + "Goal position out of bound";
        }
        return logHeader(LOG_ERROR) + "Control mode invalid";
    }
    return logHeader(LOG_ERROR) + "Torque is disabled";
}

string FakeDynamixelHandle::setPosDegree(float goal)
{
    float goal_unit_scale = 0.088; // Degree per unit
    return setPosRaw(static_cast<uint16_t>(floor(goal / goal_unit_scale)) + pos_offset_);
}

string FakeDynamixelHandle::readEncoderVel()
{
    vel_read_ = vel_sim_;

    return logHeader(LOG_INFO) + "Encoder Velocity: " + to_string(vel_read_);
}

string FakeDynamixelHandle::readVelLimit()
{
    return logHeader(LOG_INFO) + "Velocity limit : " + to_string(vel_limit_) + ", " + to_string((float)vel_limit_ * vel_unit_scale_) + " RPM";
}

string FakeDynamixelHandle::setVelRaw(int16_t goal)
{
    readVelLimit();
    if (torque_ == 1)
    {
        if (mode_ == VELOCITY_CONTROL)
        {
            if (goal >= -vel_limit_ && goal <= vel_limit_)
            {
                vel_goal_ = goal;
                vel_sim_ = vel_goal_;


                return logHeader(LOG_SUCCESS) + "Velocity set, new velocity: " + to_string(vel_goal_);
            }
            return logHeader(LOG_ERROR) + "Goal velocity out of bound";
        }
        return logHeader(LOG_ERROR) + "Control mode invalid";
    }
    return logHeader(LOG_ERROR) + "Torque is disabled";
}

string FakeDynamixelHandle::setVelRPM(float goal)
{
    return setVelRaw(static_cast<uint16_t>(floor(goal / vel_unit_scale_)));
}

float FakeDynamixelHandle::getVelRPM()
{
    readEncoderVel();
    return (float)vel_read_ * vel_unit_scale_;
}
#include "dynamixel_handle.hpp"

DynamixelHandle::DynamixelHandle(uint8_t id, uint8_t mode, shared_ptr<dynamixel::PortHandler> port, shared_ptr<dynamixel::PacketHandler> packet)
    : port_handler_(port), packet_handler_(packet), id_(id), mode_(mode)
{
}

DynamixelHandle::~DynamixelHandle()
{
}

void DynamixelHandle::setup(uint8_t id, uint8_t mode, shared_ptr<dynamixel::PortHandler> port, shared_ptr<dynamixel::PacketHandler> packet)
{
    port_handler_ = port;
    packet_handler_ = packet;
    id_ = id;
    mode_ = mode;
}

string DynamixelHandle::logHeader(uint8_t state)
{
    string status = "[dynamixel_handle ID:" + to_string(id_) + "] ";
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

string DynamixelHandle::init()
{
    // Serial port open
    if (!port_handler_->openPort())
    {
        return logHeader(LOG_ERROR) + "Port failed to open";
    }

    // Set baudrate
    if (!port_handler_->setBaudRate(BAUDRATE))
    {
        return logHeader(LOG_ERROR) + "Baudrate setting failed";
    }

    // Ping!
    com_report_ = packet_handler_->ping(
        port_handler_.get(), // Convert shared_ptr to regular ptr
        id_,
        &model_number_,
        &dxl_report_);

    if (com_report_ != COMM_SUCCESS)
    {
        return logHeader(LOG_ERROR) + "Initialization failed";
    }

    // LED off by default
    com_report_ = packet_handler_->write1ByteTxRx(
        port_handler_.get(),
        id_,
        ADDR_LED,
        led_,
        &dxl_report_);

    // Check mode
    uint8_t current_mode;
    com_report_ = packet_handler_->read1ByteTxRx(
        port_handler_.get(),
        id_,
        ADDR_OPERATING_MODE,
        &current_mode,
        &dxl_report_);

    if (current_mode != mode_)
    {
        com_report_ = packet_handler_->write1ByteTxRx(
            port_handler_.get(),
            id_,
            ADDR_OPERATING_MODE,
            mode_,
            &dxl_report_);
    }

    // Torque off by default
    deactivate();

    return logHeader(LOG_SUCCESS) + "Initialization, Mode: " + (mode_ == POSITION_CONTROL ? "Position" : "Velocity") + " control";
}

string DynamixelHandle::toggleLED()
{
    led_ = !led_;
    com_report_ = packet_handler_->write1ByteTxRx(
        port_handler_.get(),
        id_,
        ADDR_LED,
        led_,
        &dxl_report_);

    if (com_report_ != COMM_SUCCESS)
    {
        return logHeader(LOG_ERROR) + "LED toggle failed";
    }

    return logHeader(LOG_SUCCESS) + "LED toggle: " + (led_ ? "ON" : "OFF");
}

string DynamixelHandle::activate()
{
    torque_ = 1;
    com_report_ = packet_handler_->write1ByteTxRx(
        port_handler_.get(),
        id_,
        ADDR_TORQUE_ENABLE,
        torque_,
        &dxl_report_);

    if (com_report_ != COMM_SUCCESS)
    {
        return logHeader(LOG_ERROR) + "Torque enable failed";
    }

    return logHeader(LOG_SUCCESS) + "Torque ON";
}

string DynamixelHandle::deactivate()
{
    torque_ = 0;
    com_report_ = packet_handler_->write1ByteTxRx(
        port_handler_.get(),
        id_,
        ADDR_TORQUE_ENABLE,
        torque_,
        &dxl_report_);

    if (com_report_ != COMM_SUCCESS)
    {
        return logHeader(LOG_ERROR) + "Torque disable failed";
    }

    return logHeader(LOG_SUCCESS) + "Torque OFF";
}

string DynamixelHandle::readEncoderPos()
{
    com_report_ = packet_handler_->read4ByteTxRx(
        port_handler_.get(),
        id_,
        ADDR_PRESENT_POSITION,
        reinterpret_cast<uint32_t *>(&pos_read_),
        &dxl_report_);

    if (com_report_ != COMM_SUCCESS)
    {
        return logHeader(LOG_ERROR) + "Position reading failed";
    }

    return logHeader(LOG_INFO) + "Encoder position: " + to_string(pos_read_);
}

string DynamixelHandle::readPosLimit()
{
    com_report_ = packet_handler_->read4ByteTxRx(
        port_handler_.get(),
        id_,
        ADDR_POS_LIMIT_MAX,
        reinterpret_cast<uint32_t *>(&pos_limit_max_),
        &dxl_report_);

    if (com_report_ != COMM_SUCCESS)
    {
        return logHeader(LOG_ERROR) + "Position limit reading failed";
    }

    com_report_ = packet_handler_->read4ByteTxRx(
        port_handler_.get(),
        id_,
        ADDR_POS_LIMIT_MIN,
        reinterpret_cast<uint32_t *>(&pos_limit_min_),
        &dxl_report_);

    if (com_report_ != COMM_SUCCESS)
    {
        return logHeader(LOG_ERROR) + "Position limit reading failed";
    }

    return logHeader(LOG_INFO) + "Position limit max: " + to_string(pos_limit_max_) + ", min: " + to_string(pos_limit_min_);
}

float DynamixelHandle::getPosDegree()
{
    readEncoderPos();
    return ((float)(pos_read_ - pos_offset_)) / 4095 * 360;
}

string DynamixelHandle::setPosRaw(uint16_t goal)
{
    readPosLimit();
    if (mode_ == POSITION_CONTROL)
    {
        if (goal >= pos_limit_min_ && goal <= pos_limit_max_)
        {
            pos_goal_ = goal;

            com_report_ = packet_handler_->write4ByteTxRx(
                port_handler_.get(),
                id_,
                ADDR_GOAL_POSITION,
                pos_goal_,
                &dxl_report_);

            if (com_report_ != COMM_SUCCESS)
            {
                return logHeader(LOG_ERROR) + "Set position failed";
            }
            return logHeader(LOG_SUCCESS) + "Position set, new position: " + to_string(pos_goal_);
        }
        return logHeader(LOG_ERROR) + "Goal position out of bound";
    }
    return logHeader(LOG_ERROR) + "Control mode invalid";
}

string DynamixelHandle::setPosDegree(float goal)
{
    float goal_unit_scale = 0.088; // Degree per unit
    return setPosRaw(static_cast<uint16_t>(floor(goal / goal_unit_scale)) + pos_offset_);
}

string DynamixelHandle::readEncoderVel()
{
    com_report_ = packet_handler_->read4ByteTxRx(
        port_handler_.get(),
        id_,
        ADDR_PRESENT_VELOCITY,
        reinterpret_cast<uint32_t *>(&vel_read_),
        &dxl_report_);

    if (com_report_ != COMM_SUCCESS)
    {
        return logHeader(LOG_ERROR) + "Velocity reading failed";
    }

    return logHeader(LOG_INFO) + "Encoder Velocity: " + to_string(vel_read_);
}

string DynamixelHandle::readVelLimit()
{
    com_report_ = packet_handler_->read4ByteTxRx(
        port_handler_.get(),
        id_,
        ADDR_VEL_LIMIT,
        reinterpret_cast<uint32_t *>(&vel_limit_),
        &dxl_report_);

    if (com_report_ != COMM_SUCCESS)
    {
        return logHeader(LOG_ERROR) + "Velocity limit reading failed";
    }

    return logHeader(LOG_INFO) + "Velocity limit : " + to_string(vel_limit_) + ", " + to_string((float)vel_limit_ * vel_unit_scale_) + " RPM";
}

string DynamixelHandle::setVelRaw(int16_t goal)
{
    readVelLimit();
    if (mode_ == VELOCITY_CONTROL)
    {
        if (goal >= -vel_limit_ && goal <= vel_limit_)
        {
            vel_goal_ = goal;

            com_report_ = packet_handler_->write4ByteTxRx(
                port_handler_.get(),
                id_,
                ADDR_GOAL_VELOCITY,
                vel_goal_,
                &dxl_report_);

            if (com_report_ != COMM_SUCCESS)
            {
                return logHeader(LOG_ERROR) + "Set velocity failed";
            }
            return logHeader(LOG_SUCCESS) + "Velocity set, new velocity: " + to_string(vel_goal_);
        }
        return logHeader(LOG_ERROR) + "Goal velocity out of bound";
    }
    return logHeader(LOG_ERROR) + "Control mode invalid";
}

string DynamixelHandle::setVelRPM(float goal)
{
    return setVelRaw(static_cast<uint16_t>(floor(goal / vel_unit_scale_)));
}

float DynamixelHandle::getVelRPM()
{
    readEncoderVel();
    return (float)vel_read_ * vel_unit_scale_;
}

string DynamixelHandle::setPosOffset(uint16_t offset)
{
    pos_offset_ = offset;
    return logHeader(LOG_SUCCESS) + "Position offset set: " + to_string(pos_offset_);
}

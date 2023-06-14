#ifndef DYNAMIXEL_HANDLE_HPP_
#define DYNAMIXEL_HANDLE_HPP_

// Control table address for xl430-w250
#define ADDR_OPERATING_MODE     11
#define ADDR_VEL_LIMIT          44
#define ADDR_POS_LIMIT_MAX      48
#define ADDR_POS_LIMIT_MIN      52
#define ADDR_TORQUE_ENABLE      64
#define ADDR_LED                65
#define ADDR_GOAL_VELOCITY      104
#define ADDR_GOAL_POSITION      116
#define ADDR_PRESENT_VELOCITY   128
#define ADDR_PRESENT_POSITION   132

// State def for logHeader
#define LOG_SUCCESS             1
#define LOG_ERROR               2
#define LOG_INFO                3

// Control mode
#define POSITION_CONTROL        3
#define VELOCITY_CONTROL        1

#define BAUDRATE                1000000     

#include <memory>
#include <string>
#include <cmath>

#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace std;

class DynamixelHandle
{
private:
    shared_ptr<dynamixel::PortHandler> port_handler_;
    shared_ptr<dynamixel::PacketHandler> packet_handler_;

    uint8_t                 id_;
    uint8_t                 mode_;
    uint16_t                model_number_;

    bool                    led_ = 0;
    bool                    torque_ = 0;

    int32_t                 pos_read_;
    int32_t                 pos_goal_;
    int32_t                 pos_offset_ = 2047;
    uint32_t                pos_limit_min_ = 0;
    uint32_t                pos_limit_max_ = 4095;

    int16_t                 vel_read_;
    int16_t                 vel_goal_;
    int16_t                 vel_limit_ = 265;
    float                   vel_unit_scale_ = 0.229;        // RPM per unit

    uint8_t                 dxl_report_ = 0;
    int                     com_report_ = COMM_TX_FAIL;

    /**
     * Generates a log header based on the given state.
     * Parameters:
     *   - state: The state of the log (SUCCESS, ERROR, INFO, DEBUG).
     * Returns: A string containing the log header.
     */
    string logHeader(uint8_t state = 0);

public:
    /**
     * Constructor: Initializes a DynamixelHandle object.
     * Parameters:
     *   - id: The ID of the Dynamixel motor.
     *   - mode: The operating mode of the Dynamixel motor (position or velocity control).
     *   - port: A shared pointer to the PortHandler object for communication.
     *   - packet: A shared pointer to the PacketHandler object for packet handling.
     */
    DynamixelHandle() = default;

    DynamixelHandle(uint8_t id,
                    uint8_t mode,
                    shared_ptr<dynamixel::PortHandler> port,
                    shared_ptr<dynamixel::PacketHandler> packet);

    /**
     * Destructor: Cleans up the DynamixelHandle object.
     */
    ~DynamixelHandle();

    void setup(uint8_t id,
                uint8_t mode,
                shared_ptr<dynamixel::PortHandler> port,
                shared_ptr<dynamixel::PacketHandler> packet);

    /**
     * Initializes the Dynamixel motor and sets its mode and torque.
     * Returns: A string indicating the status of the initialization.
     */
    string init();

    /**
     * Toggles the LED of the Dynamixel motor.
     * Returns: A string indicating the status of the LED toggle.
     */
    string toggleLED();

    /**
     * Activates the torque of the Dynamixel motor.
     * Returns: A string indicating the status of the torque.
     */
    string activate();

    /**
     * Deactivates the torque of the Dynamixel motor.
     * Returns: A string indicating the status of the torque.
     */
    string deactivate();

    /**
     * Sets the position offset of the Dynamixel motor.
     * Parameters:
     *   - offset: The offset value to set.
     * Returns: A string indicating the status of the offset set.
     */
    string setPosOffset(uint16_t offset);

    /**
     * Sets the position of the Dynamixel motor (raw value).
     * Parameters:
     *   - goal: The goal position to set.
     * Returns: A string indicating the status of the position set.
     */
    string setPosRaw(uint16_t goal);

    /**
     * Sets the position of the Dynamixel motor in degrees.
     * Parameters:
     *   - goal: The goal position to set in degrees.
     * Returns: A string indicating the status of the position set.
     */
    string setPosDegree(float goal);

    /**
     * Sets the velocity of the Dynamixel motor (raw value).
     * Parameters:
     *   - goal: The goal velocity to set.
     * Returns: A string indicating the status of the velocity set.
     */
    string setVelRaw(int16_t goal);

    /**
     * Reads the encoder position of the Dynamixel motor.
     * Returns: A string indicating the current encoder position.
     */
    string readEncoderPos();

    /**
     * Reads the encoder velocity of the Dynamixel motor.
     * Returns: A string indicating the current encoder velocity.
     */
    string readEncoderVel();

    /**
     * Reads the min and max position limits of the Dynamixel motor.
     * Returns: A string indicating the position limits.
     */
    string readPosLimit();

    /**
     * Reads the velocity limit of the Dynamixel motor.
     * Returns: A string indicating the velocity limit.
     */
    string readVelLimit();

    /**
     * Retrieves the ID of the Dynamixel motor.
     * Returns: The ID of the motor.
     */
    uint8_t getId() { return id_; };

    /**
     * Retrieves the control mode of the Dynamixel motor.
     * Returns: The control mode of the motor.
     */
    uint8_t getMode() { return mode_; };

    /**
     * This function reads the current encoder position of the motor.
     * Returns: The raw position of the motor.
     */
    uint16_t getPosRaw() { readEncoderPos(); return pos_read_; };

    /**
     * Converts the encoder position to degrees.
     * Returns: The current position in degrees.
     */
    float getPosDegree();

    /**
     * This function reads the current encoder velocity of the motor.
     * Returns: The raw velocity of the motor.
     */
    int16_t getVelRaw() { readEncoderVel(); return vel_read_; };

    /**
     * Converts the encoder velocity to RPM.
     * Returns: The current velocity in RPM.
     */
    float getVelRPM();
};

#endif  // DYNAMIXEL_HANDLE_HPP_
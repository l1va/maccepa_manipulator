#ifndef VELOCITY_CONTROL_H
#define VELOCITY_CONTROL_H

#include <fcntl.h>
#include <termios.h>

#include <stdlib.h>
#include <stdio.h>
#include <system_error>
#include <chrono>
#include <thread>
#include <array>
#include <iostream>
#include <string>
#include <math.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "maccepa_msgs/MotorsVel.h"
#include "maccepa_msgs/MotorsPose.h"


struct Driver {
    int id;
    uint8_t goal_velocity[4];
    uint8_t goal_position[4];

    Driver(int index) : id(index) {}
};

class DynamixelMACCEPA {
    int PROTOCOL_VERSION = 2.0;
    int BAUDRATE = 1000000;

    int SET_MODE_ADRESS = 11;
    int OFFSET_ADRESS = 20;
    int TORQUE_ENABLE_ADRESS = 64;
    int GOAL_POSITION_ADRESS = 116;
    int GOAL_VELOCITY_ADRESS = 104;
    int PRESENT_POSITION_ADRESS = 132;
    int VALUES_PER_REVOLUTE = 4096;
    int MAX_VELOCITY = 200;
 
    float RAD_PER_SEC = 0.024;

    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;
    dynamixel::GroupSyncWrite *groupSyncVelWrite;
    dynamixel::GroupSyncWrite *groupSyncPoseWrite;
    dynamixel::GroupSyncRead *groupSyncRead;

    std::string port;
    std::array<Driver, 4> DXLS = {Driver(1), Driver(2), Driver(3), Driver(4)};
    std::array<int,4> dxl_goal_velocity;
    std::array<int,4> dxl_goal_position;
    std::array<int,4> dxl_init_position;
    std::array<int,4> dxl_offset;
    std::array<int,4> dxl_present_position;

    bool velocity_mode = false;
    

public:
    DynamixelMACCEPA(const ros::NodeHandle&);
    ~DynamixelMACCEPA();

    void checkCommAnswer(int, int, int);
    void setVelocity(const std::array<int, 4>&);
    void setPosition(const std::array<int, 4>&);
    void updatePresentPosition(maccepa_msgs::MotorsPose&);
    void initSettings();
    void chatterVelocityCallback(const maccepa_msgs::MotorsVel&);
    void chatterPositionCallback(const maccepa_msgs::MotorsPose&);
    bool velocityMode(){return velocity_mode;};
};

#endif //VELOCITY_CONTROL_H

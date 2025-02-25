#ifndef __INPUT_H
#define __INPUT_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <uav_utils/utils.h>
#include "PX4CtrlParam.h"

/**
 * @brief rec data
 * 
 */
class RC_Data_t
{
private:
    /* data */
public:
    double mode;    // mode: check is in hover, (if ch[4] > API_MODE_THRESHOLD_VALUE, in hover mode)
    double gear;    // gear: check if in command mode,(if in hover and ch[5] > GEAR_SHIFT_VALUE, in command mode )
    double reboot_cmd;  // check if in reboot mode, (if ch[7] > REBOOT_THRESHOLD_VALUE, toggle reboot)
    double last_mode;
    double last_gear;
    double last_reboot_cmd;
    bool have_init_last_mode{false};
    bool have_init_last_gear{false};
    bool have_init_last_reboot_cmd{false};
    double ch[4];

    mavros_msgs::RCIn msg;
    ros::Time rcv_stamp;

    bool is_command_mode;
    bool enter_command_mode;
    bool is_hover_mode;
    bool enter_hover_mode;
    bool toggle_reboot;

    static constexpr double GEAR_SHIFT_VALUE = 0.75;
    static constexpr double  API_MODE_THRESHOLD_VALUE = 0.75;
    static constexpr double REBOOT_THRESHOLD_VALUE = 0.5;
    static constexpr double DEAD_ZONE = 0.25;
    RC_Data_t(/* args */);
    ~RC_Data_t(){};
    void check_validity();
    bool check_centered();
    void feed(mavros_msgs::RCInConstPtr pMsg);
    bool is_received(const ros::Time &now_time);
};

class Odom_Data_t
{
private:
    /* data */
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Quaterniond q;
    Eigen::Vector3d w;

    nav_msgs::Odometry msg;
    ros::Time rcv_stamp;
    bool recv_new_msg;

    Odom_Data_t(/* args */);
    ~Odom_Data_t(){};
    void feed(nav_msgs::OdometryConstPtr pMsg);
};

class Start_Trigger_Data_t
{
public:
  bool recv_start_trig;

  Start_Trigger_Data_t();
  void feed(geometry_msgs::PoseStampedConstPtr pMsg);
};

class Cmd_Trigger_Data_t
{
public:
  bool recv_cmd_trig;

  Cmd_Trigger_Data_t();
  void feed(geometry_msgs::PoseStampedConstPtr pMsg);
};

class Emergency_Landing_t
{
private:
    /* data */
public:
    std_msgs::Bool msg;

    ros::Time rcv_stamp;
    bool flag_emergency_landing;

    Emergency_Landing_t(/* args */);
    ~Emergency_Landing_t(){};
    void feed(std_msgs::BoolConstPtr pMsg);
};

class Imu_Data_t
{
private:
    /* data */
public:
    Eigen::Quaterniond q;
    Eigen::Vector3d w;
    Eigen::Vector3d a;

    sensor_msgs::Imu msg;
    ros::Time rcv_stamp;

    Imu_Data_t(/* args */);
    ~Imu_Data_t(){};
    void feed(sensor_msgs::ImuConstPtr pMsg);
};

class Keyboard_t
{
private:
    /* data */
public:
    bool trigger_;
    bool land_trigger_;
    bool start_mission_;
    bool emengercy_trigger_;
    int image_yaw_state_;

    std_msgs::String msg;
    ros::Time rcv_stamp;

    Keyboard_t(/* args */);
    ~Keyboard_t(){};
    void feed(std_msgs::StringConstPtr pMsg);
};

class State_Data_t
{
private:
    /* data */
public:

    mavros_msgs::State current_state;
    mavros_msgs::State state_before_offboard;


    State_Data_t(/* args */);
    ~State_Data_t(){};
    void feed(mavros_msgs::StateConstPtr pMsg);
};

class ExtendedState_Data_t
{
private:
    /* data */
public:
    mavros_msgs::ExtendedState current_extended_state;

    ExtendedState_Data_t(/* args */);
    ~ExtendedState_Data_t(){};
    void feed(mavros_msgs::ExtendedStateConstPtr pMsg);
};

class Command_Data_t
{
private:
    /* data */
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d j;
    double yaw;
    double yaw_rate;

    quadrotor_msgs::PositionCommand msg;
    ros::Time rcv_stamp;

    Command_Data_t(/* args */);
    ~Command_Data_t(){};
    void feed(quadrotor_msgs::PositionCommandConstPtr pMsg);
};

class Battery_Data_t
{
private:
    /* data */
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double volt{0.0};
    double percentage{0.0};

    sensor_msgs::BatteryState msg;
    ros::Time rcv_stamp;

    Battery_Data_t(/* args */);
    ~Battery_Data_t(){};
    void feed(sensor_msgs::BatteryStateConstPtr pMsg);
};

class Takeoff_Land_Data_t
{
private:
    /* data */
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool triggered{false};
    uint8_t takeoff_land_cmd;

    quadrotor_msgs::TakeoffLand msg;
    ros::Time rcv_stamp;

    Takeoff_Land_Data_t(/* args */);
    ~Takeoff_Land_Data_t(){};
    void feed(quadrotor_msgs::TakeoffLandConstPtr pMsg);
};





#endif

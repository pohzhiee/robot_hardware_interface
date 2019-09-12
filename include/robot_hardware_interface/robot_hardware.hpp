#ifndef gazebo_robot_v1_hpp
#define gazebo_robot_v1_hpp

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/joint_state_handle.hpp>
#include <hardware_interface/joint_command_handle.hpp>
#include <hardware_interface/robot_hardware.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vector>
#include <string>

namespace robot_hw_interface
{

class MyRobotHardware : public hardware_interface::RobotHardware
{
public:
    MyRobotHardware() = default;
    ~MyRobotHardware() = default;
    hardware_interface::hardware_interface_ret_t init() override;
    hardware_interface::hardware_interface_ret_t read() override;
    hardware_interface::hardware_interface_ret_t write() override;

protected:

    //interfaces
    std::vector<hardware_interface::JointStateHandle> joint_state_handles_;
    std::vector<hardware_interface::JointCommandHandle> joint_command_handles_;

    std::vector<double> pos_;
    std::vector<double> vel_;
    std::vector<double> eff_;
    std::vector<double> cmd_;

    unsigned int num_joints;
    std::vector<std::string> joint_names;

    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> publishers_;
    rclcpp::Node::SharedPtr node_;
};
} // namespace robot_hw_interface

#endif
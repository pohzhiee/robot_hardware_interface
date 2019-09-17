#ifndef gazebo_robot_v1_hpp
#define gazebo_robot_v1_hpp

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/joint_state_handle.hpp>
#include <hardware_interface/joint_command_handle.hpp>
#include <hardware_interface/robot_hardware.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
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

    void set_pos(double &val, unsigned int index);
    void set_vel(double &val, unsigned int index);
    void set_eff(double &val, unsigned int index);
    void set_cmd(double &val, unsigned int index);
    std::pair<int32_t, uint32_t> get_update_time_elapsed();
protected:

    //interfaces
    std::vector<hardware_interface::JointStateHandle> joint_state_handles_;
    std::vector<hardware_interface::JointCommandHandle> joint_command_handles_;

    // std::vector<double> pos_;
    // std::vector<double> vel_;
    // std::vector<double> eff_;
    // std::vector<double> cmd_;
    std::array<double, 16> pos_;
    std::array<double, 16> vel_;
    std::array<double, 16> eff_;
    std::array<double, 16> cmd_;

    int32_t prev_update_sec;
    uint32_t prev_update_nsec;
    int32_t update_period_sec;
    uint32_t update_period_nsec;

    unsigned int num_joints;
    std::vector<std::string> joint_names_;

    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> publishers_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> subscriptions_;
    rclcpp::Node::SharedPtr node_, subscriber_node_;
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
private:
    std::vector<std::string> get_joint_names(std::string &robot_name);
    void register_joint_handles();
    void initialise_vectors();
    void create_cmd_pubs(std::string &robot_name);

    void joint_state_subscription_callback(sensor_msgs::msg::JointState::UniquePtr msg);

    std::future<void> future_handle_;
};
} // namespace robot_hw_interface

#endif
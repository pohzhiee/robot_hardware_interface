#include "ros2_control_robot/robot_hardware.hpp"
#include "parameter_server_interfaces/srv/get_all_joints.hpp"
#include <chrono>
namespace robot_hw_interface
{
using namespace std::chrono_literals;
hardware_interface::hardware_interface_ret_t MyRobotHardware::init()
{
    node_ = std::make_shared<rclcpp::Node>("robot_node");
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    // TODO: fix the robot name hardcoding
    std::string robotName = "lobot";
    // Get all the joint names Hardware the robot
    joint_names_ = get_joint_names(robotName);
    // Initialise all the related vectors to the correct size
    initialise_vectors();
    // Register all the handles
    register_joint_handles();
    // Create the subscription to joint states
    subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(), std::bind(&MyRobotHardware::joint_state_subscription_callback, this, std::placeholders::_1));
    
    cmd_publisher_ = node_->create_publisher<ros2_control_interfaces::msg::JointCommands>("/" + robotName + "/sim/cmd", rclcpp::SensorDataQoS());

    executor_->add_node(node_);
    future_handle_ = std::async(std::launch::async,[](std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe) { exe->spin(); } , executor_);

    return 0;
} // namespace robot_hw_interface

std::vector<std::string> MyRobotHardware::get_joint_names(std::string &robot_name)
{
    using getAllJoints = parameter_server_interfaces::srv::GetAllJoints;
    auto client = node_->create_client<getAllJoints>("/GetAllControlJoints");
    auto joint_names = std::vector<std::string>();
    auto retryCount = 0;
    while(retryCount < 8){
        client->wait_for_service(1s);
        if (client->service_is_ready())
        {
            auto req = std::make_shared<parameter_server_interfaces::srv::GetAllJoints::Request>();
            req->robot = robot_name;
            auto resp = client->async_send_request(req);
            RCLCPP_INFO(node_->get_logger(), "(MyRobotHardware) Sending async request...");
            auto spin_status = rclcpp::executors::spin_node_until_future_complete(*executor_, node_, resp, 5s);
            if (spin_status == rclcpp::executor::FutureReturnCode::SUCCESS)
            {
                auto status = resp.wait_for(1s);
                if (status == std::future_status::ready)
                {
                    auto res = resp.get();
                    joint_names = res->joints;
                    return joint_names;
                }
                else
                {
                    RCLCPP_ERROR(node_->get_logger(), "GetAllJoints service failed to execute");
                    retryCount++;
                }
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "GetAllJoints service failed to execute (spin failed)");
                retryCount++;
            }
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "GetAllJoints service failed to start, check that parameter server is launched");
            retryCount++;
        }
    }
    return joint_names;
}

void MyRobotHardware::register_joint_handles()
{
    for (size_t i = 0; i < joint_names_.size(); i++)
    {
        hardware_interface::JointStateHandle state_handle(joint_names_[i], &(pos_[i]), &(vel_[i]), &(eff_[i]));
        joint_state_handles_[i] = state_handle;
        if (register_joint_state_handle(&joint_state_handles_[i]) != hardware_interface::HW_RET_OK)
        {
            throw std::runtime_error("unable to register " + joint_state_handles_[i].get_name());
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "Registered joint_state_handle for %s, %u", joint_names_[i].c_str(), i);
        }
        hardware_interface::JointCommandHandle command_handle(joint_names_[i], &(cmd_[i]));
        joint_command_handles_[i] = command_handle;
        if (register_joint_command_handle(&joint_command_handles_[i]) !=
            hardware_interface::HW_RET_OK)
        {
            throw std::runtime_error("unable to register " + joint_command_handles_[i].get_name());
        }
        else
        {
            // RCLCPP_INFO(node_->get_logger(), "Registered joint_command_handle for %s, %u", joint_names_[i].c_str(), i);
        }
    }
}

void MyRobotHardware::initialise_vectors()
{
    auto length = joint_names_.size();
    joint_state_handles_ = std::vector<hardware_interface::JointStateHandle>();
    joint_state_handles_.resize(length);
    joint_command_handles_ = std::vector<hardware_interface::JointCommandHandle>();
    joint_command_handles_.resize(length);
    pos_ = std::vector<double>();
    pos_.resize(length);
    vel_ = std::vector<double>();
    vel_.resize(length);
    eff_ = std::vector<double>();
    eff_.resize(length);
    cmd_ = std::vector<double>();
    cmd_.resize(length);
}

void MyRobotHardware::joint_state_subscription_callback(sensor_msgs::msg::JointState::UniquePtr msg)
{
    // Update position states
    {
        auto pos_size = msg->position.size();
        auto pos_min_size = std::min(pos_size, pos_.size());
        if (pos_size > pos_.size())
        {
            RCLCPP_WARN_ONCE(node_->get_logger(), "Number of position data from joint states exceed number of joints in robot hardware, truncating...");
        }
        if (pos_size < pos_.size())
        {
            RCLCPP_WARN_ONCE(node_->get_logger(), "Number of position data from joint states is lower than number of joints in robot hardware");
        }
        for (size_t i = 0; i < pos_min_size; i++)
        {
            for(size_t j = 0;j<msg->name.size();j++){
                auto match = joint_state_handles_[i].get_name().compare(msg->name[j])==0;
                if(match){
                    pos_[i] = msg->position[j];
                    break;
                }
            }
        }
        // RCLCPP_INFO(node_->get_logger(), "Pos 0 set to: %f", (double)(pos_[0]));
    }
    // Update velocity states
    {
        auto vel_size = msg->velocity.size();
        auto vel_min_size = std::min(vel_size, pos_.size());
        if (vel_size > vel_.size())
        {
            RCLCPP_WARN_ONCE(node_->get_logger(), "Number of velocity data from joint states exceed number of joints in robot hardware, truncating...");
        }
        if (vel_size < vel_.size())
        {
            RCLCPP_WARN_ONCE(node_->get_logger(), "Number of velocity data from joint states is lower than number of joints in robot hardware");
        }
        for (size_t i = 0; i < vel_min_size; i++)
        {
            for(size_t j = 0;j<msg->name.size();j++){
                auto match = joint_state_handles_[i].get_name().compare(msg->name[j])==0;
                if(match){
                    vel_[i] = msg->velocity[j];
                    break;
                }
            }
        }
    }
/*     // Update effort states
    {
        auto eff_size = msg->effort.size();
        auto eff_min_size = std::min(eff_size, pos_.size());
        if (eff_size > eff_.size())
        {
            RCLCPP_WARN_ONCE(node_->get_logger(), "Number of effort data from joint states exceed number of joints in robot hardware, truncating...");
        }
        if (eff_size < eff_.size())
        {
            RCLCPP_WARN_ONCE(node_->get_logger(), "Number of effort data from joint states is lower than number of joints in robot hardware");
        }
        for (size_t i = 0; i < eff_min_size; i++)
        {
            eff_[i] = (msg->effort[i]);
            // eff_[i] = msg->effort[i];
        }
    }
    // Update time
    {
        update_period_nsec = msg->header.stamp.nanosec - prev_update_nsec;
        update_period_sec = msg->header.stamp.sec - prev_update_sec;

        prev_update_nsec = msg->header.stamp.nanosec;
        prev_update_sec = msg->header.stamp.sec;
    } */
}

hardware_interface::hardware_interface_ret_t MyRobotHardware::read()
{
    // Currently does nothing because the joint state subscription writes directly to the buffer that is supposed to be read by the controllers
    return 0;
}

hardware_interface::hardware_interface_ret_t MyRobotHardware::write()
{
    // RCLCPP_INFO(node_->get_logger(), "Write called");
    // for(size_t i = 0;i<joint_names_.size();i++){
    auto msg = ros2_control_interfaces::msg::JointCommands();
    for(auto &cmd : joint_command_handles_){
        msg.joint_names.push_back(cmd.get_name());
        msg.commands.push_back(cmd.get_cmd());
    }
    cmd_publisher_->publish(msg);
        // RCLCPP_INFO(node_->get_logger(), "Publishing msg...");
    // }
    return 0;
}

void MyRobotHardware::set_pos(double &val, unsigned int index)
{
    pos_[index] = (val);
}

void MyRobotHardware::set_vel(double &val, unsigned int index)
{
    vel_[index] = (val);
}

void MyRobotHardware::set_eff(double &val, unsigned int index)
{
    eff_[index] = (val);
}

void MyRobotHardware::set_cmd(double &val, unsigned int index)
{
    cmd_[index] = (val);
}

std::pair<int32_t, uint32_t> MyRobotHardware::get_update_time_elapsed()
{
    auto pair = std::make_pair(update_period_sec, update_period_nsec);
    return pair;
}

} // namespace robot_hw_interface

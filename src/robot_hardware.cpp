#include "robot_hardware_interface/robot_hardware.hpp"
#include "parameter_server_interfaces/srv/get_all_joints.hpp"
#include <chrono>
namespace robot_hw_interface
{
using namespace std::chrono_literals;
hardware_interface::hardware_interface_ret_t MyRobotHardware::init()
{
    using getAllJoints = parameter_server_interfaces::srv::GetAllJoints;
    node_ = std::make_shared<rclcpp::Node>("robot_node");
    auto robotName = "lobot";
    auto client  = node_->create_client<getAllJoints>("/GetAllControlJoints");
    client->wait_for_service(1s);
    // Get all the joint names for the robot
    if (client->service_is_ready())
    {
        auto req = std::make_shared<parameter_server_interfaces::srv::GetAllJoints::Request>();
        req->robot = robotName;
        auto resp = client->async_send_request(req);
        RCLCPP_INFO(node_->get_logger(), "Sending async request...");
        auto spin_status = rclcpp::spin_until_future_complete(node_, resp, 5s);
        if (spin_status == rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            auto status = resp.wait_for(1s);
            if (status == std::future_status::ready)
            {
                auto res = resp.get();
                joint_names = res->joints;
                for (auto &j : res->joints)
                {
                    RCLCPP_INFO(node_->get_logger(), "Joint: %s", j.c_str());
                }
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "GetAllJoints service failed to execute");
            }
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "GetAllJoints service failed to execute (spin failed)");
        }
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "GetAllJoints service failed to start, check that parameter server is launched");
    }


    // for (auto &joint_name : joint_names)
    // {
    //     hardware_interface::JointStateHandle state_handle(joint_name, &pos_[i], &vel_[i], &eff_[i]);
    //     joint_state_handles_[i] = state_handle;
    //     if (register_joint_state_handle(&joint_state_handles_[i]) != hardware_interface::HW_RET_OK)
    //     {
    //         throw std::runtime_error("unable to register " + joint_state_handles_[i].get_name());
    //     }

    //     hardware_interface::JointCommandHandle command_handle(joint_name, &cmd_[i]);
    //     joint_command_handles_[i] = command_handle;
    //     if (register_joint_command_handle(&joint_command_handles_[i]) !=
    //         hardware_interface::HW_RET_OK)
    //     {
    //         throw std::runtime_error("unable to register " + joint_command_handles_[i].get_name());
    //     }
    //     ++i;
    // }
    // node_ = std::make_shared<rclcpp::Node>("robot_node");
    return 0;
}

hardware_interface::hardware_interface_ret_t MyRobotHardware::read()
{
    // do robot specific stuff to update the pos_, vel_, eff_ arrays
    // auto msg1 = std_msgs::msg::Float64();
    // msg1.data = pos_[0];

    // auto msg2 = std_msgs::msg::Float64();
    // msg2.data = pos_[1];

    return 0;
}

hardware_interface::hardware_interface_ret_t MyRobotHardware::write()
{
    // do robot specific stuff to apply the command values from cmd_ to the robot
    /* for (unsigned int i = 0; i < joint_state_handles_.size(); i++)
    {
        auto stateHandle = joint_state_handles_[i];
        auto pos = stateHandle.get_position();
        pos_[i] = pos + cmd_[i] * 0.03;
    }*/
    return 0; 
}
} // namespace robot_hw_interface

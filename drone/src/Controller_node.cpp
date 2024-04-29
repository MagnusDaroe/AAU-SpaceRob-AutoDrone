#include <chrono>
#include <memory>
#include <math.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "drone/msg/drone_control_data.hpp"
#include "drone/msg/drone_command.hpp"

using namespace std::chrono_literals;

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode() : Node("controller_node")
    {
        // Subscribe to altitude reference and measurement topics
        Data_subscription_ = this->create_subscription<drone::msg::DroneControlData>("/DroneControlData", 10, std::bind(&ControllerNode::DataCallback, this, std::placeholders::_1));

        // Publish regulated altitude control value
        Control_publisher_ = this->create_publisher<drone::msg::DroneCommand>("regulated_altitude_control", 10);
    }

private:
    //reference values
    float x_ref = 1200;
    float y_ref = 1000;
    float z_ref = 500;

    float altitude_control_value = 0;
    float regulator_z_value;
    float integral;
    float prev_z_error;

    float local_error_x;
    float local_error_y;
    float pitch_angle;
    float roll_angle;
    float prev_pitch_angle;
    float prev_roll_angle;
    float regulator_pitch_value;
    float regulator_roll_value;

    rclcpp::Subscription<drone::msg::DroneControlData>::SharedPtr Data_subscription_;
    rclcpp::Publisher<drone::msg::DroneCommand>::SharedPtr Control_publisher_;

    //Controller functions
    void DataCallback(const drone::msg::DroneControlData::SharedPtr msg)
    {
        float x_pos = msg->vicon_x;
        float y_pos = msg->vicon_y;
        float z_pos = msg->vicon_z;
        float yaw = msg->vicon_yaw;

        //std::cout << "x: " << pos_x << " y: " << pos_y << " yaw: " << yaw << "z_ref: " << z_ref << std::endl;


        control_value_regulated(z_ref, z_pos);
        z_error_to_controller_value(ControllerNode::altitude_control_value);
        
        globalErrorToLocalError(x_ref, y_ref, x_pos, y_pos, yaw);
        localErrorToAngle(local_error_x, local_error_y);
        anglePD(pitch_angle, roll_angle);
        
        //RCLCPP_DEBUG(ControllerNode->get_logger(), "Regulator pitch value: %d", regulator_pitch_value);
        //RCLCPP_DEBUG(ControllerNode->get_logger(), "Regulator roll value: %d", regulator_roll_value);
        //RCLCPP_DEBUG(ControllerNode->get_logger(), "Regulator altitude value: %d", regulator_z_value);

        //std::cout << "Regulator pitch value: " << regulator_pitch_value << std::endl;
        //std::cout << "Regulator roll value: " << regulator_roll_value << std::endl;
        //std::cout << "Regulator altitude value: " << regulator_z_value << std::endl;

        // Publish regulated pitch, roll, and thrust values
        auto control_msg = drone::msg::DroneCommand();
        control_msg.cmd_auto_roll = regulator_pitch_value;
        control_msg.cmd_auto_pitch = regulator_roll_value;
        control_msg.cmd_auto_thrust = regulator_z_value;
        Control_publisher_->publish(control_msg);
    }

    void z_error_to_controller_value(float regulator_z_value)
    {
        int thrust_to_hover = 480;
        int max_value = 200;
        
        std::cout << "regulartor_z_value: " << regulator_z_value << std::endl;
        if (regulator_z_value > max_value)
        {
            altitude_control_value = thrust_to_hover + max_value;
        }
        else if (regulator_z_value < -max_value)
        {
            altitude_control_value = thrust_to_hover - max_value;
        }
        else
        {
            altitude_control_value = thrust_to_hover + regulator_z_value;
        }
        std::cout << "altitude_control_value: " << altitude_control_value << std::endl;
    }

    void control_value_regulated(float z_ref, float z_pos)
    {
        float z_error = z_ref - z_pos;
        float Kp_altitude = 100;
        float Ki_altitude = 0;
        float Kd_altitude = 80;

        integral += z_error;
        regulator_z_value = Kp_altitude * z_error + Ki_altitude * integral + Kd_altitude * (z_error - prev_z_error);
        prev_z_error = z_error;
    }

    //XY_controller functions    
    void globalErrorToLocalError(float x_ref, float y_ref, float x_global_mes, float y_global_mes, float yaw_mes)
    {
        float x_global_error = x_ref - x_global_mes;
        float y_global_error = y_ref - y_global_mes;
        float roll = 0;
        float pitch = 0;
        float yaw = yaw_mes;

        float inv_R[3][3] = {{cos(pitch) * cos(yaw), cos(pitch) * sin(yaw), -sin(pitch)},                                                                                       //Inverse rotation matrix, which takes radians
                             {cos(yaw) * sin(pitch) * sin(roll) - cos(roll) * sin(yaw), cos(roll) * cos(yaw) + sin(pitch) * sin(roll) * sin(yaw), cos(pitch) * sin(roll)},
                             {sin(roll) * sin(yaw) + cos(roll) * cos(yaw) * sin(pitch), cos(roll) * sin(pitch) * sin(yaw) - cos(yaw) * sin(roll), cos(pitch) * cos(roll)}};

        float global_error_vec[3][1] = {{x_global_error},
                                        {y_global_error},
                                        {0}};

        float result[3][1];
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 1; j++)
            {
                result[i][j] = 0;

                for (int k = 0; k < 3; k++)
                {
                    result[i][j] += inv_R[i][k] * global_error_vec[k][j];
                }
            }
        }

        local_error_x = result[0][0];
        local_error_y = result[1][0];
    }

    void localErrorToAngle(float local_error_x, float local_error_y)
    {
        int max_error = 200;
        int max_angle = 20;

        if (local_error_x > max_error)
        {
            pitch_angle = max_angle;
        }
        else if (local_error_x < -max_error)
        {
            pitch_angle = -max_angle;
        }
        else if (local_error_y > max_error)
        {
            roll_angle = max_angle;
        }
        else if (local_error_y < -max_error)
        {
            roll_angle = -max_angle;
        }
        else
        {
            pitch_angle = local_error_x / 10;
            roll_angle = local_error_y / 10;
        }
    }

    void anglePD(float pitch_angle, float roll_angle)
    {
        float Kp_pitch = 0;
        float Kd_pitch = 1;
        float Kp_roll = 0;
        float Kd_roll = 1;

        regulator_pitch_value = Kp_pitch * pitch_angle + Kd_pitch * (pitch_angle - prev_pitch_angle);
        regulator_roll_value = Kp_roll * roll_angle + Kd_roll * (roll_angle - prev_roll_angle);

        prev_pitch_angle = pitch_angle;
        prev_roll_angle = roll_angle;
    } 
};

int main(int argc, char *argv[])
{
    std::cout << "Controller_node starting up..." << std::endl;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

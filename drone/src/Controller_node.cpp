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
    float x_ref = 100;
    float y_ref = 100;
    float z_ref = 500;

    float altitude_control_value = 0;
    float regulator_z_value;
    float integral;
    float prev_z_error;

    float local_error_x;
    float local_error_y;
    float pitch_controller_value;
    float roll_controller_value;
    float prev_local_error_x;
    float prev_local_error_y;
    float regulated_x_value;
    float regulated_y_value;

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
        z_error_to_controller_value(ControllerNode::regulator_z_value);
        
        globalErrorToLocalError(x_ref, y_ref, x_pos, y_pos, yaw);
        anglePD(local_error_x, local_error_y);
        localErrorToAngle(regulated_x_value, regulated_y_value);
        
        //RCLCPP_DEBUG(ControllerNode->get_logger(), "Regulator pitch value: %d", regulator_pitch_value);
        //RCLCPP_DEBUG(ControllerNode->get_logger(), "Regulator roll value: %d", regulator_roll_value);
        //RCLCPP_DEBUG(ControllerNode->get_logger(), "Regulator altitude value: %d", regulator_z_value);

        std::cout << "pitch_controller_value: " << pitch_controller_value << std::endl;
        std::cout << "roll_controller_value: " << roll_controller_value << std::endl;
        std::cout << "altitude_controller_value: " << altitude_control_value << std::endl;

        // Publish regulated pitch, roll, and thrust values
        auto control_msg = drone::msg::DroneCommand();
        control_msg.cmd_auto_roll = pitch_controller_value;
        control_msg.cmd_auto_pitch = roll_controller_value;
        control_msg.cmd_auto_thrust = altitude_control_value;
        Control_publisher_->publish(control_msg);
    }

    void z_error_to_controller_value(float regulator_z_value)
    {
        float thrust_to_hover = 480;
        float max_value = 30000;
        float max_thrust = 200;
        if (regulator_z_value > max_value)
        {
            altitude_control_value = thrust_to_hover + max_thrust;
        }
        else if (regulator_z_value < -max_value)
        {
            altitude_control_value = thrust_to_hover - max_thrust;
        }
        else
        {
            altitude_control_value = thrust_to_hover + regulator_z_value*(max_thrust/max_value);
        }
        //std::cout << "altitude_control_value: " << altitude_control_value << std::endl;
    }

    void control_value_regulated(float z_ref, float z_pos)
    {
        //std::cout << "z_mes: " << z_pos << std::endl;
        float z_error = z_ref - z_pos;
        float Kp_altitude = 100;
        float Ki_altitude = 0;
        float Kd_altitude = 80;

        integral += z_error;
        regulator_z_value = Kp_altitude * z_error + Ki_altitude * integral + Kd_altitude * (z_error - prev_z_error);
        //print regulator z value with std::cout
        std::cout << "regulator_z_value: " << regulator_z_value << std::endl;
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

    void localErrorToAngle(float regulated_x_error, float regulated_y_error)
    {
        //float convert_to_control_value = 12.5;
        float max_error = 1000;
        float max_controller_value = 200;
        std::cout << "regulated_x_error: " << regulated_x_error << std::endl;
        std::cout << "regulated_y_error: " << regulated_y_error << std::endl;
        if (regulated_x_error > max_error)
        {
            pitch_controller_value = max_controller_value;
        }
        else if (regulated_x_error < -max_error)
        {
            pitch_controller_value = -max_controller_value;
        }
        else
        {
            pitch_controller_value = regulated_x_error*(max_controller_value/max_error);
        }
        if (regulated_y_error > max_error)
        {
            roll_controller_value = max_controller_value;
        }
        else if (regulated_y_error < -max_error)
        {
            roll_controller_value = -max_controller_value;
        }
        else
        {
            roll_controller_value = regulated_y_error*(max_controller_value/max_error);
        }
        //roll_angle = roll_angle * convert_to_control_value;
        //pitch_angle = pitch_angle * convert_to_control_value;
    }

    void anglePD(float local_error_x, float local_error_y)
    {
        float Kp_pitch = 1;
        float Kd_pitch = 1;
        float Kp_roll = 1;
        float Kd_roll = 1;

        regulated_x_value = Kp_pitch * local_error_x + Kd_pitch * (local_error_x - prev_local_error_x);
        regulated_y_value = Kp_roll * local_error_y + Kd_roll * (local_error_y - prev_local_error_y);

        prev_local_error_x = local_error_x;
        prev_local_error_y = local_error_y;
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

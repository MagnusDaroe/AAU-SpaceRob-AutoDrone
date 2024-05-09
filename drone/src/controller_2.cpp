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

    float altitude_control_value;
    float regulator_z_value;
    float integral;
    float prev_z_error = 0;
    float prev_z_ref = 0;

    float local_error_x;
    float local_error_y;
    float pitch_angle;
    float roll_angle;
    float prev_pitch_angle;
    float prev_roll_angle;

    // XY controller
    // -Ouputs
    float regulator_pitch_value;
    float regulator_roll_value;

    // -Inputs
    float x_error;
    float y_error;
    float prev_x_error;
    float prev_y_error;

    // Z controller
    // -Outputs
    float regulator_altitude_value;

    // -Inputs
    float z_error;
    float prev_z_error;

    // Yaw controller
    // -Outputs
    float regulator_yaw_value;

    // -Inputs
    float yaw_error;

    rclcpp::Subscription<drone::msg::DroneControlData>::SharedPtr Data_subscription_;
    rclcpp::Publisher<drone::msg::DroneCommand>::SharedPtr Control_publisher_;

    //Controller functions
    void DataCallback(const drone::msg::DroneControlData::SharedPtr msg) // skal ændres hvis vi vil køre på vicon data
    {
        // kører xy controller
        float x_ref_signal = ref_signal(msg->time, x_ref, 1);
        float y_ref_signal = ref_signal(msg->time, y_ref, 1);
        globalErrorToLocalError(x_ref_signal, y_ref_signal, msg->vicon_x, msg->vicon_y, msg->camera_yaw);
        XY_controller(local_error_x, local_error_y);

        // Z controller
        float z_ref_signal = ref_signal(msg->time, z_ref, 1);
        Z_controller(z_ref_signal, msg->vicon_z); // note vicon ups

        // Yaw controller
        float yaw_signal = ref_signal(msg->time, yaw_ref, 1);
        yaw_controller(yaw_signal, msg->camera_yaw);



        //RCLCPP_DEBUG(ControllerNode->get_logger(), "Regulator pitch value: %d", regulator_pitch_value);
        //RCLCPP_DEBUG(ControllerNode->get_logger(), "Regulator roll value: %d", regulator_roll_value);
        //RCLCPP_DEBUG(ControllerNode->get_logger(), "Regulator altitude value: %d", regulator_z_value);

        // Publish regulated pitch, roll, and thrust values
        auto control_msg = drone::msg::DroneCommand();
        control_msg.cmd_auto_roll = regulator_roll_value;
        control_msg.cmd_auto_pitch = regulator_pitch_value;
        control_msg.cmd_auto_thrust = regulator_altitude_value;
        control_msg.cmd_auto_yaw = regulator_yaw_value;
        Control_publisher_->publish(control_msg);
    }

    //XY_controller functions    
    void globalErrorToLocalError(float x_ref, float y_ref, float x_global_mes, float y_global_mes, float yaw_mes)
    {
        float x_global_error = x_ref - x_global_mes;
        float y_global_error = y_ref - y_global_mes;
        float roll = 0;
        float pitch = 0;
        float yaw = yaw_mes;

        float inv_R[3][3] = {{cos(pitch) * cos(yaw), cos(pitch) * sin(yaw), -sin(pitch)},
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

    void yaw_controller(float yaw_ref, float yaw_mes)
    {
        float Kp_yaw = 15;
        float saturation_value = 900;

        float yaw_error = yaw_ref - yaw_mes;

        float yaw_value = Kp_yaw * yaw_error;

        regulator_yaw_value = saturation(yaw_value, saturation_value);
    }

    void Z_controller(float z_ref, float z_mes)
    {
        float Kp_altitude = 0.01;
        float Kd_altitude = 10;
        float saturation_value = 900;

        float z_error = z_ref - z_mes;

        float altitude_value = 10*((z_error - prev_z_error)/samples_pr_sec)+0.01*z_error;

        regulator_altitude_value = saturation(altitude_value, saturation_value, 1);

        prev_z_error = z_error;
    }

    void XY_controller(float x_error, float y_error) // float pitch_angle, float roll_angle
    {
        // Define PD controller parameters
        float Kp_pitch = 0.002;
        float Kd_pitch = 0.7;
        float Kp_roll = 0.002;
        float Kd_roll = 0.7;
        float sample_time = 100;

        // Max allowed value (1000 is max max, but we aint chill like that)
        float saturation_value = 900;  

        // Discretized PD controller for x and y
        float pitch_value = (kd_pitch*(x_error-prev_x_error)/sample_time)+x_error*(kp_pitch);
        float roll_value = (kd_yaw*(y_error-prev_y_error)/sample_time)+y_error*(kp_roll);

        regulator_pitch_value = saturation(pitch_value, saturation_value);
        regulator_roll_value = saturation(roll_value, saturation_value);

        prev_x_error = x_error;
        prev_y_error = y_error;
    } 

    float saturation(float value, float max_value, int only_positive = 0)
    {
        // Saturate value to + or - max_value
        if (value > max_value)       // Saturate to max value
        {
            value = max_value;
        }
        else if (value < -max_value) // Saturate to minimum
        {
            if only_positive == 0
                value = -max_value;
            else
                value = 1; // Smallest allowed value when only saturating to positive numbers

        }
        else                         // Do nothing
        {
            ;
        }
        return value;
    }
    
    float ref_signal(float t, float ref, float delay)
    {
        float signal = ref*(1-exp(-t/delay));
        return signal;
    }

};


int main(int argc, char *argv[])
{
    //RCLCPP_DEBUG(ControllerNode->get_logger(), "My log message is: Hi");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


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
    float regulator_pitch_value;
    float regulator_roll_value;
    float altitude_control_value;

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
    void DataCallback(const drone::msg::DroneControlData::SharedPtr msg)
    {
        ramped_reference(ControllerNode::z_ref);
        control_value_regulated(ControllerNode::z_ref_ramped, msg->vicon_z);  //Skal have z_ref og z_pos
        z_error_to_controller_value(ControllerNode::regulator_z_value);
        globalErrorToLocalError(x_ref, y_ref, msg->vicon_x, msg->vicon_y, msg->camera_yaw);
        localErrorToAngle(local_error_x, local_error_y);
        
        XY_controller(local_error_x, local_error_y);
        //RCLCPP_DEBUG(ControllerNode->get_logger(), "Regulator pitch value: %d", regulator_pitch_value);
        //RCLCPP_DEBUG(ControllerNode->get_logger(), "Regulator roll value: %d", regulator_roll_value);
        //RCLCPP_DEBUG(ControllerNode->get_logger(), "Regulator altitude value: %d", regulator_z_value);

        // Publish regulated pitch, roll, and thrust values
        auto control_msg = drone::msg::DroneCommand();
        control_msg.cmd_auto_roll = regulator_roll_value;
        control_msg.cmd_auto_pitch = regulator_pitch_value;
        control_msg.cmd_auto_thrust = altitude_control_value;
        Control_publisher_->publish(control_msg);
    }
    void ramped_reference(float z_ref)
    {
        float samples_pr_sec = 100;
        float z_ref_ramped = (z_ref*samples_pr_sec+prev_z_ref)/(1+samples_pr_sec);
        prev_z_ref = z_ref_ramped;
    }

    void z_error_to_controller_value(float regulator_z_value)
    {
        int max_value = 800;

        if (regulator_z_value > max_value)
        {
            altitude_control_value = max_value;
        }
        else if (regulator_z_value < -max_value)
        {
            altitude_control_value = -max_value;
        }
        else
        {
            altitude_control_value = regulator_z_value;
        }
    }

    void control_value_regulated(float z_ref_ramped, float z_pos)
    {
        float Kp_altitude = 0.01;
        float Kd_altitude = 10;
        float z_error = z_ref_ramped - z_pos;

        regulator_z_value = 10*((z_error - prev_z_error)/samples_pr_sec)+0.01*z_error;
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

    void yaw_controller(float yaw_error)
    {
        float Kp_yaw = 15;
        float saturation_value = 900;

        float yaw_value = Kp_yaw * yaw_error;

        regulator_yaw_value = saturation(yaw_value, saturation_value);
    }

    void Z_controller(float z_error)
    {
        float Kp_altitude = 0.01;
        float Kd_altitude = 10;
        float saturation_value = 900;

        float altitude_value = 10*((z_error - prev_z_error)/samples_pr_sec)+0.01*z_error;

        regulator_altitude_value = saturation(altitude_value, saturation_value);

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

    float saturation(float value, float max_value)
    {
        // Saturate value to + or - max_value
        if (value > max_value)       // Saturate to max value
        {
            value = max_value;
        }
        else if (value < -max_value) // Saturate to minimum
        {
            value = -max_value;
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


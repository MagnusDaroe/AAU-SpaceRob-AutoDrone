#include <chrono>
#include <memory>
#include <math.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "drone/msg/vicon_data.hpp"
#include "drone/msg/drone_command.hpp"

using namespace std::chrono_literals;

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode() : Node("controller_node")
    {
        // Subscribe to altitude reference and measurement topics
        Data_subscription_ = this->create_subscription<drone::msg::ViconData>("/ViconData", 10, std::bind(&ControllerNode::DataCallback, this, std::placeholders::_1));

        // Publish regulated altitude control value
        Control_publisher_ = this->create_publisher<drone::msg::DroneCommand>("regulated_altitude_control", 10);
    }

private:
    // Variables to hold the output error after converting from global to local coordinates
    float local_error_x;
    float local_error_y;

    // XY controller function variables
    // Outputs
    float regulator_pitch_value;
    float regulator_roll_value;

    // Variables to store previous error
    float prev_x_error = 0;
    float prev_y_error = 0;

    // Z controller
    // Output
    float regulator_altitude_value;

    // Previous error value
    float prev_z_error = 0;

    // Yaw controller
    // Outputs
    float regulator_yaw_value;

    // Variable to check if waypoint is reached and new data is requested
    bool data_request = true;


    // Defining waypoints
    const static int array_size = 2;            // size of array

    float x_ref_list[array_size] = {1200, 500};
    float y_ref_list[array_size] = {1000, -700};
    float z_ref_list[array_size] = {500, 1000}; 
    float yaw_ref_list[array_size] = {0, 90};

    int array_counter = 0;                     // counter for array

    // Variables that hold individual reference points while running the controller
    float x_ref;
    float y_ref;
    float z_ref;
    float yaw_ref;

    // Variables to hold previous filter value
    float prev_filter_val = 0;   // DER SKAL SQ NOK OPRETES EN FOR HVER SLAGS MÅLT VÆRDI SÅ DER IKKE GÅR GED I DEN

    // Sample time of all controllers
    float sample_time = 0.01;

    // Defines time variables
    std::chrono::system_clock::time_point time_start;
    std::chrono::system_clock::time_point time_stop;

    // Subscribers and publishers
    rclcpp::Subscription<drone::msg::ViconData>::SharedPtr Data_subscription_;
    rclcpp::Publisher<drone::msg::DroneCommand>::SharedPtr Control_publisher_;

    //Controller functions
    void DataCallback(const drone::msg::ViconData::SharedPtr msg) // skal ændres hvis vi vil køre på kamera data data
    { 
        // Check if data is requested. Reset data and timer if so
        if (data_request == true)
        {
            // Next waypoint is saved in seperate variable
            x_ref = x_ref_list[array_counter];
            y_ref = y_ref_list[array_counter];
            z_ref = z_ref_list[array_counter]; 
            yaw_ref = yaw_ref_list[array_counter];

            time_start = std::chrono::system_clock::now(); // Timer is reset
            data_request = false;                          // Reset data request

            // Only increment if there are more waypoints
            if(array_counter < array_size-1){
                array_counter++;
            }
        }
        // Check time now and calculate time duration
        time_stop = std::chrono::system_clock::now();
        auto time_duration = std::chrono::duration_cast<std::chrono::milliseconds>(time_stop - time_start).count();
        std::cout << "Time duration: " << time_duration << std::endl;

        // XY controller
        // Generates a reference signal according to exponential function
        // Signal function requires current time, final reference signal, and time constant
        float x_ref_signal = ref_signal(time_duration/1000, x_ref, 2); // time duration is converted to seconds
        float y_ref_signal = ref_signal(time_duration/1000, y_ref, 2); // time duration is converted to seconds

        // // Denoise the recieved position variables
        // float denoised_vicon_x = low_pass(msg->vicon_x, 10);
        // float denoised_vicon_y = low_pass(msg->vicon_y, 10);
        // float denoised_vicon_z = low_pass(msg->vicon_z, 10);
        // float denoised_vicon_yaw = low_pass(msg->vicon_yaw, 10);

        // recieves the reference signal and the current position and calculates the local error
        globalErrorToLocalError(x_ref_signal, y_ref_signal, msg->vicon_x, msg->vicon_y, msg->vicon_yaw);
        // recieves the local error and calculates controller values for roll and pitch
        XY_controller(local_error_x, local_error_y);

        // Z controller
        // Generates reference signal
        float z_ref_signal = ref_signal(time_duration/1000, z_ref, 1); // time duration is converted to seconds
        // Generates controller value for altitude
        Z_controller(z_ref_signal, msg->vicon_z); // Note: skal ændres hvis kamera skal bruges

        // Yaw controller
        // Generates reference signal
        float yaw_signal = ref_signal(time_duration/1000, yaw_ref, 1); // time duration is converted to seconds
        // Generates controller value for yaw
        yaw_controller(yaw_signal, msg->vicon_yaw);

        // Shitty way to calculate the distance to the point but square roots and shit aint worth it
        float total_error = abs((msg->vicon_x + msg->vicon_y + msg->vicon_z) - (x_ref + y_ref + z_ref));

        // Check if error is under threshold to request new data
        if (total_error < 0){   // SKAL SÆTTES TIL AFSTAND LIMIT FØR SKIDTET VIRKER
            data_request = true;    // Reset data request if close to waypoint
        }
        else{
            data_request = false;   // Still false if not close
        }

        // Publish regulated pitch, roll, thrust, and yaw values
        auto control_msg = drone::msg::DroneCommand();
        control_msg.cmd_auto_roll = regulator_roll_value;
        control_msg.cmd_auto_pitch = regulator_pitch_value;
        control_msg.cmd_auto_thrust = regulator_altitude_value;
        control_msg.cmd_auto_yaw = regulator_yaw_value;
        control_msg.identifier = 1;
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
    // P controller for yaw position
    {
        float Kp_yaw = 15;              // Proportional gain
        float saturation_value = 900;   // Max and min value allowed to be sent to the drone

        float yaw_error = yaw_ref - yaw_mes;  // Error between reference and measurement

        float yaw_value = Kp_yaw * yaw_error; // P regulaed value

        regulator_yaw_value = saturation(yaw_value, saturation_value);  // Saturate value
    }

    void Z_controller(float z_ref, float z_mes)
    // PD controller for altitude
    {
        float Kp_altitude = 0.6;       // Proportional gain
        float Kd_altitude = 0.5;         // Derivative gain
        float saturation_value = 200;   // Max and min value allowed to be sent to the drone
        float hover_value = 495;        // controller value for hovering (found by m*g/thrust to newton relation)

        float z_error = z_ref - z_mes;  // Error between reference and measurement

        float altitude_value = Kd_altitude*((z_error - prev_z_error)/sample_time)+Kp_altitude*z_error; // PD regulated value

        regulator_altitude_value = saturation(altitude_value, saturation_value)+hover_value; // Saturate value and add hover value

        prev_z_error = z_error; // Update previous error
    }

    void XY_controller(float local_x_error, float local_y_error)
    // PD controller for x and y
    {
        // PD controller gains
        float Kp_pitch = 0.002; 
        float Kd_pitch = 0.7;
        float Kp_roll = 0.002;
        float Kd_roll = 0.7;

        // Max allowed value (1000 is max max, but we aint chill like that)
        float saturation_value = 900;  

        // Discretized PD controller for x and y
        float pitch_value = (Kd_pitch*(local_x_error-prev_x_error)/sample_time)+local_x_error*(Kp_pitch);
        float roll_value = (Kd_roll*(local_y_error-prev_y_error)/sample_time)+local_y_error*(Kp_roll);

        // Saturate values
        regulator_pitch_value = saturation(pitch_value, saturation_value);
        regulator_roll_value = saturation(roll_value, saturation_value);

        // Update previous errors
        prev_x_error = local_x_error;
        prev_y_error = local_y_error;
    } 

    float saturation(float value, float max_value, int only_positive = 0)
    // Saturate value to + or - max_value
    {
        if (value > max_value)       // Saturate to max value
        {
            value = max_value;
        }
        else if (value < -max_value) // Saturate to minimum
        {
            if (only_positive == 0){ // Check if only positive values are allowed
                value = -max_value;
            }
            else{
                value = 1;           // Smallest allowed value when only saturating to positive numbers
            }
        }
        else                         // Do nothing
        {
            ;
        }
        return value;
    }
    
    float ref_signal(float t, float ref, float delay)
    // Generates a reference signal according to exponential function
    {
        float signal = ref*(1-exp(-t/delay));
        return signal;
    }

    float ref_ramp(float t, float ref, float slope, float offset = 0)
    // Generates a reference signal according to ramp function
    {
        float signal = slope*t+offset;
        return saturation(signal, ref);
    }
    float low_pass(float signal, float filter_val)
    // Low pass filter for noise on the measured position signal
    {
        // Discretized low pass filter
        float lowpass_signal = (signal*filter_val*sample_time+prev_filter_val)/(1+filter_val*sample_time);
        // Update previous filter value
        prev_filter_val = lowpass_signal;

        return lowpass_signal;
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


#include <chrono>
#include <time.h>
#include <memory>
#include <math.h>
#include <iostream>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "drone/msg/vicon_data.hpp"
#include "drone/msg/drone_control_data.hpp"
#include "drone/msg/drone_command.hpp"

using namespace std::chrono_literals;

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode() : Node("controller_node")
    {
        // Subscribe to altitude reference and measurement topics
        Data_subscription_ = this->create_subscription<drone::msg::DroneControlData>("/DroneControlData", 10, std::bind(&ControllerNode::DataCallback, this, std::placeholders::_1)); //KAMERA

        // Publish regulated altitude control value
        Control_publisher_ = this->create_publisher<drone::msg::DroneCommand>("/cmd_fc", 10);
        

        // Start the control loop in a separate thread
        control_loop_thread_ = std::thread(&ControllerNode::ControlLoop, this);
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


    //Defining waypoints
    //*test1 waypoints
    //const static int array_size = 5;            // size of array
    //float x_ref_list[array_size] = {-500, -450, -290, -1500, -1500};
    //float y_ref_list[array_size] = {0, -600, 250, 420, 420};
    //float z_ref_list[array_size] = {500, 700, 500, 600, 180}; 
    //float yaw_ref_list[array_size] = {M_PI, M_PI, M_PI, M_PI, M_PI}; //Ref is in radians

    // //*task test, waypoints
    const static int array_size = 4;            // size of array
    float x_ref_list[array_size] = {-500, -1500, -1500, -1500};
    float y_ref_list[array_size] = {0, 420, 420, 420};
    float z_ref_list[array_size] = {500, 500, 0, 500};
    float yaw_ref_list[array_size] = {M_PI, M_PI, M_PI, M_PI}; //Ref is in radians


    int array_counter = 0;                     // counter for array

    // Variables that hold individual reference points while running the controller
    float x_ref;
    float y_ref;
    float z_ref;
    float yaw_ref;
    float x_ref_old = 0;
    float y_ref_old = 0;
    float z_ref_old = 0;
    float total_error;
    float z_ref_signal;

    //Variables that hold current position
    float current_x;
    float current_y;
    float current_z;
    float current_yaw;

    //Publish varible
    int cmd_auto_land = 0;

    // Variables to hold previous filter value
    float prev_filter_val = 0;   // DER SKAL SQ NOK OPRETES EN FOR HVER SLAGS MÅLT VÆRDI SÅ DER IKKE GÅR GED I DEN

    // Sample time of all controllers
    float sample_time = 0.01;

    // New msg variable
    bool new_msg = false;

    // Defines time variables
    std::chrono::system_clock::time_point disarm_start;
    std::chrono::system_clock::time_point disarm_stop;
    std::chrono::system_clock::time_point time_start;
    std::chrono::system_clock::time_point time_stop;
    std::chrono::system_clock::time_point timestamp;
    std::chrono::system_clock::duration time_since_epoch;
    int ghetto_ur = 0; 
    // Subscribers and publishers
    rclcpp::Subscription<drone::msg::DroneControlData>::SharedPtr Data_subscription_; // KAMERA
    rclcpp::Publisher<drone::msg::DroneCommand>::SharedPtr Control_publisher_;

    //Controller functions
    void DataCallback(const drone::msg::DroneControlData::SharedPtr msg) // skal ændres hvis vi vil køre på kamera data data
    { 
        current_x = msg->camera_x; //KAMERA
        current_y = msg->camera_y;
        current_z = msg->camera_z;
        current_yaw = msg->camera_yaw;
        new_msg = true;
    }

    void ControlLoop(){
        while(rclcpp::ok()){
            if(cmd_auto_land == 1){
                disarm_stop = std::chrono::system_clock::now();
                auto disarm_duration = std::chrono::duration_cast<std::chrono::milliseconds>(disarm_stop - disarm_start).count();
                std::cout<< "Auto land: 1"<< std::endl;
                if(disarm_duration > 10000){ //Waits for 10 seconds before sending the arm command
                    cmd_auto_land = 0;
                    ghetto_ur = 0;
                }
            }
            else if(new_msg == true){
                new_msg = false;
                // Check if data is requested. Reset data and timer if so
                if (data_request == true){
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
                std::cout<< "array_counter: " << array_counter-1 << std::endl;
                // Check time now and calculate time duration
                time_stop = std::chrono::system_clock::now();
                auto time_duration = std::chrono::duration_cast<std::chrono::milliseconds>(time_stop - time_start).count();
                // std::cout << "Time duration: " << time_duration << std::endl;

                // XY controller
                // Generates a reference signal according to exponential function
                // Signal function requires current time, final reference signal, and time constant
                float x_ref_signal = ref_signal(time_duration/1000, x_ref, x_ref_old, 1); // time duration is converted to seconds
                float y_ref_signal = ref_signal(time_duration/1000, y_ref, y_ref_old, 1); // time duration is converted to seconds

                // // Denoise the recieved position variables
                // float denoised_vicon_x = low_pass(msg->vicon_x, 10);
                // float denoised_vicon_y = low_pass(msg->vicon_y, 10);
                // float denoised_vicon_z = low_pass(msg->vicon_z, 10);
                // float denoised_vicon_yaw = low_pass(msg->vicon_yaw, 10);

                // recieves the reference signal and the current position and calculates the local error
                globalErrorToLocalError(x_ref_signal, y_ref_signal, current_x, current_y, current_yaw);
                // recieves the local error and calculates controller values for roll and pitch
                XY_controller(local_error_x, local_error_y);
                

                // Z controller
                // Generates reference signal
                if (z_ref != 0){
                    z_ref_signal = ref_signal(time_duration/1000, z_ref, z_ref_old, 2); // time duration is converted to seconds
                    total_error = abs((current_x + current_y + current_z/3) - (x_ref + y_ref + z_ref/3));
                }
                else{
                    z_ref_signal = ref_signal(time_duration/1000, 100, z_ref_old, 4); //100 because it is 3 cm below floor height
                    total_error = abs((current_z) - (130));
                }
                // Generates controller value for altitude
                Z_controller(z_ref_signal, current_z);
                // Yaw controller
                // Generates reference signal
                float yaw_signal = ref_signal(time_duration/1000, yaw_ref, 0, 1); // time duration is converted to seconds
                // Generates controller value for yaw 
                yaw_controller(yaw_signal, current_yaw);

                std::cout<<"total_error: "<< total_error << std::endl;
                std::cout<<"z_ref: "<< z_ref << std::endl;
                
                if (z_ref == 0 && total_error < 40){
                    ghetto_ur++;
                    std::cout<< "Ghetto ur"<< std::endl;
                    if (ghetto_ur > 200){
                        cmd_auto_land = 1; //Meaning it sends a request to disarm
                        data_request = true;
                    }
                }
                // Check if error is under threshold to request new data
                else if (total_error < 80 && z_ref != 0){   // SKAL SÆTTES TIL AFSTAND LIMIT FØR SKIDTET VIRKER //Ændre til 50
                    ghetto_ur++;
                    if (ghetto_ur > 200){
                        data_request = true;    // Reset data request if close to waypoint
                        ghetto_ur = 0;
                        x_ref_old = x_ref;
                        y_ref_old = y_ref;
                        z_ref_old = z_ref;
                    }
                }
                else{
                    data_request = false;   // Still false if not close
                }

                timestamp = std::chrono::system_clock::now();
                time_since_epoch = timestamp.time_since_epoch();
                double time_since_epoch_double = time_since_epoch.count()*pow(10, -9);
                //std::cout << "timestamp: "<< time_since_epoch_double << std::endl;

                auto control_msg = drone::msg::DroneCommand();
                // Publish regulated pitch, roll, thrust, and yaw values
                control_msg.cmd_auto_roll = static_cast<int>(-regulator_roll_value); //(minus)Because of Henriks ligninger //Kamera
                control_msg.cmd_auto_pitch = static_cast<int>(regulator_pitch_value);
                control_msg.cmd_auto_thrust = static_cast<int>(regulator_altitude_value);
                control_msg.cmd_auto_yaw = static_cast<int>(-regulator_yaw_value);  //Minus because fc coordinates system is downwards maybe              
                control_msg.identifier = 1;
                control_msg.timestamp = time_since_epoch_double;
                control_msg.cmd_auto_disarm = cmd_auto_land;
                if(z_ref == 0){
                    std::cout << "total_error: " << total_error << std::endl;
                }
                if(cmd_auto_land == 1){
                    disarm_start = std::chrono::system_clock::now();
                }
                Control_publisher_->publish(control_msg);
            }
        }
    }


    //XY_controller functions    
    void globalErrorToLocalError(float x_ref, float y_ref, float x_global_mes, float y_global_mes, float yaw_mes)
    {
        float x_global_error = x_ref - x_global_mes; 
        float y_global_error = y_ref - y_global_mes; 
        // x_global_error = -x_global_error; //Kamera
        // y_global_error = -y_global_error; //Kamera
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
        float Kp_yaw = 150;              // Proportional gain
        float saturation_value = 300;   // Max and min value allowed to be sent to the drone

        float yaw_error = yaw_ref - yaw_mes;  // Error between reference and measurement
        std::cout<<"yaw_error: "<< yaw_error << std::endl;
        float yaw_value = Kp_yaw * yaw_error; // P regulaed value

        regulator_yaw_value = saturation(yaw_value, saturation_value);  // Saturate value
    }

    void Z_controller(float z_ref, float z_mes)
    // PD controller for altitude
    {
        float Kp_altitude = 0.6;       // Proportional gain
        float Kd_altitude = 0.5;         // Derivative gain
        float saturation_value = 100;   // Max and min value allowed to be sent to the drone
        float hover_value = 560;        // controller value for hovering (found by m*g/thrust to newton relation)

        float z_error = z_ref - z_mes;  // Error between reference and measurement
        std::cout<<"z_error: "<< z_error << std::endl;

        float altitude_value = Kd_altitude*((z_error - prev_z_error)/sample_time)+Kp_altitude*z_error; // PD regulated value

        regulator_altitude_value = saturation(altitude_value, saturation_value)+hover_value; // Saturate value and add hover value

        prev_z_error = z_error; // Update previous error
    }

    void XY_controller(float local_x_error, float local_y_error)
    // PD controller for x and y
    {
        std::cout<<"local_x_error: "<< local_x_error << std::endl;
        std::cout<<"local_y_error: "<< local_y_error << std::endl;
        
        // PD controller gains
        float Kp_pitch = 0.5; 
        float Kd_pitch = 1;
        float Kp_roll = 0.5;
        float Kd_roll = 1;

        // Max allowed value (1000 is max max, but we aint chill like that)
        float saturation_value = 500;  //CHANGE

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
    
    float ref_signal(float t, float ref, float old_ref, float delay)
    // Generates a reference signal according to exponential function
    {
        float dif = ref - old_ref;
        float signal = dif*(1-exp(-t/delay))+old_ref;
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

    //float landing_signal()

    std::thread control_loop_thread_;

};


int main(int argc, char *argv[])
{
    std::cout << "Starting controller node" << std::endl;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


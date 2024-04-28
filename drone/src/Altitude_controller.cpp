// #include <iostream>     //Library for input and output
// #include <math.h>       //Library for mathematical functions
// #include <chrono>
// #include "rclcpp/logging.hpp"

// using namespace std::chrono_literals;

// class Altitude_controller
// {
//     private:
        
//     public:
//         //Public variables, which can be accessed from the object
//         float altitude_control_value;
//         float regulator_z_value;
//         float integral;
//         float prev_z_error;
//         //Constructor, which takes the z error and calculates the controller value
//         int z_error_to_controller_value(float z_ref, float z_mes);
//         //Constructor, which takes the controller value and regulates it with a PID controller
//         int control_value_regulated(float altitude_control_value);

// };

// int Altitude_controller::z_error_to_controller_value(float z_ref, float z_mes)
// {
//     int max_value = 200;                  //Maximum value of the altitude control value
//     float z_error = z_ref - z_mes;        //Finds the global error in z
//     if (z_error > max_value)                    //If the error is greater than 200, the altitude control value is set to 200
//     {
//         altitude_control_value = max_value;
//     }
//     else if (z_error < -max_value)              //If the error is less than -200, the altitude control value is set to -200
//     {
//         altitude_control_value = -max_value;
//     }
//     else
//     {
//         altitude_control_value = z_error;       //Otherwise, the altitude control value is set to the error
//     }
//     return 0;
// }

// int Altitude_controller::control_value_regulated(float altitude_control_value)
// {
//     float Kp_altitude = 5;                              //Proportional gain 
//     float Ki_altitude = 1;                              //Integral gain
//     float Kd_altitude = 10;                             //Derivative gain
                                  
//     integral = integral+altitude_control_value;         //Integral term

//     regulator_z_value = Kp_altitude*altitude_control_value + Ki_altitude*altitude_control_value + Kd_altitude*(altitude_control_value-prev_z_error); //PID controller
//     prev_z_error = altitude_control_value;     //Assigns the current altitude control value to the previous altitude control value
    
//     return 0;
// }

// int main()
// {
//     Altitude_controller measurements;                            //Creates an object of the class
//     measurements.z_error_to_controller_value(300, 200);             //Calls the function with z_ref and z_mes, which need to be specified
//     std::cout << "The altitude control value is: \n" << measurements.altitude_control_value << std::endl; //Prints the altitude control value

//     measurements.control_value_regulated(measurements.altitude_control_value); //Calls the function with the altitude control value
//     std::cout << "The regulated altitude control value is: \n" << measurements.regulator_z_value << std::endl; //Prints the regulated altitude control value
// }



#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "msg/DroneControlData.hpp"


using namespace std::chrono_literals;

class AltitudeControllerNode : public rclcpp::Node
{
public:
    AltitudeControllerNode() : Node("altitude_controller_node")
    {
        // Subscribe to altitude reference and measurement topics
        Data_subscription_ = this->create_subscription<std_msgs::msg::Float32>("altitude_reference", 10, std::bind(&AltitudeControllerNode::zRefCallback, this, std::placeholders::_1));

        // Publish regulated altitude control value
        regulator_z_value_publisher_ = this->create_publisher<std_msgs::msg::Float32>("regulated_altitude_control", 10);
    }

private:
    float altitude_control_value;
    float regulator_z_value;
    float integral;
    float prev_z_error;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr z_ref_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr z_mes_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr regulator_z_value_publisher_;

    void zRefCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        z_error_to_controller_value(msg->data);
        control_value_regulated();
    }
    
    void zMesCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        // Assuming we update z_mes here if needed
    }

    void z_error_to_controller_value(float z_ref)
    {
        int max_value = 200;
        float z_error = z_ref - altitude_control_value;

        if (z_error > max_value)
        {
            altitude_control_value = max_value;
        }
        else if (z_error < -max_value)
        {
            altitude_control_value = -max_value;
        }
        else
        {
            altitude_control_value = z_error;
        }
    }

    void control_value_regulated(float altitude_control_value)
    {
        float Kp_altitude = 5;
        float Ki_altitude = 1;
        float Kd_altitude = 10;

        integral += altitude_control_value;
        regulator_z_value = Kp_altitude * altitude_control_value + Ki_altitude * integral + Kd_altitude * (altitude_control_value - prev_z_error);
        prev_z_error = altitude_control_value;

        // Publish regulated altitude control value
        auto msg = std_msgs::msg::Float32();
        msg.data = regulator_z_value;
        regulator_z_value_publisher_->publish(msg);
    }
};

int main(int argc, char *argv[])
{
    //RCLCPP_DEBUG(AltitudeControllerNode->get_logger(), "My log message is: Hi");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AltitudeControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

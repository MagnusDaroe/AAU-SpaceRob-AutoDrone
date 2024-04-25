#include <iostream>     //Library for input and output
#include <math.h>       //Library for mathematical functions
#include <chrono>

using namespace std;

class Altitude_controller
{
    private:
        
    public:
        //Public variables, which can be accessed from the object
        float altitude_control_value;
        float regulator_z_value;
        //Constructor, which takes the z error and calculates the controller value
        int z_error_to_controller_value(float z_ref, float z_mes);
        //Constructor, which takes the controller value and regulates it with a PID controller
        int control_value_regulated(float altitude_control_value);

};

int Altitude_controller::z_error_to_controller_value(float z_ref, float z_mes)
{
    int max_value = 200;                  //Maximum value of the altitude control value
    float z_error = z_ref - z_mes;        //Finds the global error in z
    if (z_error > max_value)                    //If the error is greater than 200, the altitude control value is set to 200
    {
        altitude_control_value = max_value;
    }
    else if (z_error < -max_value)              //If the error is less than -200, the altitude control value is set to -200
    {
        altitude_control_value = -max_value;
    }
    else
    {
        altitude_control_value = z_error;       //Otherwise, the altitude control value is set to the error
    }
    return 0;
}

int Altitude_controller::control_value_regulated(float altitude_control_value)
{
    float Kp_altitude = 5;                              //Proportional gain 
    float Ki_altitude = 1;                              //Integral gain
    float Kd_altitude = 10;                             //Derivative gain
                                  
    float prev_z_error;                                 //Previous error
    float integral = integral+altitude_control_value;   //Integral term

    regulator_z_value = Kp_altitude*altitude_control_value + Ki_altitude*altitude_control_value + Kd_altitude*(altitude_control_value-prev_z_error); //PID controller
    prev_z_error = altitude_control_value;     //Assigns the current altitude control value to the previous altitude control value
    
    return 0;
}

int main()
{
    Altitude_controller measurements;                               //Creates an object of the class
    measurements.z_error_to_controller_value(300, 200);             //Calls the function with z_ref and z_mes, which need to be specified
    cout << "The altitude control value is: \n" << measurements.altitude_control_value << endl; //Prints the altitude control value

    measurements.control_value_regulated(measurements.altitude_control_value); //Calls the function with the altitude control value
    cout << "The regulated altitude control value is: \n" << measurements.regulator_z_value << endl; //Prints the regulated altitude control value
}
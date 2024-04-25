#include <iostream>     //Library for input and output
#include <math.h>       //Library for mathematical functions

using namespace std;

class XY_controller
{
    private:
        
    public:
        //Public variables, which can be accessed from the object
        float local_error_x;
        float local_error_y;
        float pitch_angle;
        float roll_angle;
        float prev_pitch_angle = 0;
        float prev_roll_angle = 0;
        float regulator_pitch_value;
        float regulator_roll_value;

        //Constructor, which takes the global errors and calculates the local errors
        int global_error_to_local_error(float x_ref, float y_ref, float x_global_mes, float y_global_mes, float yaw_mes);
        //Constructor, which takes the local errors and calculates the controller angles
        int local_error_to_angle(float local_error_x, float local_error_y);
        //Constructor, which takes the angles and calculates the PD regulated values
        int angle_PD(float pitch_angle, float roll_angle);
};

int XY_controller::global_error_to_local_error(float x_ref, float y_ref, float x_global_mes, float y_global_mes, float yaw_mes)
{  
    float x_global_error = x_ref - x_global_mes;        //Finds the global error in x  
    float y_global_error = y_ref - y_global_mes;        //Finds the global error in y
    float roll = 0;                                     //Roll is assumed to be 0
    float pitch = 0;                                    //Pitch is assumed to be 0
    float yaw = yaw_mes;                                //Yaw is set to a constant value             

    
    /*float R[3][3] = {{cos(pitch)*cos(yaw), cos(yaw)*sin(pitch)*sin(roll) - cos(roll)*sin(yaw), sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)},       //Rotation matrix, which takes radians
                     {cos(pitch)*sin(yaw), cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw), cos(roll)*sin(pitch)*sin(yaw) - cos(yaw)*sin(roll)},
                     {-sin(pitch),                               cos(pitch)*sin(roll),                               cos(pitch)*cos(roll)}};*/

    
    float inv_R[3][3] = {{                               cos(pitch)*cos(yaw),                                cos(pitch)*sin(yaw),          -sin(pitch)},    //Inverse rotation matrix, which takes radians    
                         {cos(yaw)*sin(pitch)*sin(roll) - cos(roll)*sin(yaw), cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw), cos(pitch)*sin(roll)},
                         {sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch), cos(roll)*sin(pitch)*sin(yaw) - cos(yaw)*sin(roll), cos(pitch)*cos(roll)}};

    
    
    /*cout << "Inverse of the given matrix is:\n";          //Printing the inverse of the matrix
    for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			cout<<"\t"<<inv_R[i][j];
		}
		cout<<endl;
	}*/

    float global_error_vec[3][1] = {{x_global_error},       //Global error vector
                                    {y_global_error}, 
                                    {0}};

    
    //cout << "Local error vector is:\n";
    float result[3][1];                        //Local error vector temperaty vector
    for (int i = 0; i < 3; i++) {              //Calculating the local error vector
        for (int j = 0; j < 1; j++) {
            result[i][j] = 0;

            for (int k = 0; k < 3; k++) {
                result[i][j] += inv_R[i][k] * global_error_vec[k][j];

            }
            //cout<<"\t"<<result[i][j];
        }
        //cout << endl;
    }

    local_error_x = result[0][0];                //Assigns local_error_x value to objects public variable
    local_error_y = result[1][0];                //Assigns local_error_y value to objects public variable
    return 0;
}

int XY_controller::local_error_to_angle(float local_error_x, float local_error_y)
{  
    //Ensures the angles are no more than +-20 degrees
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
        pitch_angle = local_error_x/10;
        roll_angle = local_error_y/10;
    }
    return 0;
}

int XY_controller::angle_PD(float pitch_angle, float roll_angle)
{  
    float Kp_pitch = 0;      //Proportional gain pitch
    float Kd_pitch = 1;      //Derivative gain pitch
    float Kp_roll = 0;       //Proportional gain roll
    float Kd_roll = 1;       //Derivative gain roll

    regulator_pitch_value = Kp_pitch*pitch_angle + Kd_pitch*(pitch_angle-prev_pitch_angle);  //Calculates the regulator value for pitch
    regulator_roll_value = Kp_roll*roll_angle + Kd_roll*(roll_angle-prev_roll_angle);         //Calculates the regulator value for roll

    prev_pitch_angle = pitch_angle;     //Assigns the current pitch angle to the previous pitch angle
    prev_roll_angle = roll_angle;       //Assigns the current roll angle to the previous roll angle
    return 0;
}

int main()
{
    XY_controller measurements;   //Creates an object of the XY_controller class
    measurements.global_error_to_local_error(12, 10, 3, 2, -0.6571164634);   //Calls the global_error_to_local_error method of the XY_controller object with the given values
    cout << "Local error in x and y is: \n";        //Prints "Local error in x and y is:
    cout << measurements.local_error_x << endl;      //Prints the local_error_x
    cout << measurements.local_error_y << endl;      //Prints the local_error_y

    measurements.local_error_to_angle(measurements.local_error_x, measurements.local_error_y);   //Calls the local_error_to_angle method of the XY_controller object with the given valuesreturn 0;
    cout << "Pitch and roll angle is: \n";        //Prints "Pitch and roll angle is:
    cout << measurements.pitch_angle << endl;      //Prints the pitch_angle
    cout << measurements.roll_angle << endl;      //Prints the roll_angle

    measurements.angle_PD(measurements.pitch_angle, measurements.roll_angle);   //Calls the angle_PD method of the XY_controller object with the given values
    cout << "Regulator pitch and roll value is: \n";        //Prints "Regulator pitch and roll value is:
    cout << measurements.regulator_pitch_value << endl;      //Prints the regulator_pitch_value
    cout << measurements.regulator_roll_value << endl;      //Prints the regulator_roll_value
    return 0;
}
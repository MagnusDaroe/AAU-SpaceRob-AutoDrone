#include <iostream>
#include <string>
#include <fstream>

using namespace std;

int main() {
    char csv_file_path[] = "C:\\Users\\simon\\Documents\\GitHub\\Programmer\\P4_DroneProject\\Control\\waypoints.csv";
    class wayPoints {
        public:
            int x_;
            int y_;
            int z_;

        
        int read(char* csv_file_path) {                            // Function that reads the csv file
            ifstream allWaypoints;                                 // Creates a ifstream object

            allWaypoints.open(csv_file_path);                           //opens the file

            if (allWaypoints.fail()) {                                  //checks if the file is opened correctly  
                cerr << "Unable to open file" << csv_file_path << endl;
                return 1;
            }

            int amountOfWaypoints = 1;
            for(int i = 0; i < amountOfWaypoints; i++) {                //reads specified amount of waypoints
                string x;
                string y;
                string z;
                getline(allWaypoints, x, ',');                          // reads the first int of the file
                getline(allWaypoints, y, ',');                          // reads the second int of the file
                getline(allWaypoints, z, ';');                          // reads the third int of the file        
                x_ = stoi(x), y_ = stoi(y), z_ = stoi(z);               // converts the strings to integers
                cout << x_ << " " << y_ << " " << z_ << endl;           // prints the first waypoint in the file
            }

            allWaypoints.close();
            return 0;
        };


    };
    wayPoints wp;
    wp.read(csv_file_path);                                         // Calls the read function, which reads the csv file
    cout << wp.x_ << endl;                                          // Allows us to acces the x value of the waypoint
};
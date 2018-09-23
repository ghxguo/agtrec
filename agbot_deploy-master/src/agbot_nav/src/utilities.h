#ifndef _UTILITIES_H_
#define _UTILITIES_H_

#include <cmath>
#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <Eigen/Dense>
#define _USE_MATH_DEFINES

using std::cout;
using std::vector;
using std::ifstream;
using std::string;
using std::stod;
using Eigen::MatrixXd;
using Eigen::VectorXd;


//struct to define vehicle position on a coordinate system at a certain heading
struct Point
{
    int x;
    int y;
    int inputHeading;
    Point() : x(0), y(0), inputHeading(0) {}
    Point(int xIn, int yIn, int headingIn): x(xIn), y(yIn), inputHeading(headingIn) { }
    Point(int xIn, int yIn): x(xIn), y(yIn) { }

};

//class to define vehicle parameters
struct AckermannVehicle
{
    int length;
    int maximumVelocity;
    int minTurningRadius;
    AckermannVehicle(int lengthIn, int maximumVelocityIn, int minTurningRadiusIn) : length(lengthIn), maximumVelocity(maximumVelocityIn), minTurningRadius(minTurningRadiusIn) {}
};

//class to define Pure pursuit controller parameters
class PPController
{
private:
    double leadDistance, length, turningRadius, maximumVelocity;//set the length for ppcontroller as the length of maximumSteeringAngle

    //List of waypoints: From start to end:
    vector<Point> wpList;
    //Current target waypoint index:
    int currWpIdx;
    //List of desired heading values:
    vector<double> tgtHeading;
    //List of normal vectors to segments joining the waypoints:
    vector<Point> segNormVecList;
    //Number of waypoints:
    size_t nPts;
    //Tuning gains:
    double k_theta;

    double k_delta;

    double k_vel;

    double minVelocity;
public:
    PPController(double inputLeadDistance, double inputLength = 2.065, double inputMinTurningRadius = 4.6, double inputMaximumVelocity = 0.5);
    void initialize(string filename);
    
    //Function to compute steering angle and forward velocity commands:
    //References are all return values  
    void compute_steering_vel_cmds(Point current, double &vel, double &delta, double &distance2Goal);

    //compute the steering radius of ackerman vehicle of given parameters
    void compute_turning_radius(Point current = Point(0,0,0) , Point goal = Point(0,0,0));

    //compute the steering angle of ackermann vehicle of given paramters
    double compute_steering_angle();
    
    //compute forward velocity relative to steering angle
    double compute_forward_velocity(); //added a variable velocity based on Bijo's suggestion

    //return the unit vector of the vector
    VectorXd unit_vector(VectorXd vector);

    ~PPController();
};



#endif

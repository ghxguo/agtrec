#ifndef _UTILITIES_H_
#define _UTILITIES_H_

#include <cmath>
#include <vector>
#include <fstream>
#include <string>

using std::vector;
using std::ifstream;
using std::string;
uisng std::stod;



//struct to define vehicle position on a coordinate system at a certain heading
struct Point
{
    int x;
    int y;
    int inputHeading;
    Point(int xIn, int yIn, int headingIn): x(xIn), y(yIn), inputHeading(headingIn) { }
    Point(int xIn, int yIn, int headingIn): x(xIn), y(yIn) { }

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
    double leadDistance, length, turningRadius, maximumVelocity;
    //List of waypoints: From start to end:
    vector<Point> wpList;
    //Current target waypoint index:
    int currWpIdx;
    //List of normal vectors to segments joining the waypoints:
    vector<int> tgtHeading;
    //List of normal vectors to segments joining the waypoints:
    vector<int> segNormVecList;
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
    ~PPController();
};



#endif

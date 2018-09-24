#define _USE_MATH_DEFINES
#include <cmath>
#include "utilities.h"

PPController::PPController(double inputLeadDistance, double inputLength, double inputMinTurningRadius, double inputMaximumVelocity)
{
    leadDistance = inputLeadDistance;
    length = inputLength;
    turningRadius = inputMinTurningRadius;
    maximumVelocity = inputMaximumVelocity;
    k_theta = 0.3;
    k_delta = 1.2;
    k_vel = 0.1;
    minVelocity = 0.1;
    currWpIdx = 0;
    nPts = 0;
}

void PPController::initialize(string filename)
{
    wpList.reserve(30);
    ifstream in(filename);
    string px;
    string py;
    std::getline(in, px, ',');
    std::getline(in, py, ',');
    while(!in.fail())
    {
        wpList.push_back(Point(stod(px), stod(py) ) );
        std::getline(in, px, ',');
        std::getline(in, py, ',');
    }
    nPts = wpList.size();
    segNormVecList.reserve(nPts);
    //tgtHeading = Matrix3Xd(250);
    for(size_t i = 0; i < wpList.size(); i++)
    {
        tgtHeading.push_back(atan2(wpList[i+1].y - wpList[i].y, wpList[i+1].x-wpList[i].x));
        
         double normX = wpList[i].y - wpList[i + 1].y;
         double normY = wpList[i + 1].x - wpList[i].x;

         //Calculate the norm:
         double nVecMag = sqrt(pow(normX, 2) + pow(normY, 2));

        /* segNormVecList(0, i + 1) = normX / nVecMag;
         segNormVecList(1, i + 1) = normY / nVecMag;*/
         //result shoule be double
    }
    tgtHeading[0] = tgtHeading[1];

    for(int i = 0 ; i < segNormVecList.size() ; i++)
    {
        segNormVecList[i, 0] = segNormVecList[i, 1];
    }
}
void PPController::compute_turning_radius(Point current, Point goal)
{
    
}

void PPController::compute_steering_vel_cmds(Point current, double &vel, double &delta, double &distance2Goal)
{
    // Compute vector from current position to current waypoint:
    VectorXd vecRobot2WP = VectorXd::Zero(2,1);
    vecRobot2WP(0,0) = this->wpList[this->currWpIdx].x-current.x;
    vecRobot2WP(1,0) = this->wpList[this->currWpIdx].y - current.y;
    VectorXd vecCurHeading = VectorXd::Zero(2,1);
    vecRobot2WP(0,0) = this->wpList[this->currWpIdx].x-current.x;
    vecRobot2WP(1,0) = this->wpList[this->currWpIdx].y - current.y;
    
    vecCurHeading(0,0) = cos(this->tgtHeading[this->currWpIdx]);
    vecCurHeading(1,0) = sin(this->tgtHeading[this->currWpIdx]);
    distance2Goal = vecRobot2WP.dot(vecCurHeading);
    cout<<"distance2Goal" << distance2Goal;

    //Compute the minimum distance from the current segment:
    //change this
    double minDist = 0;
    //double minDist = vecRobot2WP.dot(segNormVecList[:,self.currWpIdx]);
    double theta_gain = this->k_theta * minDist;
        if (theta_gain > M_PI /2)
        {
            theta_gain = M_PI/2;
        }
        if(theta_gain < -M_PI/2)
        {
            theta_gain = -M_PI/2;    
        }
        cout << "minDist = " << minDist;
        cout << "theta_gain = " << theta_gain;
        //Compute the desired heading angle based of target heading and the min dist:
        auto theta_des = this->tgtHeading[this->currWpIdx] + theta_gain;

        cout << "Theta des = " << theta_des;
        //Compute the steering agle command:

        //change this
        auto heading_err = theta_des - current.inputHeading;

        if(heading_err > M_PI)
        {
            heading_err = heading_err - 2 * M_PI;
        }
        else if(heading_err < -M_PI)
        heading_err = heading_err + 2 * M_PI;
        delta = this->k_delta*(heading_err);

        cout << "Target heading = " << this->tgtHeading[this->currWpIdx];

        //Compute forward velocity:
        vel = this->maximumVelocity - abs(this->k_vel * delta);

        if (vel < this->minVelocity)
            vel = this->minVelocity;
        if (delta > 1)
            delta = 1;
        if (delta < -1)
            delta = -1;

}
// compute the steering angle of ackermann vehicle of given paramters
double PPController::compute_steering_angle()
{
    // Steering angle command from Pure pursuit paper:
    // Steering angle = atan(L/R)
    double steeringAngle = atan(length / turningRadius);
    return steeringAngle;

}    

// compute forward velocity relative to steering angle
double PPController::compute_forward_velocity()
{
    // forwardVelocity = mule.maximumVelocity * (1 - atan(abs(steeringAngle))/(pi/2));  
    //this specifies the forward velocity at a given steering angle
    double forwardVelocity = 0.4;
    return forwardVelocity;
}

static VectorXd unit_vector(VectorXd vector)
{
	//TODO
	//THIS is a STUB!!!
	return VectorXd();
}

PPController::~PPController()
{
    
}

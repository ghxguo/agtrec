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
    tgtHeading.push_back(0);
    for(size_t i = 0; i < wpList.size(); i++)
    {
        tgtHeading.push_back(0);//Place holder, CHANGE!!
    }
    

        // # Loop to compute the target heading values:
        // for idx in range(0, len(self.wpList)-1):

        //     self.tgtHeading.append( math.atan2( self.wpList[idx + 1].y - self.wpList[idx].y , self.wpList[idx+1].x - self.wpList[idx].x))

        //     normX = self.wpList[idx].y - self.wpList[idx + 1].y
        //     normY = self.wpList[idx + 1].x - self.wpList[idx].x     #bug might live here

        //     # Calculate the norm:
        //     nVecMag = np.sqrt( normX**2 + normY**2)

        //     self.segNormVecList[0,idx+1] = normX/nVecMag
        //     self.segNormVecList[1,idx+1] = normY/nVecMag

        // self.tgtHeading[0] = self.tgtHeading[1]
        // self.segNormVecList[:,0] = self.segNormVecList[:,1]
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
    distance2aGoal = vecRobot2WP.dot(vecCurHeading);
    cout<<"distance2Goal" << distance2aGoal;

    //Compute the minimum distance from the current segment:
    //change this
    double minDist = vecRobot2WP.dot(segNormVecList[:,self.currWpIdx]);
    double theta_gain = this->k_theta * minDist;
        if (theta_gain > M_PI/2)
        {
            theta_gain = M_PI/2;
        }
        if(theta_gain < -M_PI/2)
        {
            theta_gain = -M_PI/2;    
        }
        cout << "minDist = " << minDist;
        cout << "theta_gain =" << theta_gain;
        //Compute the desired heading angle based of target heading and the min dist:
        auto theta_des = this->tgtHeading[this->currWpIdx] + theta_gain;

        cout << "Theta des = ",theta_des);
        //Compute the steering agle command:

        //change this
        auto heading_err = theta_des - current.heading;

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


PPController::~PPController()
{
    
}

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
        // self.segNormVecList = np.zeros((2,self.nPts))

        // self.tgtHeading.append(0)
    
    for(size_t i = 0; i < wpList.size(); i++)
    {
        tgtHeading.push_back()
    }
    

        # Loop to compute the target heading values:
        for idx in range(0, len(self.wpList)-1):

            self.tgtHeading.append( math.atan2( self.wpList[idx + 1].y - self.wpList[idx].y , self.wpList[idx+1].x - self.wpList[idx].x))

            normX = self.wpList[idx].y - self.wpList[idx + 1].y
            normY = self.wpList[idx + 1].x - self.wpList[idx].x     #bug might live here

            # Calculate the norm:
            nVecMag = np.sqrt( normX**2 + normY**2)

            self.segNormVecList[0,idx+1] = normX/nVecMag
            self.segNormVecList[1,idx+1] = normY/nVecMag

        self.tgtHeading[0] = self.tgtHeading[1]
        self.segNormVecList[:,0] = self.segNormVecList[:,1]
}

PPController::~PPController()
{
    
}

/**
 * This file is part of FSD Localization

 * Copyright (c) 2020 Renan Maffei <rqmaffei at inf dot ufrgs dot br> (Phi Robotics Research Lab - UFRGS)
 * For more information see <https://github.com/phir2-lab/FSD_localization>

 * FSD Localization is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * FSD Localization is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with FSD Localization.  If not, see <https://www.gnu.org/licenses/>.
**/

#ifndef ROBOT_H
#define ROBOT_H

#include <vector>

//#include "RobotBase.h"
#include "RobotBase_ROS.h"

#include "Grid.h"
#include "Utils.h"
#include "MCL.h"

class Robot
{
public:
    Robot(int argc, char* argv[]);
    ~Robot();

    void initialize();
    void run();

    void move(MovingDirection dir);
    void draw(float xRobot, float yRobot, float angRobot);
    void drawPath();
    void drawMCL();


    const Pose& getCurrentPose();

    bool isReady();
    bool isRunning();

    Grid* grid;
    MotionMode motionMode_;
    int viewMode;
    int numViewModes;

    MCL* mcl;
    bool mclHasConverged;


protected:

    Pose currentPose_;
    Pose curLocalizationPose_;
    Pose prevLocalizationPose_;
    std::vector<Pose> path_;

    bool ready_;
    bool running_;

    int windowSize_;

    RobotBase_ROS base;
    std::vector<Measurement> maxCloudRanges;
    std::vector<Measurement> randomCloudRanges;

    // Navigation stuff
    void wanderAvoidingCollisions();
    void wallFollow();
    bool isFollowingLeftWall_;

    // Mapping stuff
    void mappingWithHIMMUsingLaser();
    void mappingUsingDepthCloud();
    void mappingUsingRangeMeasurements();
    void cleanMapping();
    bool setOccupationFromPCL;

    void convertDepthCloudToRangeReadings();

    void saveMapTXT();
    void saveMapPNG();


    Timer controlTimer;
    Timer imageTimer;
};

#endif // ROBOT_H

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

#ifndef ROBOTBASE_H
#define ROBOTBASE_H

#include "Utils.h"
#include "Grid.h"

class RobotBase
{
public:
    RobotBase();

    // Drawing stuff
    void drawBase();
    void drawSonars(bool drawCones=false);
    void drawLasers(bool fill=true);

    // Navigation stuff
    void resumeMovement();
    void setMovementSimple(MovingDirection dir);
    void setMovementVel(MovingDirection dir);
    void setLinAngVelocity_fromWheelsVelocity(float lv, float rv);
    void setWheelsVelocity_fromLinAngVelocity(float linV, float angV);
    void setWheelsVelocity(float vl, float vr);
    void setLinAngVelocity(float linV, float angV);

    // Sensors stuff
    //bool readOdometryAndSensors();
    const Pose& getOdometry();
    const Pose& getSLAMPose();
    const std::vector<float>& getLaserReadings();
    const std::vector<float>& getSonarReadings();
    const std::vector<Point3D>& getDepthReadings();


    int getNumLasers();
    int getNumSonars();
    float getMaxLaserRange();
    float getMaxSonarRange();
    void setGrid(Grid* grid);
    void setOdometry(const Pose &o);
    void setSonarReadings(const std::vector<float> &s);
    void setLaserReadings(const std::vector<float> &l);

    float getMinSonarValueInRange(int idFirst, int idLast);
    float getMinLaserValueInRange(int idFirst, int idLast, int kernelSize=0);

    int getNearestSonarBeam(float angle);
    float getAngleOfSonarBeam(int k);
    float getKthSonarReading(int k);

    int getNearestLaserBeam(float angle);
    float getAngleOfLaserBeam(int k);
    float getKthLaserReading(int k);

    double getSensorsTime();

    // Log stuff
    void writeOnLog();
    bool readFromLog();

    std::string srcPath;
    std::string mapName;
    std::string fullMapName;
    std::string methodName;

    //Reset
//    void reset_pose(float x, float y, float th);

protected:
    Pose odometry_;
    Pose slamPose_;
    Grid* grid_;

    bool isSimulation;

    // Navigation stuff
    double vLeft_, vRight_;
    double oldVLeft_, oldVRight_;
    double linVel_, angVel_;

    // Sensors stuff
    int numSonars_;
    std::vector<float> sonars_;
    float maxSonarRange_;

    std::vector<float> lasers_;
    int numLasers_;
    float minAngleLaser_, maxAngleLaser_, angleLaserIncrement;
    float minLaserRange_, maxLaserRange_;

    std::vector<Point3D> depthCloud_;
    int numDepthPoints_;

    double seconds_,initialSeconds_;

    LogFile* logFile_;
};

#endif // ROBOTBASE_H

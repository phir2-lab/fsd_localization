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

#include "Robot.h"

#include <GL/glut.h>
#include <cmath>
#include <iostream>
#include <FreeImage.h>
#include <random>
#include <map>


//////////////////////////////////////
///// CONSTRUCTORS & DESTRUCTORS /////
//////////////////////////////////////

Robot::Robot(int argc, char *argv[]):
    base(argc,argv)
{
    ready_ = false;
    running_ = true;

    grid = new Grid();
    mcl = NULL;

    // variables used for navigation
    isFollowingLeftWall_=false;

    // variables used for visualization
    viewMode=1;
    numViewModes=5;
    motionMode_ = MANUAL_SIMPLE;

    mclHasConverged = false;
    setOccupationFromPCL = true;

    windowSize_ = 3.0*grid->getMapScale();

}

Robot::~Robot()
{
    if(grid!=NULL)
        delete grid;
}

////////////////////////////////////
///// INITIALIZE & RUN METHODS /////
////////////////////////////////////

void Robot::initialize()
{
    // initialize ARIA
    bool success = base.initialize();

    Pose initialPose;
    std::string mapName;

    mcl = new MCL(initialPose,base.getMaxLaserRange(),base.getNumCams(),base.fullMapName,base.methodName,grid->mutex);

    ready_ = true;
    controlTimer.startLap();
    imageTimer.startLap();

}

int step=1;

void Robot::run()
{
    controlTimer.waitTime(0.1);

    //cleanMappingUsingDepthCloud();
    bool success = base.readOdometryAndSensors();
    if(!success){
        usleep(50000);
        base.resumeMovement();
        return;
    }

    currentPose_ = base.getSLAMPose();
    Pose odometry = base.getOdometry();

//    std::cout << "TIME " << base.getSensorsTime() << std::endl;

    convertDepthCloudToRangeReadings();

    pthread_mutex_lock(grid->mutex);
    cleanMapping();
    mappingWithHIMMUsingLaser();
    setOccupationFromPCL = !setOccupationFromPCL;
    mappingUsingDepthCloud();
    setOccupationFromPCL = !setOccupationFromPCL;
    mappingUsingRangeMeasurements();
    grid->iterations++;
    pthread_mutex_unlock(grid->mutex);


    Action u;
    u.rot1 = atan2(odometry.y-prevLocalizationPose_.y,odometry.x-prevLocalizationPose_.x)-DEG2RAD(odometry.theta);
    u.trans = sqrt(pow(odometry.x-prevLocalizationPose_.x,2)+pow(odometry.y-prevLocalizationPose_.y,2));
    u.rot2 = DEG2RAD(odometry.theta)-DEG2RAD(prevLocalizationPose_.theta)-u.rot1;

    // check if there is enough robot motion
    if(u.trans > 0.1 || fabs(odometry.theta-prevLocalizationPose_.theta) > 30)
    {
        std::cout << "Current Pose " << currentPose_ << std::endl;
        double dist = sqrt(pow(currentPose_.x,2.0)+pow(currentPose_.y,2.0));
        std::cout << "current Dist " << dist << std::endl;
//        std::cout << "Trans " << u.trans << " rot1 " << RAD2DEG(u.rot1) << " rot2 " << RAD2DEG(u.rot2) << std::endl;
        path_.push_back(odometry);

        std::array<float,2> density = mcl->computeDensityInGrid(currentPose_,grid);
        std::cout << "DENSITY " << density[0] << " " << density[1] << std::endl;

        mcl->run(base.getSensorsTime(), currentPose_, u,base.getLaserReadings(),density,randomCloudRanges);
        curLocalizationPose_ = mcl->getMeanParticlePose();

        prevLocalizationPose_ = odometry;
    }


    // Save path traversed by the robot
//    if(base.isMoving()){
//        path_.push_back(base.getOdometry());
//        path_.push_back(currentPose_);
//    }

    // Navigation
    switch(motionMode_){
        case WANDER:
            wanderAvoidingCollisions();
            break;
        case WALLFOLLOW:
            wallFollow();
            break;
        case ENDING:
            // saveMapPNG();
            // saveMapTXT();
            running_=false;
            break;
        default:
            break;
    }

//    if(imageTimer.getLapTime() > 10){
//        saveMapPNG();
//        imageTimer.startLap();
//    }

    base.resumeMovement();

    usleep(50000);
}

///////////////////////////
///// MAPPING METHODS /////
///////////////////////////

void Robot::mappingWithHIMMUsingLaser()
{
    // TODO: update cells in the sensors' field-of-view
    // Follow the example in mappingWithLogOddsUsingLaser()

    int scale = grid->getMapScale();

    float alpha = 0.05; //10 cm
    float beta = 0.25;  //0.5 degrees

    int kernel=1;

    int robotX=currentPose_.x*scale;
    int robotY=currentPose_.y*scale;

    float maxRange = base.getMaxLaserRange();
    int maxRangeInt = maxRange*scale;

    float robotAngle = currentPose_.theta;

    Cell* r=grid->getCell(robotX,robotY);
    r->himm -= 1;
    if(r->himm<0)
        r->himm=0;

    const std::vector<float>& laser = base.getLaserReadings();

//    for(int i=robotX-maxRangeInt; i<=robotX+maxRangeInt; i++){
//        for(int j=robotY-maxRangeInt; j<=robotY+maxRangeInt; j++){

    for(int i=robotX-windowSize_; i<=robotX+windowSize_; ++i){
        for(int j=robotY-windowSize_; j<=robotY+windowSize_; ++j){

            float r = sqrt(pow(i-robotX,2.0)+pow(j-robotY,2.0));
            if(r > windowSize_)
                continue;

            Cell* c = grid->getCell(i,j);

            r = r/scale;
            float phi = RAD2DEG(atan2(j-robotY,i-robotX)) - robotAngle;
            phi = normalizeAngleDEG(phi);

            int k = base.getNearestLaserBeam(phi);

            if(phi < 90.0+beta && phi > -90.0-beta){
                if(laser[k] < maxRange && fabs(r-laser[k])<alpha){
                    c->himm += 3;
                    if(c->himm>15)
                        c->himm=15;

//                    for(int i2=i-kernel;i2<=i+kernel;i2++)
//                        for(int j2=j-kernel;j2<=j+kernel;j2++){
//                            Cell* t = grid->getCell(i2,j2);
//                            t->himm += 3;
//                            if(t->himm>15)
//                                t->himm=15;
//                        }

                    if(setOccupationFromPCL == false){
                        if(c->occType == UNEXPLORED && c->himm>9)
                            c->occType = OCCUPIED;
                        else if(c->occType == FREE && c->himm>7)
                            c->occType = OCCUPIED;
                    }

                }
                else if(r <= laser[k] && r < maxRange){
                    c->himm -= 1;
                    if(c->himm<0)
                        c->himm=0;

                    if(setOccupationFromPCL == false){
                        if(c->occType == UNEXPLORED && c->himm<5)
                            c->occType = FREE;
                        else if(c->occType == OCCUPIED && c->himm<10)
                            c->occType = FREE;
                    }

                }
            }

        }
    }

}

void Robot::cleanMapping()
{
    int scale = grid->getMapScale();
    int robotX=currentPose_.x*scale;
    int robotY=currentPose_.y*scale;

    for(int i=robotX-windowSize_*2; i<robotX+windowSize_*2; ++i){
        for(int j=robotY-windowSize_*2; j<robotY+windowSize_*2; ++j){
            float r = sqrt(pow(i-robotX,2.0)+pow(j-robotY,2.0));

            Cell* c = grid->getCell(i,j);

            if(r > windowSize_){
                c->himm=7;
                c->himm_cloud=7;
                c->himm_cloud2=7;
                c->occType = UNEXPLORED;
            }
            else{
                if(c->occType == KERNEL)
                    c->occType = FREE;
                else if(c->occType == KERNEL_UNK)
                    c->occType = UNEXPLORED;
            }
        }
    }

}

void Robot::mappingUsingDepthCloud()
{
    int scale = grid->getMapScale();
    int robotX=currentPose_.x*scale;
    int robotY=currentPose_.y*scale;

    float alpha = 0.05; //10 cm
    float beta = 0.25;  //0.5 degrees

    const std::vector<Point3D>& cloud = base.getDepthReadings();

    for(unsigned int p=0; p<cloud.size(); p++){
        if(cloud[p].z<0.5 || cloud[p].z>2.0)
            continue;

        int i=cloud[p].x*scale;
        int j=cloud[p].y*scale;
        float r = sqrt(pow(i-robotX,2.0)+pow(j-robotY,2.0));
        if(r > windowSize_)
            continue;

        grid->updateBoundaries(i,j);

        Cell* c = grid->getCell(i,j);
//        c->cloud = grid->iterations;

        // raycast from robot to cloud cell
        float deltaX=i-robotX;
        float deltaY=j-robotY;
        float delta = std::max(fabs(deltaX),fabs(deltaY));
        deltaX /= delta;
        deltaY /= delta;

        // Filling empty space
        float x=robotX;
        float y=robotY;
        for(int k=0;k<(int)delta;k++){
            x+=deltaX;
            y+=deltaY;

            c=grid->getCell((int)x,(int)y);
            c->cloud = grid->iterations;


            c->himm_cloud -= 1;
            if(c->himm_cloud<0)
                c->himm_cloud=0;

            if(setOccupationFromPCL){
                if(c->occType == UNEXPLORED && c->himm_cloud<5)
                    c->occType = FREE;
                else if(c->occType == OCCUPIED && c->himm_cloud<10)
                    c->occType = FREE;
            }
        }

        // Filling obstacle space
        for(int k=0;k<3;k++){
            x+=deltaX;
            y+=deltaY;
            c=grid->getCell((int)x,(int)y);

//            if(c->cloud != grid->iterations){
//                c->cloud = grid->iterations;


                for(int u=-3;u<=3;u++)
                    for(int v=-3;v<=3;v++){
                        Cell* n = grid->getCell((int)x + u,(int)y + v);
                        if(n->cloud != grid->iterations){
                            n->cloud = grid->iterations;
                            n->himm_cloud += 3;
                            if(n->himm_cloud>15)
                                n->himm_cloud=15;

                            if(setOccupationFromPCL){
                                if(n->occType == UNEXPLORED && n->himm_cloud>9)
                                    n->occType = OCCUPIED;
                                else if(n->occType == FREE && n->himm_cloud>7)
                                    n->occType = OCCUPIED;
                            }
                        }

                    }

//                c->himm_cloud += 3;
//                if(c->himm_cloud>15)
//                    c->himm_cloud=15;
//            }


//            if(c->occType == UNEXPLORED && c->himm_cloud>9)
//                c->occType = OCCUPIED;
//            else if(c->occType == FREE && c->himm_cloud>7)
//                c->occType = OCCUPIED;

        }


    }
//    std::cout << "Limits: X " << grid->minX << ' ' << grid->maxX << " Y " << grid->minY << ' ' << grid->maxY << std::endl;

}

void Robot::mappingUsingRangeMeasurements()
{
    int scale = grid->getMapScale();
    int robotX=currentPose_.x*scale;
    int robotY=currentPose_.y*scale;

    Cell* c;

    for(unsigned int p=0; p<maxCloudRanges.size(); p++){
        double angle = maxCloudRanges[p].phi + currentPose_.theta;
//        std::cout << " angle " << angle << "|" << maxCloudRanges[p].r << std::flush;
        angle = DEG2RAD(angle);

        float deltaX = cos(angle)*maxCloudRanges[p].r*scale;
        float deltaY = sin(angle)*maxCloudRanges[p].r*scale;

        if(maxCloudRanges[p].r*scale > windowSize_){
            deltaX = cos(angle)*windowSize_;
            deltaY = sin(angle)*windowSize_;
        }

        // raycast from robot to cloud cell
        float delta = std::max(fabs(deltaX),fabs(deltaY));
        deltaX /= delta;
        deltaY /= delta;

        // Filling empty space
        float x=robotX;
        float y=robotY;
        for(int k=0;k<(int)delta;k++){
            x+=deltaX;
            y+=deltaY;

            c=grid->getCell((int)x,(int)y);
            c->cloud2 = grid->iterations;


            c->himm_cloud2 -= 1;
            if(c->himm_cloud2<0)
                c->himm_cloud2=0;

            if(setOccupationFromPCL){
                if(c->occType == UNEXPLORED && c->himm_cloud2<5)
                    c->occType = FREE;
                else if(c->occType == OCCUPIED && c->himm_cloud2<10)
                    c->occType = FREE;
            }
        }

        if(maxCloudRanges[p].r*scale > windowSize_){
            continue;
        }

        // Filling obstacle space
        for(int k=0;k<3;k++){
            x+=deltaX;
            y+=deltaY;
            c=grid->getCell((int)x,(int)y);

                for(int u=-2;u<=2;u++)
                    for(int v=-2;v<=2;v++){
                        Cell* n = grid->getCell((int)x + u,(int)y + v);
                        if(n->cloud2 != grid->iterations){
                            n->cloud2 = grid->iterations;
                            n->himm_cloud2 += 3;
                            if(n->himm_cloud2>15)
                                n->himm_cloud2=15;

                            if(setOccupationFromPCL){
                                if(n->occType == UNEXPLORED && n->himm_cloud2>9)
                                    n->occType = OCCUPIED;
                                else if(n->occType == FREE && n->himm_cloud2>7)
                                    n->occType = OCCUPIED;
                            }
                        }

                    }
        }


    }
//    std::cout << "Limits: X " << grid->minX << ' ' << grid->maxX << " Y " << grid->minY << ' ' << grid->maxY << std::endl;

}

void Robot::convertDepthCloudToRangeReadings()
{
    const std::vector<Point3D>& cloud = base.getDepthReadings();

    std::vector<Measurement> z;
    std::cout << "cloud.size() " << cloud.size() << std::endl;

    for(unsigned int p=0; p<cloud.size(); p++){
//        std::cout << "p " << p << std::endl;

        if(cloud[p].z<0.5 || cloud[p].z>2.0)
            continue;



        Measurement m;
        m.r = sqrt(pow(cloud[p].x-currentPose_.x,2.0)+pow(cloud[p].y-currentPose_.y,2.0));
        m.phi = RAD2DEG(atan2(cloud[p].y-currentPose_.y,cloud[p].x-currentPose_.x)) - currentPose_.theta;
        m.phi = normalizeAngleDEG(m.phi);

        float r3D = sqrt(pow(cloud[p].x-currentPose_.x,2.0)+pow(cloud[p].y-currentPose_.y,2.0) + pow(cloud[p].z-1.045,2.0));
        m.cos_pitch = m.r/r3D;
        m.pitch = acos(m.cos_pitch);
        m.cos_pitch = fabs(m.cos_pitch);

//        std::cout << "m " << m.r << " " << m.phi << std::endl;


        z.push_back(m);

//        std::cout << "z.size() " << z.size() << std::endl;
    }

    std::cout << "CLOUD SIZE " << z.size() << std::endl;

    if(z.empty())
        return;

    // Select random ranges
    int K=mcl->numRanges;
    randomCloudRanges.resize(K);
    int total = z.size();

//    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(0,total-1);

    for(unsigned int i=0; i<K; i++)
    {
        int r = distribution(*(mcl->generator));

        randomCloudRanges[i] = z[r];
//        std::cout << "r " << r << " angle " << randomCloudRanges[i].phi << "|" << randomCloudRanges[i].r << " ";
    }

    // Find max ranges in all ranges

    std::map<int,float> maxRanges;
    std::map<int,float>::iterator it;

    for(unsigned int p=0; p<z.size(); p++){
        int a = z[p].phi;

        it = maxRanges.find(a);
        if (it != maxRanges.end()){
            if(z[p].r > maxRanges[a])
                maxRanges[a] = z[p].r;
        }else if(z[p].r > 0.1)
            maxRanges[a] = z[p].r;
    }

    maxCloudRanges.resize(maxRanges.size());
    unsigned int p=0;
    for(it=maxRanges.begin(); it!=maxRanges.end(); ++it){
        maxCloudRanges[p].phi = it->first;
        maxCloudRanges[p].r = it->second;
        p++;
//        std::cout << maxCloudRanges[p].phi << "|" << maxCloudRanges[p].r << " ";
    }
//    std::cout << std::endl;

}

void Robot::saveMapTXT()
{
    std::string path = ros::package::getPath("fsd_localization");
    std::cout << path << std::endl;

    LogFile mapFile(MAP, path+"/maps/");

    int border = 5;

    std::cout << "Limits: X " << grid->minX << ' ' << grid->maxX << " Y " << grid->minY << ' ' << grid->maxY << std::endl;

    int width = grid->maxX-grid->minX+1+2*border;
    int height = grid->maxY-grid->minY+1+2*border;

    mapFile.writeInt(width);
    mapFile.writeInt(height);
    mapFile.breakLine();

    for(int j=grid->minY-border; j<=grid->maxY+border; ++j){
        for(int i=grid->minX-border; i<=grid->maxX+border; ++i){
            Cell* c = grid->getCell(i,j);
            if(c->himm_cloud > 12)
                mapFile.writeInt(1);
            else if(c->himm_cloud < 3)
                mapFile.writeInt(0);
            else {
                mapFile.writeString("- ");
            }
        }
        mapFile.breakLine();
    }
}

void Robot::saveMapPNG()
{
    std::string path = ros::package::getPath("fsd_localization");
    std::cout << path << std::endl;

    int border = 5;

    int width = grid->maxX-grid->minX+1+2*border;
    int height = grid->maxY-grid->minY+1+2*border;

    // Make the BYTE array, factor of 3 because it's RBG.
    BYTE* pixelsRGBD = new BYTE[ 3 * width * height];
    BYTE* pixelsLASER = new BYTE[ 3 * width * height];

    int p=0;
    for(int j=grid->minY-border; j<=grid->maxY+border; ++j){
        for(int i=grid->minX-border; i<=grid->maxX+border; ++i){
            Cell* c = grid->getCell(i,j);
            if(c->himm_cloud > 12){
                pixelsRGBD[p] = pixelsRGBD[p+1] = pixelsRGBD[p+2] = 0;
            }else if(c->himm_cloud < 3)
                pixelsRGBD[p] = pixelsRGBD[p+1] = pixelsRGBD[p+2] = 255;
            else {
                pixelsRGBD[p] = pixelsRGBD[p+1] = pixelsRGBD[p+2] = 127;
            }

            if(c->himm > 12){
                pixelsLASER[p] = pixelsLASER[p+1] = pixelsLASER[p+2] = 0;
            }else if(c->himm < 3)
                pixelsLASER[p] = pixelsLASER[p+1] = pixelsLASER[p+2] = 255;
            else {
                pixelsLASER[p] = pixelsLASER[p+1] = pixelsLASER[p+2] = 127;
            }

            p=p+3;
        }
    }

    time_t t = time(0);
    struct tm *now = localtime(&t);
    std::stringstream ss;
    std::string imgName;
    ss << path+"/maps/map-" << -100+now->tm_year
                    << std::setfill('0') << std::setw(2) << 1+now->tm_mon
                    << std::setfill('0') << std::setw(2) << now->tm_mday << '-'
                    << std::setfill('0') << std::setw(2) << now->tm_hour
                    << std::setfill('0') << std::setw(2) << now->tm_min
                    << std::setfill('0') << std::setw(2) << now->tm_sec << "-depth.png";
    ss >> imgName;

    // Convert to FreeImage format & save to file
    FIBITMAP* imageRGBD = FreeImage_ConvertFromRawBits(pixelsRGBD, width, height, 3 * width, 24, 0xFF0000, 0x0000FF, 0xFF0000, false);
    FreeImage_Save(FIF_PNG, imageRGBD, imgName.c_str(), 0);

    ss.clear();
    ss << path+"/maps/map-" << -100+now->tm_year
                    << std::setfill('0') << std::setw(2) << 1+now->tm_mon
                    << std::setfill('0') << std::setw(2) << now->tm_mday << '-'
                    << std::setfill('0') << std::setw(2) << now->tm_hour
                    << std::setfill('0') << std::setw(2) << now->tm_min
                    << std::setfill('0') << std::setw(2) << now->tm_sec << "-laser.png";
    ss >> imgName;

    // Convert to FreeImage format & save to file
    FIBITMAP* imageLASER = FreeImage_ConvertFromRawBits(pixelsLASER, width, height, 3 * width, 24, 0xFF0000, 0x0000FF, 0xFF0000, false);
    FreeImage_Save(FIF_PNG, imageLASER, imgName.c_str(), 0);

    // Free resources
    FreeImage_Unload(imageRGBD);
    FreeImage_Unload(imageLASER);

    delete [] pixelsRGBD;
    delete [] pixelsLASER;
}

//////////////////////////////
///// NAVIGATION METHODS /////
//////////////////////////////

void Robot::move(MovingDirection dir)
{
    switch(dir){
        case FRONT:
            std::cout << "moving front" << std::endl;
            break;
        case BACK:
            std::cout << "moving back" << std::endl;
            break;
        case LEFT:
            std::cout << "turning left" << std::endl;
            break;
        case RIGHT:
            std::cout << "turning right" << std::endl;
            break;
        case STOP:
            std::cout << "stopping robot" << std::endl;
    }

    if(motionMode_==MANUAL_SIMPLE)
        base.setMovementSimple(dir);
    else if(motionMode_==MANUAL_VEL)
        base.setMovementVel(dir);
    else if(motionMode_=WALLFOLLOW)
        if(dir==LEFT)
            isFollowingLeftWall_=true;
        else if(dir==RIGHT)
            isFollowingLeftWall_=false;
}

void Robot::wanderAvoidingCollisions()
{
    float linVel=0;
    float angVel=0;

    //TODO - implementar desvio de obstaculos




    base.setWheelsVelocity_fromLinAngVelocity(linVel, angVel);
}

void Robot::wallFollow()
{
    float linVel=0;
    float angVel=0;

    if(isFollowingLeftWall_)
        std::cout << "Following LEFT wall" << std::endl;
    else
        std::cout << "Following RIGHT wall" << std::endl;

    //TODO - implementar wall following usando PID




    base.setWheelsVelocity_fromLinAngVelocity(linVel, angVel);
}


////////////////////////
///// DRAW METHODS /////
////////////////////////

void Robot::draw(float xRobot, float yRobot, float angRobot)
{
    float scale = grid->getMapScale();
    glTranslatef(xRobot,yRobot,0.0);    
    glRotatef(angRobot,0.0,0.0,1.0);

    glScalef(1.0/scale,1.0/scale,1.0/scale);

    // sonars and lasers draw in cm
    if(viewMode==1)
        base.drawSonars(true);
    else if(viewMode==2)
        base.drawSonars(false);
    else if(viewMode==3)
        base.drawLasers(true);
    else if(viewMode==4)
        base.drawLasers(false);

    // robot draw in cm
    base.drawBase();

    glScalef(scale,scale,scale);
    glRotatef(-angRobot,0.0,0.0,1.0);
    glTranslatef(-xRobot,-yRobot,0.0);
}

void Robot::drawPath()
{
    float scale = grid->getMapScale();

    if(path_.size() > 1){
        glScalef(scale,scale,scale);
        glLineWidth(3);
        glBegin( GL_LINE_STRIP );
        {
            for(unsigned int i=0;i<path_.size()-1; i++){
                glColor3f(1.0,0.0,1.0);

                glVertex2f(path_[i].x, path_[i].y);
                glVertex2f(path_[i+1].x, path_[i+1].y);
            }
        }
        glEnd();
        glLineWidth(1);
        glScalef(1.0/scale,1.0/scale,1.0/scale);

    }
}

void Robot::drawMCL()
{
    mcl->draw();
}

/////////////////////////
///// OTHER METHODS /////
/////////////////////////

bool Robot::isReady()
{
    return ready_;
}

bool Robot::isRunning()
{
    return running_;
}

const Pose& Robot::getCurrentPose()
{
    return currentPose_;
}


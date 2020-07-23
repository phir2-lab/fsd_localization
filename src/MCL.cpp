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

#include "MCL.h"

#include <GL/glut.h>
#include <FreeImage.h>

#include <iostream>
#include <string>
#include <fstream>
#include <string>
#include <chrono>
#include <sys/stat.h>

#include <unistd.h>
#include <sstream>
#include <iomanip>


//////////////////////////////////////
///// CONSTRUCTORS & DESTRUCTORS /////
//////////////////////////////////////

MCL::MCL(Pose initialPose, float maxRange, int nc, std::string mapName, std::string methodName, pthread_mutex_t* m):
    odomPose(initialPose), maxRange(maxRange), mutex(m), numCams(nc)
{
    viewMode = 0;
    numViewModes = 5;
    totalDistance = 0;

    // construct a trivial random generator engine from a time-based seed:
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator = new std::default_random_engine(seed);

    readMap(mapName);
    scale = 20;
    transparency = true;

    kernelWidth = 1.5*scale;
    angleStep = DEG2RAD(1.0);
    kernelCells = computeMaxDensityCellsValue();
    numRanges = 100;

    if(readMapOfDensities(mapName) == false){
        computeMapOfDensities();
        saveMapOfDensities(mapName);
    }

    computeLikelihoodMap();

    numParticles = 20000;

    std::cout << "Method Name: " << methodName << std::endl;

    wt = ABS_PCL_FSD;
    if(methodName == "PCL_FSD")
        wt = ABS_PCL_FSD;
    else if(methodName == "PCL_FSD_UNK")
        wt = INT_PCL_FSD;
    else if(methodName == "CLikelihood")
        wt = CLOUD_LIKELIHOOD;
    else if(methodName == "CRaycast")
        wt = CLOUD_RAYCAST;
    else if(methodName == "LRaycast")
        wt = LASER_RAYCAST;
    else if(methodName == "Motion")
        wt = SIMPLE_WEIGHTS;

    covMajorAxis=-1;

    pthread_mutex_lock(mutex);
    initParticles();
    pthread_mutex_unlock(mutex);
    updateMeanAndCovariance();

    initLogFile(mapName);

}

////////////////////////////////////
///// INITIALIZE & RUN METHODS /////
////////////////////////////////////

void MCL::initParticles()
{
    particles.resize(numParticles);

    std::uniform_real_distribution<double> randomX(0.0,mapWidth/scale);
    std::uniform_real_distribution<double> randomY(0.0,mapHeight/scale);
    std::uniform_real_distribution<double> randomTh(-M_PI,M_PI);

    // generate initial set
    for(int i=0; i<numParticles; i++){

        bool valid = false;
        do{
            // sample particle pose
            particles[i].p.x = randomX(*generator);
            particles[i].p.y = randomY(*generator);
            particles[i].p.theta = randomTh(*generator);

            //particles[i].p.theta = truePose.theta;

            // check if particle is valid (known and not obstacle)
            if(mapCells[(int)(particles[i].p.x*scale)][(int)(particles[i].p.y*scale)] == FREE)
                valid = true;

        }while(!valid);

        std::cout << "Particle (" << i << "): "
                  << particles[i].p.x << ' '
                  << particles[i].p.y << ' '
                  << RAD2DEG(particles[i].p.theta) << std::endl;
    }
}

void MCL::readMap(std::string mapName)
{
    std::string name = mapName+"/fullhouse1.txt";
    std::ifstream file;
    file.open(name.c_str(), std::ifstream::in);

    if( !file.good() )
    {
        std::cerr << "The file \"" << name << "\"  does not exist!" << std::endl;
        return;
    }

//     Read goal pose
//    file >> goal.x >> goal.y;
//    std::cout << "Goal x " << goal.x << " y " << goal.y << std::endl;

//     Read initial pose
    file >> initialPose .x >> initialPose.y >> initialPose.theta;
    std::cout << "Initial pose x " << initialPose.x << " y " << initialPose.y << " th " << initialPose.theta << std::endl;
    odomPose = initialPose;


    // Read dimensions.
    file >> mapWidth >> mapHeight;
    std::cout << "map.width " << mapWidth << " map.height " << mapHeight << std::endl;

    mapCells = new CellOccType*[mapWidth];
        for(int i=0;i<mapWidth;i++)
            mapCells[i] = new CellOccType[mapHeight];

    // Read grid from file.
    char read;
    for(int y=0; y < mapHeight; y++)
    {
        for(int x=0; x < mapWidth; x++)
        {
            file >> read;
            switch(read)
            {
                case '1':
                    mapCells[x][mapHeight-y-1] = OCCUPIED;
                    break;
                case '0':
                    mapCells[x][mapHeight-y-1] = FREE;
                    break;
                case '-':
                    mapCells[x][mapHeight-y-1] = UNEXPLORED;
                    break;
            }
        }
    }

    file.close();
}

void MCL::run(double ts, const Pose& tp, const Action &u, const std::vector<float> &z_laser, const std::array<float,2> &z_density, const std::vector<Measurement> &z_ranges)
{
    timeStamp = ts;

    // updating true pose
    odomPose.x += u.trans*cos(odomPose.theta + u.rot1);
    odomPose.y += u.trans*sin(odomPose.theta + u.rot1);
    odomPose.theta += u.rot1 + u.rot2;
    std::cout << "TruePose " <<odomPose << std::endl;

    // find relative pose
    Pose relative;
    relative.x = cos(initialPose.theta)*(odomPose.x-initialPose.x) -sin(initialPose.theta)*(odomPose.y-initialPose.y);
    relative.y = sin(initialPose.theta)*(odomPose.x-initialPose.x) +cos(initialPose.theta)*(odomPose.y-initialPose.y);
    relative.theta = odomPose.theta - initialPose.theta;
//    std::cout << "Relative " << relative << std::endl;

//    std::cout << "TP " << tp << std::endl;

    truePose.x = cos(initialPose.theta)*(tp.x) -sin(initialPose.theta)*(tp.y) + initialPose.x;
    truePose.y = sin(initialPose.theta)*(tp.x) +cos(initialPose.theta)*(tp.y) + initialPose.y;
    truePose.theta = DEG2RAD(tp.theta) + initialPose.theta;
    std::cout << "truePose " << truePose << std::endl;

    std::cout << "MCL Diff " << odomPose.x-truePose.x << " " << odomPose.y-truePose.y << std::endl;


//    double dist = sqrt(pow(truePose.x-initialPose.x,2.0)+pow(truePose.y-initialPose.y,2.0));
//    std::cout << "Dist " << dist << std::endl;

    odomPath.push_back(odomPose);
    truePath.push_back(truePose);
    totalDistance += u.trans;

    cleanCellsAroundPose(truePose.x*scale, truePose.y*scale);
    currentFSD = computeDensity(truePose.x*scale, truePose.y*scale);
    std::cout << "Density " << currentFSD << std::endl;

    measuredFSD    = z_density[0];
    measuredFSDUNK = z_density[1];

    t1.startCounting();
    sampling(u);
    t3.startCounting();
    weighting(wt,z_laser,z_density,z_ranges);
    t3.stopCounting();

    pthread_mutex_lock(mutex);
    resampling();
    updateMeanAndCovariance();
    pthread_mutex_unlock(mutex);

    t1.stopCounting();


    computeError();


//    screenshot();

//    std::cout << "GOAL " << goal.x << ' ' << goal.y << " - Mean " << meanParticle[0] << " " << meanParticle[1] << std::endl;
}

///////////////////////////////////////////
///// SAMPLING, WEIGHTING, RESAMPLING /////
///////////////////////////////////////////

void MCL::sampling(const Action &u)
{
//    std::cout << "rot1 " << RAD2DEG(u.rot1) << " trans " << u.trans << " rot2 " << RAD2DEG(u.rot2) << std::endl;

    double alpha1 = 0.01;
    double alpha2 = 0.001;
    double alpha3 = 0.01;
    double alpha4 = 0.001;

    std::normal_distribution<double> randomR1(0.0,alpha1*u.rot1 + alpha2*u.trans);
    std::normal_distribution<double> randomT1(0.0,alpha3*u.trans + alpha4*(u.rot1+u.rot2));
    std::normal_distribution<double> randomR2(0.0,alpha1*u.rot2 + alpha2*u.trans);

    for(int i=0; i<numParticles; i++){

        double rot1  = u.rot1  - randomR1(*generator);
        double trans = u.trans - randomT1(*generator);
        double rot2  = u.rot2  - randomR2(*generator);

        // sample particle pose
        particles[i].p.x = particles[i].p.x + trans*cos(particles[i].p.theta+rot1);
        particles[i].p.y = particles[i].p.y + trans*sin(particles[i].p.theta+rot1);
        particles[i].p.theta = particles[i].p.theta + rot1 + rot2;
    }
}

void MCL::weighting(WeightingType type, const std::vector<float> &z, const std::array<float,2> density, const std::vector<Measurement> &z_ranges)
{
    if(type == SIMPLE_WEIGHTS){
        for(int i=0; i<numParticles; i++){
            if(mapCells[(int)(particles[i].p.x*scale)][(int)(particles[i].p.y*scale)] == OCCUPIED || mapCells[(int)(particles[i].p.x*scale)][(int)(particles[i].p.y*scale)] == UNEXPLORED){
                particles[i].w = 0.0;
            }else{
                particles[i].w = 1.0;
            }
        }
    }else if(type == LASER_RAYCAST){
        for(int i=0; i<numParticles; i++){
            if(mapCells[(int)(particles[i].p.x*scale)][(int)(particles[i].p.y*scale)] == OCCUPIED || mapCells[(int)(particles[i].p.x*scale)][(int)(particles[i].p.y*scale)] == UNEXPLORED){
                particles[i].w = 0.0;
            }else{
                double prob = 1.0;
                double logProb = 0.0;
                double hitVariance = 50;

                // Compute likelihood of each laser beam
                for(int l=0; l<z.size(); l=l+10){

                    float z_measured = std::min(z[l],maxRange);

                    // Find z_expected via ray-casting
                    float z_expected = computeExpectedMeasurement(l, particles[i].p);
                    //std::cout << z_measured << ' ' << z_expected << " - ";

                    // Compute likelihood using z_measured and z_expected
                    double p_hit = std::max(measurementLikelihood(z_measured,z_expected,hitVariance),0.0001);
                    prob *= p_hit;
                }
                //std::cout << std::endl;

                particles[i].w = prob;
            }
        }
    }else if(type == CLOUD_RAYCAST){
        for(int i=0; i<numParticles; i++){
            if(mapCells[(int)(particles[i].p.x*scale)][(int)(particles[i].p.y*scale)] == OCCUPIED || mapCells[(int)(particles[i].p.x*scale)][(int)(particles[i].p.y*scale)] == UNEXPLORED){
                particles[i].w = 0.0;
            }else{
                double prob = 1.0;
                double logProb = 0.0;
                double hitVariance = 50;

                // Compute likelihood of each laser beam
                for(int l=0; l<z_ranges.size(); l++){

                    float z_measured = std::min(z_ranges[l].r,maxRange);

                    // Find z_expected via ray-casting
                    float z_expected = computeExpectedMeasurement(-1, particles[i].p, z_ranges[l].phi);
                    //std::cout << z_measured << ' ' << z_expected << " - ";

                    // Compute likelihood using z_measured and z_expected
                    double p_hit = std::max(measurementLikelihood(z_measured,z_expected,hitVariance),0.0001);
                    prob *= p_hit;
                }
                //std::cout << std::endl;

                particles[i].w = prob;
            }
        }
    }else if(type == CLOUD_LIKELIHOOD){

        float varAngle = pow(0.18,2.0);
        float minLikeAngle = 1/800;

        for(int i=0; i<numParticles; i++){
            if(mapCells[(int)(particles[i].p.x*scale)][(int)(particles[i].p.y*scale)] == OCCUPIED || mapCells[(int)(particles[i].p.x*scale)][(int)(particles[i].p.y*scale)] == UNEXPLORED){
                particles[i].w = 0.0;
            }else{
                double prob = 1.0;
                double logProb = 0.0;

                // Compute likelihood of each laser beam
                for(int l=0; l<z_ranges.size(); l++){
                    float angle = particles[i].p.theta + DEG2RAD(z_ranges[l].phi);
                    float z_measured = std::min(z_ranges[l].r,maxRange);


                    float angleWeight = (1.0-minLikeAngle)/(sqrt(2*M_PI*varAngle))*exp(-0.5*(pow(z_ranges[l].cos_pitch-1.0,2)/varAngle))+minLikeAngle;
                    logProb += log(angleWeight);
//                    prob *= angleWeight;

                    int x = (particles[i].p.x + cos(angle)*z_measured)*scale;
                    int y = (particles[i].p.y + sin(angle)*z_measured)*scale;
                    if(x<0 || x>=mapWidth || y<0 || y>=mapHeight){
                        logProb += logMinLikeValue;
//                        prob *= minLikeValue;
                    }else{
//                        prob *= mapLikelihood[x][y];
                        logProb += mapLogLikelihood[x][y];
                    }
                }
//                particles[i].w = prob;
                particles[i].w = exp(logProb);
            }
        }
    }else if(type == ABS_PCL_FSD){
        for(int i=0; i<numParticles; i++){
            if(mapCells[(int)(particles[i].p.x*scale)][(int)(particles[i].p.y*scale)] == OCCUPIED || mapCells[(int)(particles[i].p.x*scale)][(int)(particles[i].p.y*scale)] == UNEXPLORED){
                particles[i].w = 0.0;
            }else{
                float d=mapDensities[(int)(particles[i].p.x*scale)][(int)(particles[i].p.y*scale)];

                float diff = fabs(density[0] - d);

                double prob = 1.0 - std::min(diff,rangeDensity)/rangeDensity;

                particles[i].w = prob;
            }
        }
    }else if(type == INT_PCL_FSD){
        for(int i=0; i<numParticles; i++){
            if(mapCells[(int)(particles[i].p.x*scale)][(int)(particles[i].p.y*scale)] == OCCUPIED || mapCells[(int)(particles[i].p.x*scale)][(int)(particles[i].p.y*scale)] == UNEXPLORED){
                particles[i].w = 0.0;
            }else{
                float d=mapDensities[(int)(particles[i].p.x*scale)][(int)(particles[i].p.y*scale)];
                double prob;

                if(d>=density[0] && d<=density[1]){
                    prob = 1.0;
                }else{
                    float diff;
                    if(d<density[0])
                        diff = fabs(density[0] - d);
                    else
                        diff = fabs(d - density[1]);

                    prob = 1.0 - std::min(diff,rangeDensity)/rangeDensity;
                }

                particles[i].w = prob;
            }
        }
    }

    // compute sum of all particles weights
    double sumWeights = 0.0;
    for(int i=0; i<numParticles; i++)
        sumWeights += particles[i].w;
//    std::cout << "SUM "<< sumWeights << std::endl;

    // normalize particles weights
    if(sumWeights!=0){
        for(int i=0; i<numParticles; i++)
            particles[i].w /= sumWeights;
    }else{
        std::cout << "SUM ZERO" << std::endl;
        double w = 1.0/(double)numParticles;
        for(int i=0; i<numParticles; i++)
            particles[i].w = w;
    }
}

void MCL::resampling()
{
    // low variance resampling

    std::vector<int> children;
    children.resize(numParticles,0);

    std::vector<MCLparticle> nextGeneration;
    nextGeneration.reserve(numParticles);

    std::uniform_real_distribution<double> randomValue(0.0,1.0/((double)numParticles));

    int i=0;
    double r=randomValue(*generator);
    double c=particles[i].w;

    for(int j=0; j<numParticles; j++){
        double U = r + j*1.0/(double)numParticles;
        while(U>c){
            i++;
            c+=particles[i].w;
        }
        nextGeneration.push_back(particles[i]);
    }

    // update particles set
    particles = nextGeneration;
}


///////////////////////
///// FSD METHODS /////
///////////////////////

float MCL::computeDensity(int x, int y)
{
    float total = 0;

    for(float a=0.0; a<2.0*M_PI; a=a+angleStep){
        float dx = cos(a);
        float dy = sin(a);

        float dmax = std::max(fabs(dx),fabs(dy));
        dx /= dmax;
        dy /= dmax;


        float X=x;
        float Y=y;
        for(float d=0; d<=dmax*kernelWidth; d+=1.0){

            int u = X;
            int v = Y;

            if(mapCells[u][v] == FREE){
                mapCells[u][v] = KERNEL;
                total += 1;
            }else if(mapCells[u][v] == UNEXPLORED)
                mapCells[u][v] = KERNEL_UNK;
            else if(mapCells[u][v] == OCCUPIED)
                break;

            X += dx;
            Y += dy;
        }
    }

    return total/kernelCells;
}

std::array<float,2> MCL::computeDensityInGrid(Pose pr, Grid* grid)
{
    int x = pr.x * scale;
    int y = pr.y * scale;


    for(int u=-2*kernelWidth; u<=2*kernelWidth; u++){
        for(int v=-2*kernelWidth; v<=2*kernelWidth; v++){
            int X=x+u;
            int Y=y+v;
            if(X>=0 && X<grid->getMapWidth() && Y>=0 && Y<grid->getMapHeight()){
                Cell* c = grid->getCell(X,Y);
                if(c->occType == KERNEL)
                    c->occType = FREE;
                else if(c->occType == KERNEL_UNK)
                    c->occType = UNEXPLORED;
            }
        }
    }

    float total = 0, unk_total=0;

    for(float a=0.0; a<2.0*M_PI; a=a+angleStep){
        float dx = cos(a);
        float dy = sin(a);

        float dmax = std::max(fabs(dx),fabs(dy));
        dx /= dmax;
        dy /= dmax;


        float X=x;
        float Y=y;
        for(float d=0; d<=dmax*kernelWidth; d+=1.0){

            int u = X;
            int v = Y;

            Cell* c = grid->getCell(u,v);

            if(c->occType == FREE){
                c->occType = KERNEL;
                total += 1;
            }else if(c->occType == UNEXPLORED){
                c->occType = KERNEL_UNK;
                unk_total += 1;
            }else if(c->occType == OCCUPIED)
                break;

            X += dx;
            Y += dy;
        }
    }

    std::array<float,2> p;
    p[0]  = total/kernelCells;
    p[1] = (total+unk_total)/kernelCells;

    return p;
}

float MCL::computeMaxDensityCellsValue()
{
    CellOccType mapAux[2*kernelWidth+1][2*kernelWidth+1];

    int x,y;
    x = y = kernelWidth;

    for(int u=x-kernelWidth; u<=x+kernelWidth; u++){
        for(int v=y-kernelWidth; v<=y+kernelWidth; v++){
            mapAux[u][v] = FREE;
        }
    }

    float total = 0;

    for(float a=0.0; a<2.0*M_PI; a=a+angleStep){
        float dx = cos(a);
        float dy = sin(a);

        float dmax = std::max(fabs(dx),fabs(dy));
        dx /= dmax;
        dy /= dmax;


        float X=x;
        float Y=y;
        for(float d=0; d<=dmax*kernelWidth; d+=1.0){

            int u = X;
            int v = Y;

            if(mapAux[u][v] == FREE){
                mapAux[u][v] = KERNEL;
                total += 1;
            }else if(mapAux[u][v] == UNEXPLORED)
                mapAux[u][v] = KERNEL_UNK;
            else if(mapAux[u][v] == OCCUPIED)
                break;

            X += dx;
            Y += dy;
        }
    }

    return total;
 }

void MCL::cleanCellsAroundPose(int x, int y)
{
    for(int u=-2*kernelWidth; u<=2*kernelWidth; u++){
        for(int v=-2*kernelWidth; v<=2*kernelWidth; v++){
            int X=x+u;
            int Y=y+v;
            if(X>=0 && X<mapWidth && Y>=0 && Y<mapHeight){
                if(mapCells[X][Y] == KERNEL || mapCells[X][Y] == KERNEL_UNK)
                    mapCells[X][Y] = FREE;
            }
        }
    }


}

void MCL::computeMapOfDensities()
{
    std::cout << "Computing map of densities" << std::endl;

    mapDensities = new float*[mapWidth];
    for(int i=0;i<mapWidth;i++){
        mapDensities[i] = new float[mapHeight];
        for(int j=0; j<mapHeight; j++)
            mapDensities[i][j] = -1.0;
    }

    minDensity = 1.0;
    maxDensity = 0.0;

    for(int i=0;i<mapWidth;i++){
        for(int j=0; j<mapHeight; j++){
            if(mapCells[i][j] == FREE){
                mapDensities[i][j] = computeDensity(i,j);
                cleanCellsAroundPose(i,j);

                if(mapDensities[i][j] > maxDensity)
                    maxDensity = mapDensities[i][j];
                if(mapDensities[i][j] < minDensity)
                    minDensity = mapDensities[i][j];
            }
        }
    }

    rangeDensity = maxDensity - minDensity;

    std::cout << "Done!" << std::endl;
}

void MCL::saveMapOfDensities(std::string mapName)
{
    std::string name = mapName+"/densities_W"+std::to_string(kernelWidth)+"_A"+std::to_string(RAD2DEG(angleStep))+".txt";
    std::ofstream file;
    file.open(name.c_str(), std::ofstream::out);

//    if( !file.good() )
//    {
//        std::cerr << "The file \"" << name << "\"  does not exist!" << std::endl;
//        return;
//    }

    // Write dimensions.
    file << mapWidth << " " << mapHeight << std::endl;
    std::cout << "WRITING map.width " << mapWidth << " map.height " << mapHeight << std::endl;

    // Write min, max, range of densities
    file << minDensity << " " << maxDensity << " " << rangeDensity << std::endl;
    std::cout << "WRITING minDensity " << minDensity << " maxDensity " << maxDensity << " rangeDensity " << rangeDensity << std::endl;

    // Write grid on file.
    for(int y=0; y < mapHeight; y++)
    {
        for(int x=0; x < mapWidth; x++)
        {
            file << mapDensities[x][y] << " ";
        }
        file << std::endl;
    }

    file.close();
}

bool MCL::readMapOfDensities(std::string mapName)
{
    std::string name = mapName+"/densities_W"+std::to_string(kernelWidth)+"_A"+std::to_string(RAD2DEG(angleStep))+".txt";
    std::ifstream file;
    file.open(name.c_str(), std::ifstream::in);

    if( !file.good() )
    {
        std::cerr << "The file \"" << name << "\"  does not exist!" << std::endl;
        return false;
    }

    // Read dimensions.
    int W, H;
    file >> W >> H;
    std::cout << "READING map.width " << W << " map.height " << H << std::endl;

    // Write min, max, range of densities
    file >> minDensity >> maxDensity >> rangeDensity;
    std::cout << "READING minDensity " << minDensity << " maxDensity " << maxDensity << " rangeDensity " << rangeDensity << std::endl;


    mapDensities = new float*[mapWidth];
    for(int i=0;i<mapWidth;i++){
        mapDensities[i] = new float[mapHeight];
    }

    // Read grid from file.
    for(int y=0; y < mapHeight; y++)
    {
        for(int x=0; x < mapWidth; x++)
        {
            file >> mapDensities[x][y];
        }
    }

    file.close();

}

void MCL::computeLikelihoodMap()
{
    std::cout << "Computing likelihood map" << std::endl;

    mapLikelihood = new float*[mapWidth];
    mapLogLikelihood = new float*[mapWidth];
    for(int i=0;i<mapWidth;i++){
        mapLikelihood[i] = new float[mapHeight];
        mapLogLikelihood[i] = new float[mapHeight];
        for(int j=0; j<mapHeight; j++)
            mapLikelihood[i][j] = -1.0;
    }

    vector<vector<double> > distMap;
    distMap.resize(mapWidth);
    for(int i=0;i<mapWidth;i++){
        distMap[i].resize(mapHeight);
        for(int j=0;j<mapHeight; j++)
        {
            if(mapCells[i][j]==OCCUPIED)
                distMap[i][j]=0;
            else
                distMap[i][j]=INF;
        }

    }

    // Compute distance for nearest obstacle for all cells
    dt.computeSquareEuclideanDT(distMap);

    // Find maximum value
    double maxDist=-1.0;
    for(int i=0; i<mapWidth; ++i){
        for(int j=0; j<mapHeight; ++j){
            if(distMap[i][j] > maxDist)
                maxDist = distMap[i][j];
        }
    }

    // Normalize distances using the maximum value
    mapDistances = new float*[mapWidth];
    for(int i=0;i<mapWidth;i++){
        mapDistances[i] = new float[mapHeight];
        for(int j=0; j<mapHeight; ++j){
            mapDistances[i][j] = (maxDist-distMap[i][j])/maxDist;
        }
    }

    // Attributes of likelihood map
    double gaussStdev = 0.52;
    gaussStdev *= scale;
    double gaussVar = pow(gaussStdev,2.0);
    double gaussNu = 1.0/sqrt(2.0*M_PI*gaussVar);
    minLikeValue = 0.4;//1/80;
    logMinLikeValue = log(minLikeValue);

    // Compute likelihood of all cells
    for(int i=0; i<mapWidth; ++i){
        for(int j=0; j<mapHeight; ++j){
            // prob = 1/sqrt(2*M_PI*sigma^2) * e^(-1/2 * dist^2/sigma^2)
            // prob = nu * e^(-0.5 * sqrdist/var)
            mapLikelihood[i][j] = (1.0-minLikeValue)*gaussNu*exp(-0.5*distMap[i][j]/gaussVar)+minLikeValue;
            mapLogLikelihood[i][j] = log(mapLikelihood[i][j]);
        }
    }

    // Find maximum value
    double maxLike=-1.0;
    for(int i=0; i<mapWidth; ++i){
        for(int j=0; j<mapHeight; ++j){
            if(mapLikelihood[i][j] > maxLike)
                maxLike = mapLikelihood[i][j];
        }
    }

    // Normalize distances using the maximum value
    for(int i=0; i<mapWidth; ++i){
        for(int j=0; j<mapHeight; ++j){
            mapLikelihood[i][j] = (mapLikelihood[i][j])/maxLike;
        }
    }

    std::cout << "Done!" << std::endl;
}

/////////////////////////////
///// AUXILIARY METHODS /////
/////////////////////////////

void MCL::initLogFile(std::string mapName)
{
    int status = mkdir(mapName.c_str(),0777);
    std::cout << "mkdir " << mapName << ' ' << status << std::endl;

    std::string name = mapName+"/Results/";
    status = mkdir(name.c_str(),0777);
    std::cout << "mkdir " << name << ' ' << status << std::endl;

    if(wt == SIMPLE_WEIGHTS)
        name = name+"Motion/";
    else if(wt == LASER_RAYCAST)
        name = name+"LRaycast/";
    else if(wt == CLOUD_RAYCAST)
        name = name+"CRaycast/";
    else if(wt == CLOUD_LIKELIHOOD)
        name = name+"CLikelihood/";
    else if(wt == ABS_PCL_FSD)
        name = name+"ABS_PCL_FSD/";
    else if(wt == INT_PCL_FSD)
        name = name+"INT_PCL_FSD/";
    status = mkdir(name.c_str(),0777);
    std::cout << "mkdir " << name << ' ' << status << std::endl;        

    time_t t = time(0);
    struct tm *now = localtime(&t);
    std::stringstream ss;
    ss << name << "teste-P" << numParticles;
    if(wt == CLOUD_LIKELIHOOD || wt == CLOUD_RAYCAST)
        ss << "-N" << numRanges;
    else if(wt == ABS_PCL_FSD)
        ss << "-K" << kernelWidth;

    if(numCams!=4)    
        ss << "-singleCam";

    ss << "-" << -100+now->tm_year
       << std::setfill('0') << std::setw(2) << 1+now->tm_mon
       << std::setfill('0') << std::setw(2) << now->tm_mday << '-'
       << std::setfill('0') << std::setw(2) << now->tm_hour
       << std::setfill('0') << std::setw(2) << now->tm_min
       << std::setfill('0') << std::setw(2) << now->tm_sec << "/";
    ss >> pathName;
    std::cout << "NAME " << pathName << std::endl;

    status = mkdir(pathName.c_str(),0777);
    std::cout << "mkdir " << pathName << ' ' << status << std::endl;

    logFile.open((pathName+"log.txt").c_str(), std::ofstream::out);
    logFile << "# timestamp errorMeanParticlePose meanParticleError stdevParticleError stdevParticlePose covMajorAxis covMinorAxis " << std::endl;
    logFile << "# meanParticlePoseAngle meanDiffAngle stdevDiffAngle stdevParticlePoseAngle less100 less75 less50 less25 weightingTime MCLtime GlobalTime" << std::endl;
}

void MCL::computeError()
{
    // PRINT timestamp
    logFile << timeStamp << " ";

    // PRINT errorMeanParticlePose
    double errorMeanParticlePose = sqrt(pow(truePose.x-meanParticle[0],2.0)+pow(truePose.y-meanParticle[1],2.0));
    logFile << errorMeanParticlePose << " ";

    double less25 = 0, less50 = 0, less75 = 0, less100 = 0;
    double meanParticleError = 0;
    double sumSqrErr = 0;
    double norm = 0;
    for(unsigned int i=0; i<numParticles; i++){
        double err = sqrt(pow(truePose.x-particles[i].p.x,2.0)+pow(truePose.y-particles[i].p.y,2.0));

        if(err<=0.25){
            less25  += 1.0;
            less50  += 1.0;
            less75  += 1.0;
            less100 += 1.0;
        }else if(err<=0.50){
            less50  += 1.0;
            less75  += 1.0;
            less100 += 1.0;
        }else if(err<=0.75){
            less75  += 1.0;
            less100 += 1.0;
        }else if(err<=1.0){
            less100 += 1.0;
        }

        meanParticleError += err*particles[i].w;
        sumSqrErr += err*err*particles[i].w;
        norm += particles[i].w;

    }

    if(norm == 0.0)
        return;

    // PRINT errorMeanParticlePose
    meanParticleError /= norm;
    logFile << meanParticleError << " ";

    // PRINT stdevParticleError
    double stdevParticleError = sqrt(sumSqrErr/norm - meanParticleError*meanParticleError);
    logFile << stdevParticleError << " ";

    // PRINT stdevParticlePose
    double sumSqrDist = 0;
    for(unsigned int i=0; i<numParticles; i++){
        double dist = sqrt(pow(meanParticle[0]-particles[i].p.x,2.0)+pow(meanParticle[1]-particles[i].p.y,2.0));
        sumSqrDist += dist*dist*particles[i].w;
    }
    double stdevParticlePose = sqrt(sumSqrDist/norm);
    logFile << stdevParticlePose << " ";

    // PRINT covMajorAxis & covMinorAxis
    logFile << covMajorAxis << " " << covMinorAxis << " ";

    // PRINT orientation errors
    double meanDiffAngle = 0;
    double sumSqrDiffAngle = 0;
    double sumSqrDiffMeanAngle = 0;
    for(unsigned int i=0; i<numParticles; i++){
        double oDiff = particles[i].p.theta-truePose.theta;
        oDiff = fabs(normalizeAngleRAD(oDiff));

        meanDiffAngle += oDiff*particles[i].w;
        sumSqrDiffAngle += oDiff*oDiff*particles[i].w;

        double oDiffMean = particles[i].p.theta-meanParticle[2];
        oDiffMean = fabs(normalizeAngleRAD(oDiffMean));
        sumSqrDiffMeanAngle += oDiffMean*oDiffMean*particles[i].w;
    }

    double meanParticlePoseAngle = meanParticle[2] - truePose.theta;
    meanParticlePoseAngle = fabs(normalizeAngleRAD(meanParticlePoseAngle));
    double stdevParticlePoseAngle = sqrt(sumSqrDiffMeanAngle/norm);
    meanDiffAngle = meanDiffAngle/norm;
    double stdevDiffAngle = sqrt(sumSqrDiffAngle/norm - meanDiffAngle*meanDiffAngle);
    logFile << meanParticlePoseAngle << " " << meanDiffAngle << " " << stdevDiffAngle << " " << stdevParticlePoseAngle << " ";

    // PRINT less100, less75, less50, less25s
    logFile << less100/numParticles << " " << less75/numParticles << " " << less50/numParticles << " " << less25/numParticles << " ";

    // PRINT times
    float time1 = t1.getTotalTime();
    float time3 = t3.getTotalTime();
    float time2 = t2.getLapTime();
    t2.startLap();
    logFile << time3 << " " << time1 << " " << time2 << " ";

    logFile << totalDistance << " ";

    logFile << std::endl;

}

double MCL::measurementLikelihood(double value, double mean, double var)
{
    return 1.0/(sqrt(2*M_PI*var))*exp(-0.5*(pow(value-mean,2)/var));
}

float MCL::computeExpectedMeasurement(int index, Pose &pose, double phi)
{
    double angle;

    if(index==-1){
        angle = pose.theta + DEG2RAD(phi);
    }else{
        angle = pose.theta + double(90-index)*M_PI/180.0;
    }

    // Ray-casting using DDA
    double dist;
    double difX=cos(angle);
    double difY=sin(angle);
    double deltaX, deltaY;

    if(tan(angle)==1 || tan(angle)==-1){
        deltaX=deltaY=1.0;
        dist = difX*maxRange;
    }else if(difX*difX > difY*difY){
        deltaX=1.0;
        deltaY=difY/difX;
        dist = difX*maxRange;
    }else{
        deltaX=difX/difY;
        deltaY=1.0;
        dist = difY*maxRange;
    }
    if(deltaX*difX < 0.0)
        deltaX = -deltaX;
    if(deltaY*difY < 0.0)
        deltaY = -deltaY;
    if(dist < 0.0)
        dist = -dist;

    dist *= scale;

    double i=pose.x*scale;
    double j=pose.y*scale;
    for(int k=0;k<(int)(dist);k++){

        if(mapCells[(int)i][(int)j] == OCCUPIED){
            // the real obstacle is one step ahead due to wall thickening
            return sqrt(pow(pose.x*scale-(i+deltaX),2)+pow(pose.y*scale-(j+deltaY),2))/scale;
        }

        i+=deltaX;
        j+=deltaY;
    }

    return maxRange;
}

void MCL::updateMeanAndCovariance()
{
    // Compute Mean
    float sx=0, cx=0;
    double norm=0.0;
    meanParticle[0] = meanParticle[1] = 0.0;
    for(unsigned int i=0; i<numParticles; i++){
        meanParticle[0] += particles[i].p.x*particles[i].w;
        meanParticle[1] += particles[i].p.y*particles[i].w;
        sx += sin(particles[i].p.theta)*particles[i].w;
        cx += cos(particles[i].p.theta)*particles[i].w;
        norm += particles[i].w;
    }
    meanParticle[0] /= norm;
    meanParticle[1] /= norm;
    meanParticle[2] = atan2(sx/norm,cx/norm);

    // Compute Covariance Matrix (considering only x and y)
    float covariance[2][2];
    for(unsigned int l=0; l<2; l++)
        for(unsigned int c=0; c<2; c++)
            covariance[l][c] = 0;

    float diffx, diffy;
    for(unsigned int i=0; i<numParticles; i++){
        diffx  = (meanParticle[0]-particles[i].p.x);
        diffy  = (meanParticle[1]-particles[i].p.y);

        covariance[0][0] += diffx*diffx*particles[i].w;    covariance[0][1] += diffx*diffy*particles[i].w;
        covariance[1][0] += diffy*diffx*particles[i].w;    covariance[1][1] += diffy*diffy*particles[i].w;
    }

    for(unsigned int l=0; l<2; l++)
        for(unsigned int c=0; c<2; c++)
            covariance[l][c] /= norm;

    // Compute EigenValues and EigenVectors of covariance matrix
    float T = covariance[0][0] + covariance[1][1]; // Trace
    float D = covariance[0][0]*covariance[1][1] - covariance[0][1]*covariance[1][0]; // Determinant

    if((pow(T,2.0)/4.0 - D)<0.0)
        return;

//    std::cout << "Covariance [" << covariance[0][0] << " " << covariance[0][1]
//                        << "; " << covariance[1][0] << " " << covariance[1][1] << std::endl;

    float lambda1 = T/2.0 + sqrt(pow(T,2.0)/4.0 - D);
    float lambda2 = T/2.0 - sqrt(pow(T,2.0)/4.0 - D);
    float eigvec1[2], eigvec2[2];

    if(covariance[1][0]!=0.0){
        eigvec1[0] = lambda1 - covariance[1][1];    eigvec2[0] = lambda2 - covariance[1][1];
        eigvec1[1] = covariance[1][0];              eigvec2[1] = covariance[1][0];
    }else if(covariance[0][1]!=0.0){
        eigvec1[0] = covariance[0][1];              eigvec2[0] = covariance[0][1];
        eigvec1[1] = lambda1 - covariance[0][0];    eigvec2[1] = lambda2 - covariance[0][0];
    }else if(covariance[1][0]==0.0 && covariance[0][1]==0.0){
        eigvec1[0] = 1;    eigvec2[0] = 0;
        eigvec1[1] = 0;    eigvec2[1] = 1;
    }

//    std::cout << "lambda " << lambda1 << " and " << lambda2 << std::endl;
//    std::cout << "eigvectors [" << eigvec1[0] << "; " << eigvec1[1]
//              << "] and [" << eigvec1[0] << "; " << eigvec1[1] << "]" << std::endl;

    // Compute direction of covariance ellipse
    //1st - Calculate the angle between the largest eigenvector and the x-axis
    covAngle = RAD2DEG(atan2(eigvec1[1], eigvec1[0]));

    //2nd - Calculate the size of the minor and major axes
    covMajorAxis = sqrt(lambda1);
    covMinorAxis = sqrt(lambda2);
//    std::cout << "covAngle " << covAngle << " covMajorAxis " << covMajorAxis << " covMinorAxis " << covMinorAxis << std::endl;
}


bool MCL::hasConverged()
{
    if(covMajorAxis>1)
        return false;
    else
        return true;
}

Pose MCL::getMeanParticlePose()
{
    meanParticlePose.x = meanParticle[0];
    meanParticlePose.y = meanParticle[1];
    meanParticlePose.theta = RAD2DEG(meanParticle[2]);

    return meanParticlePose;
}

void MCL::copyMapToGrid(Grid* grid)
{

    for(int y=0; y < mapHeight; y++){
        for(int x=0; x < mapWidth; x++){
            Cell* c = grid->getCell(x,y);
            c->occType = mapCells[x][y];
        }
    }
}


///////////////////////////
///// DRAWING METHODS /////
///////////////////////////

void MCL::draw()
{
    // Draw map
    for(int x=0;x<mapWidth;x++){
        for(int y=0;y<mapHeight;y++){

            if(viewMode == 3){
                if(mapDistances[x][y]<0)
                    glColor3f(1.0,1.0,1.0);
                else
                    setGLColorFromGrayscale(mapDistances[x][y]);
            }else if(viewMode == 4){
                if(mapLikelihood[x][y]<0)
                    glColor3f(1.0,1.0,1.0);
                else
                    setGLColorFromGrayscale(mapLikelihood[x][y]);
            }

            else if(mapCells[x][y] == OCCUPIED)
                glColor3f(0.0,0.0,0.0);
            else if (mapCells[x][y] == UNEXPLORED)
                glColor3f(0.5,0.5,0.5);
            else{
                if(viewMode == 1){
                     if (mapCells[x][y] == KERNEL)
                        setGLColorFromGrayscale(1.0-currentFSD);
                     else if (mapCells[x][y] == KERNEL_UNK)
                        glColor3f(0.7,0.7,0.7);
                     else
                        glColor3f(1.0,1.0,1.0);
                }
                else if(viewMode == 2){
                    if(mapDensities[x][y]<0)
                        glColor3f(1.0,1.0,1.0);
                    else
                        setGLColorFromGrayscale(mapDensities[x][y]);
                }
                else{
    //                if(x%20==0 || y%20==0)
    //                    glColor3f(0.6,0.6,1.0);
    //                else
                        glColor3f(1.0,1.0,1.0);
                }
            }

            glBegin( GL_QUADS );
            {
                glVertex2f(x  ,y  );
                glVertex2f(x+1,y  );
                glVertex2f(x+1,y+1);
                glVertex2f(x  ,y+1);
            }
            glEnd();
        }
    }

    // Draw colorbar
    if(mapWidth > 0.6*mapHeight){

        int x1, x2, y1, y2;

        x1 = 0.2*mapWidth;
        x2 = 0.8*mapWidth;
        y1 = 0.05*mapWidth;
        y2 = 0.025*mapWidth;

        float numSteps = 40;
        float step = (x2-x1)/numSteps;
        for(float i=0; i<numSteps; i+=1.0){

           setGLColorFromGrayscale(i/numSteps);

           glBegin( GL_QUADS );
           {
                glVertex2f(x1+i*step, y1);
                glVertex2f(x1+(i+1)*step, y1);
                glVertex2f(x1+(i+1)*step,y2);
                glVertex2f(x1+i*step, y2);
           }
           glEnd();
        }


        glColor3f(0.0,0.0,0.0);
        glLineWidth(2);
        glBegin( GL_LINE_STRIP );
        {
            glVertex2f(x1, y1);
            glVertex2f(x2, y1);
            glVertex2f(x2, y2);
            glVertex2f(x1, y2);
            glVertex2f(x1, y1);
        }
        glEnd();

        // minDensity e maxDensity
        glLineWidth(4);
        glBegin( GL_LINES );
        {
            glVertex2f(x1 + (minDensity)*(x2-x1), y1+5);
            glVertex2f(x1 + (minDensity)*(x2-x1), y2-5);
        }
        glEnd();
        glBegin( GL_LINES );
        {
            glVertex2f(x1 + (maxDensity)*(x2-x1), y1+5);
            glVertex2f(x1 + (maxDensity)*(x2-x1), y2-5);
        }
        glEnd();

        // currentFSD e measuredFSD
        glColor3f(0.0,0.6,0.0);
        glBegin( GL_LINES );
        {
            glVertex2f(x1 + (currentFSD)*(x2-x1), y1+8);
            glVertex2f(x1 + (currentFSD)*(x2-x1), y2-8);
        }
        glEnd();
        glColor3f(0.6,0.0,0.0);
        if(wt != INT_PCL_FSD){
            glBegin( GL_LINES );
            {
                glVertex2f(x1 + (measuredFSD)*(x2-x1), y1+8);
                glVertex2f(x1 + (measuredFSD)*(x2-x1), y2-8);
            }
            glEnd();
        }else{
            glBegin( GL_LINES );
            {
                glVertex2f(x1 + (measuredFSD)*(x2-x1), y1+8);
                glVertex2f(x1 + (measuredFSD)*(x2-x1), y2-8);

                glVertex2f(x1 + (measuredFSDUNK)*(x2-x1), y1+8);
                glVertex2f(x1 + (measuredFSDUNK)*(x2-x1), y2-8);

                glVertex2f(x1 + (measuredFSD)*(x2-x1), y1+8);
                glVertex2f(x1 + (measuredFSDUNK)*(x2-x1), y1+8);

                glVertex2f(x1 + (measuredFSD)*(x2-x1), y2-8);
                glVertex2f(x1 + (measuredFSDUNK)*(x2-x1), y2-8);
            }
            glEnd();
        }
    }


    double dirScale=5;
    glPointSize(4);
    glLineWidth(2);

    float alpha;
    if(transparency)
        alpha = 100.0/numParticles;
    else
        alpha = 1.0;

    // Draw particles
    for(int p=0;p<particles.size();p++){

        double x=particles[p].p.x*scale;
        double y=particles[p].p.y*scale;
        double th=particles[p].p.theta;

        // Draw point
        glColor4f(1.0,0.0,0.0,alpha);
        glBegin( GL_POINTS );
        {
            glVertex2f(x, y);
        }
        glEnd();

        // Draw direction
        glColor4f(0.0, 0.0, 1.0, alpha);
        glBegin( GL_LINES );
        {
            glVertex2f(x, y);
            glVertex2f(x+dirScale*cos(th), y+dirScale*sin(th));
        }
        glEnd();
    }
    glLineWidth(1);

//    // Draw goal pose
//    double xGoal = goal.x*scale;
//    double yGoal = goal.y*scale;
//    glTranslatef(xGoal,yGoal,0.0);
//    glScalef(1.0/2.0,1.0/2.0,1.0/2.0);
//    glColor3f(1.0,0.0,0.0);
//    glBegin( GL_POLYGON );
//    {
//        glVertex2f(-12, -8);
//        glVertex2f(  8, 12);
//        glVertex2f( 12,  8);
//        glVertex2f( -8,-12);
//    }
//    glEnd();
//    glBegin( GL_POLYGON );
//    {
//        glVertex2f(-12,  8);
//        glVertex2f( -8, 12);
//        glVertex2f( 12, -8);
//        glVertex2f(  8,-12);
//    }
//    glEnd();
//    glScalef(2,2,2);
//    glTranslatef(-xGoal,-yGoal,0.0);

    // Draw mean
    double xRobot = meanParticle[0]*scale;
    double yRobot = meanParticle[1]*scale;
    double angRobot = RAD2DEG(meanParticle[2]);

    if(viewMode==0 && covMajorAxis>-1){

        glTranslatef(xRobot,yRobot,0.0);
        glRotatef(angRobot,0.0,0.0,1.0);
        glScalef(1.0/5.0,1.0/5.0,1.0/5.0);
        glColor3f(0.7,0.5,0.0);
        glBegin( GL_POLYGON );
        {
            glVertex2f(-20, -8);
            glVertex2f(-13, -15);
            glVertex2f(8, -15);
            glVertex2f(15, -8);
            glVertex2f(15, 8);
            glVertex2f(8, 15);
            glVertex2f(-13, 15);
            glVertex2f(-20, 8);
        }
        glEnd();
        glColor3f(0.0,0.0,0.0);
        glBegin( GL_LINE_STRIP );
        {
            glVertex2f(0, 0);
            glVertex2f(30, 0);
        }
        glEnd();
        glScalef(5,5,5);
        glRotatef(-angRobot,0.0,0.0,1.0);
        glTranslatef(-xRobot,-yRobot,0.0);

        // Draw Covariance Ellipse
        glColor3f(0.5,0.4,0.0);
        glLineWidth(2);
        double chisquare_val = 2.4477; // 95% confidence interval
        glTranslatef(xRobot,yRobot,0.0);
        MCL::Ellipse(chisquare_val*covMajorAxis*scale, chisquare_val*covMinorAxis*scale,covAngle);
        glTranslatef(-xRobot,-yRobot,0.0);
        glLineWidth(1);

    }

 /*   // Draw odom pose
    xRobot = odomPose.x*scale;
    yRobot = odomPose.y*scale;
    angRobot = RAD2DEG(odomPose.theta);

    glTranslatef(xRobot,yRobot,0.0);
    glRotatef(angRobot,0.0,0.0,1.0);
    glScalef(1.0/5.0,1.0/5.0,1.0/5.0);

    glColor3f(0.8,0.0,0.0);
    glBegin( GL_POLYGON );
    {
        glVertex2f(-20, -8);
        glVertex2f(-13, -15);
        glVertex2f(8, -15);
        glVertex2f(15, -8);
        glVertex2f(15, 8);
        glVertex2f(8, 15);
        glVertex2f(-13, 15);
        glVertex2f(-20, 8);
    }
    glEnd();
    glColor3f(0.0,0.0,0.0);
    glBegin( GL_LINE_STRIP );
    {
        glVertex2f(0, 0);
        glVertex2f(30, 0);
    }
    glEnd();

    glScalef(5,5,5);
    glRotatef(-angRobot,0.0,0.0,1.0);
    glTranslatef(-xRobot,-yRobot,0.0);

    if(odomPath.size() > 1){
        glScalef(scale,scale,scale);
        glColor3f(0.6,0.0,0.0);
        glLineWidth(3);
        glBegin( GL_LINE_STRIP );
        {
            for(unsigned int i=0;i<odomPath.size()-1; i++){
                glVertex2f(odomPath[i].x, odomPath[i].y);
                glVertex2f(odomPath[i+1].x, odomPath[i+1].y);
            }
        }
        glEnd();
        glLineWidth(1);
        glScalef(1.0/scale,1.0/scale,1.0/scale);

    }
*/
    // Draw SLAM pose
    xRobot = truePose.x*scale;
    yRobot = truePose.y*scale;
    angRobot = RAD2DEG(truePose.theta);

    glTranslatef(xRobot,yRobot,0.0);
    glRotatef(angRobot,0.0,0.0,1.0);
    glScalef(1.0/5.0,1.0/5.0,1.0/5.0);

    glColor3f(0.0,1.0,0.0);
    glBegin( GL_POLYGON );
    {
        glVertex2f(-20, -8);
        glVertex2f(-13, -15);
        glVertex2f(8, -15);
        glVertex2f(15, -8);
        glVertex2f(15, 8);
        glVertex2f(8, 15);
        glVertex2f(-13, 15);
        glVertex2f(-20, 8);
    }
    glEnd();
    glColor3f(0.0,0.0,0.0);
    glBegin( GL_LINE_STRIP );
    {
        glVertex2f(0, 0);
        glVertex2f(30, 0);
    }
    glEnd();

    glScalef(5,5,5);
    glRotatef(-angRobot,0.0,0.0,1.0);
    glTranslatef(-xRobot,-yRobot,0.0);

    if(truePath.size() > 1){
        glScalef(scale,scale,scale);
        glColor3f(0.0,0.6,0.0);
        glLineWidth(3);
        glBegin( GL_LINE_STRIP );
        {
            for(unsigned int i=0;i<truePath.size()-1; i++){
                glVertex2f(truePath[i].x, truePath[i].y);
                glVertex2f(truePath[i+1].x, truePath[i+1].y);
            }
        }
        glEnd();
        glLineWidth(1);
        glScalef(1.0/scale,1.0/scale,1.0/scale);

    }

}

void MCL::Ellipse(float rx, float ry, float angle, int num_segments)
{
    float theta = 2 * M_PI / float(num_segments);
    float c = cos(theta);//precalculate the sine and cosine
    float s = sin(theta);
    float t;

    float x = 1;//we start at angle = 0
    float y = 0;

    glRotatef(angle,0,0,1);
    glBegin(GL_LINE_LOOP);
    for(int ii = 0; ii < num_segments; ii++)
    {
        glVertex2f(x*rx, y*ry);//output vertex

        //apply the rotation matrix
        t = x;
        x = c * x - s * y;
        y = s * t + c * y;
    }
    glEnd();
    glRotatef(-angle,0,0,1);
}

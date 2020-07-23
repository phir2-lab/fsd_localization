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

#ifndef MCL_H
#define MCL_H

#include <string>
#include <vector>
#include <array>
#include <random>

#include "Grid.h"
#include "Utils.h"
#include "DistanceTransform.h"

enum WeightingType {SIMPLE_WEIGHTS, LASER_RAYCAST, CLOUD_RAYCAST, CLOUD_LIKELIHOOD, ABS_PCL_FSD, INT_PCL_FSD};

typedef struct{
    double rot1;
    double trans;
    double rot2;
} Action;

typedef struct {
    float r, phi, pitch, cos_pitch;
} Measurement;

typedef struct{
    Pose p;
    double w;
} MCLparticle;

class MCL
{
public:
    MCL(Pose initialPose, float maxRange, int numCams, std::string mapName, string methodName, pthread_mutex_t* m);
    ~MCL();

    void run(double ts, const Pose& tp, const Action &u, const std::vector<float> &z_laser, const std::array<float,2> &z_density, const std::vector<Measurement> &z_ranges);

    void draw();

    bool hasConverged();
    Pose getMeanParticlePose();
    void copyMapToGrid(Grid* grid);

    std::array<float,2> computeDensityInGrid(Pose p, Grid* grid);

    std::string pathName;
    int mapWidth;
    int mapHeight;
    int numRanges;
    int numCams;
    double totalDistance;

    bool transparency;

    Pose goal;

    int viewMode;
    int numViewModes;

    std::default_random_engine* generator;

private:
    void readMap(std::string mapName);
    void initParticles();
    void initLogFile(std::string mapName);
    void computeError();

    void sampling(const Action &u);
    void weighting(WeightingType type, const std::vector<float> &z, const std::array<float,2> density, const std::vector<Measurement> &z_ranges);
    void resampling();

    float computeDensity(int x, int y);
    float computeMaxDensityCellsValue();
    void cleanCellsAroundPose(int x, int y);
    void computeMapOfDensities();
    bool readMapOfDensities(std::string mapName);
    void saveMapOfDensities(std::string mapName);

    void computeLikelihoodMap();


    float computeExpectedMeasurement(int index, Pose &pose, double phi=0.0);
    double measurementLikelihood(double value, double mean, double var);

    void updateMeanAndCovariance();

    std::ofstream logFile;

    CellOccType** mapCells;
    float** mapDensities;
    float** mapLikelihood;
    float** mapLogLikelihood;
    float** mapDistances;
    float minDensity, maxDensity, rangeDensity;
    double scale;
    float maxRange;

    double minLikeValue, logMinLikeValue;

    DistanceTransform dt;

    WeightingType wt;

    int kernelWidth;
    float angleStep;
    float kernelCells;
    float currentFSD;
    float measuredFSD, measuredFSDUNK;

    Pose truePose, odomPose, initialPose;
    std::vector<Pose> odomPath, truePath;
    double timeStamp;

    int numParticles;
    std::vector<MCLparticle> particles;

    float meanParticle[3];
    Pose meanParticlePose;
    float covAngle, covMajorAxis, covMinorAxis;

    pthread_mutex_t* mutex;

    static void Ellipse(float rx, float ry, float angle, int num_segments=80);

    Timer t1, t2, t3;


};

#endif // MCL_H

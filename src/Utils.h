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

#ifndef UTILS_H
#define UTILS_H

#include <sys/time.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include <vector>

enum ConnectionMode {SIMULATION, SERIAL, WIFI};
enum LogMode { NONE, RECORDING, PLAYBACK, MAP};
enum MotionMode {MANUAL_SIMPLE, MANUAL_VEL, WANDER, WALLFOLLOW, ENDING};
enum MovingDirection {STOP, FRONT, BACK, LEFT, RIGHT, RESTART, DEC_ANG_VEL, INC_ANG_VEL, INC_LIN_VEL, DEC_LIN_VEL};

#define DEG2RAD(x) x*M_PI/180.0
#define RAD2DEG(x) x*180.0/M_PI

float normalizeAngleDEG(float a);
float normalizeAngleRAD(float a);

void setGLColorFromGrayscale(float grayscale);

typedef struct {
    float x,y,z;
} Point3D;

class Pose{
    public:
        Pose();
        Pose(float a, float b, float c);

        friend std::ostream& operator<<(std::ostream& os, const Pose& p);

        float x, y, theta;
};

class LogFile
{
    public:
        LogFile(LogMode mode, std::string name);

        Pose readPose(std::string info);
        std::vector<float> readSensors(std::string info);

        void writePose(std::string info, Pose pose);
        void writeSensors(std::string s, std::vector<float> sensors);
        void writeString(std::string s);
        void writeInt(int x);
        void breakLine();

        bool hasEnded();

    private:
        std::fstream file;
        std::string filename;
};

class Timer{
    public:
        Timer();

        void startCounting();
        void startLap();
        void stopCounting();

        float getTotalTime();
        float getLapTime();

        void waitTime(float t);

    private:
        struct timeval tstart, tlapstart, tnow;
};

#endif // UTILS_H

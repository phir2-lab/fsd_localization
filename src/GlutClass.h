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

#ifndef __GLUTCLASS_H__
#define __GLUTCLASS_H__

#include "Robot.h"
#include "Utils.h"

class GlutClass
{
    public:
        static GlutClass* getInstance();

        void initialize();
        void process();
        void terminate();

        void screenshot();

        void setRobot(Robot* r);

        bool drawRobotPath;

        int glutWindowSize;
        int frame;

        int halfWindowSize, x_aux, y_aux;
        int halfWindowSize_MCL, x_aux_MCL, y_aux_MCL;
        bool viewingMCL;


    private:
        GlutClass ();
        static GlutClass* instance;

        Robot* robot_;
        Grid* grid_;
        Timer timer;

        int halfWindowSizeX_, halfWindowSizeY_;
        bool lockCameraOnRobot;

        int id_;

	    void render();

        static void display();
        static void reshape(int w, int h);
        static void keyboard(unsigned char key, int x, int y);
        static void specialKeys(int key, int x, int y);
};

#endif /* __GLUT_H__ */



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

#include <pthread.h>
#include <iostream>
#include <string.h>

#include "Robot.h"
#include "GlutClass.h"

void* startRobotThread (void* ref)
{
    Robot* robot=(Robot*) ref;

    robot->initialize();

    while(robot->isRunning()){
        robot->run();
    }

	return NULL;
}

void* startGlutThread (void* ref)
{
    GlutClass* glut=GlutClass::getInstance();
    glut->setRobot((Robot*) ref);

    glut->initialize();

    glut->process();

	return NULL;
}

int main(int argc, char* argv[])
{
    Robot* r;
    r = new Robot(argc,argv);

    r->grid->mutex = new pthread_mutex_t;
    if (pthread_mutex_init(r->grid->mutex,NULL) != 0){
        printf("\n mutex init failed\n");
        return 1;
    }

    pthread_t robotThread, glutThread;
    pthread_create(&(robotThread),NULL,startRobotThread,(void*)r);
    pthread_create(&(glutThread),NULL,startGlutThread,(void*)r);

    pthread_join(robotThread, 0);
    pthread_join(glutThread, 0);

    return 0;
}


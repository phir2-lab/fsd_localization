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

#include <GL/glut.h>
#include <FreeImage.h>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <cmath>
#include <unistd.h>

#include "GlutClass.h"

/////////////////////////////////////////////
///// CONSTRUCTOR & CLASS INSTANTIATION /////
/////////////////////////////////////////////

GlutClass::GlutClass(){
}

GlutClass* GlutClass::instance = 0; 

GlutClass* GlutClass::getInstance ()
{
    if (instance == 0){
        instance = new GlutClass;
    }
    return instance;
}

//////////////////////////
///// PUBLIC METHODS /////
//////////////////////////

void GlutClass::initialize()
{
    halfWindowSize = 50;
    x_aux = 0;
    y_aux = 0;
    glutWindowSize = 700;

    viewingMCL = true;


    // Wait for the robot's initialization
    while(robot_->isReady() == false){
        usleep(100000);
    }

    grid_ = robot_->grid;

	int argc=0;char** argv=0;
    glutInit(&argc, argv);
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize (glutWindowSize,glutWindowSize);

    id_ = glutCreateWindow("Janela");
    lockCameraOnRobot = true;
    drawRobotPath = true;
    frame = 0;

    timer.startCounting();

    glClearColor (1.0, 1.0, 1.0, 0.0);
    glClear (GL_COLOR_BUFFER_BIT);

    glEnable (GL_BLEND); glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glutDisplayFunc(display); 
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(specialKeys);     
}

void GlutClass::process()
{
    glutMainLoop();
}

void GlutClass::terminate()
{
    robot_->motionMode_ = ENDING;
}

void GlutClass::setRobot(Robot *r)
{
    robot_=r;
}

///////////////////////////
///// PRIVATE METHODS /////
///////////////////////////

void GlutClass::render()
{
    if(robot_->isRunning() == false){
        exit(0);
    }

    if(viewingMCL){
        // Desenha MCL
        int h = robot_->mcl->mapHeight;
        int w = robot_->mcl->mapWidth;

        // Atualiza a região da janela
        glMatrixMode (GL_PROJECTION);
        glLoadIdentity ();
        if(w > h){
            glOrtho (0 + x_aux_MCL + halfWindowSize_MCL,
                     w + x_aux_MCL - halfWindowSize_MCL,
                     -(w-h)/2 + y_aux_MCL + halfWindowSize_MCL,
                     h+(w-h)/2 + y_aux_MCL - halfWindowSize_MCL,
                     -1, 50);
        }else{
            glOrtho (-(h-w)/2 + x_aux_MCL + halfWindowSize_MCL,
                     w+(h-w)/2 + x_aux_MCL - halfWindowSize_MCL,
                     0 + y_aux_MCL + halfWindowSize_MCL,
                     h + y_aux_MCL - halfWindowSize_MCL,
                     -1, 50);
        }

        glMatrixMode (GL_MODELVIEW);

        glClearColor(1.0, 1.0, 1.0, 1.0);
        glClear (GL_COLOR_BUFFER_BIT);

        pthread_mutex_lock(grid_->mutex);
        robot_->drawMCL();
        pthread_mutex_unlock(grid_->mutex);

        // Take a screenshot per second
        if(timer.getLapTime()>3.0){
            screenshot();
            timer.startLap();
        }

        glutSwapBuffers();
        glutPostRedisplay();
        return;
    }

    int mapWidth = grid_->getMapWidth();

    int scale = grid_->getMapScale();

    Pose robotPose;

    robotPose = robot_->getCurrentPose();

    float xRobot = robotPose.x*scale;
    float yRobot = robotPose.y*scale;
    float angRobot = robotPose.theta;

    float xCenter, yCenter;
    if(lockCameraOnRobot){
        xCenter=xRobot;
        yCenter=yRobot;
    }

    // Update window region
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    glOrtho ((int)(xCenter) + x_aux - halfWindowSize, (int)(xCenter) + x_aux + halfWindowSize-1,
             (int)(yCenter) - y_aux - halfWindowSize, (int)(yCenter) - y_aux + halfWindowSize-1,-1, 50);
    glMatrixMode (GL_MODELVIEW);
    glClearColor(1.0, 1.0, 1.0, 0);
    glClear (GL_COLOR_BUFFER_BIT);

    // Compute limits of visible section of the grid
    int xi, yi, xf, yf;
    int x = xCenter + mapWidth/2 - 1;
    int y = mapWidth/2 - yCenter;

    xi = x + x_aux - halfWindowSize;
    if( xi < 0 ){
        xi = 0;
        xf = halfWindowSize*2 - 1;
    }else{
        xf = x + x_aux + halfWindowSize - 1;
        if( xf > mapWidth - 1){
            xi = mapWidth - 2*halfWindowSize;
            xf = mapWidth - 1;
        }
    }

    yi = y + y_aux - halfWindowSize;
    if( yi < 0 ){
        yi = 0;
        yf = halfWindowSize*2 - 1;
    }else{
        yf = y + y_aux + halfWindowSize - 1;
        if( yf > mapWidth - 1){
            yi = mapWidth - 2*halfWindowSize;
            yf = mapWidth - 1;
        }
    }

    // Draw grid
    pthread_mutex_lock(grid_->mutex);
    grid_->draw(xi, yi, xf, yf);
    pthread_mutex_unlock(grid_->mutex);

    // Draw robot path
    if(drawRobotPath){
        robot_->drawPath();
    }

    // Draw robot
    robot_->draw(xRobot,yRobot,angRobot);

    // Take a screenshot per second
    if(timer.getLapTime()>1.0){
        screenshot();
        timer.startLap();
    }

    glutSwapBuffers();
    glutPostRedisplay();

    usleep(5000);
}

void GlutClass::screenshot()
{

    std::stringstream ss;
    std::string imgName;

    ss << robot_->mcl->pathName << "frame-" << std::setfill('0') << std::setw(6) << frame << ".png";
    ss >> imgName;

    int width = glutWindowSize;
    int height = glutWindowSize;

    // Make the BYTE array, factor of 3 because it's RBG.
    BYTE* pixels = new BYTE[ 3 * width * height];

    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels);

    BYTE aux;
    for(int p=0;p<3*width*height;p=p+3){
        aux=pixels[p+2];
        pixels[p+2]=pixels[p];
        pixels[p]=aux;
    }

    // Convert to FreeImage format & save to file
    FIBITMAP* image = FreeImage_ConvertFromRawBits(pixels, width, height, 3 * width, 24, 0xFF0000, 0x0000FF, 0xFF0000, false);
    FreeImage_Save(FIF_PNG, image, imgName.c_str(), 0);

    // Free resources
    FreeImage_Unload(image);
    delete [] pixels;

    frame++;
}

/////////////////////////////////////////////////////
///// STATIC FUNCTIONS PASSED AS GLUT CALLBACKS /////
/////////////////////////////////////////////////////

void GlutClass::display()
{
    instance->render();
}

void GlutClass::reshape(int w, int h)
{
    glViewport (0, 0, (GLsizei) w, (GLsizei) h); 
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    glOrtho (-100,100,-100,100,0, 50);
    glMatrixMode (GL_MODELVIEW);

    glClearColor(0.8, 0.8, 0.8, 0);
    glClear (GL_COLOR_BUFFER_BIT);
}

void GlutClass::keyboard(unsigned char key, int x, int y)
{
    // key: the value of the pressed key

    switch(key) {
        case 27:
            instance->terminate();
            break;
        case ' ':
            instance->robot_->move(STOP);
            instance->robot_->motionMode_ = MANUAL_SIMPLE;
            std::cout << "MotionMode: 1 - MANUAL_SIMPLE" << std::endl;
            break;
        case '1':
            if(instance->robot_->motionMode_!=MANUAL_SIMPLE){
                instance->robot_->move(STOP);
                instance->robot_->motionMode_ = MANUAL_SIMPLE;
                std::cout << "MotionMode: 1 - MANUAL_SIMPLE" << std::endl;
            }
            break;
        case '2':
            if(instance->robot_->motionMode_!=MANUAL_VEL){
                instance->robot_->move(STOP);
                instance->robot_->motionMode_ = MANUAL_VEL;
                std::cout << "MotionMode: 2 - MANUAL_VEL" << std::endl;
            }
            break;
        case '3':
            instance->robot_->motionMode_ = WANDER;
            std::cout << "MotionMode: 3 - WANDER" << std::endl;
            break;
        case '4':
            instance->robot_->motionMode_ = WALLFOLLOW;
            std::cout << "MotionMode: 4 - WALLFOLLOW" << std::endl;
            break;
        case 'l': //Lock camera
            if(instance->lockCameraOnRobot == true){
                instance->lockCameraOnRobot = false;
                Pose p = instance->robot_->getCurrentPose();
                instance->x_aux = p.x*instance->grid_->getMapScale();
                instance->y_aux = -p.y*instance->grid_->getMapScale();
            }else{
                instance->lockCameraOnRobot = true;
                instance->x_aux = 0;
                instance->y_aux = 0;
            }
            break;
        case 'r': //robot view mode
            instance->robot_->viewMode++;
            if(instance->robot_->viewMode == instance->robot_->numViewModes)
                instance->robot_->viewMode = 0;
            break;
        case 'v': //view mode
            instance->grid_->viewMode++;
            if(instance->grid_->viewMode == instance->grid_->numViewModes)
                instance->grid_->viewMode = 0;

            if(instance->grid_->viewMode == 0)
                instance->viewingMCL = true;
            else
                instance->viewingMCL = false;
            break;
        case 'b': //view mode
            instance->grid_->viewMode--;
            if(instance->grid_->viewMode == -1)
                instance->grid_->viewMode = instance->grid_->numViewModes-1;

            if(instance->grid_->viewMode == 0)
                instance->viewingMCL = true;
            else
                instance->viewingMCL = false;
           break;
        case 't': //MCL transparency mode
            if(instance->robot_->mcl != NULL){
                instance->robot_->mcl->transparency = !instance->robot_->mcl->transparency;
            }
            break;
        case 'y': //MCL view mode
            instance->robot_->mcl->viewMode++;
            if(instance->robot_->mcl->viewMode == instance->robot_->mcl->numViewModes)
                instance->robot_->mcl->viewMode = 0;
            break;
        case 'w':
            if(instance->viewingMCL){
                instance->y_aux_MCL -= 10;
            }else{
                instance->y_aux -= 10;
                std::cout << "x_aux: " << instance->x_aux << " y_aux: " << instance->y_aux << " halfWindowSize:" << instance->halfWindowSize << std::endl;
            }
            break;
        case 'd':
            if(instance->viewingMCL){
                instance->x_aux_MCL += 10;
            }else{
                instance->x_aux += 10;
                std::cout << "x_aux: " << instance->x_aux << " y_aux: " << instance->y_aux << " halfWindowSize:" << instance->halfWindowSize << std::endl;
            }
            break;
        case 'a':
            if(instance->viewingMCL){
                instance->x_aux_MCL -= 10;
            }else{
                instance->x_aux -= 10;
                std::cout << "x_aux: " << instance->x_aux << " y_aux: " << instance->y_aux << " halfWindowSize:" << instance->halfWindowSize << std::endl;
            }
            break;
        case 's':
            if(instance->viewingMCL){
                instance->y_aux_MCL += 10;
            }else{
                instance->y_aux += 10;
                std::cout << "x_aux: " << instance->x_aux << " y_aux: " << instance->y_aux << " halfWindowSize:" << instance->halfWindowSize << std::endl;
            }
            break;
        case 'm':
            instance->screenshot();
            break;
        case '-':
            if(instance->viewingMCL){
                instance->halfWindowSize_MCL -= 10;

                int size = std::max(instance->robot_->mcl->mapHeight,instance->robot_->mcl->mapWidth);
                if(instance->halfWindowSize_MCL < -size/3)
                    instance->halfWindowSize_MCL = -size/3;
            }else{
                instance->halfWindowSize += 10;
                if((unsigned int)instance->halfWindowSize > instance->grid_->getMapWidth()/2)
                    instance->halfWindowSize = instance->grid_->getMapWidth()/2;
            }
            break;
        case '+':
        case '=':
            if(instance->viewingMCL){
                instance->halfWindowSize_MCL += 10;

                int size = std::max(instance->robot_->mcl->mapHeight,instance->robot_->mcl->mapWidth);
                if(instance->halfWindowSize_MCL > size/3)
                    instance->halfWindowSize_MCL = size/3;
            }else{
                instance->halfWindowSize -= 10;
                if(instance->halfWindowSize < instance->grid_->getMapScale())
                    instance->halfWindowSize = instance->grid_->getMapScale();
            }
            break;
        default:
            break;
    }
}

void GlutClass::specialKeys(int key, int x, int y)
{
    // key: the value of the pressed key

    if(instance->robot_->motionMode_ == MANUAL_VEL)
        switch(key) {
            case GLUT_KEY_UP:
                instance->robot_->move(INC_LIN_VEL);
                break;
            case GLUT_KEY_RIGHT:
                instance->robot_->move(DEC_ANG_VEL);
                break;
            case GLUT_KEY_LEFT:
                instance->robot_->move(INC_ANG_VEL);
                break;
            case GLUT_KEY_DOWN:
                instance->robot_->move(DEC_LIN_VEL);
                break;
            default:
                break;
        }
    else
        switch(key) {
            case GLUT_KEY_UP:
                instance->robot_->move(FRONT);
                break;
            case GLUT_KEY_RIGHT:
                instance->robot_->move(RIGHT);
                break;
            case GLUT_KEY_LEFT:
                instance->robot_->move(LEFT);
                break;
            case GLUT_KEY_DOWN:
                instance->robot_->move(BACK);
                break;
            default:
                break;
        }
}


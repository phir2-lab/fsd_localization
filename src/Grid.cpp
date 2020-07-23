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

#include <cstdio>
#include <GL/glut.h>
#include <iostream>
#include <sstream>
#include <iomanip>

#include "Grid.h"
#include "Utils.h"
#include "math.h"

Grid::Grid ()
{
    mapScale_ = 20;
    mapWidth_ = mapHeight_ = 2000;
    numCellsInRow_=mapWidth_;
    halfNumCellsInRow_=mapWidth_/2;

    cells_ = new Cell[mapWidth_*mapHeight_];

    for (unsigned int j = 0; j < numCellsInRow_; ++j)
    {
        for (unsigned int i = 0; i < numCellsInRow_; ++i)
        {
            unsigned int c = j*numCellsInRow_ + i;
            cells_[c].x = -halfNumCellsInRow_ + 1 + i;
            cells_[c].y =  halfNumCellsInRow_ - j;

            cells_[c].val=1.0;
            cells_[c].himm=7;
            cells_[c].himm_cloud=7;
            cells_[c].himm_cloud2=7;
            cells_[c].cloud=-100;
            cells_[c].cloud2=-100;
            cells_[c].occType=UNEXPLORED;
        }
    }

    minX = minY = 1000;
    maxX = maxY =-1000;

    numViewModes=5;
    viewMode=0;
    iterations = 0;

    showValues=false;
}

Cell* Grid::getCell (int x, int y)
{
    int i=x+halfNumCellsInRow_-1;
    int j=halfNumCellsInRow_-y;
    return &(cells_[j*numCellsInRow_ + i]);
}

int Grid::getMapScale()
{
    return mapScale_;
}

int Grid::getMapWidth()
{
    return mapWidth_;
}

int Grid::getMapHeight()
{
    return mapHeight_;
}

void Grid::updateBoundaries(int i, int j)
{
    if(i < minX) minX = i;
    if(i > maxX) maxX = i;
    if(j < minY) minY = j;
    if(j > maxY) maxY = j;

}

void Grid::draw(int xi, int yi, int xf, int yf)
{
    glLoadIdentity();

    for(int i=xi; i<=xf; ++i){
        for(int j=yi; j<=yf; ++j){
            drawCell(i+j*numCellsInRow_);
        }
    }

    if(showValues){
        for(int i=xi; i<=xf; i++){
            for(int j=yi; j<=yf; j++){
                drawText(i+j*numCellsInRow_);
            }
        }
    }
}

void Grid::drawCell(unsigned int n)
{
    float aux;

    if(viewMode==1){
        aux=(16.0-cells_[n].himm)/16.0;
        glColor3f(aux,aux,aux);
//        if(aux>0.7 && (cells_[n].x%20==0 || cells_[n].y%20==0))
//            glColor3f(0.6,0.6,1.0);

    }else if(viewMode==2){
        if(cells_[n].cloud2 > iterations-3 && cells_[n].occType==OCCUPIED)
            glColor3f(0.0,0.0,0.8);
        else if(cells_[n].cloud2 > iterations-3 && cells_[n].occType==FREE)
            glColor3f(0.5,0.5,0.8);
        else
        if(cells_[n].occType==OCCUPIED)
            glColor3f(0.0,0.0,0.0);
        else if(cells_[n].occType==FREE)
            glColor3f(1.0,1.0,1.0);
        else if(cells_[n].occType==KERNEL)
            setGLColorFromGrayscale(1.0-0.7);
        else if(cells_[n].occType==KERNEL_UNK)
            setGLColorFromGrayscale(1.0-0.6);
        else
            glColor3f(0.6,0.6,0.6);
//        }
    }else if(viewMode==3){
            aux=(16.0-cells_[n].himm_cloud)/16.0;
            glColor3f(aux,aux,aux);
    }else{
            aux=(16.0-cells_[n].himm_cloud2)/16.0;
            glColor3f(aux,aux,aux);
    }


    glBegin( GL_QUADS );
    {
        glVertex2f(cells_[n].x+1, cells_[n].y+1);
        glVertex2f(cells_[n].x+1, cells_[n].y  );
        glVertex2f(cells_[n].x  , cells_[n].y  );
        glVertex2f(cells_[n].x  , cells_[n].y+1);
    }
    glEnd();
}

void Grid::drawText(unsigned int n)
{
    glRasterPos2f(cells_[n].x+0.25, cells_[n].y+0.25);
    std::stringstream s;
    glColor3f(0.5f, 0.0f, 0.0f);
    s << cells_[n].val;

    std::string text=s.str();
    for (unsigned int i=0; i<text.size(); i++)
    {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, text[i]);
    }
}

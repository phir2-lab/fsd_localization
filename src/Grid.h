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

#ifndef __GRID_H__
#define __GRID_H__

#include <pthread.h>

enum CellOccType {OCCUPIED, UNEXPLORED, FREE, NEAROBSTACLE, KERNEL, KERNEL_UNK};
enum CellPlanType {REGULAR, FRONTIER, MARKED_FRONTIER, PATH, LOCALGOAL};

#define UNDEF -10000000

class Cell
{
    public:
        int x,y, himm, himm_cloud, himm_cloud2, cloud, cloud2;
        float val;

        CellOccType occType;
        CellPlanType planType;
};

class Grid
{
    public:
        Grid();
        Cell* getCell(int x, int y);

        int getMapScale();
        int getMapWidth();
        int getMapHeight();

        void draw(int xi, int yi, int xf, int yf);

        void updateBoundaries(int i, int j);
        int minX, minY, maxX, maxY;


        int numViewModes;
        int viewMode;
        bool showValues;

        int iterations;

        pthread_mutex_t* mutex;


    private:
        int mapScale_; // Number of cells per meter
        int mapWidth_, mapHeight_; // in cells
        int numCellsInRow_, halfNumCellsInRow_;

        Cell* cells_;

        void drawCell(unsigned int i);
        void drawVector(unsigned int i);
        void drawText(unsigned int n);
};

#endif // __GRID_H__

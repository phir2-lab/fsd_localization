/**
 * This file is part of FSD Localization

 * This file is based on dt.h <http://cs.brown.edu/people/pfelzens/dt/>.
 * An implementation of the distance transform algorithm described in the paper <http://dx.doi.org/10.4086/toc.2012.v008a019>

 * Copyright (c) 2006 Pedro Felzenszwalb
 * Modifications Copyright (c) 2020 Renan Maffei <rqmaffei at inf dot ufrgs dot br> (Phi Robotics Research Lab - UFRGS)
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

#ifndef DISTANCETRANSFORM_H
#define DISTANCETRANSFORM_H

#include <vector>
#include <queue>
using namespace std;

#define INF 1E20

template <class T>
inline T square(const T &x) { return x*x; }

class DistanceTransform
{
public:
    DistanceTransform();

    void computeManhattanDT(vector<vector<double> >& distMap, queue<pair<int, int> > &cellsQueue); // 4-neighbor
    void computeChebyshevDT(vector<vector<double> >& distMap, queue<pair<int, int> > &cellsQueue); // 8-neighbor
    void computeEuclideanDT(vector<vector<double> >& distMap); // euclidean
    void computeSquareEuclideanDT(vector<vector<double> >& distMap); // euclidean

private:
    static double* dt1D(double *f, int n);
};

#endif // DISTANCETRANSFORM_H

# FSD Localization package

This package implements the Monte Carlo Localization technique using Free-Space Density (FSD) as observation model, as proposed by [Renan Maffei](http://www.inf.ufrgs.br/~rqmaffei/) et al. [[1]](#1)

## 1. Installation 

1. This package works in **ROS melodic** (and **kinetic**). For instructions on how to install and configure a ROS workspace see
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

    - The framework has two dependencies that must be installed
        - **Glut**:      graphical user interface
        - **FreeImage:** screenshot saving

        `sudo apt-get install freeglut3-dev libfreeimage-dev`

2. Copy the folder of 'fsd_localization' package into the src folder. It should look like this:  
```
    ROS_WORKSPACE_PATH/
        build/
        devel/
        src/
          fsd_localization/
```

3. To compile the package, just type catkin_make inside the main folder of the workspace.  
    `catkin_make`

## 2. Running the program

The program can be executed as follows  
`rosrun fsd_localization fsd_localization`

Parameters:
- `mapName`: The program loads a 2d grid map (in .txt format). The map should be located in the 'maps' folder within the package.   
This work was developed using data from the [Robot@home dataset](http://mapir.isa.uma.es/mapirwebsite/index.php/mapir-downloads/203-robot-at-home-dataset.htmlurl), converted to the ROS format using the **robotathome_at_ros** package, available at https://github.com/phir2-lab/robotathome_at_ros.  
The **default** option is `alma`. Other options are `anto`, `pare`, `rx2`. All these maps are contained in the 'maps' folder.

- `method`: Different particle weighting strategies are implemented in the [MCL.cpp](src/MCL.cpp) file, and their use can be selected with this parameter. 
    - `ABS_PCL_FSD`: **Absolute FSD** - Free-Space Density using depth information obtained by RGB-D cameras;
    - `INT_PCL_FSD` (**default**): **Interval FSD** - An Interval-based extension of the previous approach that computes the Free-Space Density using not only the known free-space, but also the possible free-space in unknown regions;
    - `CLikelihood`: scan matching between K random measurements from the point cloud, analyzing beam endpoints using a likelihood map, as used in [[2]](#2)
    - `CRaycast`: similar to the previous method, but performing raycasting to obtain more precise results;
    - `LRaycast`: scan matching using the 2D laser readings;
    - `Motion`: No observation model - only removing particles that go over obstacles or outside the map

- `datasetType`: There are two option: `multi_cam` (**default**) or `single_cam`. The first option uses the 4 RGB-D cameras of the Robot@Home dataset, and the second uses only the frontal RGB-D camera.

Ex:  
    `rosrun fsd_localization fsd_localization _mapName:="anto" _method:="ABS_PCL_FSD"`

## 3. Running everything together with a launch file

In the 'launch' folder is available a launch file to simultaneously run the program, the RViz viewer and bagfiles with data from the **Robot@home dataset** converted to the ROS format using the [**robotathome_at_ros**](https://github.com/phir2-lab/robotathome_at_ros) package.

For proper operation, you need to install beforehand the **robotathome_at_ros** package in the same ROS workspace and generate the bagfiles following the instructions in LINK_AQUI.

To run the launch file, type  
`roslaunch fsd_localization run_MCL.launch`

The same three parameters of the FSD localization program (`mapName, method, datasetType`) can be set by the user in the launch file.

Ex:  
`roslaunch fsd_localization run_MCL.launch mapName:="rx2" datasetType:="single_cam"`

## 4. Framework controls

`w, a, s, d` : scrolls viewed area

`+`, `-`: changes zoom

`v`, `b`: changes view mode

`ESC`: close program


## References

<a id="1">[1]</a> Maffei, R. Q., Pittol, D., Mantelli, M., Prestes, E. Kolberg, M. Global Localization Over 2D Floor Plans with Free-Space Density Based on Depth Information. Accepted in: 2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Las Vegas, USA. 2020. [(Accepted version)](http://www.inf.ufrgs.br/~rqmaffei/files/papers/Maffei2020_IROS.pdf)

<a id="2">[2]</a> 
W. Winterhalter, F. Fleckenstein, B. Steder, L. Spinello, and W. Burgard, 
“Accurate indoor localization for rgb-d smartphones and tablets given 2d floor plans,” 
in Proc. of IROS, 2015, pp. 3138–3143.

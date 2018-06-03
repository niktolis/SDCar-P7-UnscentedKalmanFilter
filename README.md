[//]: # (Image References)

[image1]: ./screenshots/Dataset1.PNG "Dataset1 Performance"
[image2]: ./screenshots/Dataset2.PNG "Dataset2 Performance"
[image3]: ./screenshots/LaserNIS_Dataset1.png "Laser NIS of Dataset 1"
[image4]: ./screenshots/RadarNIS_Dataset1.png "Radar NIS of Dataset 1"
[image5]: ./screenshots/LaserNIS_Dataset2.png "Laser NIS of Dataset 2"
[image6]: ./screenshots/RadarNIS_Dataset2.png "Radar NIS of Dataset 2"

# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Note that the programs that need to be written to accomplish the project are src/ukf.cpp, src/ukf.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurment that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

### Linux
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF` Previous versions use i/o from text files.  The current state uses i/o
from the simulator.

### Visual Studio on Windows

1. Clone this repo.
2. Use [this](https://github.com/fkeidel/CarND-Term2-ide-profile-VisualStudio) repo to install the necessary dependencies for Visual Studio.
   Do not copy the `main.cpp` file and don't perform any changes on the `CMakeLists.txt` file as they are already modified to facilitate compilation on both VS and GNU environment.
3. Open the solution generated inside `./products` folder on your Visual Studio Environment.

During the installation of some libraries with vcpkg some of the mirrors may be inactive. You may alter the vcpkg to search into another mirror.


## Results

Following screenshots show the performance of the UKF algorithm in the first and second dataset

### Dataset1

We see in the screenshot of the simulation that the RMSE for all measurements are well below the project target.

![alt text][image1]

And here are the printed values of the NIS for both sensors:

![alt text][image3]
![alt text][image4]

### Dataset2

In the second dataset we see from the screenshots that we are away the project target only in `Vx` RMSE. This is because of the 
initial values of P which are picked to satisfy the first dataset but create higher initial error on the second one. We see that as the time passes the filter converges to the measurement
values and the RMSE is decreasing.
![alt text][image2]

And here are the printed values of the NIS for both sensors again:

![alt text][image5]
![alt text][image6]



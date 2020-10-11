# Extended Kalman Filter Project 
Self-Driving Car Engineer Nanodegree Program
This is my completed assignment.

This project utilizes a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.  The kalman filter recieves alternating sensor measurements from lidar and radar sensors supplied by a simulator.  It goes through a constant cycle of predicting the location and velocity after a small step in time, then updating its belief based on a sensor measurement, factoring in uncertainty in both the prediction and update operations.  The lidar measurements are supplied in a cartesian coordinate system.  However, the radar measurements are supplied in a polar coordinate system which requires a nonlinear transformation using a first order Taylor series expansion (Jacobian matrix).
When the simulator and the C++ program are run together, the simulator starts out displaying an X and Y axis at the starting point where the car begins.  The sensor measurements are given as if the sensors were located at that origin and measuring the position of the car.  When the simulation is started, the car begins to move and the car's position is considered the ground truth.  The scattered red and blue dots represent the noisy radar and lidar measurements, which are passed to the C++ program.  The C++ program returns its beliefs about the car's position to the simulator and the simulator plots those coordinates as green triangles.  As the car travels its path, you can see that the extended kalman filter beliefs are very close to the car's ground truth position.  The Root Mean Squared error between the ground truth and the extended kalman filter's beliefs is calculated at each step and returned to the simulator where you can see the X position, Y position, X velocity, and Y velocity RMSE values.

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).  The linux simulator does not work with Ubuntu 20 (Ubuntu 16 or 18 recommended).  When starting the simulator, select the EKF simulation.

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Install it by running the appropriate .sh file in the terminal.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the build directory

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

Then start the Term 2 Simulator.

The C++ files are located in the src directory.  After cloning the original repo from Udacity, the changes I made to complete the assignment included editing FusionEKF.cpp, kalman_filter.cpp, and tools.cpp.  These files are commented with `TODO:` sections which are the areas where I wrote code to complete the project.

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

**INPUT**: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


**OUTPUT**: values provided by the c++ program to the simulator

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

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `



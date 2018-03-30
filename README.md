# Unscented Kalman Filter
## Author : Sandeep Patil


[nis_value]: ./sample_images/nis_value.JPG "Nis value"
[nis]: ./sample_images/nis.png "nis plot"


#### Objective of this project is to do sensor fusion of lidar and radar data using unscented kalman filter.   

In this project we will utilize unscented kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.  


--- 
### Installations and build 

This project involves the Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases) .

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. 

For Linux  
`./install-ubuntu.sh`

For Mac  
`./install-mac.sh`


Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF`

---

### Cummunication between main.cpp (server) and simulator
Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

OUTPUT: values provided by the c++ program to the simulator  

["estimate_x"] <= kalman filter estimated position x  
["estimate_y"] <= kalman filter estimated position y  
["rmse_x"]  
["rmse_y"]  
["rmse_vx"]  
["rmse_vy"]  

---

### Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)

---
### NIS value for order 3 equations.

![margin][nis_value]  


The 95% nis values are blow 7.815.

![nis plot][nis]


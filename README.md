# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

In this project I implemented Extended Kalman filter to do sensor fusion, estimate the state of a moving object of interest with noisy lidar and radar measurements.

By correctly implementing sensor fusion, this implementation obtains RMSE values that are lower that the tolerance outlined in the project rubric: px, py, vx, vy output coordinates must have an RMSE <= [.11, .11, 0.52, 0.52] .

Simulation results are recorded as following youtube videos.

Video with Dataset 1:
https://youtu.be/E0pFsr8ao94

Video with Dataset 2:
https://youtu.be/tRFBBEhw9wU

[image3]: ./images/img3.png
[image4]: ./images/img4.png

The main flow is in `main.cpp`, and the algorithm implementation is in:

`FusionEKF.cpp`, values initialization, and flow of "Measurement", "Prediction", Sensor fusion.

`KalmanFilter.cpp`, implementation Kalman filter for Lidar and Radar datas.

`tools.cpp`, helper functions to calculate `Jacobian` and `RMSE`.

`KalmanFilter.cpp` illustrated (taken from udacity course video):
![alt text][image3]

`FusionEKF.cpp` illustrated (taken from udacity course video):
![alt text][image4]



# Basic Usage
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

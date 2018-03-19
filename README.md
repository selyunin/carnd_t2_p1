# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program


This project implements Extended Kalman filter to estimate position and velocity of a moving pedestrian.
The state of a moving object of interest with noisy lidar and radar measurements. 
In order to satisfy the project rubric requirements, EKF with RMSE below the pre-defined bound has been devised.

This project should be run  in the Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)


Cmake is used as a build system for the project. In order to compile the code:
1. mkdir build
2. cd build
3. cmake ../src/
4. make
5. ./ExtendedKF


## Important Dependencies

The project has been tested / run on Ubuntu.

* cmake >= 3.5
* make >= 4.1 (Linux, Mac),
* gcc/g++ >= 5.4

## RMSE

[image1]: ./img/ekf_pic.png

The Extended Kalman filter is run against the test data set, obtaining the following error:

![EKF result pic][image1]


In order to come to a correct results:
* initialize previous time stamp when seeing the first time stamp;
* update for Radar: (1) roll the angle to `[-pi, pi]` interval after calculating `y`
* update for Radar: (2) avoid infinitesimal numbers when calculating rho ( or h(0) ), in order to 
  avoid division by zero in rho_dot (or h(2) ).

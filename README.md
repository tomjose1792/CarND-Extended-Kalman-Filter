# CarND-Extended-Kalman-Filter-Project



[//]: # (Image References)

[image1]: ./RMSE_Values_images/Dataset-1.png "Dataset-1 result"
[image2]: ./RMSE_Values_images/Dataset-2.png "Dataset-2 result"

### Overview

I have implemented Extended Kalman Filter (EKF) in C++ on LIDAR and RADAR measurement data from simulator provided by Udacity. The EKF initialises, predicts and updates the postions and velocities in X & Y of the vehicle.
To measure the performance of the filter, the Root Mean Square Error between the filter results and the actual ground truth is calculated.

The RMSE values of px,py,vx,vy need to be less than or equal to 
 [.11, .11, 0.52, 0.52] to meet the project milestone.
 
Note:

For comprehensive instructions on how to install and run project, please, refer to the following repo, which was used as a skeleton for this project: https://github.com/udacity/CarND-Extended-Kalman-Filter-Project

 
### Files in the Github src Folder


main.cpp - Communicates with the Term 2 Simulator receiving data measurements, calls a function to run the Kalman filter, calls a function to calculate RMSE

FusionEKF.cpp - Initializes the filter, calls the predict function, calls the update function

kalman_filter.cpp - Defines the predict function, the update function for lidar, and the update function for radar

tools.cpp - Function to calculate RMSE and the Jacobian matrix


### Basic Build instructions

    $ mkdir build && cd build
    $ cmake .. && make
    $ ./ExtendedKF

Or use the `$ bash build_and_run.sh` provided to execute the above commands.

### Final results after Filter:


On the simulator: 
The red circles are LIDAR measurements and blue circles are RADAR measurements data. The green triangles are the estimated markers. The simulator provides the script the measured data (either lidar or radar), and the script feeds back the measured estimation marker, and RMSE values from its Kalman filter.

Dataset-1 results:

![alt text][image1]


Dataset-2 results:

![alt text][image2]


Also a recording of the estimation by the EKF is provided, "Extended Kalman Filter_Video" 



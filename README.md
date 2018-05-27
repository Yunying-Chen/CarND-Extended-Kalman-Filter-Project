# CarND-Extended-Kalman-Filter-Project
## Intro
In this project， a kalman filter will be built to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined.


## Dependencies
cmake >= 3.5                                     
make >= 4.1                                       
gcc/g++ >= 5.4                                        



## Files
  * src/kalman_filter.cpp ————  defines the predict function and the update functions;
  * src/FusionEKF.cpp ————  initializes the filter;
  * src/tools.cpp ————  calculates the RMSE and Jacobian matrix

## Performance
From the build directory, execute ./ExtendedKF                                                               
With Dataset 1 the result is as follows:                                                       
![Image text](https://github.com/Yunying-Chen/CarND-Extended-Kalman-Filter-Project/blob/master/imgs/result1.png)     







With Dataset 2 the result is as follows: 
![Image text](https://github.com/Yunying-Chen/CarND-Extended-Kalman-Filter-Project/blob/master/imgs/result2.png)     

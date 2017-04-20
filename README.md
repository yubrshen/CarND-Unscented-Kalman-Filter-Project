# Unscented Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

---
## RMSE

Here is the RMSE for the data set "obj_pose-laser-radar-synthetic-input.txt" of the UKF implementation:

(Rubric's requirement: [.09, .10, .40, .30])

The best so far with std\_a = 0.23, std\_yawdd = 0.23: 

    0.0691687
    0.0971907
     0.346969
     0.239235

They meet the rubric requirement. 

How I found the combination of the parameters?

1. compute the std of velocity acceleration, the std of yaw rate acceleration based on the measurement data
2. use the computed as starting points
3. observe the NIS distribution, determine the possibility to increase std\_a, and std\_yawdd
4. in small increments to increase std\_a, and std\_yawdd, until the RMSE meets the rubric requirement. 

Experiment with LIDAR or RADAR alone also helped to develop experience of the possible range of std\_a, and std\_yawdd

It's still very blind search. 

## NIS

The NIS curvees shows the guess of std\_a, and std\_yawdd are acceptable. 
It's not too certain with majority of NIS values near the 95% threshold. It's not too random, with a few outliers of NIS above the 95 percentile. 

![NIS of LIDAR for Object_Pose](./data/nis-lidar-obj-pose.png)
![NIS of RADAR for Object_Pose](./data/nis-radar-obj-pose.png)

## Estimation Visualization

![Position for Object_Pose](./data/position-obj-pose.png)

![Velocity for Object_Pose](./data/velocity-obj-pose.png)
![Yaw for Object_Pose](./data/yaw-obj-pose.png)

## Estimation Accuracy of Different Sensors

Experiments were conducted with LIDAR alone, and RADAR alone. The overall estimation performance were compared. The following are the order of the performance, from the best to the worst: 

1. LIDAR and RADAR combined
2. LIDAR alone
3. RADAR alone

With LIDAR alone, the estimations of positions, velocity, and turning angle (yaw) were fairly accurate. 

With RADAR alone, the estimation is much coarse. 

## Further Studies

More study is needed to understand the characteristics of numerical calculation to address the tendency of divergence to "nan" values. 

I also observe that converting the computed yaw to be within [-PI, PI] may cause discontinuity or inaccuracy in numerical computation. 
I used to convert it every time when yaw calculation is involved. I observed that there was some jumps in the estimated yaw angle, and also 
phase lag in the velocity estimation. At the end, I only do the conversion once at the end of processing one measurement. Then the estimations of yaw angle, and velocity become much smooth. 
The mathematical explanation is not yet clear to me. I would like to understand what should be the proper mathematical sound procedure. 

Furthermore, when there is more time, I'd like to implement more validation and test support for assurance of implementation correctness. 
It's quite error prone in the implementation, causing doubt of the nature of the problems whether it's of the appropriateness of algorithm, 
or the correctness of implementation. 


## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./UnscentedKF ../data/obj_pose-laser-radar-synthetic-input.txt`

## Project Instructions and Rubric

This information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/c3eb3583-17b2-4d83-abf7-d852ae1b9fff/concepts/f437b8b0-f2d8-43b0-9662-72ac4e4029c1)
for instructions and the project rubric.

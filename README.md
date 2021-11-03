# SLAM LECTURES (Claus Brenner)
This repository contains assignments solution in SLAM Course by Claus Brenner.

## Overview
The Course discusses algorithms and techniques used in SLAM (Simultanuous Localization And Mapping) for robots and it's devided into 7 units as follows:

### Unit A
In this unit, the robot is steered through the arena and the motor ticks in addition to the LIDAR scans are recorded, then he discusses the motion model for the robot and uses the motor ticks to determine the robot trajectory in the arena, later the LIDAR scans are used to detect landmarks in the arena and compare their positions to the actual positions in the arena. [READ MORE](./Unit_A/README.md)

### Unit B
Using the detected landmark positions from Unit A, they will be assigned to actual landmarks in the arena based on their proximity and those point pairs will be used in a least squares estimation of a similarity transformation to correct the robot trajectory. Although this approach works, it results in a very jagged trajectory because sometimes there are few observed features (cylinder point pairs), so another techinque is used where points on the fence of the arena are assigned to the nearest possible partners to determine the transformation using an algorithm called Iterative Closest Point (ICP) which leads in a smoother trajectory but unfortuantely with a high cost. [READ MORE](./Unit_B/README.md)

### Unit C
This unit introduces the uncertainty of the robot and models it using probability distributions. First it shows the effect of the robot movement on the uncertainty of its state (the more the robot moves, the higher the uncertainty of its state is).Then it shows the effect of measurements on the uncertainty of the robot state (measurement reduces the uncertainty of the robot state). By combining those two steps; movement (prediction step) and measurement (correction step) this introduces the Bayes filter (which is the base filter for iterative filters). Then it discusses the situation where the distribution is a Gaussian/Normal distribution and derives the kalman filter for 1D case. [READ MORE](./Unit_C/README.md)

### Unit D
The unit starts by exploring the multivariate Gaussian/Normal distribution, and then it generalizes the kalman filter from unit c to the multi-dimensional case. Then it tries to apply this filter on our robot but since the robot motion model is non-linear, another version of kalman filter is introduced which is the Extended kalman filter. [READ MORE](./Unit_D/README.md)

### Unit E
First have a look at the particle filter which represents the distribution by hypothetical states (particles), and this filter also consists of two steps prediction and correction, where in the prediction we compute set of particles that diverge, while the correction step can be summed up in two steps: compute importance factors and perform importance sampling. This filter is able to recover the robot trajectory even if it doesn't know the initial state. [READ MORE](./Unit_E/README.md)

### Unit F

### Unit G

### Unit PP
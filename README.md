# Calibration of a Hexapod Manipulator

Using parameters given in Table 1 and 2, wrote a MATLAB code to calibrate the hexapod Stewart platform.

![image](https://github.com/ankushsingh999/error-hexapod-WS/assets/64325043/403ff405-154a-43b9-9cba-25f8630e1859)

![image](https://github.com/ankushsingh999/error-hexapod-WS/assets/64325043/aca6e24b-1244-468a-8b7c-6da96f1841cf)

Used XYZ Euler sequence.

**For selecting configurations, it does not matter how many configurations you choose, but it should not be less than 10 and your configurations should be chosen very close to the boundary of the workspace. This is because of the error observability consideration. When you choose your configurations close to the workspace boundary, you will get better calibration results. Also, try to choose your configurations to be distributed among the whole workspace boundary. In other words, do not choose your configurations very close to one another.**

Used **Least Square method** to minimize the cost function. Used **‘lsqnonlin’** function on MATLAB to minimize the cost function and find the identified real values. For simplification purpose, assume that the error of measurement device, called ‘measurement error’, is zero/negligible when measuring the position/orientation (configuration) as well as when applying leg lengths variation to the robot.

Functions / codes used:
1) NIK.m - Inverse Kinematics of the parallel manipulator using the nominal parameters.
2) NFK.m - Forward Kinematics of the parallel manipulator using the nominal parameters.
3) jacobianV.m - Velocity Jacobian used in NFK.
4) RIK.m - Inverse Kinematics of the parallel manipulator using the simulated real values.
5) RFK.m - Forwards Kinematics of the parallel manipulator using the simulated real values.
6) jacobianRV.m - Velocity Jacobian used in RFK.
7) config.m - To find the points on the workspace boundary.
8) CF.m - Cost Function used to minimize the error.
9) Calibrate.m - Calibration.

![image](https://github.com/ankushsingh999/calibration-hexapod-manipulator/assets/64325043/f5777eee-3721-41fb-8251-b4de05b1a090)

**Inverse Kinematics – NIK & RIK**
The inverse Kinematic code takes the Pose (P - 6x1) of the end effector of the manipulator as its input and calculates the leg lengths (lg - 6x1) required by the leg lengths to achieve the desired position and orientation. The nominal IK uses the nominal values of S and U which are the vectors from the center of the top platform to leg and center of the bottom platform to the leg. Meanwhile the real IK utilizes the simulated real values of the same.

**Forward Kinematics – NFK & RFK**
The forward Kinematics calculates the pose (P – 6x1) of the end effector when the leg lengths (lg) are given. The code follows an iterative process to calculate the pose, by taking the initial guess and finally reaching the actual pose by minimizing the leg length or pose error. I have taken the leg length error for the iteration and pose calculation in my code. The real forward kinematics code also has a component where it calculates the error in leg lengths; which is obtained by subtracting the simulated real values and the nominal values. This error is added to the leg lengths that are provided as the input to the real forward kinematic code.

**Calibration**
The first part of calibration is to write the cost function.

**Cost Function:**
The system has 42 kinematic parameters, 7 for each leg (Si, Ui, Loi). We would be needing more than 7 configurations for reliable parameter identification. To Test the code I had taken 12 random configurations, and checked if they are present in the workspace by running those in the NIK code and cascading the input to NFK to verify the output.

**config.m**
The code config.m is used to calculate the planar workspace of hexapod at heights at 750, 800, 1000 and 1050. The plot shown below is a collection of points with a step size 5. The points are taken near the boundary as instructed. Since this is a constant orientation workspace, the euler angles would be 0. The points shown are in the format [ X Y 750/800/100/1050 0 0 0 ].

![image](https://github.com/ankushsingh999/calibration-hexapod-manipulator/assets/64325043/fe03ec92-4aaf-47d6-9819-3d34ac8e4292)

**Random Configurations taken:**
[ 0 0 800 0 0 0], [120 380 782 2 3 7], [-67 -78 800 0 0 0], [45 76 1000 0 0 0], [56 80 750 3 5 6], [45 3 816 9 8 5], [32 28 760 1 0 8], [0 0 1045 8 7 5], [9 8 800 9 8 3], [54 -45 900 0 4 5], [5 67 900 8 7 2] and [4 5 782 9 2 3]

**Configuration close to workspace boundary:**
[210 532 750 0 0 0]; [-500 312 750 0 0 0]; [-370 -518 750 0 0 0]; [385 -428 750 0 0 0]; [-85 542 800 0 0 0]; [-535 -3 800 0 0 0]; [-60 -543 800 0 0 0];[ 595 2 800 0 0 0]; [-190 172 1000 0 0 0]; [-195 -158 1000 0 0 0]; [170 -163 1000 0 0 0]; [205 127 1000 0 0 0]; [-15 117 1050 0 0 0]; [-105 -3 1050 0 0 0]; [-10 -113 1050 0 0 0]; [140 2 1050 0 0 0]

All these points lie inside the workspace and have been verified by running the inverse and forward kinematics.
From the nominal values, we know that the leg length is 604.8652.
The measured pose is taken by calculating the real forward kinematics. The input of the RFK uses the output from the NIK for each configuration.
J is the number of error functions present which is declared. The loop is run for ‘m’ configurations for each of the six legs. Inverse Kinematics is calculated for the measured pose in the loop. This is used to calculate the leg variation (delta l ) which we will need to apply to the nominal leg to reach the desired position. “delta_l” is calculated by subtracting the l from IK with the nominal leg length value.

***The cost function is given by:***

![image](https://github.com/ankushsingh999/calibration-hexapod-manipulator/assets/64325043/283d531d-b553-4599-b477-d51d7457ec4e)

The O is the distance from the center (origin of the base frame) of the base platform to the end effector which is taken to be the center of the top platform. The values can be obtained from the pose as the first three components of P contain the position of the end effector with respect to the base frame. The R is calculated by taking the 4,5 and 6 th values of the P. We are considering “XYZ” euler angles in the code (all NIK, RIK and CF). The S and U are the nominal S and U are obtained from the initial guess, in my code I have considered the nominal values to be my initial guess. The first three values of the initial guess are S(x,y,z) and U(x,y,z) For the second part of the cost function, Loi is the nominal leg length which is obtained from the last value (7) of the guess. The delta_l is calculated and explained in the previous paragraphs.

# Calibration (calibrate.m):
The calibrate function contains the Initial guess, which I have considered as the nominal values. There are total of 7 parameters (Sx,y,z Ux,y,z Loi). These parameters are used to call the cost function. The identified values are obtained by using lsqnonlin to minimize the cost function. The identified values are then compared with the simulated real values. On observation we notice that the error between the identified values and the simulated values id very less. The graphs are plotted for randomly taken configurations and configurations on or close to the workspace. 
*The results are as displayed below:*

For Configurations taken on the boundary of the workspace :

**IdentifiedValues =**
[ Sx Sy Sz Ux Uy Uz Loi ]

 96.6611 81.7606 1.0684 305.2600 115.0700 2.6210 604.4299

22.2475 125.2510 -0.5530 -55.2815 322.9818 4.2181 607.2473

-122.4645 36.6422 4.3538 -244.8080 208.0058 3.9348 600.4438

-120.6838 -34.4564 -4.9010 -252.5734 -211.8781 -3.0122 605.9031

24.7768 -125.0488 -4.8473 -53.9679 -320.6115 4.3181 604.5251

91.3449 -80.9842 0.2519 302.4251 -109.4326 3.3816 600.0615

The error when compared with the real simulated values was:

**error_in_values =**

-0.0001 -0.0004 0.0000 -0.0001 -0.0005 0.0000 -0.0000

0.0001 0.0001 -0.0000 0.0001 0.0001 -0.0000 0.0000

0.0126 0.0031 0.0009 0.0126 0.0029 0.0017 0.0003

-0.0021 -0.0001 -0.0004 -0.0021 -0.0002 -0.0006 0.0000

0.0001 -0.0001 0.0000 0.0001 -0.0000 0.0000 -0.0000

0.0013 -0.0024 -0.0004 0.0015 -0.0025 -0.0004 0.0001
 
***We notice that the error is very small, the values are pretty close.***

# RESULT:

***The following is the bar graph obtained from the error:***

![image](https://github.com/ankushsingh999/calibration-hexapod-manipulator/assets/64325043/510c8a4a-c8e5-4363-9c43-59a5c9a94ccc)

# Reference
A total solution to kinematic calibration of hexapod machine tools with a minimum number of measurement configurations and superior accuracies M.J. Nategh a, M.M. Agheli b


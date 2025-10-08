# Six DOF Robot Arm

## Overview
This is a desktop robot arm that I am building as a personal project to gain experience. It has six degrees of freedom and is made from PLA 3D printed parts. I designed it in Solidworks and wrote the code for it on a Raspberry Pi 4B in ROS2/Python.

## Code
There are several ROS Nodes set up in this workspace.  


## Math
### Equation 1
I solved the inverse kinematics for this 6 DOF robot arm using geometry. Referring to the image above one can see how I derived my equations (except for this first equation). Firstly, we know  
  
 ``` theta_1 = arctan(y/x) ```  
   
This is the first angle rotating around the z axis of the base of the robot, dictating where the robot turns to (yaw).

### Equation 2
Next, we can see a right triangle in the image which gives us  

``` r = sqrt(x**2 + y**2) ```  

### Equation 3
Next, we can derive a relation between phi_1 and phi_2 to find theta_2. However, we don't know phi_1 and phi_2. But we do have enough information to figure them out.  

``` theta_2 = phi_2 + phi_1 ```  


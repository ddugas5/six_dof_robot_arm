# Six DOF Robot Arm

YouTube Demo: https://youtube.com/shorts/wcTCWknwm1I?feature=share
  
## Overview
This is a desktop robot arm that I am building as a personal project to gain experience as a robotics engineer. It has six degrees of freedom and is made from PLA 3D printed parts. I designed it in Solidworks and wrote the code for it on a Raspberry Pi 4B in ROS2/Python. As for the rest of my electronics, I used an Adafruit PCA9685 module for my servo driver and this was powered using a Rapthor 12V 2600mAh battery. I stepped this down to 5V using a (add regulator name).

## Code
There are several ROS Nodes set up in this workspace. These are the core of what makes the robot arm move and how they all work together to make the robot arm perform.
* ik_solver_node
  * Subscribes to the topic "/ee_goal" to receive a position and orientation, do the inverse kinematics to find the joint angles to get the wrist center there, and publish the joint_angles to the "/joint_command" topic
* joint_command_node
  * Subscribes to the topic "/joint_commands" where it receives a list of joint angles. The joint angles are then mapped to their associated joint names and then published to the topic "/joint_angles"
* servo_node
  * Subscribes to "/joint_angles" and then outputs a pwm to servo motors to put them to that angle
  * Also subscribes to "/servo/joint_5/cmd" to set the gripper to open or closed
* gripper_node
  * Subscribes to "/gripper_command" and publishes an "open" or "close" to the servo_node's topic "/joint_angles" to control the gripper
* trajectory_control_node
  * Subscribes to "/joint_commands" and uses linear interpolation to produce 150 points between the first and second location(if there is no previous location stored, the code assumes the robot is starting at the zero position). These values are published to the topic "/ee_goal" to be converted to joint angles by the ik solver

## Math  
    
<img width="563" height="679" alt="Robot Arm FBD" src="https://github.com/user-attachments/assets/c9e46a22-2c18-4b20-a8fe-cf9b8742baaa" />  
  
This is an image of the first four degrees of freedom (DOF) of the robot. I am calculating the inverse kinematics for the wrist center of the robot, which is depicted as the final DOF that you can see in the upmost right corner of the image.  
  
### Equation 1
I solved the inverse kinematics for this 6 DOF robot arm using geometry. Referring to the image above, one can see how I derived my equations (except for this first equation). Firstly, we know  
  
 ``` theta_1 = arctan(y/x) ```  
   
This is the first angle rotating around the z axis of the base of the robot, dictating where the robot turns to face (yaw).

### Equation 2
Next, we can see a right triangle in the image which gives us  

``` r = sqrt(x**2 + y**2) ```  

### Equation 3
Next, we can derive a relation between phi_1 and phi_2 to find theta_2. However, we don't know phi_1 and phi_2. But we do have enough information to figure them out.  

``` theta_2 = phi_2 + phi_1 ```  

From the law of cosines, we can derive an equation to find phi_1  

``` L3**2 = L**2 + r**2 - 2 * L2 * r cos(phi_1) ```  

``` 2 * L2 * r * cos(phi_1) = L2**2 + r**2 - L3**2 ```  

``` cos(phi_1) = (L2**2 + r**2 - L3**2) / (2 * L2 * r) ```  

``` phi_1 = arccos((L2**2 + r**2 - L3**2) / (2 * L2 * r)) ```

### Equation 4  
From the triangle, we can see that phi_2 is  

``` phi_2 = arctan(z/x) ```  

Now, we can use Equation 3  

``` theta_2 = phi_2 + phi_1 ```  

### Equation 5  
Lastly, we can see from the triangle that  

```theta_3 = phi_3 + 180```  

### Equation 6  
From the law of cosines, again, we can solve for phi_3  

``` r**2 = L3**2 + L2**2 - 2 * L3 * L2 * cos(phi_3) ```  

``` 2 * L3 * L2 * cos(phi_3) = L3**2 + L2**2 - r**2 ```  

``` phi_3 = arccos((L3**2 + L2**2 -r**2)/(2*L3*L2)) ```  

Now, we can use Equation 5  

``` theta_3 = phi_3 +180 ```  

In the code, you will notice that I manipulate this value a bit more by adding 2*pi radians to it and multiplying it by -1. This is because of the way I have my servos mounted and it required me to put an offset on them to get from joint space to motor space.  


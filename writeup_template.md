## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code.


[//]: # (Image References)

[im1]: ./misc_images/img1
[im2]: ./misc_images/img2
[im3]: ./misc_images/img3
[im4]: ./misc_images/img4
[im5]: ./misc_images/img5
[im6]: ./misc_images/img6
[im7]: ./misc_images/img7
[im8]: ./misc_images/img8

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The kinematic structure of the robotic arm was drown below:

![The kinematic structure of the robotic arm][im1]

From this schematic, a modified DH parameters were derived:

![DH paremeters table][im2]

The direction of theta anlges was obtaind from forward_kinematics demo. The values of constant DH parameters were obtained from urdf file:

d_1 = 0.44+0.42 = 0.75

a_1 = 0.35

a_2 = 1.25

a_3 = -0.054

d_4 = 0.96 + 0.54 = 1.5

d_G = 0.193 + 0.11 = 0.303



#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.
The DH parameter table was derived:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | d_1 | q_1
1->2 | - pi/2 | a_1 | 0 | -pi/2 + q_2
2->3 | 0 | a_2 | 0 | q_3
3->4 |  -pi/2 | a_3 | d_4 | q_4
4->5 | pi/2 | 0 | 0 | q_5
5->6 | -pi/2 | 0 | 0 | q_6
6->EE | 0 | 0 | d_G | 0

The values were substituted into transforomation matrices for each joitn: T0_1 ... T6_G. Every transformation matrix Ti-1_i had the following form:

![Transromation matrix][im3]

To account for the discrepancy between ROS and modified DH one needs a correction matrix:

T_corr = Rot_z(180) x Rot_y(-90)          (1)

The coordinate frame transition can be seen below:

![Coordinate frame transition][im7]

The complete transformation from the base link to the gripper link is equal to:

T0_G = T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_G*T_corr          (2)

The result of forward kinematics was tested with calc_mat.py script with points obtained from ROS simulation.

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.
In this task one obtains the end effector position and orientation.

##### Inverse position kinematics
First we need to find the position of the spherical wrist center from end effector position and its orientation. To do so, one has to change end effector orientation from kwaternion form to Euler angles form with usage of appropriate function. Then w can turn euler angles into rotation matrix by multiplying three extrinsic rotation matrices:

Rrpy_gaz = RotZ(yaw) * RotY(pitch) * RotX(roll)         (3)

The first column of this matrix is a versor (vector of length 1) which is oriented along the x axis of the local coordinate frame of the gripper. To find the wrist center we have to create vector of length d_G which is oriented along x axis of the local coordinate frame. To obtain wrist center position in global coordinate frame, this vector needs to be subtracted from the vector representing position of end effector (in global coordinate frame):

Wrc = [x_g, y_g, z_g] - [l_x, l_y, l_z] x d_G         (4)

Theta 1 is a rotation in the x,y plane and can be found as:

theta1 = atan2(Wrc_y, Wrc_x)          (5)

To simplify calculations (and drawing) I moved the center of coordinate frame to joint 2. To find wrist center relative to joint 2 the following equation was applied:

Wrc2 =  [ sqrt(Wrc_x^2+Wrc_y^2)-0.35,    0,     Wrc_z-0.75])          (6)

![Kinematics of wrist center][im5]

The drawing for angle derivation and resulting equations are shown below:

![Angle derivation for theta 2 and 3][im6]

##### Inverse orientation kinematics

The complete rotation from base link to gripper link in gazebo notation can be written as:

Rrpy_gaz = R0_3 x R3_G * R_corr          (7)

To find the R3_G matrix we pre-multiply by inverse of R0_3 evaluated with previously found theta 1-3 and post-multiply with inverse of R_corr matrix evaluated before:

R3_G = inv(R0_3)* Rrpy_gaz * inv(R_corr)          (8)

Tot get rid of theta 1-3 angles in this equation we substitute thetas computed in the previous step.

One can derive the same matrix from forward kinematics as:

R3_G = T3_4*T4_5*T5_6*T6_G          (9)

Which gives:

R3_G =
![R3_G matrix][im8]

By equating (8) and (9) one can derive equations for theta 4-6 as:

theta4 = atan2( R3_6[2,2], -R3_6[0,2] )

theta5 = atan2( sqrt(R3_6[0,2]^2 + R3_6[2,2]^2), R3_6[1,2] )

theta6 = atan2( -R3_6[1,1], R3_6[1,0] )


The whole derivation was tested in IK_debug script.
For matrix inversion, a method using the adjugate matrix and a determinant was used as it gave better result:

End effector error for x position is: 0.00000000

End effector error for y position is: 0.00000000

End effector error for z position is: 0.00000000

Overall end effector offset is: 0.00000000 units

The results were better for all 3 cases when inverse_ADJ instead of inverse_LU was used. Script 'inversion_problem.py' shows how little numerical errors in matrix can lead to totally different and erroneous inversion results when using LU factorization method.  

The IK_debug.py script took around 40s to compute. Therefor metohd needs some optimization before using it in inverse kinematics service.

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.

The script IK_debug.py was used to compute and print matrices needed for the analysis:
- Rrpy_gaz (representing rotation derived from Euler angles)
- R0_3 (representing rotation from base link to link 3)
- R_corr_inv (representig the rotation from ROS gripper coordinate frame to modified DH notation)
The precomputed matrices were hardcoded in the script.

The rest of the code follow procedure show above.

Running the simulation proved that IK_server.py allows to pick and place the cylinder in almost any case. After every test there was a need to restart computer for the simulation to work properly. In my opinion the trajectory planner should be improved remove curly artifacts in the trajectory.

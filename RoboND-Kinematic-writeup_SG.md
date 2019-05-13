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

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[image4]: ./misc_images/Kuka_Xacro.png
[image5]: ./misc_images/Kuka_DH.jpg
[image6]: ./misc_images/DH_Caln_Graphical.jpg
[image7]: ./misc_images/Theta1.jpg
[image8]: ./misc_images/Theta2.jpg
[image9]: ./misc_images/Theta3.jpg
[image10]: ./misc_images/Theta_4_5_6.jpg

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Following snapshot shows Xacro file from where Kua robot's DH parameters are being picked up

![alt text][image4]

Following snapshot shows drived table from Xacro / URDF

![alt text][image5]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 | - pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | q7

Following snapshot shows graphical calculation & DH table derivation

![alt text][image6]

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Theta 1 Calculation

![alt text][image7]

Theta 2 Calculation

![alt text][image8]

Theta 3 Calculation

![alt text][image9]

Theta 4,5 & 6 Calculation

![alt text][image10]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  

1. Forward Kinematics implemeted as compositions of succesive transformations
    ```
    #Composition - Forward
    T0_2 = (T0_1 * T1_2)
    T0_3 = (T0_2 * T2_3)
    T0_4 = (T0_3 * T3_4)
    T0_5 = (T0_4 * T4_5)
    T0_6 = (T0_5 * T5_6)
    T0_7 = (T0_6 * T6_7)
    ```
2. Transformations between successive joints impemented as per DH table
    ```
    ### Homogenous Transform
	#Base link to Link1
	T0_1 = Matrix([[             cos(q1),             -sin(q1),            0,              a0],
		       [ sin(q1)*cos(alpha0),  cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
		       [ sin(q1)*sin(alpha0),  cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
		       [                   0,                    0,            0,               1]])
	T0_1 = T0_1.subs(s)

	#Link1 to Link2
	T1_2 = Matrix([[             cos(q2),             -sin(q2),            0,              a1],
		       [ sin(q2)*cos(alpha1),  cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
		       [ sin(q2)*sin(alpha1),  cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
		       [                   0,                    0,            0,               1]])
	T1_2 = T1_2.subs(s)

	#Link2 to Link3
	T2_3 = Matrix([[             cos(q3),             -sin(q3),            0,              a2],
		       [ sin(q3)*cos(alpha2),  cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
		       [ sin(q3)*sin(alpha2),  cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
		       [                   0,                    0,            0,               1]])
	T2_3 = T2_3.subs(s)

	#Link3 to Link4
	T3_4 = Matrix([[             cos(q4),             -sin(q4),            0,              a3],
		       [ sin(q4)*cos(alpha3),  cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
		       [ sin(q4)*sin(alpha3),  cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
		       [                   0,                    0,            0,               1]])
	T3_4 = T3_4.subs(s)

	#Link4 to Link5
	T4_5 = Matrix([[             cos(q5),             -sin(q5),            0,              a4],
		       [ sin(q5)*cos(alpha4),  cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
		       [ sin(q5)*sin(alpha4),  cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
		       [                   0,                    0,            0,               1]])
	T4_5 = T4_5.subs(s)

	#Link5 to Link6
	T5_6 = Matrix([[             cos(q6),             -sin(q6),            0,              a5],
		       [ sin(q6)*cos(alpha5),  cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
		       [ sin(q6)*sin(alpha5),  cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
		       [                   0,                    0,            0,               1]])
	T5_6 = T5_6.subs(s)

	#Link6 to Gripper
	T6_7 = Matrix([[             cos(q7),             -sin(q7),            0,              a6],
		       [ sin(q7)*cos(alpha6),  cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
		       [ sin(q7)*sin(alpha6),  cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
		       [                   0,                    0,            0,               1]])
	T6_7 = T6_7.subs(s)
    ```
3. Parameter table calculated / derived first - Using URDF files (Xacro files), which describes different parameters of the Kuka arm
4. DH table calculated as per the guidelines of the Udacity - Which was fun
    ```
    # Create Modified DH parameters
    s = {alpha0:    0,  a0:     0,  d1:     0.75,
	     alpha1:-hfPi,  a1:  0.35,  d2:        0,   q2: q2-hfPi,
	     alpha2:    0,  a2:  1.25,  d3:        0,
	     alpha3:-hfPi,  a3:-0.054,  d4:      1.5,
	     alpha4: hfPi,  a4:     0,  d5:        0,
	     alpha5:-hfPi,  a5:     0,  d6:        0,
	     alpha6:    0,  a6:     0,  d7:    0.303,   q7: 0}	
    ```
5. Inverse kinematics, first divided into 2 seperate problem of Wrist center position due to joint 1,2 & 3. And rotations (Spherical wrist) due to joint 4,5 & 6
   - Dividing this way helps solve the IK using analytic solution
6. Theta 1,2 & 3 calculated using trigonometry - Cosine Laws. Joint 3 was harder to visulize, where I needed some help.
    ```
    # Calculate joint angles using Geometric IK method
    wcx = px - (d7).subs(s) * Rrpy[0,2]
    wcy = py - (d7).subs(s) * Rrpy[1,2]
    wcz = pz - (d7).subs(s) * Rrpy[2,2]

    theta1 = atan2(wcy, wcx)
 
    r_2 = sqrt(wcx * wcx + wcy * wcy) - (a1).subs(s)
    s_2 = wcz - (d1).subs(s)

    sideB = sqrt(r_2 * r_2 + s_2 * s_2)
    sideA = sqrt((d4).subs(s) * (d4).subs(s) + (a3).subs(s) * (a3).subs(s))
    sideC = (a2).subs(s)

    #Using cosine rule
    angleA = acos((sideB * sideB + sideC * sideC - sideA * sideA) / (2 * sideB * sideC))
    theta2 = (pi/2) - ((angleA) + atan2(s_2, r_2))

    angleDef = atan2(-(a3).subs(s), (d4).subs(s))
    angleB = acos((sideA * sideA + sideC * sideC - sideB * sideB) / (2 * sideA * sideC))
    theta3 = (pi/2) - angleB - angleDef
    ```
7. Theta 4,5 & 6 calculated as per 
    ```
    #Calculate R0_3
    R0_3 = T0_3[0:3,0:3]
    R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

    R3_6 = R0_3.inv("LU") * Rrpy[0:3,0:3]

    #Calculate theta 4,5 & 6
    theta5 = atan2(sqrt(R3_6[0,2] * R3_6[0,2] + R3_6[2,2] * R3_6[2,2]),R3_6[1,2])
    theta4 = atan2(R3_6[2,2],-R3_6[0,2])
    theta6 = atan2(-R3_6[1,1], R3_6[1,0])
    ```

NOTE:
> Error between calculated EE position & received position displayed on the terminal

Observation & Improvements:
1. Due to possibility of multiple solutions, Theta 4,5 & 6 usually affecting end effector precise calculation
2. Iteratively error is adding up in EE calculation
3. Error correction mechanism as per actual received EE location & one calculated through Forward Kinematics, might be able to improve the error

And just for fun, another example image:
![alt text][image3]



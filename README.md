## Project: Kinematics Pick & Place
Welcome to the implementation of Autonomous Pick and Place operation!

[//]: # (Image References)
[image1]: ./misc_images/AutonomousPickAndPlace.png
[image2]: ./misc_images/DHParameterSupportingImage.jpeg
[image3]: ./misc_images/GetWC.png
[image4]: ./misc_images/IKCalc.jpeg
[image5]: ./misc_images/IKFormula.png

![][image1]

---


**Steps to review the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/saurabdixit/RoboND-Kinematics-Project.git) into the ***src*** directory of your ROS Workspace.  
3. Perform catkin_make after cloning.
4. After catkin_make, run following command:

   ```Shell
   rosrun kuka_arm safe_spawnder.sh
   ```

   Note: IK_Server.py will run automatically once you run above command. I have added the IK_server node in inverse_kinematics.launch.

   In code, I am checking whether the "demo" parameter value is set to TRUE or FALSE. Based on the value, I am making the decision whether to initialize the node or not.
   ```python
   def IK_server():
       # initialize node and declare calculate_ik service
       if rospy.get_param('/trajectory_sampler/demo'):
           print '-------------------------------------------------------------------------------'
           print 'Not starting IK Server as the /trajectory_sampler/demo parameter is set to TRUE'
           print '-------------------------------------------------------------------------------'
           pass
       else:
           print '-------------------------------------------------------------------------------'
           print '/trajectory_sampler/demo parameter is set to FALSE. Initiating IK Server-------'
           print '-------------------------------------------------------------------------------'
           rospy.init_node('IK_server')
           s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
           print "Ready to receive an IK request"
           rospy.spin()
   ```

5. Once the environment is loaded, please continue clicking next to see the robot performing autonomous pick and place



## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
This section documents the process of Kinematic Analysis and project implementation.

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.
Here is the Joint diagram for Kr210 robot.

![alt text][image2]

From the image above, the DH parameter table can be derived as follows:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
---   | --- 	   | --- 	| --- 	 | ---
0->1  | 0 		   | 0 		| d1 	 | theta1
1->2  | - pi/2     | a1		| 0 	 | theta2-pi/2
2->3  | 0  		   | a2 	| 0 	 | theta3
3->4  | -pi/2 	   | a3		| d4 	 | theta4
4->5  | pi/2 	   | 0 		| 0 	 | theta5
5->6  | -pi/2 	   | 0 		| 0 	 | theta6
6->EE | 0 		   | 0 		| dG 	 | theta7

In above table, we have some variable terms i.e. thetas and constant terms a1, a2, a3, d1, d4, and dG. All the thetas are the joint angles around z-axis. These are the one which will decide the position of end effector relative to robot base. Let's keep that variable in the equation so that if we need to we can get the end effector position when we plug in the value of theta. In code, we are using symbol q1,...,q7 in place of thetas. We can get values of the constant from urdf file. Here are the values:

Parameters 	| Values
---			| ---
a1			| 0.35
a2 			| 1.25
a3 			| -0.054
d1 			| 0.75
d4 			| 1.5
dG 			| 0.303

The final DH-Parameter table looks like follows:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
---   | --- 	   | --- 	| --- 	 | ---
0->1  | 0 		   | 0 		| 0.75	 | q1
1->2  | - pi/2     | 0.35	| 0 	 | q2-pi/2
2->3  | 0  		   | 1.25 	| 0 	 | q3
3->4  | -pi/2 	   | -0.054	| 1.5 	 | q4
4->5  | pi/2 	   | 0 		| 0 	 | q5
5->6  | -pi/2 	   | 0 		| 0 	 | q6
6->EE | 0 		   | 0 		| 0.303	 | q7

Let's dive into the transformation matrices used about each joint.


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

As the homogenious transformation matrices can be defined with the help of four generalized DH-parameter. I am using a function to generate those Transformation matrix with the input as DH-parameters. 

```python
def TF_Matrix(alp, a, d, q):
    TF = Matrix([
                [            cos(q),            -sin(q),            0,            a ],
                [   sin(q)*cos(alp),    cos(q)*cos(alp),    -sin(alp),  -sin(alp)*d ],
                [   sin(q)*sin(alp),    cos(q)*sin(alp),     cos(alp),   cos(alp)*d ],
                [                 0,                  0,            0,            1 ]
                    ])
	return TF
```
As we have a function now, we can populate the homogenious transformation for each link connection.

```python
T0_1 = TF_Matrix(alp0, a0, d1, q1).subs(DH_tab)
T1_2 = TF_Matrix(alp1, a1, d2, q2).subs(DH_tab)
T2_3 = TF_Matrix(alp2, a2, d3, q3).subs(DH_tab)
T3_4 = TF_Matrix(alp3, a3, d4, q4).subs(DH_tab)
T4_5 = TF_Matrix(alp4, a4, d5, q5).subs(DH_tab)
T5_6 = TF_Matrix(alp5, a5, d6, q6).subs(DH_tab)
T6_E = TF_Matrix(alp6, a6, d7, q7).subs(DH_tab)
T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_E

```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.
We are getting the End-Effector(EE) position and Rotation in roll, pitch, and yaw from the scene. We need to generate rotation matrix using roll, pitch, and yaw and compensate for rotation discrepancy between DH parameters and Gazebo. 
Initially, We need to identify Wrist Center(WC) so that we can solve two different equations for joints 0_3 and 3_6 to reduce the calculation time.
We can identify WC using following equation.

![alt text][image3]

Once we get the WC, we can find theta1, theta2, and theta3 using following method:

![alt text][image4]

From above calculation, we can find the values of all joint angles.


### Project Implementation
I have updated IK_server.py with the Forward Kinematics and inverse kinematics that we have discussed previously. I am populating the matrices before the for loop so that we can save some computation. The only calculation that are performed inside the for loop are dependent on wrist position and orientation that is sent when rosservice call is made.

I am not using Inverse for the calculation of R3_6.
![alt text][image5]

I am using transpose instead of Inverse.
```python
R3_6 = R0_3.T * ROT_EE
```

The reason I am mentioning this is because I was getting some inconsistent results before making this change.









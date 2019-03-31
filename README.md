## Project: Kinematics Pick & Place
Welcome to the implementation of Autonomous Pick and Place operation!

[//]: # (Image References)
[image1]: ./misc_images/AutonomousPickAndPlace.png
[image2]: ./misc_images/DHParameterSupportingImage.jpeg
[image3]: ./misc_images/misc2.png

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
0->1  | 0 		   | 0 		| d1 	 | qi
1->2  | - pi/2     | a2 	| 0 	 | -pi/2 + q2
2->3  | 0  		   | 0 		| 0 	 | 0
3->4  | 0 		   | 0 		| 0 	 | 0
4->5  | 0 		   | 0 		| 0 	 | 0
5->6  | 0 		   | 0 		| 0 	 | 0
6->EE | 0 		   | 0 		| 0 	 | 0

We need to get the parameter values of d1, d4, dG, a2, and a3. We got following values from urdf file:
Parameters | Values
d1 | 
a2 |
a3 |
d4 |
dG | 



#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.



#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]



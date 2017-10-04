/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

//Visulaization tools
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <stdio.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>

#include <math.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <cmath>
//custom message
#include <cool400_tests/joint_control.h>

//variable for getting joint_values of robot
float joint_val[6];

//variable for storing safety limits for each joints
double max[6] = {3.14,0,3.14,2.09,1.57,3.14};
double min[6] = {-3.14,-3.14,0,-2.09,-1.57,-3.14};

//Safety limit check function
bool safety_check(int joint, double position)
{
  if ((position>=min[joint]) && (position<=max[joint]))
    return true;
  else
    return false;
}

//Subscriber callback function for publishing values
void joint_states_Callback(const sensor_msgs::JointState::ConstPtr& jointstates)
{
  for(int i=1;i<=6;i++)
  {
   
   if(i<=6)
   {
	  joint_val[i] = jointstates->position[i];
   }
  }
  }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "lnt_position_control");
  ros::NodeHandle node_handle;
  ros::NodeHandle n;
  
  //Subscriber for joint positions
  ros::Subscriber sub = node_handle.subscribe("joint_states", 1000, joint_states_Callback);
  
  //Publisher for individual joint states
  ros::Publisher joint1 = n.advertise<cool400_tests::joint_control>("/joint1_controller/command", 1);
  
  //Create a message for joint commands
  cool400_tests::joint_control msg;
 
  //spinner thread for subscribers
  ros::AsyncSpinner spinner(3);
  spinner.start();
  
   //Define the planning group 
  static const std::string PLANNING_GROUP = "manipulator";

  //Invoke the movegroup interface for the planning group which we want
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup *joint_model_group =
  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  
  // Visualization
  // ^^^^^^^^^^^^^
  // Adding visualization tools
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();
  
  
  // Rviz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 0.75; // above head of PR2
  visual_tools.publishText(text_pose, "L&T Manipulator", rvt::WHITE, rvt::XLARGE);
  
  // Batch publishing is used to reduce the number of messages being sent to Rviz for large visualizations
  visual_tools.trigger();
  
  int packt;
  bool success;
  
  //Packet code
  std::cout<<"Enter packet code";
  std::cin>>packt;
  
  //Packet data:5 - Publishing individual joint commands
  if(packt == 5)
  {
	  //variable for storing joint angles(radians) 
	  std_msgs::Float64 position; 
	  
	  //variable for storing the joint number
	  int joint_num;
	  
	  std::cout<<"Enter the joint number and Angles(Degrees):";
	  std::cin>>joint_num>>position.data;
	  
	  //Radians Conversion
	  position.data = (position.data *3.14)/180;
	  
	  //Check safety limits and execute
	 if(safety_check(joint_num,position.data))  
     {
      //Create a robotstate object and store the current state information(position/accleration/velocity)
      moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  
      // Next get the current set of joint values for the group.
      std::vector<double> joint_group_positions;
      current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

      // Modify the joint state accordingly
      joint_group_positions[joint_num] = position.data;  // radians
      move_group.setJointValueTarget(joint_group_positions);
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      success = move_group.plan(my_plan);
      ROS_INFO_NAMED("Executing joint space goal %s", success ? "" : "FAILED");

      // Visualize the plan in Rviz
      visual_tools.deleteAllMarkers();
      visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
      visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
      visual_tools.trigger();
	  
	  //Execute in real hardware
	  move_group.move(); 
	 }
	 else
	 {
		 std::cout<<"Value:Out of range";	 
    }
   }
   //Packet data:2 - Publishing multiple joint commands at same time
   else if(packt==6)
   {
	 std_msgs::Float64 position[6];
	 std::cout<<"Enter Angles(Degrees) for each joint successively";
	 std::cin>>position[0].data>>position[1].data>>position[2].data>>position[3].data>>position[4].data>>position[5].data;
	 
	 for(int i=0;i<6;i++)
	 {
	  position[i].data = (position[i].data *3.14)/180;
     } 
	 
		 
	  //Create a robotstate object and store the current state information(position/accleration/velocity)
      moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  
      // Next get the current set of joint values for the group.
      std::vector<double> joint_group_positions;
      current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

      // Modify the joint state accordingly
      for(int i=0;i<6;i++)
      {
	   if(safety_check(i,position[i].data))
	   {	  
        joint_group_positions[i] = position[i].data;
	   }
	   else
	   {
		   std::cout<<"Joint"<<i<<" out of range"<<std::endl;
       }
      }
      move_group.setJointValueTarget(joint_group_positions);
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      success = move_group.plan(my_plan);
      ROS_INFO_NAMED("Executing joint space goal(Multiple Joints) %s", success ? "" : "FAILED");

      // Visualize the plan in Rviz
      visual_tools.deleteAllMarkers();
      visual_tools.publishText(text_pose, "Joint Space Goal(Multiple)", rvt::WHITE, rvt::XLARGE);
      visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
      visual_tools.trigger();
	  
	  //Execute in real hardware
	  move_group.move();  
	  }
   
  else if(packt==1 || packt ==2)
  { 
   char mode;  
   float x,y,z,alpha,beta,gama;
   float r,theta;
  
	if(packt==1)
	{
          std::cout<<"Enter x-increment:";
		  std::cin>>x;
		  std::cout<<"Enter y-increment:";
		  std::cin>>y;
		  std::cout<<"Enter z-increment:";
		  std::cin>>z;
		  std::cout<<"Enter roll increment:";
		  std::cin>>alpha;
		  std::cout<<"Enter pitch increment:";
		  std::cin>>beta;
		  std::cout<<"Enter yaw increment:";
		  std::cin>>gama;	
		  
		  //Input endeffector mode(none/orientation/position)
	      std::cout<<"Enter end_effector mode";
	      std::cin>>mode;
	  }
	  else
	  {
		  std::cout<<"Enter r:";
		  std::cin>>r;
		  std::cout<<"Enter theta:";
		  std::cin>>theta;
		  std::cout<<"Enter pi:";
		  std::cin>>z;
		  //Cylindrical to cartesian conversion
		  x= r*cos(theta);
	      y= r*sin(theta);
	      mode = '1';
	   }
	
	
	
	// Planning to a start position
    geometry_msgs::Pose start_pose;
    start_pose.position.x = 0.0309533713758;
    start_pose.position.y =-0.708873151926;
    start_pose.position.z =   0.392867713827; 
    start_pose.orientation.x =  0.500662419524;
    start_pose.orientation.y =  0.424772096872;
    start_pose.orientation.z = -0.506079028547;
    start_pose.orientation.w =  0.559276160998;
     
    move_group.setPoseTarget(start_pose);

    //Creating a plan object
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
   
    //planning to the corresponding setpose target
    bool success = move_group.plan(my_plan);
     
    ROS_INFO_NAMED("Visualizing Start Pose %s", success ? "" : "FAILED");

    // Visualizing plans
    // ^^^^^^^^^^^^^^^^^
    // We can also visualize the plan as a line with markers in Rviz.
    ROS_INFO("Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(start_pose, "Start_pose");
    visual_tools.publishText(text_pose, "Start_pose", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
         

    //Executing in the real hardware
    move_group.move(); 
    
    //End Effector Mode selection
   	switch(mode){
		case '1':
		{
		    
	   //Planning with orientation Constraints
        moveit_msgs::OrientationConstraint ocm;
		ocm.link_name = "link_6";
		ocm.header.frame_id = "base_link";
		ocm.orientation.x =   0.500662419524;
		ocm.orientation.y = 0.424772096872;
		ocm.orientation.z = -0.506079028547;
		ocm.orientation.w = 0.559276160998;
		ocm.absolute_x_axis_tolerance = 0.1;
		ocm.absolute_y_axis_tolerance = 0.1;
		ocm.absolute_z_axis_tolerance = 0.1;
		ocm.weight = 0.5;

		//Now, set it as the path constraint for the group.
		moveit_msgs::Constraints test_constraints;
		test_constraints.orientation_constraints.push_back(ocm);
		move_group.setPathConstraints(test_constraints);
	
	    //Get the current state and set it as start state -- (For Actual Hardware)
		//robot_state::RobotState start_state(*move_group.getCurrentState());
	    //move_group.setStartState(start_state);
     
		geometry_msgs::Pose target_pose;
		target_pose.position.x = start_pose.position.x+x;
		target_pose.position.y = start_pose.position.y+y;
		target_pose.position.z =   start_pose.position.z+z; 
		target_pose.orientation.x =   0.500662419524;
		target_pose.orientation.y =  0.424772096872;
		target_pose.orientation.z = -0.506079028547;
		target_pose.orientation.w =  0.559276160998;
		
		move_group.setPoseTarget(target_pose);
		move_group.setPlanningTime(5.0);
			
		success = move_group.plan(my_plan);
	
   		ROS_INFO_NAMED("Visualizing orientation (constraints) %s", success ? "" : "FAILED");
    
		// Visualize the plan in Rviz
		visual_tools.deleteAllMarkers();
		visual_tools.publishAxisLabeled(start_pose, "start");
		visual_tools.publishAxisLabeled(target_pose, "goal");
		visual_tools.publishText(text_pose, "Orientation constrained Goal", rvt::WHITE, rvt::XLARGE);
		visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
		visual_tools.trigger();
  
 
		//Executing in real robot
		move_group.move(); 
		
  
		// Clearing path constraint
		move_group.clearPathConstraints();
	    }
		break;
	    
	    case '0':
	    {
		  //Assigning the start pose orientation into an quaternion 	
		  Eigen::Matrix3f mat1;
          Eigen::Quaternionf q1(mat1);
          q1.x()=	start_pose.orientation.x;
          q1.y()=	start_pose.orientation.y;
          q1.z()=	start_pose.orientation.z;
          q1.w()=	start_pose.orientation.w;
			
		  //Converting quaternion to euler angles
		  auto euler = q1.toRotationMatrix().eulerAngles(0,1,2);
          //std::cout << "Euler from quaternion in roll, pitch, yaw"<< std::endl << euler << std::endl;
          
          alpha= alpha+euler[0];
          beta= beta+euler[2];
          gama=gama+euler[1];
          
	     		  
		  //Converting Euler angle representation to Quaternion
		  Eigen::Matrix3f mat;
          Eigen::Quaternionf q(mat);
          q = Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitX())* Eigen::AngleAxisf(beta,  Eigen::Vector3f::UnitY())* Eigen::AngleAxisf(gama, Eigen::Vector3f::UnitZ());
	      
          //Apply the corresponding incrementation
		  geometry_msgs::Pose target_pose;
		  target_pose.position.x = start_pose.position.x+x;
		  target_pose.position.y = start_pose.position.y+y;
		  target_pose.position.z =   start_pose.position.z+z; 
		  target_pose.orientation.x =  q.x();
		  target_pose.orientation.y = q.y();
		  target_pose.orientation.z = q.z();
		  target_pose.orientation.w =  q.w();
		  
		  				  
		  //Creating a plan object
		  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		  
		  move_group.setPoseTarget(target_pose);
		  move_group.setPlanningTime(2.0);
   
		  //planning to the corresponding setpose target
		  bool success = move_group.plan(my_plan);
     
		  ROS_INFO_NAMED("Visualizing plan (unconstrained goal) %s", success ? "" : "FAILED");

		  // Visualizing plans
          // ^^^^^^^^^^^^^^^^^
          // We can also visualize the plan as a line with markers in Rviz.
          ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
		  visual_tools.publishAxisLabeled(start_pose, "Unconstrained_pose");
		  visual_tools.publishText(text_pose, "Start_pose", rvt::WHITE, rvt::XLARGE);
          visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
          visual_tools.trigger();

     	   //Executing in the real hardware
           move_group.move(); 
          }
		  break;
		  
		case '2':
		{
		  //Assigning the start pose orientation into an quaternion 	
		  Eigen::Matrix3f mat1;
          Eigen::Quaternionf q1(mat1);
          q1.x()=	start_pose.orientation.x;
          q1.y()=	start_pose.orientation.y;
          q1.z()=	start_pose.orientation.z;
          q1.w()=	start_pose.orientation.w;
			
		  //Converting quaternion to euler angles
		  auto euler = q1.toRotationMatrix().eulerAngles(0,1,2);
          //std::cout << "Euler from quaternion in roll, pitch, yaw"<< std::endl << euler << std::endl;
          
          alpha= alpha+euler[0];
          beta= beta+euler[2];
          gama=gama+euler[1];
          
	     		  
		  //Converting Euler angle representation to Quaternion
		  Eigen::Matrix3f mat;
          Eigen::Quaternionf q(mat);
          q = Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitX())* Eigen::AngleAxisf(beta,  Eigen::Vector3f::UnitY())* Eigen::AngleAxisf(gama, Eigen::Vector3f::UnitZ());
	     		  
		  //Set Position constraints
		  moveit_msgs::PositionConstraint pcm;
	      pcm.link_name = "link_6";
	      pcm.header.frame_id = "base_link";
          pcm.target_point_offset.x =  0.0309533713758;
	      pcm.target_point_offset.y =  -0.708873151926;
	      pcm.target_point_offset.z =   0.392867713827; 
	      pcm.weight = 1.0;
	
	      moveit_msgs::Constraints pos_constraints;
	      pos_constraints.position_constraints.push_back(pcm);
	      move_group.setPathConstraints(pos_constraints);
	          
		  
		  geometry_msgs::Pose target_pose;
		  target_pose.position.x = start_pose.position.x;
		  target_pose.position.y = start_pose.position.y;
		  target_pose.position.z =   start_pose.position.z; 
		  target_pose.orientation.x =  q.x();
		  target_pose.orientation.y = q.y();
		  target_pose.orientation.z = q.z();
		  target_pose.orientation.w = q.w();
		  
		  				  
		  //Creating a plan object
		  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		  
		  move_group.setPoseTarget(target_pose);
		  move_group.setPlanningTime(4.0);
   
		  //planning to the corresponding setpose target
		  bool success = move_group.plan(my_plan);
     
		  ROS_INFO_NAMED("Visualizing plan 1 (pose constrained goal) %s", success ? "" : "FAILED");

   	      //Executing in the real hardware
          move_group.move(); 
         }
         break;
		
		default:
		  std::cout<<"Invalid input";
	  }  
	}
	else
	  std::cout<<"Invalid Input";

  ros::Rate loop_rate(2);
  loop_rate.sleep(); 
  ros::shutdown();
  return 0;
}

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
double max[7] = {0,250950,250950,0,101265,75950,151900};
double min[7] = {0,-250950,0,-250950,-101265,-75950,-151900};
//variable for getting joint number and angle


//Safety limit check function
bool safety_check(int joint, double position)
{
  if ((position>min[joint]) && (position<max[joint]))
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
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in Rviz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();
  
  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in Rviz
  visual_tools.loadRemoteControl();

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
  
  //Packet data:1 - Publishing individual joint commands
  if(packt == 5)
  {
	  std_msgs::Float64 position; 
	  int joint_num;
	  std::cout<<"Enter the joint number and position(radians):";
	  std::cin>>joint_num>>position.data;
	  	  
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
   
   //Packet data:2 - Publishing multiple joint commands at same time
   else if(packt==6)
   {
	 std_msgs::Float64 position[6];
	 std::cout<<"Enter position for each joint successively";
	 std::cin>>position[0].data>>position[1].data>>position[2].data>>position[3].data>>position[4].data>>position[5].data;
	 
		 
	  //Create a robotstate object and store the current state information(position/accleration/velocity)
      moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  
      // Next get the current set of joint values for the group.
      std::vector<double> joint_group_positions;
      current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

      // Modify the joint state accordingly
      for(int i=0;i<6;i++)
      {
       joint_group_positions[i] = position[i].data;
      }
      
      
      
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
  
  
  else if(packt==1 || packt ==2)
  { 
  char mode;  
  float x,y,z,alpha,beta,gama;
  
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
	  }
	  else
	  {
		  std::cout<<"Enter alpha:";
		  std::cin>>alpha;
		  std::cout<<"Enter beta:";
		  std::cin>>beta;
		  std::cout<<"Enter gamma:";
		  std::cin>>gama;
		  
		  //Cylindrical to cartesian conversiom
		  x= alpha*cos(beta);
	      y= alpha*sin(beta);
	      z=z;
	   }
	//Input endeffector mode(none/orientation/position)
	std::cout<<"Enter end_effector mode";
	std::cin>>mode;
	
	
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
     
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // Visualizing plans
    // ^^^^^^^^^^^^^^^^^
    // We can also visualize the plan as a line with markers in Rviz.
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(start_pose, "pose1");
    visual_tools.publishText(text_pose, "Start_pose", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
         

    //Executing in the real hardware
    move_group.move(); 
    ROS_INFO("Reached Start_position");
	
	switch(mode){
		case '1':
		{
		    
	   // Planning with orientation Constraints
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

		// Now, set it as the path constraint for the group.
		moveit_msgs::Constraints test_constraints;
		test_constraints.orientation_constraints.push_back(ocm);
		move_group.setPathConstraints(test_constraints);
	
	    //Get the current state and set it as start state
		robot_state::RobotState start_state(*move_group.getCurrentState());
	
		geometry_msgs::Pose target_pose;
		target_pose.position.x = start_pose.position.x+x;
		target_pose.position.y = start_pose.position.y+y;
		target_pose.position.z =   start_pose.position.z+z; 
		target_pose.orientation.x =   0.500662419524;
		target_pose.orientation.y =  0.424772096872;
		target_pose.orientation.z = -0.506079028547;
		target_pose.orientation.w =  0.559276160998;
		move_group.setStartState(start_state);
     
		move_group.setPoseTarget(target_pose);
		move_group.setPlanningTime(5.0);
			
		success = move_group.plan(my_plan);
	
   		ROS_INFO_NAMED("Visualizing orientation (constraints) %s", success ? "" : "FAILED");
    
		// Visualize the plan in Rviz
		visual_tools.deleteAllMarkers();
		visual_tools.publishAxisLabeled(start_pose, "start");
		visual_tools.publishAxisLabeled(target_pose, "goal");
		visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
		visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
		visual_tools.trigger();
  
 
		//Executing in real robot
		move_group.move(); 
		ROS_INFO("Reached Constrained_goal_position");
  
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
			
		  //converting quaternion to euler angles
		  auto euler = q1.toRotationMatrix().eulerAngles(0,1,2);
          std::cout << "Euler from quaternion in roll, pitch, yaw"<< std::endl << euler << std::endl;
          
          alpha= alpha+euler[0];
          beta= beta+euler[1];
          gama=gama+euler[2];
          
	     		  
		  //Converting Euler angle representation to Quaternion
		  Eigen::Matrix3f mat;
          Eigen::Quaternionf q(mat);
          q = Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitX())* Eigen::AngleAxisf(beta,  Eigen::Vector3f::UnitY())* Eigen::AngleAxisf(gama, Eigen::Vector3f::UnitZ());
	      std::cout<<std::endl<<q.x()<<std::endl<<q.y()<<std::endl<<q.z()<<std::endl<<q.w();

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
     
		  ROS_INFO_NAMED("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

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
           ROS_INFO("Reached the Position without constraints");
		  
		  
		 }
		  break;
		  
		case '2':
		{
		 
		  
		  //Converting Euler angle representation to Quaternion
		  Eigen::Matrix3f mat;
          Eigen::Quaternionf q(mat);
          q = Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitX())* Eigen::AngleAxisf(beta,  Eigen::Vector3f::UnitY())* Eigen::AngleAxisf(gama, Eigen::Vector3f::UnitZ());
	      std::cout<<std::endl<<q.x()<<std::endl<<q.y()<<std::endl<<q.z()<<std::endl<<q.w();
		  
		  //Set Position constraints
		  moveit_msgs::PositionConstraint pcm;
	      pcm.link_name = "link_6";
	      pcm.header.frame_id = "base_link";
          pcm.target_point_offset.x =  0.0309533713758;
	      pcm.target_point_offset.y =  -0.708873151926;
	      pcm.target_point_offset.z =   0.392867713827; 
	      pcm.weight = 0.5;
	
	      moveit_msgs::Constraints pos_constraints;
	      pos_constraints.position_constraints.push_back(pcm);
	      move_group.setPathConstraints(pos_constraints);
	      
	      
		  
		  geometry_msgs::Pose target_pose;
		  target_pose.position.x = start_pose.position.x;
		  target_pose.position.y = start_pose.position.y;
		  target_pose.position.z =   start_pose.position.z; 
		  target_pose.orientation.x =  start_pose.orientation.x+q.x();
		  target_pose.orientation.y = start_pose.orientation.y+q.y();
		  target_pose.orientation.z = start_pose.orientation.z+q.z();
		  target_pose.orientation.w =  start_pose.orientation.w;
		  
		  				  
		  //Creating a plan object
		  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		  
		  move_group.setPoseTarget(target_pose);
		  move_group.setPlanningTime(2.0);
   
		  //planning to the corresponding setpose target
		  bool success = move_group.plan(my_plan);
     
		  ROS_INFO_NAMED("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

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
		default:
		  std::cout<<"Invalid input";
	  }  
			  
   }

else if(packt==32)
{
	// Planning to a home position
    geometry_msgs::Pose start_pose;
    start_pose.position.x = 0.0309533713758;
    start_pose.position.y =-0.708873151926;
    start_pose.position.z =   0.392867713827; 
    start_pose.orientation.x =   0.500662419524;
    start_pose.orientation.y =  0.424772096872;
    start_pose.orientation.z = -0.506079028547;
    start_pose.orientation.w =  0.559276160998;
     
    move_group.setPoseTarget(start_pose);

    //Creating a plan object
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
   
    //planning to the corresponding setpose target
    bool success = move_group.plan(my_plan);
     
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // Visualizing plans
    // ^^^^^^^^^^^^^^^^^
    // We can also visualize the plan as a line with markers in Rviz.
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(start_pose, "pose1");
    visual_tools.publishText(text_pose, "Start_pose", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
         

    //Executing in the real hardware
    move_group.move(); 
    ROS_INFO("Reached Start_position");
	

	moveit_msgs::PositionConstraint pcm;
	pcm.link_name = "link_6";
	pcm.header.frame_id = "base_link";
	pcm.target_point_offset.x =  0.0309533713758;
	pcm.target_point_offset.y =  -0.708873151926;
	pcm.target_point_offset.z =   0.392867713827; 
	pcm.weight = 0.5;
	
	moveit_msgs::Constraints pos_constraints;
	pos_constraints.position_constraints.push_back(pcm);
	move_group.setPathConstraints(pos_constraints);
	
	robot_state::RobotState start_state(*move_group.getCurrentState());
	geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.0309533713758;
    target_pose.position.y =   -0.708873151926;
    target_pose.position.z =   0.392867713827; 
    target_pose.orientation.x =  start_pose.orientation.x;
	target_pose.orientation.y = start_pose.orientation.y;
	target_pose.orientation.z = start_pose.orientation.z;
	target_pose.orientation.w = 1;
    move_group.setStartState(start_state);
     
    //Creating a plan object
    //moveit::planning_interface::MoveGroupInterface::Plan my_plan; 
     
    move_group.setPoseTarget(target_pose);
    move_group.setPlanningTime(5.0);
    //bool success;
    success = move_group.plan(my_plan);
    
     ROS_INFO_NAMED("Visualizing Position (constraints) %s", success ? "" : "FAILED");
    
    // Visualize the plan in Rviz
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(start_pose, "start");
    visual_tools.publishAxisLabeled(target_pose, "goal");
    visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
  
 
  //Exexuting in real robot
    
   ROS_INFO("Reached Constrained_goal_position");
  
  // When done with the path constraint be sure to clear it.
  move_group.clearPathConstraints();
    
    move_group.move(); 
	
}

  ros::Rate loop_rate(2);
  loop_rate.sleep(); 
  ros::shutdown();
  return 0;
}

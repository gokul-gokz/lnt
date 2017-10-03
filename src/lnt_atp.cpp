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

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <stdio.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>

#include <math.h>
#include "Quat.h"
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
	 // ROS_INFO("joint%d : [%f]",i , jointstates->position[i]);
	  joint_val[i] = jointstates->position[i];
   }
  }
  }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "lnt_position_control");
  ros::NodeHandle node_handle;
   ros::NodeHandle n;
  
  ros::Subscriber sub = node_handle.subscribe("joint_states", 1000, joint_states_Callback);
  
  //Publisher for individual joint states
  ros::Publisher joint1 = n.advertise<cool400_tests::joint_control>("/joint1_controller/command", 1);
  
  cool400_tests::joint_control msg;
  //spinner thread for subscribers
  ros::AsyncSpinner spinner(3);
  spinner.start();
  
  //Temporary assignment of safety limits of joints
    int packt;
    bool success;
  std::cout<<"Enter packet code";
  std::cin>>packt;
  if(packt == 1)
  {
	  std_msgs::Float64 position; 
	  int joint_num;
	  std::cout<<"Enter the joint number and position";
	  std::cin>>joint_num>>position.data;
	  success = safety_check(joint_num,position.data);
	  if(success)
	  {
	   msg.joint_number = joint_num;
	   msg.position[0]   = position.data;
	   joint1.publish(msg);
   }
	 
   }
  /* switch(joint_num)
   {
	   case 1:
	       if(position.data>min[1] && position.data<max[1])
	       {
			   joint1.publish(position);
			   }
	       else
	          std::cout<<"Value out of range";
	  	   break;
	   case 2:
	       if(position.data>min[2] && position.data<max[2])
	       {
			   joint2.publish(position);
			   }
	       else
	          std::cout<<"Value out of range";
	  	   break;
	   case 3:
	       if(position.data>min[3] && position.data<max[3])
	       {
			   joint3.publish(position);
			   }
	       else
	          std::cout<<"Value out of range";
	  	   break;
	   case 4:
	       if(position.data>min[4] && position.data<max[4])
	       {
			   joint4.publish(position);
			   }
	       else
	          std::cout<<"Value out of range";
	  	   break;
	   case 5:
	      if(position.data>min[5] && position.data<max[5])
	       {
			   joint5.publish(position);
			   }
	       else
	          std::cout<<"Value out of range";
	  	   break;
	   case 6:
	       if(position.data>min[6] && position.data<max[6])
	       {
			   joint6.publish(position);
			   }
	       else
	          std::cout<<"Value out of range";
	  	   break;
	  default:
	        std::cout<<"Invalid joint number" ;	 
		} */
				
	 
	 else if(packt==2)
	 {
		 //float angle[6];
		 std_msgs::Float64 position[6];
		 std::cout<<"Enter position for each joint successively";
		 std::cin>>position[1].data>>position[2].data>>position[3].data>>position[4].data>>position[5].data>>position[6].data;
		 std::cout<<"Corresponding position"<<position[1].data<<position[2].data<<position[3].data<<position[4].data<<position[5].data<<position[6].data;
	
         for(int i=1;i<7;i++)
         {
			success=safety_check(i,position[i].data);
			std::cout<<success;
			if (success)
			    msg.position[i-1] = position[i].data;
			else
			    msg.position[i-1] = 99999;
		}	
		msg.joint_number=7;
		joint1.publish(msg);
	
	
	
	/*for (int i=1;i<7;i++)
	{	 
      switch(i)
      {
	   case 1:
	       if(position[1].data>min[1] && position[1].data<max[1])
	       {
			   joint1.publish(position[1]);
			   }
	       else
	          std::cout<<"Value out of range";
	  	   break;
	   case 2:
	       if(position[2].data>min[2] && position[2].data<max[2])
	       {
			   joint2.publish(position[2]);
			   }
	       else
	          std::cout<<"Value out of range";
	  	   break;
	   case 3:
	       if(position[3].data>min[3] && position[3].data<max[3])
	       {
			   joint3.publish(position[3]);
			   }
	       else
	          std::cout<<"Value out of range";
	  	   break;
	   case 4:
	       if(position[4].data>min[4] && position[4].data<max[4])
	       {
			   joint4.publish(position[4]);
			   }
	       else
	          std::cout<<"Value out of range";
	  	   break;
	   case 5:
	      if(position[5].data>min[5] && position[5].data<max[5])
	       {
			   joint5.publish(position[5]);
			   }
	       else
	          std::cout<<"Value out of range";
	  	   break;
	   case 6:
	       if(position[6].data>min[6] && position[6].data<max[6])
	       {
			   joint6.publish(position[6]);
			   }
	       else
	          std::cout<<"Value out of range";
	  	   break;
	  default:
	        std::cout<<"Invalid joint number" ;	 
		} 
		
	 }
 }*/
 
 
   //time for the joints to execute the command     
     //ros::Rate loop_rate(2);
	 //loop_rate.sleep(); 
 	//else
	  //  std::cout<<"Invalid packet number";
    
    
	   

}
  


  else if(packt==3)
  {
  //Define the planning group 
  static const std::string PLANNING_GROUP = "manipulator";

  //Invoke the movegroup interface for the planning group which we want
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
 
  // We can print the name of the reference frame for this robot.
  //ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  //ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
  
   // Planning to a home position
  
    geometry_msgs::Pose start_pose;
    start_pose.position.x = -0.00913318298593;
    start_pose.position.y =-0.0041092969613;
    start_pose.position.z =  0.391565521503; 
    start_pose.orientation.x =  0.00811752067547;
    start_pose.orientation.y =  0.0092369720666;
    start_pose.orientation.z = -0.705957432088;
    start_pose.orientation.w = 0.708147504609;
     
    move_group.setPoseTarget(start_pose);

    //Creating a plan object
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
   
    //planning to the corresponding setpose target
    bool success = move_group.plan(my_plan);

    //Executing in the real hardware
    move_group.move(); 
    ROS_INFO("Reached home_position");
    
   /* robot_state::RobotState start_state(*move_group.getCurrentState());
    geometry_msgs::Pose target_pose;
    target_pose.position.x = -0.0532916332999;
    target_pose.position.y =   0.0645766065631;
    target_pose.position.z =   0.372744202366; 
    target_pose.orientation.x =  0.00811752067547;
    target_pose.orientation.y =  0.0092369720666;
    target_pose.orientation.z = -0.705957432088;
    target_pose.orientation.w = 0.708147504609;*/
    

  // Planning with Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Path constraints can easily be specified for a link on the robot.
  // Let's specify a path constraint and a pose goal for our group.
  // First define the path constraint.
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "link6";
  ocm.header.frame_id = "arm_base_link";
  ocm.orientation.x =  0.00811752067547;
  ocm.orientation.y =  0.0092369720666;
  ocm.orientation.z = -0.705957432088;
  ocm.orientation.w = 0.708147504609;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 0.5;

  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);
  
    robot_state::RobotState start_state(*move_group.getCurrentState());
  
    geometry_msgs::Pose target_pose;
    target_pose.position.x = -0.0532916332999;
    target_pose.position.y =   0.0645766065631;
    target_pose.position.z =   0.372744202366; 
    target_pose.orientation.x =  -0.14811752067547;
    target_pose.orientation.y =  -0.0092369720666;
    target_pose.orientation.z = -0.705957432088;
    target_pose.orientation.w = 0.708147504609;
    move_group.setStartState(start_state);
     
    move_group.setPoseTarget(target_pose);
    move_group.setPlanningTime(15.0);
    
    
    
    success = move_group.plan(my_plan);
 
    move_group.move(); 
    ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");
    move_group.clearPathConstraints();


}

else if(packt==4)
{
	static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
     move_group.clearPathConstraints();

	moveit_msgs::PositionConstraint pcm;
	pcm.link_name = "link6";
	pcm.header.frame_id = "arm_base_link";
	pcm.target_point_offset.x =  -0.0532916332999;
	pcm.target_point_offset.y =  0.0645766065631;
	pcm.target_point_offset.z =   0.372744202366; 
	pcm.weight = 0.5;
	
	moveit_msgs::Constraints pos_constraints;
	pos_constraints.position_constraints.push_back(pcm);
	move_group.setPathConstraints(pos_constraints);
	
	robot_state::RobotState start_state(*move_group.getCurrentState());
	geometry_msgs::Pose target_pose;
    target_pose.position.x = -0.0532916332999;
    target_pose.position.y =   0.0645766065631;
    target_pose.position.z =   0.372744202366; 
    target_pose.orientation.x =  1.14811752067547;
    target_pose.orientation.y =  0.152369720666;
    target_pose.orientation.z = 0.215957432088;
    target_pose.orientation.w = 0.70847504609;
    move_group.setStartState(start_state);
     
    //Creating a plan object
    moveit::planning_interface::MoveGroupInterface::Plan my_plan; 
     
    move_group.setPoseTarget(target_pose);
    move_group.setPlanningTime(5.0);
    bool success;
    success = move_group.plan(my_plan);
    move_group.move(); 
	
}

else if(packt==5)
{
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
    
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i = 0; i < joint_names.size(); ++i)
    {
     ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
          }
          
     /* Set one joint in the right arm outside its joint limit */
    joint_values[0] =5;
    kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

   /* Check whether any joint is outside its joint limits */
    ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
    
    kinematic_state->enforceBounds();
    ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

}
else if(packt == 6)
{
	float r,z,theta;
	float x,y;
	std::cout<<"Enter cylindrical coorinates ";
	std::cin>>r>>theta>>z;
	x= r*cos(theta);
	y= r*sin(theta);
	z=z;
	std::cout<<"x="<<x<<"y="<<y<<"z="<<z;
}

/*else if (packt ==7)
{
	float r,z,theta;
	float x,y;
	std::cout<<"Enter cylindrical coorinates ";
	std::cin>>r>>theta>>z;
	Quaternion q;
	q = Eigen::AngleAxisf(r, Eigen::Vector3f::UnitX())* Eigen::AngleAxisf(theta,  Eigen::Vector3f::UnitY())* Eigen::AngleAxisf(z, Eigen::Vector3f::UnitZ());
	std::cout << "Quaternion" << std::endl << q.coeffs() << std::endl;
}*/
	
	ros::Rate loop_rate(2);
	loop_rate.sleep(); 
  ros::shutdown();
  return 0;
}

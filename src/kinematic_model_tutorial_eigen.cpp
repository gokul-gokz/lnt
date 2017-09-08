/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Sachin Chitta */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

ros::Subscriber* sub;

//Subscriber callback
std::vector<double> joint_value;
int flag;


void joint_states_Callback(const sensor_msgs::JointState::ConstPtr& jointstates)
{
  for(int i=1;i<=6;i++)
  {
   
   if(i<=6)
   {
	  ROS_INFO("joint%d : [%f]",i , jointstates->position[i]);
	  joint_value[i] = jointstates->position[i];
   }
  }
  flag=0;
  sub->shutdown(); 
  return;
 }



int main(int argc, char **argv)
{
  ros::init (argc, argv, "cool400_kinematics");
  
  ros::NodeHandle n;
  
  //Load robot model
  
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  //Create the group and the planning scene
  moveit::planning_interface::MoveGroup group("manipulator");
  
    
  //Creating a robot state and assign default values
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  
  //Create a variable current state of robot
  robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
  
  //Extracting the group and joint names of that group
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
  const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
  
  
  //Get the current state
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  kinematic_state->setToRandomPositions(joint_model_group);
  flag = 1;
  //Subscribing joint values of joints
  
  //sub = new ros::Subscriber;
  
  
 // *sub = n.subscribe("joint_states", 10, joint_states_Callback);
  
  
 for(int i=0;i<15;i++)
 { 
  joint_values[0] = -2.351592547828688+i*0.2;
  joint_values[1] = -0.8283496254582462+i*0.2;
  joint_values[2] = 1.8177672336444848+i*0.2;
  joint_values[3] = -0.26691265709210155+i*0.2;
  joint_values[4] = 0.7976700097005334+i*0.2;
  joint_values[5] = 0.4049709280018093+i*0.2;
  kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
  current_state.setJointGroupPositions(joint_model_group, joint_values);
  
  
  
 //Assigning the cool400hardware values to joint 
  //for(std::size_t i = 0; i < joint_names.size(); ++i)
  //{
    //ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  //}
  
  
  //Find the FK for current state
  
  const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("link6");
  ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
  ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());
  
  
  
  //Self collision checking
   
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;

  collision_request.contacts = true;
  collision_request.max_contacts = 1000; 

  collision_request.group_name = "manipulator";
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 3: Current state is "
                  << (collision_result.collision ? "in" : "not in")
                  << " self collision");
               
  collision_detection::CollisionResult::ContactMap::const_iterator it;
  for(it = collision_result.contacts.begin();
      it != collision_result.contacts.end();
      ++it)
  {
    ROS_INFO("Contact between: %s and %s",
             it->first.first.c_str(),
             it->first.second.c_str());
  }
  
 // Find IK if no collision occurs
  
 if(!collision_result.collision)
 
 {
  
  //IK for the state
  
   bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);

  // Now, we can print out the IK solution (if found):
  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i=0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
   }
   else
   {
     ROS_INFO("Did not find IK solution");
   } 
   
   std_msgs::Float64 msg1,msg2,msg3,msg4,msg5,msg6;
  
   ros::Publisher joint1 = n.advertise<std_msgs::Float64>("/joint1_controller/command", 1);
   
   ros::Publisher joint2 = n.advertise<std_msgs::Float64>("/joint2_controller/command", 1);
   
   ros::Publisher joint3 = n.advertise<std_msgs::Float64>("/joint3_controller/command", 1);
   
   ros::Publisher joint4 = n.advertise<std_msgs::Float64>("/joint4_controller/command", 1);
   
   ros::Publisher joint5 = n.advertise<std_msgs::Float64>("/joint5_controller/command", 1);
   
   ros::Publisher joint6 = n.advertise<std_msgs::Float64>("/joint6_controller/command", 1);
    int k=0,i;
  
    ros::Rate loop_rate(2);
	for(int i=0; i<6; i++)
	{
	   k=i*0.1;
	   
	  //ROS_INFO("k= %f",k );
	 
      msg1.data = joint_values[0];
    
      msg2.data = joint_values[1];
    
      msg3.data = joint_values[2];
    
      msg4.data = joint_values[3];
    
      msg5.data = joint_values[4];
    
      msg6.data = joint_values[5];
      
       ROS_INFO("Hi %f",msg4.data);
     
      joint1.publish(msg1);
    
      joint2.publish(msg2);
    
      joint3.publish(msg3);
    
      joint4.publish(msg4);
      
      joint5.publish(msg5);
       
      joint6.publish(msg6);

     
   
     loop_rate.sleep();
      }
    
   }
 }
  return 0;
}

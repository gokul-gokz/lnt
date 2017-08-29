#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include <ros/callback_queue.h>


#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>


nt main(int argc, char **argv)
{
  ros::init (argc, argv, "cool400_kinematics");
  
  ros::NodeHandle n;
  
  //Load robot model
  
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  //Create the group and the planning scene
  moveit::planning_interface::MoveGroup group("manipulator");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
   //Defining the goal
    
    geometry_msgs::Pose target_pose1;
    
    target_pose1.position.x = -0.146053124104;
    target_pose1.position.y = 0.0725365430437;
    target_pose1.position.z =  0.312597077414; 
    target_pose1.orientation.x = -7.94596890849e-05;
    target_pose1.orientation.y =  6.3415560941e-05;
    target_pose1.orientation.z = -0.707122605895;
    target_pose1.orientation.w = 0.707090948816;
    group.setPoseTarget(target_pose1);
    
    //Planning
    
     moveit::planning_interface::MoveGroup::Plan my_plan;
     moveit::planning_interface::MoveGroup::Plan my_plan1;
     sleep(5.0);
     bool success = group.plan(my_plan);
     sleep(5.0);
     ROS_INFO("plan success ");   
    //Visualizations
    //sleep(5.0);
    
    
    //cartesian
    //cartesian_path
    std::vector<geometry_msgs::Pose> waypoints;
    
    //target_pose3.position.x -= 0.1;
    //target_pose3.position.y -= 0.1;
    //target_pose3.position.y += 0.1;
    
    target_pose3.orientation.x += 0.1;
    target_pose3.orientation.y += 0.1;
    target_pose3.orientation.z += 0.1;
    target_pose3.orientation.w += 0.1;
    ROS_INFO("TargetPose.x-%f",target_pose3.position.x);
    waypoints.push_back(target_pose3);
    
    target_pose3.orientation.x += 0.1;
    target_pose3.orientation.y += 0.1;
    target_pose3.orientation.z += 0.1;
    target_pose3.orientation.w += 0.1;
    ROS_INFO("TargetPose.x-%f",target_pose3.position.x);
    waypoints.push_back(target_pose3);
    
    //execute
    //moveit_msgs::RobotTrajectory trajectory_msg;
    group.setPlanningTime(10.0);
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = group.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               25.0,   // jump_threshold
                                               trajectory);
    // ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
      //  fraction * 100.0);   
   
        
     

  
   // The trajectory needs to be modified so it will include velocities as well.
  // create an RobotTrajectory object
 
  
  robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "manipulator");
  
  rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);
  
   // create an IterativeParabolicTimeParameterization object
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  
  success = iptp.computeTimeStamps(rt);
  //ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
  
  rt.getRobotTrajectoryMsg(trajectory);
  
  my_plan1.trajectory_ = trajectory;
  sleep(2.0);
  
  //Executing
  group.asyncMove();
  
  ros::AsyncSpinner spinner(8);
  spinner.start();
  
   ros::waitForShutdown();
   return 0;
   
    
    

   

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

class RobotArm{
	public:
	      ros::NodeHandle n1;
	      
          double POS_TOLARENCE, ANG_TOLARENCE, PLANING_TIME;
          moveit::planning_interface::MoveGroup group;
	      boost::scoped_ptr<move_group_interface::MoveGroup> move_group_;
	      RobotArm()
	      {
             POS_TOLARENCE = 0.01;
             ANG_TOLARENCE= 0.1;
             PLANING_TIME = 10.0; 
             group::options("manipulator",);
             //move_group_->options->group_name_="manipulator";
	         move_group_->setPlanningTime(PLANING_TIME);
             move_group_->setGoalOrientationTolerance(ANG_TOLARENCE);
             move_group_->setGoalPositionTolerance(POS_TOLARENCE);
            
          } 
           void movePos()
           {
			    geometry_msgs::Pose target_pose1;
    
				target_pose1.position.x = -0.146053124104;
				target_pose1.position.y = 0.0725365430437;
				target_pose1.position.z =  0.312597077414; 
				target_pose1.orientation.x = -7.94596890849e-05;
				target_pose1.orientation.y =  6.3415560941e-05;
				target_pose1.orientation.z = -0.707122605895;
				target_pose1.orientation.w = 0.707090948816;
				group.setPoseTarget(target_pose1);
				moveit::planning_interface::MoveGroup::Plan my_plan;
				RobotArm::move_group_.plan(my_plan);
				RobotArm::move_group_.asyncMove();
			}
	   };






void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
 ros::NodeHandle node_handle1;	
 for (unsigned i = 0; i < msg->axes.size(); ++i) {
    ROS_INFO("Axis %d is now at position %f", i, msg->axes[i]);
   }
    
    //sleep(5.0);
    ROS_INFO("Before_moveit");
    moveit::planning_interface::MoveGroup group("manipulator");
    
    ROS_INFO("group created");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface; 
    ROS_INFO("planning scene ");
    //ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    //moveit_msgs::DisplayTrajectory display_trajectory;

    
    
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
   
    
    

    //Executing
    group.asyncMove();
    geometry_msgs::Pose target_pose3 = target_pose1;
  
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
  
   /* if (1)
    {
     ROS_INFO("Visualizing carftesian plan");    
     display_trajectory.trajectory_start = my_plan1.start_state_;
     display_trajectory.trajectory.push_back(my_plan1.trajectory_);
     display_publisher.publish(display_trajectory);
     sleep(5.0);
    }*/
  //ROS_INFO("executing carftesian plan-%d-%.2f%% achieved",i,fraction*100.0);
  group.asyncExecute(my_plan1);
  //ROS_INFO("Executed trajectory %s",check?"SUCCEDED":"FAILED");
  //ros::spin();
  
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Cartesian_joystick_control");
	ros::NodeHandle node_handle;
	ros::CallbackQueue joy_queue; 
	node_handle.setCallbackQueue(&joy_queue);
	joy_queue.callAvailable(ros::WallDuration());
        
    ros::Subscriber sub = node_handle.subscribe("joy", 10, joyCallback);
    
                                         
   
    //sleep(5.0);
   //ros::shutdown(); 
   ros::AsyncSpinner spinner(8, &joy_queue);
   spinner.start();
   RobotArm controller;
   controller.movePos();    
   
   ros::waitForShutdown();
    return 0;
}
     
     

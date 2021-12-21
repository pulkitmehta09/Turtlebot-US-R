/**
 * @file main.cpp
 * @author Anirudh Krishnan Komaralingam (UID: 117446432)
 * @author Pulkit Mehta (UID: 117551693)
 * @author Shubham Takbhate (UID: 118359502)
 * @brief 
 * @version 0.1
 * @date 2021-12-15
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <array>
#include <nav_msgs/Odometry.h>
#include <utility>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/*                                      DATA STRUCTURES                                     */

/**
 * @brief Struct holding information about follower pose.
 * 
 * 
 */
struct follower_pose {
  // Member 'x' contains x-coordinate of follower target location in map frame.
  double x = 0;                                                                                                                         
  // Member 'y' contains y-coordinate of follower target location in map frame.
  double y = 0;                                                                                                                         
  // Member 'fiducial_id' contains the fiducial_id of the explorer-detected aruco tag.
  unsigned short int fiducial_id = 0;                                                                                                   
};

// Follower locations
follower_pose follower_locations[5];                                                                                                    

/**
 * @brief Struct holding information about the current status/progress.
 * 
 */
struct program_status_indicator{
// Member 'current_explorer_target' contains the target number to be visited by explorer.
short int current_explorer_target = -1;                                                                                                 
// Member 'current_explorer_fiducial_id' contains the latest fiducial_id detected by explorer.                    
short int current_explorer_fiducial_id = -1;                                                                                            
// Member 'current_follower_target' contains the target number to be visited by follower.
short int current_follower_target = -1;                                                                                                 
}psi;

/*                                      FUNCTION PROTOTYPES                                     */

/**
 * @brief Method to get the explorer targets locations.
 * 
 * @param nh Nodehandle for a ROS node
 * @param arr Array holding target locations.
 */
void get_explorer_targets(ros::NodeHandle &nh, std::array<std::array<double, 2>, 5>& arr);
/**
 * @brief Method to set the next goal location for the explorer.
 * 
 * @param explorer_goal variable containing the explorer goal.
 * @param explorer_targets Array containing explorer target locations.
 */
void next_explorer_goal(move_base_msgs::MoveBaseGoal& explorer_goal, const std::array<std::array<double, 2>, 5>& explorer_targets);
/**
 * @brief Method to display the resulting locations found by the explorer.
 * 
 */
void explorer_summary();
/**
 * @brief variable containing the follower goal.
 * 
 * @param follower_goal 
 */
void next_follower_goal(move_base_msgs::MoveBaseGoal& follower_goal);

/**
 * @brief Callback function for subscriber which broadcasts a new frame of the detected aruco marker into the map.
 * 
 * @param msg Contains the ros message published to fiducial_transforms topic.
 */
void fiducial_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr &msg)
{
  //check if the marker is detected
  if (!msg->transforms.empty())
  { 
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    //broadcast the new frame to /tf Topic
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
    transformStamped.child_frame_id = "marker_frame";

    psi.current_explorer_fiducial_id = msg->transforms[0].fiducial_id;

    transformStamped.transform.translation.x = msg->transforms[0].transform.translation.x;
    transformStamped.transform.translation.y = msg->transforms[0].transform.translation.y;
    transformStamped.transform.translation.z = msg->transforms[0].transform.translation.z;
    transformStamped.transform.rotation.x = msg->transforms[0].transform.rotation.x;
    transformStamped.transform.rotation.y = msg->transforms[0].transform.rotation.y;
    transformStamped.transform.rotation.z = msg->transforms[0].transform.rotation.z;
    transformStamped.transform.rotation.w = msg->transforms[0].transform.rotation.w;

    geometry_msgs::TransformStamped anotherOne = transformStamped;
    anotherOne.header.frame_id = "marker_frame";
    anotherOne.child_frame_id = "another_frame";
    anotherOne.transform.translation.x = 0;
    anotherOne.transform.translation.y = 0;
    anotherOne.transform.translation.z = 0.4;
    anotherOne.transform.rotation.y = 0;
    anotherOne.transform.rotation.z = 0;
    anotherOne.transform.rotation.x = 0;
    anotherOne.transform.rotation.w = 1;
    
    //broadcast the transforms on /tf Topic
    br.sendTransform(transformStamped); 
    br.sendTransform(anotherOne);
  }
}

/**
 * @brief Method for localizing the marker frame in the map. 
 * 
 * @param tfBuffer Contains known stored frames
 * @param status Bool value as a flag to check status
 */
void listen(tf2_ros::Buffer &tfBuffer, bool& status)
{
  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = tfBuffer.lookupTransform("map", "another_frame", ros::Time(0));
    auto trans_x = transformStamped.transform.translation.x;
    auto trans_y = transformStamped.transform.translation.y;
    auto trans_z = transformStamped.transform.translation.z;

    ROS_INFO_STREAM("Position in map frame: ["
                    << trans_x << ","
                    << trans_y << "]");

    follower_locations[psi.current_explorer_fiducial_id].x = trans_x;
    follower_locations[psi.current_explorer_fiducial_id].y = trans_y;
    follower_locations[psi.current_explorer_fiducial_id].fiducial_id = psi.current_explorer_fiducial_id;
    status = true;
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}



/*                                      MAIN                                              */

int main(int argc, char **argv)
{
 
  // boolean to check if explorer goal is sent.
  bool explorer_goal_sent = false;                                                                    
  // boolean to check if follower goal is sent.
  bool follower_goal_sent = false;                                                                    
  // boolean to check if any more goals are remaining.
  bool no_more_exploring = false;                                                                     
  // boolean to check if the aruco marker frame is localized in map.
  bool job_done = false;                                                                              
  // boolean to determine when to start looking for aruco marker.
  bool start_looking = false;                                                                         

  // Array holding target locations of explorer. 
  std::array<std::array<double, 2>, 5> explorer_targets{};                                            
  // Array holding target locations of follower.
  std::array<std::pair<double, double>, 5> follower_targets{};                                        

  // Initiate node
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;

  // Tell the action client that we want to spin a thread by default
  MoveBaseClient explorer_client("/explorer/move_base", true);
  // Tell the action client that we want to spin a thread by default
  MoveBaseClient follower_client("/follower/move_base", true);

  // Wait for the action server to come up
  while (!explorer_client.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up for explorer");
  }

  while (!follower_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for follower");
  }

  move_base_msgs::MoveBaseGoal explorer_goal;
  move_base_msgs::MoveBaseGoal follower_goal;

  // Publisher to make the explorer rotate when it reaches a target location so as to start detection of the aruco marker.  
  ros::Publisher explorer_pub = nh.advertise<geometry_msgs::Twist>("explorer/cmd_vel", 5);            
  // Subscriber for retrieving information about aruco tag.
  ros::Subscriber sub = nh.subscribe("/fiducial_transforms", 5, &fiducial_callback);                  

  get_explorer_targets(nh, explorer_targets);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    // Check if explorer needs to move to a target location.
    if (!no_more_exploring){
      // Check if a goal is sent to explorer.
      if (!explorer_goal_sent){
        next_explorer_goal(explorer_goal, explorer_targets);
        ROS_INFO_STREAM("Sending goal # " << psi.current_explorer_target << " for explorer");
        explorer_client.sendGoal(explorer_goal); 
        explorer_goal_sent = true;
        job_done = false;
        start_looking = false;
      }
      // Check if explorer reached a goal.
      if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO_STREAM("Hooray, explorer reached goal " << psi.current_explorer_target);
        // Check if all targets locations are reached by explorer 
        if(psi.current_explorer_target >= 4){
          // Stop explorer from exploring new targets.
          no_more_exploring = true;

          // Add home position for follower
          follower_locations[4].x = -4;
          follower_locations[4].y = 3.5;

          explorer_summary();

          // Publish a message to rotate explorer
          geometry_msgs::Twist msg;
          msg.linear.x = 0;
          msg.angular.z = 0.0;
          explorer_pub.publish(msg);

          ROS_INFO_STREAM("EXPLORER JOB DONE!");
          continue;
        }
        
        // Publish a message to rotate explorer
        geometry_msgs::Twist msg;
        msg.linear.x = 0;
        msg.angular.z = 0.1;
        explorer_pub.publish(msg);
        start_looking = true;
        // Check if marker is found and localized
        if(job_done){
          // Raise flag for next goal
          explorer_goal_sent = false;
        }
      }
    }

    // Check if follower needs to move to a target location.
    if (no_more_exploring){
      // Check if a goal is sent to follower.
      if (!follower_goal_sent) {
        next_follower_goal(follower_goal);
        ROS_INFO_STREAM("Sending goal # " << psi.current_follower_target << " to follower");
        ROS_INFO_STREAM("goal => x: " << follower_goal.target_pose.pose.position.x << 
                        "\ty: "     << follower_goal.target_pose.pose.position.y );
        follower_client.sendGoal(follower_goal);
        follower_goal_sent = true;
      }
      // Check if explorer reached a goal.
      if (follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO_STREAM("Follower reached goal # " << psi.current_follower_target);
        // Check if all targets locations are reached by follower.
        if (psi.current_follower_target >= 4){
          ROS_INFO_STREAM("PROJECT FINISHED!!!");
          ros::shutdown();
        }
        follower_goal_sent = false;
      }
    }
    // Check if aruco marker needs to be detected.
    if (start_looking){
      listen(tfBuffer, job_done);
      ros::spinOnce();
    }
    loop_rate.sleep();
  }
}

/*                                      FUNCTION DEFINITIONS                                     */

void get_explorer_targets(ros::NodeHandle &nh, std::array<std::array<double, 2>, 5>& arr){
  for (int i = 0; i < 4 ; i++){
    XmlRpc::XmlRpcValue my_list;
    char t_name[32];
    sprintf(t_name, "aruco_lookup_locations/target_%d", i+1);
    nh.getParam(t_name, my_list);
    // Add x-coordinate of target into array.
    arr[i][0] = my_list[0];                                                                           
    // Add y-coordinate of target into array.
    arr[i][1] = my_list[1];                                                                           
  }
  // Add predefined home position of explorer.
  arr[4][0] = -4;
  arr[4][1] = 2.5;
}


void next_explorer_goal(move_base_msgs::MoveBaseGoal& explorer_goal, const std::array<std::array<double, 2>, 5>& explorer_targets){
  psi.current_explorer_target += 1;
  explorer_goal.target_pose.header.frame_id = "map";
  explorer_goal.target_pose.header.stamp = ros::Time::now();
  explorer_goal.target_pose.pose.position.x = explorer_targets[psi.current_explorer_target][0];
  explorer_goal.target_pose.pose.position.y = explorer_targets[psi.current_explorer_target][1];
  explorer_goal.target_pose.pose.orientation.w = 1.0;
}

void next_follower_goal(move_base_msgs::MoveBaseGoal& follower_goal){
  psi.current_follower_target += 1;
  follower_goal.target_pose.header.frame_id = "map";
  follower_goal.target_pose.header.stamp = ros::Time::now();
  follower_goal.target_pose.pose.position.x = follower_locations[psi.current_follower_target].x;
  follower_goal.target_pose.pose.position.y = follower_locations[psi.current_follower_target].y;
  follower_goal.target_pose.pose.orientation.w = 1.0;
}

void explorer_summary(){
  ROS_INFO_STREAM("=============");
  for (int i = 0; i < 4; i++){
    ROS_INFO_STREAM("follower goals: " << follower_locations[i].fiducial_id << "  " 
                                       << follower_locations[i].x << "  "
                                       << follower_locations[i].y << "  ");
  }
  ROS_INFO_STREAM("=============");
}
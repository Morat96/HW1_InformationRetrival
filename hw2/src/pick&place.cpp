#include <ros/ros.h>
#include <iostream>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <std_msgs/Header.h>
#include <Eigen/Geometry>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include "geometry_msgs/PoseStamped.h"
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotOutput.h"
using namespace std;


int main(int argc, char **argv)
{

ros::init(argc, argv, "move");
ros::NodeHandle n;

// subscribe topic from Apriltag in order to obtain all position of the object in the table, and apply to them collision avoidance 
boost::shared_ptr<apriltag_ros::AprilTagDetectionArray const> msg = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("tag_detections");

// subscribe from tf and obtain TF from /camera to /world in order to give to ur5 poses from its referement system  
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener tf2_listener(tfBuffer);

// retrive trasformation from reference system /camera to /base
geometry_msgs::TransformStamped cameratobase;
cameratobase = tfBuffer.lookupTransform("world", "camera_rgb_optical_frame", ros::Time(0), ros::Duration(1.0) );

// compute the pose0 that is pose of the object in the reference system of the camera
// pose1 is the pose trasformed in reference system of /world 
geometry_msgs::PoseStamped pose1;
geometry_msgs::PoseStamped pose0;
pose0.header = msg -> detections[0].pose.header;
pose0.pose = msg -> detections[0].pose.pose.pose;
// compute the transformation
tf2::doTransform(pose0, pose1, cameratobase);

// visualize poses
apriltag_ros::AprilTagDetection detection;
detection = msg -> detections[0];
cout << "CAMERA POSE" << endl;
cout << "x: " << detection.pose.pose.pose.position.x << endl;
cout << "y: " << detection.pose.pose.pose.position.y << endl;
cout << "z: " << detection.pose.pose.pose.position.z << endl;
cout << "WORLD POSE" << endl;
cout << "x: " << pose1.pose.position.x << endl;
cout << "y: " << pose1.pose.position.y << endl;
cout << "z: " << pose1.pose.position.z << endl;

// obtain a pose that have EEF facing down to grasp
tf::Quaternion quat(pose1.pose.orientation.x, pose1.pose.orientation.y, pose1.pose.orientation.z, pose1.pose.orientation.w);

double roll, pitch, yaw;

// get the RPY of the pose
tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

// compute a rotation of the pitch of 90 degrees 
pitch = pitch + 1.571;

tf::Quaternion quat1(0,0,0,1);
// set the new orientation and transform it to quaternion
quat1.setRPY(roll, pitch, yaw);

// BEGIN moveit
ros::AsyncSpinner spinner(1);
spinner.start();

// planning group
static const std::string PLANNING_GROUP = "manipulator";


moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

// Getting Basic Information

// name of the reference frame for this robot.
ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

// name of the end-effector link for this group.
ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

// list of all the groups in the robot:
ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

// util for visualizzation on RVIZ
namespace rvt = rviz_visual_tools;
moveit_visual_tools::MoveItVisualTools visual_tools("shoulder_link");
visual_tools.deleteAllMarkers();
Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
text_pose.translation().z() = 0.0;
visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

// Planning

// POSE 1: using joint-space
// position the arm at the top and in the center of the table

/*
  x: 0.530604
  y: 0.0777674
  z: 1.30427
orientation: 
  x: 0.708776
  y: -0.0147633
  z: 0.0169541
  w: 0.705075
*/
// get the current state
moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
std::vector<double> joint_group_positions;
current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

joint_group_positions[0] = 1.59495;  // radians
joint_group_positions[1] = -1.79504;  // radians
joint_group_positions[2] = -1.12234;  // radians
joint_group_positions[3] = -1.80584;  // radians
joint_group_positions[4] = 1.55196; // radians
joint_group_positions[5] = 0.0245287;  // radians

move_group.setJointValueTarget(joint_group_positions);

// Planning
moveit::planning_interface::MoveGroupInterface::Plan my_plan;

bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

// compute the plan and move the robot	
move_group.move();

// POSE 2: over the object
geometry_msgs::Pose target_pose1;

target_pose1.position.x = pose1.pose.position.x;
target_pose1.position.y = pose1.pose.position.y;
target_pose1.position.z = pose1.pose.position.z + 0.30;
target_pose1.orientation.w = quat1.w();
target_pose1.orientation.x = quat1.x();
target_pose1.orientation.y = quat1.y();
target_pose1.orientation.z = quat1.z();

// set the target pose
move_group.setPoseTarget(target_pose1);

success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

// show trajectory in RVIZ
ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
visual_tools.publishAxisLabeled(target_pose1, "pose1");
visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);

// compute the plan and move the robot 
move_group.move();


// PUBLISHER
ros::Publisher pub = n.advertise<robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput>("/left_hand/command", 1000);

robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput str;
// OPEN GRIPPER
str.rACT = 1;
str.rMOD = 0;
str.rGTO = 1; 
str.rATR = 0; 
str.rGLV = 0; 
str.rICF = 0; 
str.rICS = 0;
str.rPRA = 0; 
str.rSPA = 200; 
str.rFRA = 0; 
str.rPRB = 0; 
str.rSPB = 0; 
str.rFRB = 0; 
str.rPRC = 0; 
str.rSPC = 0; 
str.rFRC = 0; 
str.rPRS = 0; 
str.rSPS = 0; 
str.rFRS = 0;
// open
//rACT: 1, rMOD: 0, rGTO: 1, rATR: 0, rGLV: 0, rICF: 0, rICS: 0, rPRA: 0, rSPA: 200, rFRA: 0, rPRB: 0, rSPB: 0, rFRB: 0, rPRC: 0, rSPC: 0, rFRC: 0, rPRS: 0, rSPS: 0, rFRS: 0
// close
//rACT: 1, rMOD: 0, rGTO: 1, rATR: 0, rGLV: 0, rICF: 0, rICS: 0, rPRA: 250, rSPA: 200, rFRA: 200, rPRB: 0, rSPB: 0, rFRB: 0, rPRC: 0, rSPC: 0, rFRC: 0, rPRS: 0, rSPS: 0, rFRS: 0
ros::Time beginTime = ros::Time::now();
ros::Duration secondsIWantToSendMessagesFor = ros::Duration(3); 
ros::Time endTime = beginTime + secondsIWantToSendMessagesFor;
    while(ros::Time::now() < endTime )
    {
        pub.publish(str);

        // Time between messages, so you don't blast out an thousands of 
        // messages in your 3 secondperiod
        ros::Duration(0.1).sleep();
    }



// POSE 3: approach the object by going down

move_group.setStartStateToCurrentState();

// save waypoints
std::vector<geometry_msgs::Pose> waypoints;
geometry_msgs::Pose target_pose3 = move_group.getCurrentPose().pose;

// move the robot down of 15 cm
target_pose3.position.z -= 0.15;
waypoints.push_back(target_pose3);

// set parameters
move_group.setMaxVelocityScalingFactor(0.1);
move_group.setPlanningTime(10.0);
moveit_msgs::RobotTrajectory trajectory;
const double jump_threshold = 0.0;
const double eef_step = 0.01;

// compute the path
double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

// plan and do the trajectory
my_plan.trajectory_ = trajectory;
move_group.execute(my_plan);

// Visualize the plan in RViz
visual_tools.deleteAllMarkers();
visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
for (std::size_t i = 0; i < waypoints.size(); ++i) visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);

// CLOSE GRIPPER
str.rACT = 1;
str.rMOD = 0;
str.rGTO = 1; 
str.rATR = 0; 
str.rGLV = 0; 
str.rICF = 0; 
str.rICS = 0;
str.rPRA = 250; 
str.rSPA = 200; 
str.rFRA = 200; 
str.rPRB = 0; 
str.rSPB = 0; 
str.rFRB = 0; 
str.rPRC = 0; 
str.rSPC = 0; 
str.rFRC = 0; 
str.rPRS = 0; 
str.rSPS = 0; 
str.rFRS = 0;

beginTime = ros::Time::now();
secondsIWantToSendMessagesFor = ros::Duration(3); 
endTime = beginTime + secondsIWantToSendMessagesFor;
    while(ros::Time::now() < endTime )
    {
        pub.publish(str);

        // Time between messages, so you don't blast out an thousands of 
        // messages in your 3 secondperiod
        ros::Duration(0.1).sleep();
    }

// POSE 4: pull up the object
sleep(0.5);
move_group.setStartStateToCurrentState();

// save waypoints
std::vector<geometry_msgs::Pose> waypoints1;

// move the robot down of 15 cm
target_pose3.position.z += 0.15;
waypoints1.push_back(target_pose3);

// set parameters
move_group.setMaxVelocityScalingFactor(0.1);
move_group.setPlanningTime(10.0);
moveit_msgs::RobotTrajectory trajectory1;

// compute the path
fraction = move_group.computeCartesianPath(waypoints1, eef_step, jump_threshold, trajectory1);
ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

// plan and do the trajectory
my_plan.trajectory_ = trajectory1;
move_group.execute(my_plan);


visual_tools.trigger();
visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

return 0;  

}

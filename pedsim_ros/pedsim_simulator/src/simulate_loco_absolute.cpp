#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

double g_updateRate, g_simulationFactor;
std::string g_worldFrame, g_robotFrame;
geometry_msgs::Twist g_currentTwist;
nav_msgs::Odometry loco_Pose;
tf::Transform g_currentPose;
boost::shared_ptr<tf::TransformBroadcaster> g_transformBroadcaster;
boost::mutex mutex;

/// Simulates robot motion of a differential-drive robot with translational and
/// rotational velocities as input
/// These are provided in the form of a geometry_msgs::Twist, e.g. by
/// turtlebot_teleop/turtlebot_teleop_key.
/// The resulting robot position is published as a TF transform from world -->
/// base_footprint frame.
void updateLoop() {
  ros::Rate rate(g_updateRate);

  
  while (true) {
    // Get current pose
    double x,y, qx, qy, qz, qw;
    {
      boost::mutex::scoped_lock lock(mutex);
      x = loco_Pose.pose.pose.position.x;
      y = loco_Pose.pose.pose.position.y;
      qx = loco_Pose.pose.pose.orientation.z;
      qy = loco_Pose.pose.pose.orientation.y;
      qz = loco_Pose.pose.pose.orientation.z;
      qw = loco_Pose.pose.pose.orientation.w;
    }
    if(qw == 0){ //error if qw is 0, is later always != 0 
    	qw = 0.00001;}
    	
    g_currentPose.getOrigin().setX(x);
    g_currentPose.getOrigin().setY(y);
    g_currentPose.setRotation(tf::Quaternion(0,0, qz, qw ));
    
    // Broadcast transform
    g_transformBroadcaster->sendTransform(tf::StampedTransform(
        g_currentPose, ros::Time::now(), g_worldFrame, g_robotFrame));

    rate.sleep();
  }
}

//void onTwistReceived(const geometry_msgs::Twist::ConstPtr& twist) {
//  boost::mutex::scoped_lock lock(mutex);
//  g_currentTwist = *twist;
//}

void onPoseReceived(const nav_msgs::Odometry::ConstPtr& Odometry) {
  boost::mutex::scoped_lock lock(mutex);
  loco_Pose = *Odometry;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simulate_diff_drive_robot");
  ros::NodeHandle nodeHandle("");
  ros::NodeHandle privateHandle("~");

  // Process parameters
  privateHandle.param<std::string>("world_frame", g_worldFrame, "odom");
  privateHandle.param<std::string>("robot_frame", g_robotFrame,
                                   "base_footprint");

  privateHandle.param<double>("/pedsim_simulator/simulation_factor", g_simulationFactor,
                              1.0);  // set to e.g. 2.0 for 2x speed
  privateHandle.param<double>("/pedsim_simulator/update_rate", g_updateRate, 25.0);  // in Hz

  double initialX = 0.0, initialY = 0.0, initialTheta = 0.0;
  privateHandle.param<double>("pose_initial_x", initialX, 0.0);
  privateHandle.param<double>("pose_initial_y", initialY, 0.0);
  privateHandle.param<double>("pose_initial_theta", initialTheta, 0.0);

  g_currentPose.getOrigin().setX(initialX);
  g_currentPose.getOrigin().setY(initialY);
  g_currentPose.setRotation(tf::createQuaternionFromRPY(0, 0, initialTheta));

  // Create ROS subscriber and TF broadcaster
  g_transformBroadcaster.reset(new tf::TransformBroadcaster());
  //ros::Subscriber twistSubscriber =
  //    nodeHandle.subscribe<geometry_msgs::Twist>("cmd_vel", 3, onTwistReceived);
      
  ros::Subscriber poseSubscriber =
      nodeHandle.subscribe<nav_msgs::Odometry>("/locobot/mobile_base/absolute_position", 3, onPoseReceived);    

  // Run
  boost::thread updateThread(updateLoop);
  ros::spin();
}

#ifndef ROBOT_PATH_PLANNING_MOVEMENT_NODE_H
#define ROBOT_PATH_PLANNING_MOVEMENT_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

namespace robot_path_planning
{

class MovementNode
{
public:
    MovementNode();
    ~MovementNode() = default;

private:
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void timerCallback(const ros::TimerEvent&); // Declare the timer callback

    // Publisher to send velocity commands
    ros::Publisher cmd_vel_publisher_;

    // Subscribers to receive goals and paths
    ros::Subscriber goal_subscriber_;
    ros::Subscriber path_subscriber_;

    // Timer for publishing velocity commands
    ros::Timer timer_;

    // Current target pose
    geometry_msgs::PoseStamped current_goal_;

    // Node handle
    ros::NodeHandle nh_; // Add NodeHandle here
};

} // namespace robot_path_planning

#endif // ROBOT_PATH_PLANNING_MOVEMENT_NODE_H

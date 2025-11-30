// include/robot_path_planning/localization_node.h

#ifndef ROBOT_PATH_PLANNING_LOCALIZATION_NODE_H
#define ROBOT_PATH_PLANNING_LOCALIZATION_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

namespace robot_path_planning
{

class LocalizationNode
{
public:
    LocalizationNode();
    ~LocalizationNode() = default;

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    // Subscribers for odometry and pose
    ros::Subscriber odom_subscriber_;
    ros::Subscriber pose_subscriber_;

    // Publisher for localized pose
    ros::Publisher localized_pose_publisher_;

    // Node handle
    ros::NodeHandle nh_;
};

} // namespace robot_path_planning

#endif // ROBOT_PATH_PLANNING_LOCALIZATION_NODE_H

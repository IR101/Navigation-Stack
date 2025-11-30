// include/robot_path_planning/slam_node.h

#ifndef ROBOT_PATH_PLANNING_SLAM_NODE_H
#define ROBOT_PATH_PLANNING_SLAM_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

namespace robot_path_planning
{

class SlamNode
{
public:
    SlamNode();
    ~SlamNode() = default;

private:
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    // Subscriber for laser scans
    ros::Subscriber laser_subscriber_;

    // Publisher for map data
    ros::Publisher map_publisher_;

    // Node handle
    ros::NodeHandle nh_;
};

} // namespace robot_path_planning

#endif // ROBOT_PATH_PLANNING_SLAM_NODE_H

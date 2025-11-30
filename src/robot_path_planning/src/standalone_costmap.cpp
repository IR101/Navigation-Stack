#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/layered_costmap.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "standalone_costmap");
    ros::NodeHandle nh("~");

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // Initialize costmap
    costmap_2d::Costmap2DROS costmap("costmap", tf_buffer);
    
    ros::Rate loop_rate(10);  // 10 Hz

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

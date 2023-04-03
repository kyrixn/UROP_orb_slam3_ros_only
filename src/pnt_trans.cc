#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

ros::Subscriber sub;
ros::Publisher pub;

sensor_msgs::PointCloud2 pntCloud;

void publishPnt(sensor_msgs::PointCloud2 pc) {
    pub.publish(pc);
}

void pntCallback(const sensor_msgs::PointCloud2 cloud)
{
    pntCloud = cloud;
    publishPnt(pntCloud);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pnt_trans");
    ros::NodeHandle nh;

    sub = nh.subscribe("/orb_slam3/tracked_points", 1, pntCallback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/transformed_points", 1);

    ros::spin();
    return 0;
}

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

ros::Subscriber sub;
ros::Publisher pub;

//create map online/offline
bool create_grid_offline;

void pntCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Perform the actual filtering
    pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> ror;
    ror.setInputCloud (cloudPtr);
    ror.setRadiusSearch(0.1);
    ror.setMinNeighborsInRadius(5);
    ror.filter (cloud_filtered);
    


    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_filtered, output);

    // Publish the data
    pub.publish (output);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pnt_trans");
    ros::NodeHandle nh;

    nh.param<bool>("offline", create_grid_offline, false);

    // initROR();

    ROS_INFO("Start\n");

    sub = nh.subscribe("/orb_slam3/all_points", 1, pntCallback);

    pub = nh.advertise<sensor_msgs::PointCloud2>("/transformed_points", 1);

    ros::spin();
}

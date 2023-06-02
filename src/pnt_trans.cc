#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>    
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/convolution.h>
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <math.h>

#define PI 3.14159265

ros::Subscriber sub;
ros::Publisher pub;

//create map online/offline
bool create_grid_offline;
bool start_publish = false;

int frame_count = 0;

sensor_msgs::PointCloud2 output;

pcl::PointCloud<pcl::PointXYZ> pXYZ_filtered;
pcl::PointCloud<pcl::PointXYZ> pXYZ_ground;
pcl::PointCloud<pcl::PointXYZ> pXYZ_no_ground;
pcl::PointCloud<pcl::PointXYZ> pXYZ_scaled;
pcl::PointCloud<pcl::PointXYZ> pXYZ_densified;


pcl::PointCloud<pcl::PointXYZ>::Ptr cld(new pcl::PointCloud<pcl::PointXYZ>);

double x_rad, y_rad, z_rad;

void mappointsDelay(const ros::TimerEvent&)
{
    ROS_INFO("start sending transformed pnt cloud");
}

Eigen::Matrix4f transform_mat() {
    
    Eigen::Matrix4f transform_scale = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f rotate_x = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f rotate_y = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f rotate_z = Eigen::Matrix4f::Identity();

    transform_scale(0,0) = 2;
    transform_scale(1,1) = 2;
    transform_scale(2,2) = 1;


    rotate_x(1, 1) = cos(PI*x_rad);
    rotate_x(1, 2) = -sin(PI*x_rad);
    rotate_x(2, 1) = sin(PI*x_rad);
    rotate_x(2, 2) = cos(PI*x_rad);


    rotate_y(0, 0) = cos(PI*y_rad);
    rotate_y(0, 2) = sin(PI*y_rad);
    rotate_y(2, 0)= -sin(PI*y_rad);
    rotate_y(2, 2) = cos(PI*y_rad);


    rotate_z(0, 0) = cos(PI*z_rad);
    rotate_z(0, 1)= -sin(PI*z_rad);
    rotate_z(1, 0) = sin(PI*z_rad);
    rotate_z(1, 1) = cos(PI*z_rad);

    return rotate_z*rotate_y*rotate_x*transform_scale;
}

void densify() {

    pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
    
    
    kd_tree.setInputCloud(cld);
 
    int k = 10;
    std::vector<int> pt_idx_search(k); 
    std::vector<float> pt_sqr_dis(k);
    
    pcl::PointXYZ search_pt;
    pcl::PointXYZ new_pt;
    
    pXYZ_densified = pXYZ_filtered;

    for(auto iterA = pXYZ_filtered.begin(); iterA != pXYZ_filtered.end(); iterA++) {
        
        search_pt.x = iterA->x;
        search_pt.y = iterA->y;
        search_pt.z = iterA->z;

        //threshold: [0.1,0.3]
        if (kd_tree.nearestKSearch(search_pt, k, pt_idx_search, pt_sqr_dis) > 0) {
            for(int i =0; i <k; i++) {
                if(pt_sqr_dis[i] < 0.01 || pt_sqr_dis[i] > 0.09) {
                    continue;
                }

                new_pt.x = (pXYZ_filtered[pt_idx_search[i]].x + search_pt.x)/2.0;
                new_pt.y = (pXYZ_filtered[pt_idx_search[i]].y + search_pt.y)/2.0;
                new_pt.x = (pXYZ_filtered[pt_idx_search[i]].y + search_pt.y)/2.0;
            
                pXYZ_densified.push_back(new_pt);
            }
        }
    }

}

void pntCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 

    // Container for original & filtered data
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    //remove ground points
    pcl::fromPCLPointCloud2(*cloud, pXYZ_ground);

    Eigen::Matrix4f transform = transform_mat();

    pcl::transformPointCloud(pXYZ_ground, pXYZ_no_ground, transform);


    for(auto iterA = pXYZ_no_ground.begin(); iterA != pXYZ_no_ground.end(); iterA++){
        if(iterA->z <= -0.30) {
            iterA->z = -10;
        }
    }

    pcl::PCLPointCloud2* tmp = new pcl::PCLPointCloud2;
    pcl::toPCLPointCloud2(pXYZ_no_ground, *tmp);
    pcl::PCLPointCloud2ConstPtr tmpPtr(tmp);

    // Perform the actual filtering
    pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> ror;
    ror.setInputCloud(tmpPtr);
    ror.setRadiusSearch(0.3);
    ror.setMinNeighborsInRadius(15);
    // ror.setMinNeighborsInRadius(10);
    ror.filter(cloud_filtered);
    
    pcl::fromPCLPointCloud2(cloud_filtered, pXYZ_filtered);

    densify();

    pcl::toROSMsg(pXYZ_densified, output);
    // pcl_conversions::fromPCL(cloud_scaled, output);

    frame_count++;
}

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "pnt_trans");
    ros::NodeHandle nh;

    nh.param<bool>("offline", create_grid_offline, false);

    nh.param<double>("x_rad", x_rad, -0.07);
    nh.param<double>("y_rad", y_rad, 0);
    nh.param<double>("z_rad", z_rad, 0.07);

    
    cld = pXYZ_filtered.makeShared();
    // initROR();

    ros::Timer timer = nh.createTimer(ros::Duration(10.0), mappointsDelay, true);
    
    ROS_INFO("------------------Start----------------------\n");

    // sub = nh.subscribe("/orb_slam3/tracked_points", 1, pntCallback);
    sub = nh.subscribe("/orb_slam3/all_points", 1, pntCallback);
    
    pub = nh.advertise<sensor_msgs::PointCloud2>("/transformed_points", 1);

    ros::Rate r(3);
    while(ros::ok()) {
        ros::spinOnce();

        // if(frame_count >= 10 || start_publish) {
        if(frame_count >= 50 || start_publish) {
            start_publish = true;
            pub.publish(output);
        }

        r.sleep();
    }

}

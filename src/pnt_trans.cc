#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>    
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/convolution.h>
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/filters/voxel_grid.h>

ros::Subscriber sub;
ros::Publisher pub;

//create map online/offline
bool create_grid_offline;
bool start_publish = false;

int frame_count = 0;

sensor_msgs::PointCloud2 output;

void mappointsDelay(const ros::TimerEvent&)
{
    ROS_INFO("start sending transformed pnt cloud");
}


void pntCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    pcl::PointCloud<pcl::PointXYZ> pXYZ_filtered;
    pcl::PointCloud<pcl::PointXYZ> pXYZ_Guassian_out;
    pcl::PointCloud<pcl::PointXYZ> pXYZ_scaled;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Perform the actual filtering
    pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> ror;
    ror.setInputCloud (cloudPtr);
    ror.setRadiusSearch(0.2);
    ror.setMinNeighborsInRadius(10);
    // ror.setMinNeighborsInRadius(7);

    ror.filter(cloud_filtered);
    
    pcl::fromPCLPointCloud2(cloud_filtered, pXYZ_filtered);
    // *tmp = pXYZ_filtered;

    // Gaussian filter 

	// pcl::filters::Convolution<pcl::PointXYZ, pcl::PointXYZ>  convolution;
	// Eigen::ArrayXf gaussian_kernel(5);
	// gaussian_kernel << 1.f / 16, 1.f / 4, 3.f / 8, 1.f / 4, 1.f / 16;
	// convolution.setBordersPolicy(
	// pcl::filters::Convolution<pcl::PointXYZ, pcl::PointXYZ>::BORDERS_POLICY_IGNORE);
	// convolution.setDistanceThreshold(static_cast<float> (0.1));
	// convolution.setInputCloud(pXYZ_filtered);
	// convolution.setKernel(gaussian_kernel);
	// convolution.convolve(pXYZ_Guassian_out);
    
    // least square method filter

    // if(frame_count >=100 || pXYZ_filtered->width !=0) {
	//     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	//     tree->setInputCloud(pXYZ_filtered);
    // }
	// pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
	// mls.setComputeNormals(false);
	// mls.setInputCloud(tmp);
	// mls.setPolynomialFit(false);
	// mls.setPolynomialOrder(2);
	// mls.setSearchMethod(tree);
	// mls.setSearchRadius(0.1);
	// mls.process(pXYZ_Guassian_out);

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0,0) = 2;
    transform(1,1) = 2;
    transform(2,2) = 4;

    pcl::transformPointCloud(pXYZ_filtered, pXYZ_scaled, transform);

    pcl::toROSMsg(pXYZ_scaled, output);
    // pcl_conversions::fromPCL(cloud_scaled, output);

    frame_count++;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pnt_trans");
    ros::NodeHandle nh;

    nh.param<bool>("offline", create_grid_offline, false);

    // initROR();

    ros::Timer timer = nh.createTimer(ros::Duration(10.0), mappointsDelay, true);
    
    ROS_INFO("------------------Start----------------------\n");

    // sub = nh.subscribe("/orb_slam3/tracked_points", 1, pntCallback);
    sub = nh.subscribe("/orb_slam3/all_points", 1, pntCallback);
    
    pub = nh.advertise<sensor_msgs::PointCloud2>("/transformed_points", 1);

    ros::Rate r(4);
    while(ros::ok()) {
        ros::spinOnce();

        // if(frame_count >= 10 || start_publish) {
        if(frame_count >= 100 || start_publish) {
            start_publish = true;
            pub.publish(output);
        }

        r.sleep();
    }

}

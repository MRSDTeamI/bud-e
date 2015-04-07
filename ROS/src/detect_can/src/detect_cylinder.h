/*
 * Node class for cylinder parser.
 */

#ifndef DETECT_CYLINDER_H
#define DETECT_CYLINDER_H

// Ros includes
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>

#include <typeinfo>

typedef pcl::PointXYZ PointT;

class DetectCylinder
{
public:
	
	DetectCylinder();
	DetectCylinder(ros::NodeHandle nh, std::string, std::string);

	~DetectCylinder();

	// Callback function for subscriber
	void detect_cyl_callback(const sensor_msgs::PointCloud2ConstPtr &input);

	void detect_for_cylinder();
	void display_stats(double seg_frac, long unsigned int cluster_points);
	void down_sample();
	void euclidean_cluster();
	void pass_through_filter();
	void publish(ros::Publisher *pub_cloud, ros::Publisher *pub_center);
	void remove_outliers();
	void remove_planes();
	void segment_cylinder(pcl::PointCloud<PointT>::Ptr);	

private:

	// For statistics tracking
	int not_cyl_cnt;
	int cyl_cnt;
	double start_time;
	double not_cyl_frac;	// fraction when not cylinder
	double cyl_frac;		// fraction when cylinder

	// For publishing / subscribing
	ros::Subscriber sub;
	ros::Publisher pub_cloud;
	ros::Publisher pub_center;

	sensor_msgs::PointCloud2ConstPtr orig_input;	// original input cloud
	pcl::PointCloud<PointT>::Ptr cloud;				// cloud to work with
	pcl::PointCloud<PointT>::Ptr cloud_cylinder;	// final cylinder(s) cloud
	geometry_msgs::Vector3 center_coord;			// cylinder center coordinate 
    std::vector<pcl::PointIndices> cluster_indices;	// store clusters of objects we detect

	static const int PTS_IN_PLANE = 4;

};

#endif


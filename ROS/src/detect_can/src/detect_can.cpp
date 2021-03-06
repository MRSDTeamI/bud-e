#include <ros/ros.h>
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

#include <geometry_msgs/Vector3.h>

#include <typeinfo>

ros::Publisher pub_cloud;
ros::Publisher pub_coord;

ros::Publisher pub_temp;

typedef pcl::PointXYZ PointT;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	//pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

	// Convert sensor_msgs to PCLPointCloud2 to pcl::PointXYZ
	pcl::PCLPointCloud2 temp_cloud;
	pcl_conversions::toPCL (*input, temp_cloud);
	pcl::fromPCLPointCloud2(temp_cloud, *cloud);

	pcl::PointCloud<PointT>::Ptr cloud_v (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_in (new pcl::PointCloud<PointT>);
	//pcl::PCLPointCloud2::Ptr cloud_v (new pcl::PCLPointCloud2);
	//pcl::PCLPointCloud2::Ptr cloud_f (new pcl::PCLPointCloud2);
	//pcl::PCLPointCloud2::Ptr cloud_in (new pcl::PCLPointCloud2);
	//pcl::PassThrough<pcl::PCLPointCloud2> pass;
	pcl::PassThrough<PointT> pass;
	pcl::ExtractIndices<PointT> extract;
	pcl::SACSegmentation<PointT> seg_plane;
	//pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_cyl;
	//pcl::VoxelGrid<pcl::PCLPointCloud2> sor2;
	pcl::VoxelGrid<PointT> sor2;

	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

	// For converting back to output
	pcl::PCLPointCloud2::Ptr object (new pcl::PCLPointCloud2);
	sensor_msgs::PointCloud2 output;

	/*
	// Probably don't need this b/c for seg cylinder
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	//pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
	*/
	
	pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);

	// Don't think this helps very much
	// RUN PASS THROUGH FILTER TO REMOVE POINTS OUTSIDE OF DESIRED VIEW
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0,1.5);  // filter out points greater than 1.5m
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-0.1,1);
	//pass.setFilterFieldName("y");
	//pass.setFilterLimits(0,8);
	//pass.setFilterLimitsNegative(true);
	pass.filter(*cloud_in);

	// REMOVE STATISTICAL OUTLIERS 
	pcl::StatisticalOutlierRemoval<PointT> sor1;
	sor1.setInputCloud(cloud_in);  // if using pass through filter
	//sor1.setInputCloud(cloud);
	sor1.setMeanK(10);  //50  10
	sor1.setStddevMulThresh (0.005);  //1  0.5
	sor1.filter(*cloud_v);

	// DOWNSAMPLE USING VOXELS
	sor2.setInputCloud (cloud_v);
	sor2.setLeafSize(0.01,0.01,0.01);
	sor2.filter(*cloud_v);

/*
    // Try removing statistical outlier again
    sor1.setInputCloud(cloud_v);  // if using pass through filter
    sor1.setMeanK(10);  //50  10
    sor1.setStddevMulThresh (0.05);  //1  0.5
    sor1.filter(*cloud_v);
*/

//	// ESTIMATE POINT NORMALS
//	ne.setSearchMethod (tree);
//	ne.setInputCloud (cloud_v);
//	ne.setKSearch (50);
//	ne.compute (*cloud_normals);

	// EXTRACT PLANES FROM IMAGE
	pcl::PointCloud<PointT>::Ptr inPCseg (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr outPCseg (new pcl::PointCloud<PointT>);
	//pcl::fromPCLPointCloud2 (*cloud_v,*inPCseg);
	*inPCseg = *cloud_v;
	//pcl::fromPCLPointCloud2 (*cloud,*inPCseg);
	//pcl::fromPCLPointCloud2 (*cloud_in,*inPCseg);

	pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
	pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);

	seg_plane.setOptimizeCoefficients (true);
	seg_plane.setModelType (pcl::SACMODEL_PLANE);
	seg_plane.setMethodType (pcl::SAC_RANSAC);
	seg_plane.setMaxIterations (100);
	seg_plane.setDistanceThreshold (0.007);  // 0.01 good but some artifacts

std::cerr << "Points: " << inPCseg->points.size() << std::endl;

	int i = 0, count = 0, nr_points = (int) inPCseg->points.size();
	// Keep segmenting planes until 10% of points are left
	while (inPCseg->points.size() > 0.06 * nr_points) {  // 0.06
	//while (inPCseg->points.size() > 0.1 * nr_points) {  // 0.06

		seg_plane.setInputCloud (inPCseg);
		//seg.setInputNormals (cloud_normals);
		// Obtain the plane inliers and coefficients
		seg_plane.segment (*inliers_plane, *coefficients_plane);

		if (inliers_plane->indices.size() < 4) {
			std::cerr << "Could not estimate plane." << std::endl;
			break;
		} else {
			std::cerr << "Plane segmentation successful." << std::endl;
		}

		// Extract inliers
		extract.setInputCloud (inPCseg);
		extract.setIndices (inliers_plane);
		extract.setNegative (true);
		extract.filter (*outPCseg);
		 
		//// Probably don't need this b/c for segmenting cylinder
		//extract_normals.setNegative (true);
		//extract_normals.setInputCloud (cloud_normals);
		//extract_normals.setIndices (inliers_plane);
		//extract_normals.filter (*cloud_normals2);

		std::cerr << "Planar component: " << outPCseg->width * outPCseg->height 
		<< " data points." << std::endl;

		inPCseg.swap(outPCseg);
		count++;
	}
    // Try removing statistical outlier again
    sor1.setInputCloud(outPCseg);  // if using pass through filter
    sor1.setMeanK(10);  //50  10
    sor1.setStddevMulThresh (0.05);  //1  0.5
    sor1.filter(*outPCseg);

	// Creating the KdTree object for the search method of the extraction
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (outPCseg);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.018); // 2cm  0.018
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (2500);
	ec.setSearchMethod (tree);
	ec.setInputCloud (outPCseg);
	ec.extract (cluster_indices);

std::cerr << "Cluster size: " << cluster_indices.size() << std::endl;
if (cluster_indices.size() > 0)
{
Eigen::Vector4f centroid;
pcl::compute3DCentroid(*outPCseg, cluster_indices[0], centroid);
std::cerr << centroid[0] << " " << centroid[1] << " " << centroid[2] << std::endl;
geometry_msgs::Vector3 can_coord;
can_coord.x = centroid[0];
can_coord.y = centroid[1];
can_coord.z = centroid[2];
pub_coord.publish(can_coord);

/* print cluster point cloud
extract.setInputCloud (outPCseg);
extract.setIndices (cluster_indices[0]);
extract.setNegative (true);
extract.filter (*cloud_v);
	pcl::toPCLPointCloud2(*cloud_v, *object);  // PointXYZ to PointCloud2
	pcl_conversions::fromPCL(*object, output); // PointCloud2 to sensor-msgs
	output.header.frame_id = input->header.frame_id;

pub_temp.publish(output);
*/
}

/*
	// ESTIMATE POINT NORMALS
	ne.setSearchMethod (tree);
	ne.setInputCloud (outPCseg);
	ne.setKSearch (50);
	ne.compute (*cloud_normals);

	// SEGMENTATION FOR CYLINDER OBJECT
	seg_cyl.setOptimizeCoefficients (true);
	seg_cyl.setModelType (pcl::SACMODEL_CYLINDER);
	seg_cyl.setMethodType (pcl::SAC_RANSAC);
	seg_cyl.setNormalDistanceWeight (0.1);
	seg_cyl.setMaxIterations (100);
	seg_cyl.setDistanceThreshold (0.1);
	seg_cyl.setRadiusLimits (0.01, 0.03);
	seg_cyl.setInputCloud (outPCseg);
	seg_cyl.setInputNormals (cloud_normals);

	// OBTAIN CYLINDER INLIERS AND COEFFICIENTS
	seg_cyl.segment (*inliers_cylinder, *coefficients_cylinder);
	std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

	extract.setInputCloud (outPCseg);
	extract.setIndices (inliers_cylinder);
	extract.setNegative (false);
	pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
	extract.filter (*cloud_cylinder);
	if (cloud_cylinder->points.empty ()) {
		std::cerr << "Can't find the cylindrical component." << std::endl;
	// return?
	}
*/

	// Convert to ROS data type
	//pcl::toPCLPointCloud2(*outPCseg,*cloud_f);
	//pcl::toPCLPointCloud2(*cloud_cylinder,*cloud_f);
	//pcl::toPCLPointCloud2(*cloud_cylinder, *object);  // PointXYZ to PointCloud2
	pcl::toPCLPointCloud2(*outPCseg, *object);  // PointXYZ to PointCloud2
	pcl_conversions::fromPCL(*object, output); // PointCloud2 to sensor-msgs
	output.header.frame_id = input->header.frame_id;
	
	pub_cloud.publish(output);
}

int
main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "detect_can");
	ros::NodeHandle nh;
	
	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb);
	//ros::Subscriber sub = nh.subscribe ("/camera/rgb/points", 1, cloud_cb);
	
	// Create a ROS publisher for the output cloud
	pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("detect_can_cloud", 1);
	// Create a ROS publisher for cluster center
	pub_coord = nh.advertise<geometry_msgs::Vector3> ("cluster_center", 1);

	pub_temp = nh.advertise<sensor_msgs::PointCloud2> ("test_cloud", 1);

	// Spin
	ros::spin ();
}

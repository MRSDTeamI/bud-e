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
#include <std_msgs/Bool.h>

#include <typeinfo>

ros::Publisher pub_cloud;
ros::Publisher pub_coord;

ros::Publisher pub_plane_seg;
ros::Publisher pub_temp;

// To decide if we should start running vision system
bool g_run_vision = false;

// For calculating rate of cylinder count
int g_not_cylinder = 0;
int g_cylinder = 0;
double g_start_time = 0;
double g_not_cyl_frac = 0;
double g_cyl_frac = 0;

typedef pcl::PointXYZ PointT;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	// Return if we're not supposed to start running vision system yet
	if (!g_run_vision)
		return;

	//pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

	// Convert sensor_msgs to PCLPointCloud2 to pcl::PointXYZ
	pcl::PCLPointCloud2 temp_cloud;
	pcl_conversions::toPCL (*input, temp_cloud);
	pcl::fromPCLPointCloud2(temp_cloud, *cloud);

	pcl::PointCloud<PointT>::Ptr cloud_v (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_in (new pcl::PointCloud<PointT>);

	pcl::PassThrough<PointT> pass;
	pcl::ExtractIndices<PointT> extract;
	pcl::SACSegmentation<PointT> seg_plane;
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_cyl;
	pcl::VoxelGrid<PointT> sor2;

	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

	// For converting back to output
	pcl::PCLPointCloud2::Ptr object (new pcl::PCLPointCloud2);
	sensor_msgs::PointCloud2 output;

	// For cylinder segmentation
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
	
	pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
	pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
	pcl::PointCloud<PointT>::Ptr cloud_cylinders (new pcl::PointCloud<PointT> ());

	// Don't think this helps very much
	// RUN PASS THROUGH FILTER TO REMOVE POINTS OUTSIDE OF DESIRED VIEW
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0,1.2);  // filter out points greater than 1.5m
	pass.setFilterFieldName("x");
	pass.setFilterLimits(0,1);
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

//    // Remove statistical outlier again to remove extra points
//    sor1.setInputCloud(cloud_v);  // if using pass through filter
//    sor1.setMeanK(50);  //50  10
//    sor1.setStddevMulThresh (0.001);  //1  0.5
//    sor1.filter(*cloud_v);

	// EXTRACT PLANES FROM IMAGE
	pcl::PointCloud<PointT>::Ptr inPCseg (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr outPCseg (new pcl::PointCloud<PointT>);
	*inPCseg = *cloud_v;

	pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
	pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);

	seg_plane.setOptimizeCoefficients (true);
	seg_plane.setModelType (pcl::SACMODEL_PLANE);
	seg_plane.setMethodType (pcl::SAC_RANSAC);
	seg_plane.setMaxIterations (100);
	seg_plane.setDistanceThreshold (0.007);  // 0.01 good but some artifacts

//std::cerr << "Points: " << inPCseg->points.size() << std::endl;

	int i = 0, count = 0, nr_points = (int) inPCseg->points.size();
	// Keep segmenting planes until 10% of points are left
	//while (inPCseg->points.size() > 0.06 * nr_points) {  // 0.06
	while (inPCseg->points.size() > 0.1 * nr_points) {  // 0.08

		seg_plane.setInputCloud (inPCseg);
		//seg.setInputNormals (cloud_normals);
		// Obtain the plane inliers and coefficients
		seg_plane.segment (*inliers_plane, *coefficients_plane);

		if (inliers_plane->indices.size() < 4) {
			std::cerr << "Could not estimate plane." << std::endl;
		} else {
			std::cerr << "Plane segmentation successful." << std::endl;
		}

		// Extract inliers
		extract.setInputCloud (inPCseg);
		extract.setIndices (inliers_plane);
		extract.setNegative (true);
		extract.filter (*outPCseg);

//		// Remove plane normals of the plane we just removed
//		extract_normals.setNegative (true);
//        extract_normals.setInputCloud (cloud_normals);
//        extract_normals.setIndices (inliers_plane);
//        extract_normals.filter (*cloud_normals);

		//std::cerr << "Planar component: " << outPCseg->width * outPCseg->height 
		//<< " data points." << std::endl;

		inPCseg.swap(outPCseg);
		count++;
	}

	// One more check to see if there's another plane.
	// If there are no objects then there may be one more plane left.
	seg_plane.setInputCloud(outPCseg);
	seg_plane.segment(*inliers_plane, *coefficients_plane);
	extract.setInputCloud (outPCseg);
	extract.setIndices (inliers_plane);
    extract.setNegative (true);
	extract.filter (*outPCseg);
	//// Remove plane normals of the plane we just removed
    //extract_normals.setNegative (true);
    //extract_normals.setInputCloud (cloud_normals);
    //extract_normals.setIndices (inliers_plane);
    ////extract_normals.filter (*cloud_normals2);
    //extract_normals.filter (*cloud_normals);

    // Remove statistical outlier again to remove extra points
    sor1.setInputCloud(outPCseg);  // if using pass through filter
    sor1.setMeanK(50);  //50  10
    sor1.setStddevMulThresh (0.001);  //1  0.5
    sor1.filter(*outPCseg);

// Pass to "plane_seg" topic for debugging
	// Convert to ROS
    pcl::toPCLPointCloud2(*outPCseg, *object);  // PointXYZ to PointCloud2
    pcl_conversions::fromPCL(*object, output); // PointCloud2 to sensor-msgs
    output.header.frame_id = input->header.frame_id;
    pub_plane_seg.publish(output);

	// Creating the KdTree object for the search method of the extraction
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (outPCseg);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.018); // 2cm  0.018
	ec.setMinClusterSize (100);     // 100
	ec.setMaxClusterSize (2500);   // 2500
	ec.setSearchMethod (tree);
	ec.setInputCloud (outPCseg);
	ec.extract (cluster_indices);

	//std::cerr << "Cluster size: " << cluster_indices.size() << std::endl;
	if (cluster_indices.size() > 0) {
		/*
		 * To separate each cluster out of the vector<PointIndices> we have to iterate through
		 * cluster_indices, create a new PointCloud for each entry and write all points of the
		 * current cluster in the Pointcloud.
		 */
		pcl::PointCloud<PointT>::Ptr clustered_cloud (new pcl::PointCloud<PointT>);
		std::vector<pcl::PointIndices>::const_iterator it;
		std::vector<int>::const_iterator pit;
		for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
			pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);

			for (pit = it->indices.begin(); pit != it->indices.end(); pit++) {
				// Add a point to the end of the existing vector
				cloud_cluster->points.push_back(outPCseg->points[*pit]);
			}

			// Check if it is a cylinder
		    // ESTIMATE POINT NORMALS
		    ne.setSearchMethod (tree);
		    ne.setInputCloud (cloud_cluster);
		    //ne.setKSearch (50);
			ne.setRadiusSearch (0.01);
		    ne.compute (*cloud_normals);
		
		    // SEGMENTATION FOR CYLINDER OBJECT
		    seg_cyl.setOptimizeCoefficients (true);
		    seg_cyl.setModelType (pcl::SACMODEL_CYLINDER);
		    seg_cyl.setMethodType (pcl::SAC_RANSAC);
		    seg_cyl.setMaxIterations (10);
		    seg_cyl.setDistanceThreshold (0.002);
		    seg_cyl.setRadiusLimits (0.02, 0.06);
		    seg_cyl.setInputCloud (cloud_cluster);
		    seg_cyl.setNormalDistanceWeight (0);
		    seg_cyl.setInputNormals (cloud_normals);
		
		    // OBTAIN CYLINDER INLIERS AND COEFFICIENTS
		    seg_cyl.segment (*inliers_cylinder, *coefficients_cylinder);
		    std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
			extract.setInputCloud (cloud_cluster);
			extract.setIndices (inliers_cylinder);
			extract.setNegative (false);
			extract.filter (*cloud_cylinder);
		// Pass to "test_cloud" topic to debug
		// Convert to ROS
		pcl::toPCLPointCloud2(*cloud_cylinder, *object);  // PointXYZ to PointCloud2
	   	//pcl::toPCLPointCloud2(*outPCseg, *object);  // PointXYZ to PointCloud2
    	pcl_conversions::fromPCL(*object, output); // PointCloud2 to sensor-msgs
    	output.header.frame_id = input->header.frame_id;
    	pub_temp.publish(output);
			ROS_INFO("cloud_cylinder POINTS: %lu\n",cloud_cylinder->points.size());
			ROS_INFO("cloud_cluster POINTS: %lu\n",cloud_cluster->points.size());
			double seg_frac = (double)cloud_cylinder->points.size()/(double)cloud_cluster->points.size();
			ROS_INFO("Fraction: %f\n",seg_frac);

			float point_threshold = 0.5;
			if (cloud_cylinder->points.size () <= point_threshold * cloud_cluster->points.size()) {
				std::cerr << "NOT CYLINDER" << std::endl;
				g_not_cylinder++;
				g_not_cyl_frac += seg_frac;
			} else {
				std::cerr << "CYLINDER" << std::endl;
				*clustered_cloud += *cloud_cylinder;
				g_cylinder++;
				g_cyl_frac += seg_frac;

				// Output centroid coordinate
				Eigen::Vector4f centroid;
				//pcl::compute3DCentroid(*outPCseg, cluster_indices[0], centroid);
				pcl::compute3DCentroid(*cloud_cylinder, centroid);
				std::cerr << centroid[0] << " " << centroid[1] << " " << centroid[2] << std::endl;
				geometry_msgs::Vector3 can_coord;
				can_coord.x = centroid[0];
				can_coord.y = centroid[1];
				can_coord.z = centroid[2];
				pub_coord.publish(can_coord);

			}
			double now_time = ros::Time::now().toSec() - g_start_time;
			ROS_INFO("Time: %f\n", now_time);

			ROS_INFO("Not Cylinder/second: %f\n", g_not_cylinder/now_time);
			ROS_INFO("Avg Not Cylinder fraction: %f\n", g_not_cyl_frac/g_not_cylinder);
			ROS_INFO("Percent Not Cylinder: %f\n", (float)g_not_cylinder/(float)(g_cylinder+g_not_cylinder));

			ROS_INFO("Cylinder/second: %f\n", g_cylinder/now_time);
			ROS_INFO("Avg Cylinder fraction: %f\n", g_cyl_frac/g_cylinder);
			ROS_INFO("Percent Cylinder: %f\n", (float)g_cylinder/(float)(g_cylinder+g_not_cylinder));

		}

		// Pass to "test_cloud" topic to debug
		// Convert to ROS
		pcl::toPCLPointCloud2(*cloud_normals, *object);  // PointXYZ to PointCloud2
    	pcl_conversions::fromPCL(*object, output); // PointCloud2 to sensor-msgs
    	output.header.frame_id = input->header.frame_id;
    	pub_temp.publish(output);

	}

	// Convert to ROS data type
	pcl::toPCLPointCloud2(*cloud_cylinder, *object);  // PointXYZ to PointCloud2
	pcl_conversions::fromPCL(*object, output); // PointCloud2 to sensor-msgs
	output.header.frame_id = input->header.frame_id;
	
	pub_cloud.publish(output);
}

void 
vision_cb (const std_msgs::Bool input) {
	if (input.data) 
		g_run_vision = true;
	else
		g_run_vision = false;
}

int
main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "detect_can");
	ros::NodeHandle nh;

	g_start_time = ros::Time::now().toSec();

	// Create subscriber to check if we should start running
	ros::Subscriber start_vision = nh.subscribe("start_vision", 1, vision_cb);

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb);
		
	// Create a ROS publisher for the output cloud
	pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("detect_can_cloud", 1);
	// Create a ROS publisher for cluster center
	pub_coord = nh.advertise<geometry_msgs::Vector3> ("cluster_center", 1);

	pub_temp = nh.advertise<sensor_msgs::PointCloud2> ("test_cloud", 1);
	pub_plane_seg = nh.advertise<sensor_msgs::PointCloud2> ("plane_seg", 1);

	// Spin
	ros::spin ();
}

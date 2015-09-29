
#include "detect_cylinder.h"

DetectCylinder::DetectCylinder() {

	// Init for statistics
	start_time = ros::Time::now().toSec();
	not_cyl_cnt = 0;
	cyl_cnt = 0;
	not_cyl_frac = 0.0;	// fraction when not cylinder detected
	cyl_frac = 0.0;		// fraction when cylinder detected
}

/*
 * Constructor that takes care of publishers and subscribers
 * INPUT:
 * 	nh - ROS node handle that we can use to publish/subscribe
 * 	pub_cloud - name of topic we'll publish cylinder cloud to
 *	pub_center - name of topic we'll publish center coordinates to
 */
DetectCylinder::DetectCylinder(ros::NodeHandle nh, std::string cloud, std::string center) {
    sub = nh.subscribe("/camera/depth/points", 1, &DetectCylinder::detect_cyl_callback,
         this);
	pub_cloud = nh.advertise<sensor_msgs::PointCloud2>(cloud,1);
    pub_center = nh.advertise<geometry_msgs::Vector3>(center,10);

    // Init for statistics
    start_time = ros::Time::now().toSec();
    not_cyl_cnt = 0;
    cyl_cnt = 0;
    not_cyl_frac = 0.0; // fraction when not cylinder detected
    cyl_frac = 0.0;     // fraction when cylinder detected

}

DetectCylinder::~DetectCylinder() {
}

/*
 * Callback function for subscriber when it gets new point cloud data
 * INPUT
 * 	sensor_msg ptr from subscribing to Kinect's point cloud
 */
void
DetectCylinder::detect_cyl_callback(const sensor_msgs::PointCloud2ConstPtr& input) {

	// Re-initialize our pointers
	cloud.reset(new pcl::PointCloud<PointT>);
	cloud_cylinder.reset(new pcl::PointCloud<PointT>);
	orig_input = input;			// save original input because we'll need header later

	// Convert sensor_msgs to PCLPointCloud2 to pcl::PointXYZ so we can use it with PCL 
	pcl::PCLPointCloud2 temp_cloud;
    pcl_conversions::toPCL (*input, temp_cloud);
    pcl::fromPCLPointCloud2(temp_cloud, *cloud);

	// Pass through filter to remove points outside of desired view
	DetectCylinder::pass_through_filter();

	// Remove statistical outliers
	DetectCylinder::remove_outliers();

	// Downsample using voxels
	DetectCylinder::down_sample();

	// Remove outliers again for better perf?
	//DetectCylinder::remove_outliers();

	// Extract planes from point cloud
	DetectCylinder::remove_planes();

	// Remove outliers again
	DetectCylinder::remove_outliers();

	// Euclidean clustering
	DetectCylinder::euclidean_cluster();

	// Detect cylinder
	DetectCylinder::detect_for_cylinder();

	// Publish cylinder cloud and center coordinates
	DetectCylinder::publish(&pub_cloud, &pub_center);
}

/*
 * Check for each cluster in cluster_indices for cylinder. 
 */
void
DetectCylinder::detect_for_cylinder() {

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
                cloud_cluster->points.push_back(cloud->points[*pit]);
            }

			// Check if current cluster is a cylinder
			DetectCylinder::segment_cylinder(cloud_cluster);
		}
	}
}

/*
 * Display statistics from segmenting cylinder.
 * INPUT:
 * 	seg_frac - segment fraction. Fraction of segmented points over all points in cluster
 * 	cluster_points - number of total cluster points.
 */
void
DetectCylinder::display_stats(double seg_frac, long unsigned int cluster_points) {
	double now_time = ros::Time::now().toSec() - start_time;

	ROS_INFO("cloud_cylinder POINTS: %lu\n", cloud_cylinder->points.size());
	ROS_INFO("cloud_cluster POINTS: %lu\n", cluster_points);
	ROS_INFO("Fraction: %f\n", seg_frac);

	ROS_INFO("Time: %f\n", now_time);

	ROS_INFO("Not Cylinder/second: %f\n", not_cyl_cnt / now_time);
	ROS_INFO("Avg Not Cylinder fraction: %f\n", not_cyl_frac / not_cyl_cnt);
	ROS_INFO("Percent Not Cylinder: %f\n", (float)not_cyl_cnt / (float)(cyl_cnt + not_cyl_cnt));

	ROS_INFO("Cylinder/second: %f\n", cyl_cnt / now_time);
	ROS_INFO("Avg Cylinder fraction: %f\n", cyl_frac / cyl_cnt);
	ROS_INFO("Percent Cylinder: %f\n", (float)cyl_cnt / (float)(cyl_cnt + not_cyl_cnt));
}

/*
 * Downsample using Voxels.
 */
void 
DetectCylinder::down_sample() {

    pcl::VoxelGrid<PointT> sor2;

	sor2.setInputCloud (cloud);
    sor2.setLeafSize(0.01,0.01,0.01);
    sor2.filter(*cloud);
}

/*
 * Euclidean clustering
 */
void
DetectCylinder::euclidean_cluster() {
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

	// Create the KdTree object for the search method of the extraction
    tree->setInputCloud (cloud);

    ec.setClusterTolerance (0.018); // 2cm  0.018
    ec.setMinClusterSize (100);     // 100
    ec.setMaxClusterSize (2500);    // 2500
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

	ROS_INFO("Cluster size: %lu\n", cluster_indices.size());
}

/*
 * Pass through filter to remove points outside of desired view.
 */
void
DetectCylinder::pass_through_filter() {

    pcl::PassThrough<PointT> pass;

    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0,1.5);  	// filter out points greater than 1.5m
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0,1);
    //pass.setFilterFieldName("y");
    //pass.setFilterLimits(0,8);
    //pass.setFilterLimitsNegative(true);
    pass.filter(*cloud);			// write back to cloud
}

/*
 * Publish segmented cylinder cloud and cylinder center coordinate.
 * INPUT:
 * 	pub_cloud - Publisher to publish cylinder cloud to
 * 	pub_center - Publisher to publish center coordinate to
 */
void
DetectCylinder::publish(ros::Publisher *pub_cloud, ros::Publisher *pub_center) {

	// Convert to ROS data type
    pcl::PCLPointCloud2::Ptr object (new pcl::PCLPointCloud2);
    sensor_msgs::PointCloud2 output;

    pcl::toPCLPointCloud2(*cloud, *object);	// PointXYZ to PointCloud2
    //pcl::toPCLPointCloud2(*cloud_cylinder, *object);	// PointXYZ to PointCloud2
    pcl_conversions::fromPCL(*object, output); 			// PointCloud2 to sensor-msgs
    output.header.frame_id = orig_input->header.frame_id;

	pub_cloud->publish(output);
	pub_center->publish(center_coord);
}

/*
 * Remove statistical outliers.
 */
void
DetectCylinder::remove_outliers() {

    pcl::StatisticalOutlierRemoval<PointT> sor1;

    sor1.setInputCloud(cloud);
    sor1.setMeanK(50); 				// these are empirical values
    sor1.setStddevMulThresh (0.005);
    sor1.filter(*cloud);			// write back to cloud
}

/*
 * Remove planes from point cloud.
 */
void
DetectCylinder::remove_planes() {
	pcl::SACSegmentation<PointT> seg_plane;
	pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
	pcl::ExtractIndices<PointT> extract;

	// Remove plan using PCL RANSAC
	seg_plane.setOptimizeCoefficients (true);
    seg_plane.setModelType (pcl::SACMODEL_PLANE);
    seg_plane.setMethodType (pcl::SAC_RANSAC);
    seg_plane.setMaxIterations (100);
    seg_plane.setDistanceThreshold (0.007);  // 0.01 good but some artifacts

    int cloud_pts = cloud->points.size();
    // Keep segmenting planes until 10% of points are left
    while (cloud->points.size() > 0.1 * cloud_pts) {  // 0.08 also ok

        seg_plane.setInputCloud (cloud);
        // Obtain the plane inliers and coefficients
        seg_plane.segment (*inliers_plane, *coefficients_plane);

        //if (inliers_plane->indices.size() < 4) {
        if (inliers_plane->indices.size() < DetectCylinder::PTS_IN_PLANE) {
            ROS_INFO("Could not estimate plane.\n");
        } else {
            ROS_INFO("Plane segmentation successful.\n");
        }

		// Extract inliers
        extract.setInputCloud (cloud);
        extract.setIndices (inliers_plane);
        extract.setNegative (true);
        extract.filter (*cloud);

		ROS_INFO("Planar component: %d data points", cloud->width * cloud->height);
	}

	// Run one more plane removal b/c if there are no objects then there may be
	// one plane left.
	seg_plane.setInputCloud(cloud);
    seg_plane.segment(*inliers_plane, *coefficients_plane);
    extract.setInputCloud (cloud);
    extract.setIndices (inliers_plane);
    extract.setNegative (true);
    extract.filter (*cloud);
}

/*
 * Detect whether there is a cylinder in the given cluster indices.
 * NOTE: THIS ONLY SEGMENTS _ONE_ CYLINDER IN SCENE. THE "LAST" ONE IN THE CLUSTER.
 * If it is a cylinder:
 * 		(1) calculate center coordinate cylinder and save it
 *		(2) add cylinder cluster to cloud_cylinder variable
 * INPUT:
 * 	cloud_cluster - the ONE cluster of an object we'll be trying to segment a cylinder with
 */
void
DetectCylinder::segment_cylinder(pcl::PointCloud<PointT>::Ptr cloud_cluster) {

	// For segmenting cylinder
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_cyl;
	pcl::ExtractIndices<PointT> extract;
	pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    pcl::PointCloud<PointT>::Ptr temp_cluster;    // store extracted "cylinder" before verification

	// Estimate point normals
	ne.setSearchMethod (tree);
	ne.setInputCloud (cloud_cluster);
	ne.setRadiusSearch (0.01);
	ne.compute (*cloud_normals);
	
	// Segmentation for cylinder object
	seg_cyl.setOptimizeCoefficients (true);
	seg_cyl.setModelType (pcl::SACMODEL_CYLINDER);
	seg_cyl.setMethodType (pcl::SAC_RANSAC);
	seg_cyl.setMaxIterations (10);
	seg_cyl.setDistanceThreshold (0.002);
	seg_cyl.setRadiusLimits (0.02, 0.06);
	seg_cyl.setInputCloud (cloud_cluster);
	seg_cyl.setNormalDistanceWeight (0);
	seg_cyl.setInputNormals (cloud_normals);
	
	// Obtain cylinder inliers and coefficients
	seg_cyl.segment (*inliers_cylinder, *coefficients_cylinder);
	extract.setInputCloud (cloud_cluster);
	extract.setIndices (inliers_cylinder);
	extract.setNegative (false);
	extract.filter (*temp_cluster);

	// Fraction of points in the cylinder we segmented out vs the points of that cluster
	double seg_frac = (double)temp_cluster->points.size() /
		(double)cloud_cluster->points.size();           // for statistics

	// See 'cylinder accuracy' test results on how this threshold was chosen
	float point_threshold = 0.5;
	if (temp_cluster->points.size () <= point_threshold * cloud_cluster->points.size()) {
		not_cyl_cnt++;              // for statistics
		not_cyl_frac += seg_frac;   // for statistics
	} else {
		// build cluster(s) of accepted cylinder(s)
		*cloud_cylinder += *temp_cluster;
		cyl_cnt++;                  // for statistics
		cyl_frac += seg_frac;       // for statistics
		
		// Calculate and store centroid coordinate
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*temp_cluster, centroid);
		ROS_INFO("%f %f %f\n", centroid[0], centroid[1], centroid[2]);
		center_coord.x = centroid[0];
		center_coord.y = centroid[1];
		center_coord.z = centroid[2];
	}

	// Display statistics for accuracy, number of cylinders detected, etc.
	display_stats(seg_frac, cloud_cluster->points.size());
}

/*
 *  MAIN
 */
int
main (int argc, char** argv) {
	// Initialize ROS
	ros::init(argc, argv, "detect_cylinder");
	ros::NodeHandle nh;

	DetectCylinder *detect_cyl = new DetectCylinder(nh, "cylinder_cloud", "cluster_center");

	ros::Rate r(1);

	// Spin
	while(nh.ok()) {
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}

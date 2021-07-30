/** @file
 *
 * This class detects potential objects in point clouds
 *
 * @author Jan Razlaw
 */

#ifndef DETECTOR_H
#define DETECTOR_H

#include <mutex>
#include <chrono>
#include <iostream>
#include <fstream>
#include <math.h>

#include <laser_segmentation/point_type.h>

#include <multi_hypothesis_tracking_msgs/HypothesesFull.h>
#include <multi_hypothesis_tracking_msgs/ObjectDetections.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf/transform_listener.h>

#include <geometry_msgs/PoseArray.h>

#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>

#include <tf_conversions/tf_eigen.h>

#include <eigen_conversions/eigen_msg.h>

#include <pcl/common/transforms.h>

#include <pcl/segmentation/extract_clusters.h>

#include <pcl_ros/point_cloud.h>

namespace object_detection
{

/**
 *  Class to detect objects in 3D point clouds.
 */

class Detector
{
public:

  typedef laser_segmentation::PointXYZSegmentation     	  InputPoint;
	typedef pcl::PointCloud<InputPoint> 							      InputPointCloud;

	/**
	 * @brief Constructor.
	 *
	 * Parameters are needed to use this class within a ros node or nodelet.
	 */
	Detector(ros::NodeHandle node, ros::NodeHandle private_nh);
	virtual ~Detector();

private:

  /**
	 * @brief Tries to get the transform from from to to.
	 *
	 * @param[in]  to          frame to transform to.
	 * @param[in]  from        frame to transform from.
	 * @param[in]  when        time stamp of transformation.
	 * @param[out] transform   resulting transform.
	 *
	 * @return true if transform successfully found, false otherwise
	 */
  bool tryGetTransform(const std::string& to,
                       const std::string& from,
                       const ros::Time& when,
                       Eigen::Affine3f& transform);

	/**
	 * @brief Transforms the cloud into the target_frame.
	 *
	 * @param[in,out] cloud 				the cloud that is transformed.
	 * @param[in] 		target_frame 	the frame the cloud should be transformed to.
	 *
	 * @return true if transform was successful, false otherwise
	 */
	bool transformCloud(InputPointCloud::Ptr& cloud, std::string target_frame);

	/**
	 * @brief Performs euclidean clustering on the points in cloud.
	 *
	 * Each cluster is represented by point indices that are stored in the entries
	 * of the cluster_indices vector.
	 *
	 * @param[in] 	cloud 						input cloud.
	 * @param[out] 	cluster_indices 	vector of clusters - each cluster represented
	 * 																	by several point indices.
	 * @param[in] 	cluster_tolerance distance threshold for euclidean clustering.
	 * @param[in] 	min_cluster_size 	minimal number of points a cluster has to have.
	 * @param[in] 	max_cluster_size 	maximal number of points a cluster is allowed to have.
	 *
	 * @return true if clustering was successful, false otherwise
	 */
	bool euclideanClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
													 std::vector<pcl::PointIndices>& cluster_indices,
													 float cluster_tolerance,
													 int min_cluster_size,
													 int max_cluster_size);

	/**
	 * @brief Computes the mean point and the size of a cluster.
	 *
	 * @param[in] 	cloud 				input cloud.
	 * @param[in] 	point_indices indices of the cluster's points.
	 * @param[out] 	mean 					mean point of the cluster's points.
	 * @param[out] 	min 					minimum in x, y and z direction of all points.
	 * @param[out] 	max 					maximum in x, y and z direction of all points.
	 */
	void getClusterProperties(const InputPointCloud::Ptr cloud,
														const pcl::PointIndices& point_indices,
														Eigen::Vector3f& mean,
														Eigen::Array3f& min,
														Eigen::Array3f& max,
                            bool& peripheral);

  /**
   * @brief Save currently tracked hypotheses.
   *
   * @param[in] 	hypotheses 	hypotheses from tracking.
   */
  void handleTracks(const multi_hypothesis_tracking_msgs::HypothesesFull::ConstPtr& hypotheses);

  /**
	 * @brief Detect all objects in a cloud that fit the description.
	 *
	 * First filter out all background points according to segmentation value in each point.
	 * Then cluster the remaining points by distance.
	 * Then filter clusters by description.
	 *
	 * @param[in] 	segmentation 	input cloud with segmentation values for each point.
	 */
	void handleCloud(const InputPointCloud::ConstPtr& segmentation);

	/**
	 * @brief Publishes empty messages in case no detection was made.
	 *
	 * This is needed for a tracking that should know that there are no detections for
	 * a specific time.
	 *
	 * @param[in] header  header for empty messages.
	 */
  void publishEmpty(const pcl::PCLHeader& header);

  tf::TransformListener m_tf;

  ros::Subscriber m_sub_cloud;
  ros::Subscriber m_sub_tracks;
  ros::Publisher m_pub_detections;
  ros::Publisher m_pub_vis_marker;
  ros::Publisher m_pub_clustered_cloud;

  float m_min_certainty_thresh;

  float m_cluster_tolerance;
  int m_min_cluster_size;
  int m_max_cluster_size;

	float m_min_object_height;
	float m_max_object_height;
	float m_max_object_width;
	float m_max_object_altitude;

  std::string m_fixed_frame;
  std::string m_sensor_frame;

  multi_hypothesis_tracking_msgs::HypothesesFull m_current_hypotheses;
  std::mutex m_hypotheses_mutex;

  float m_amplify_threshold;

  int m_region_growing_radius;
  float m_region_growing_distance_threshold;

  bool m_measure_time;
  std::chrono::microseconds m_summed_time_for_callbacks;
  int m_number_of_callbacks;
  std::ofstream m_time_file;
};

}

#endif

/** @file
 *
 * This class detects potential objects in point clouds
 *
 * @author Jan Razlaw
 */

#include <object_detection/detector.h>

namespace object_detection
{

Detector::Detector(ros::NodeHandle node, ros::NodeHandle private_nh)
 : m_min_certainty_thresh(0.5f)
   , m_cluster_tolerance(1.f)
   , m_min_cluster_size(4)
   , m_max_cluster_size(25000)
   , m_min_object_height(0.5f)
   , m_max_object_height(1.8f)
   , m_max_object_width(2.f)
   , m_max_object_altitude(2.f)
   , m_fixed_frame("world")
   , m_sensor_frame("velodyne")
   , m_amplify_threshold(1.f)
   , m_region_growing_radius(1)
   , m_region_growing_distance_threshold(1.f)
   , m_measure_time(false)
   , m_number_of_callbacks(0)
{
	ROS_INFO("Object_detector: Init...");
  
  private_nh.getParam("certainty_thresh", m_min_certainty_thresh);
  
  private_nh.getParam("cluster_tolerance", m_cluster_tolerance);
  private_nh.getParam("min_cluster_size", m_min_cluster_size);
  private_nh.getParam("max_cluster_size", m_max_cluster_size);

  private_nh.getParam("min_object_height", m_min_object_height);
  private_nh.getParam("max_object_height", m_max_object_height);
  private_nh.getParam("max_object_width", m_max_object_width);
  private_nh.getParam("max_object_altitude", m_max_object_altitude);

  private_nh.getParam("fixed_frame", m_fixed_frame);
  private_nh.getParam("sensor_frame", m_sensor_frame);
  private_nh.getParam("amplify_threshold", m_amplify_threshold);
  private_nh.getParam("region_growing_radius", m_region_growing_radius);
  private_nh.getParam("region_growing_distance_threshold", m_region_growing_distance_threshold);
  private_nh.getParam("measure_time", m_measure_time);
  if(m_measure_time)
  {
    std::string path_to_results_file = "/tmp/times_object_detection";
    m_time_file.open(path_to_results_file);
  }
  m_summed_time_for_callbacks = std::chrono::microseconds::zero();

  m_pub_detections = node.advertise<multi_hypothesis_tracking_msgs::ObjectDetections>("object_detections", 1);
  m_pub_vis_marker = node.advertise<visualization_msgs::MarkerArray>("object_detection_markers", 1);
  m_pub_clustered_cloud = node.advertise<InputPointCloud>("object_detection_cloud", 1);

  std::string tracks_topic = "/multi_object_tracking/hypotheses_full";
  private_nh.getParam("tracks_topic", tracks_topic);
  m_sub_tracks = node.subscribe(tracks_topic, 1, &Detector::handleTracks, this);

  std::string segments_topic = "/laser_segmenter_objects";
  private_nh.getParam("segments_topic", segments_topic);
  m_sub_cloud = node.subscribe(segments_topic, 1, &Detector::handleCloud, this);
}

Detector::~Detector()
{
  m_time_file.close();
}

bool Detector::tryGetTransform(const std::string& to,
                               const std::string& from,
                               const ros::Time& when,
                               Eigen::Affine3f& transform)
{
  if(!m_tf.waitForTransform(to, from, when, ros::Duration(0.5)))
  {
    ROS_ERROR_STREAM("Could not wait for transform from " << from << " to " << to);
    return false;
  }

  tf::StampedTransform transformTF;
  try
  {
    m_tf.lookupTransform(to, from, when, transformTF);
  }
  catch(tf::TransformException& e)
  {
    ROS_ERROR("Could not lookup transform to frame: '%s'", e.what());
    return false;
  }

  Eigen::Affine3d transform_double;
  tf::transformTFToEigen(transformTF, transform_double);
  transform = transform_double.cast<float>();
  return true;
}

bool Detector::transformCloud(InputPointCloud::Ptr& cloud, std::string target_frame)
{
  if(cloud->header.frame_id == target_frame)
    return true;

  ros::Time stamp = pcl_conversions::fromPCL(cloud->header.stamp);

  Eigen::Affine3f transform;
  if(!tryGetTransform(target_frame, cloud->header.frame_id, stamp, transform))
    return false;

  InputPointCloud::Ptr cloud_transformed(new InputPointCloud);
  pcl::transformPointCloud(*cloud, *cloud_transformed, transform);
  cloud_transformed->header = cloud->header;
  cloud_transformed->header.frame_id = target_frame;

  cloud.swap(cloud_transformed);

  return true;
}

bool Detector::euclideanClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                   std::vector<pcl::PointIndices>& cluster_indices,
                                   float cluster_tolerance,
                                   int min_cluster_size,
                                   int max_cluster_size)
{
  try
  {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance); // in m
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
  }
  catch(...)
  {
    ROS_ERROR("Object_detection: Clustering failed.");
    return false;
  }

  ROS_DEBUG_STREAM("Object_detector: Number of found clusters is " << cluster_indices.size() << ". ");
  return true;
}

void Detector::getClusterProperties(const InputPointCloud::Ptr cloud,
                                    const pcl::PointIndices& point_indices,
                                    Eigen::Vector3f& mean,
                                    Eigen::Array3f& min,
                                    Eigen::Array3f& max,
                                    bool& peripheral)
{
  mean = Eigen::Vector3f::Zero();
  min = Eigen::Array3f::Constant(std::numeric_limits<float>::max());
  max = Eigen::Array3f::Constant(-std::numeric_limits<float>::max());

  for(const auto& idx : point_indices.indices)
  {
    if(!peripheral && ((*cloud)[idx].ring == 0 || (*cloud)[idx].ring == 15))
      peripheral = true;

    auto pos = (*cloud)[idx].getVector3fMap();
    mean += pos;

    for(std::size_t i = 0; i < 3; ++i)
    {
      min[i] = std::min(min[i], pos[i]);
      max[i] = std::max(max[i], pos[i]);
    }
  }

  mean /= point_indices.indices.size();
}

void Detector::handleTracks(const multi_hypothesis_tracking_msgs::HypothesesFull::ConstPtr& hypotheses)
{
  std::lock_guard<std::mutex> guard(m_hypotheses_mutex);
  m_current_hypotheses = *hypotheses;
}

float getXYAngle(const Eigen::Vector3f& a, const Eigen::Vector3f& b)
{
  return std::atan2(a.x()*b.y()-a.y()*b.x(), a.x()*b.x()+a.y()*b.y());
}

void Detector::handleCloud(const InputPointCloud::ConstPtr& input_cloud)
{
  // start when subscribers are available
  if(m_pub_vis_marker.getNumSubscribers() == 0 &&
     m_pub_detections.getNumSubscribers() == 0 &&
     m_pub_clustered_cloud.getNumSubscribers() == 0)
  {
    ROS_DEBUG_STREAM("Object_detector: No subscriber.");
    return;
  }

  auto callback_start_time = std::chrono::high_resolution_clock::now();

  if(input_cloud->empty())
  {
    ROS_DEBUG("Object_detection: Received empty cloud.");
    publishEmpty(input_cloud->header);
    return;
  }

  InputPointCloud::Ptr segments_cloud(new InputPointCloud);
  std::vector<pcl::PointIndices> cluster_indices;

  std_msgs::Header header = pcl_conversions::fromPCL(input_cloud->header);
  // if cloud organized
  bool cloud_is_organized = input_cloud->height > 1;

  if(cloud_is_organized)
  {
    // get transform to sensor frame - needed for overlap estimation
    Eigen::Affine3f transform;
    if(!tryGetTransform(m_sensor_frame, header.frame_id , header.stamp, transform))
      return;

    pcl::copyPointCloud(*input_cloud, *segments_cloud);
    segments_cloud->is_dense = false; // needed to keep cloud organized through pcl::transformPointCloud later on

    // test if start and end of scan overlap
    float angle_between_ring_start_and_end = 0.f;
    for(int i = 0; i < 16; i++)
    {
      Eigen::Vector3f first_point_in_ring = segments_cloud->points[i * segments_cloud->width].getVector3fMap();
      Eigen::Vector3f last_point_in_ring = segments_cloud->points[(i + 1) * segments_cloud->width - 1].getVector3fMap();

      if(std::isfinite(first_point_in_ring.x()) && std::isfinite(last_point_in_ring.x()))
      {
        // transform points to sensor frame to be able to apply getXYAngle function correctly
        first_point_in_ring = transform * first_point_in_ring;
        last_point_in_ring = transform * last_point_in_ring;

        angle_between_ring_start_and_end = getXYAngle(first_point_in_ring, last_point_in_ring) * static_cast<float>(180.0 / M_PI);
        break;
      }

      ROS_ERROR("Couldn't estimate the overlap of the scan ends.");
    }

    // region growing to extract segments
    uint8_t cluster_id = 2;
    for(int col = 0; col < (int)segments_cloud->width; col++)
    {
      for(int row = 0; row < (int)segments_cloud->height; row++)
      {
        int point_index = col + segments_cloud->width * row;
        // if segment and not visited yet, start region growing
        if(segments_cloud->points[point_index].segmentation == 1)
        {
          pcl::PointIndices current_cluster_indices;
          std::queue<int> points_to_check;
          points_to_check.push(point_index);
          segments_cloud->points[point_index].segmentation = cluster_id;
          current_cluster_indices.indices.push_back(point_index);

          while(!points_to_check.empty())
          {
            int current_index = points_to_check.front();
            int current_col = current_index % (int)segments_cloud->width;
            int current_row = current_index / (int)segments_cloud->width;
            points_to_check.pop();

            // compute indices of neighbors that need to be checked wrt. radius
            int window_col_min = std::max(0, current_col - m_region_growing_radius);
            int window_col_max = std::min((int)segments_cloud->width - 1, current_col + m_region_growing_radius);
            int window_row_min = std::max(0, current_row - m_region_growing_radius);
            int window_row_max = std::min((int)segments_cloud->height - 1, current_row + m_region_growing_radius);
            for(int search_col = window_col_min; search_col <= window_col_max; search_col++)
            {
              for(int search_row = window_row_min; search_row <= window_row_max; search_row++)
              {
                int dist_to_current = abs(search_col - current_col) + abs(search_row - current_row);
                if(dist_to_current > m_region_growing_radius)
                  continue;

                int neighbor_index = search_col + segments_cloud->width * search_row;
                if(segments_cloud->points[neighbor_index].segmentation == 1)
                {
                  // if one object occludes another their points could be neighbors in the ordered cloud
                  // comparing their distances to the sensor is a simple way to keep them apart
                  Eigen::Vector3f current_point = segments_cloud->points[neighbor_index].getVector3fMap();
                  Eigen::Vector3f neighbor = segments_cloud->points[current_index].getVector3fMap();
                  current_point = transform * current_point;
                  neighbor = transform * neighbor;

                  float distance = fabsf(current_point.norm() - neighbor.norm());
                  if(distance > m_region_growing_distance_threshold)
                    continue;

                  points_to_check.push(neighbor_index);
                  segments_cloud->points[neighbor_index].segmentation = cluster_id;
                  current_cluster_indices.indices.push_back(neighbor_index);
                }
              }
            }
          }

          if((int)current_cluster_indices.indices.size() >= m_min_cluster_size &&
             (int)current_cluster_indices.indices.size() <= m_max_cluster_size)
          {
            cluster_indices.push_back(current_cluster_indices);
          }

          cluster_id++;

          if(cluster_id == UINT8_MAX)
            cluster_id = 2; // the exact cluster_id is for visualization purposes only. what matters is it being >1

        }
      }
    }


    // if start and end overlap, merge clusters that represent the same object in the real world
    if(angle_between_ring_start_and_end <= 0.f)
    {
      // convert the overlapping angle into the number of points that are in the overlapping region on one side of the ring
      const float absolute_angle_between_ring_start_and_end = fabsf(angle_between_ring_start_and_end);
      float degrees_per_point = (360.f + absolute_angle_between_ring_start_and_end) / (float)segments_cloud->width;
      int overlap_in_number_of_points = (int)std::ceil(absolute_angle_between_ring_start_and_end / degrees_per_point);
      int min_bound = overlap_in_number_of_points;
      int max_bound = (int)segments_cloud->width - overlap_in_number_of_points;

      int points_without_overlap = (int)segments_cloud->width - overlap_in_number_of_points;

      // find all clusters that could be in the overlapping region
      std::vector<size_t> horizontally_peripheral_clusters;
      for(size_t cluster_index = 0; cluster_index < cluster_indices.size(); cluster_index++)
      {
        for(size_t point_index = 0; point_index < cluster_indices[cluster_index].indices.size(); point_index++)
        {
          int current_col = cluster_indices[cluster_index].indices[point_index] % (int)segments_cloud->width;
          if(current_col <= min_bound || current_col >= max_bound)
          {
            horizontally_peripheral_clusters.push_back(cluster_index);
            break;
          }
        }
      }

      // for those clusters compute min and max indices ~ bounding boxes
      std::vector<std::pair<size_t, size_t>> min_max_col_indices;
      std::vector<std::pair<size_t, size_t>> min_max_row_indices;
      for(size_t i = 0; i < horizontally_peripheral_clusters.size(); i++)
      {
        size_t cluster_index = horizontally_peripheral_clusters[i];
        int min_col_index = std::numeric_limits<int>::max();
        int max_col_index = -std::numeric_limits<int>::max();
        int min_row_index = std::numeric_limits<int>::max();
        int max_row_index = -std::numeric_limits<int>::max();
        for(size_t point_index = 0; point_index < cluster_indices[cluster_index].indices.size(); point_index++)
        {
          int current_col = cluster_indices[cluster_index].indices[point_index] % (int)segments_cloud->width;
          min_col_index = std::min(min_col_index, current_col);
          max_col_index = std::max(max_col_index, current_col);
          int current_row = cluster_indices[cluster_index].indices[point_index] / (int)segments_cloud->width;
          min_row_index = std::min(min_row_index, current_row);
          max_row_index = std::max(max_row_index, current_row);
        }
        // "modulo" on those that lay in the overlapping area at the end of a scan
        if(max_col_index >= points_without_overlap)
        {
          max_col_index -= points_without_overlap;
          min_col_index -= points_without_overlap;
          min_col_index = std::max(min_col_index, 0);
        }
        min_max_col_indices.emplace_back(std::pair<size_t, size_t>(min_col_index, max_col_index));
        min_max_row_indices.emplace_back(std::pair<size_t, size_t>(min_row_index, max_row_index));
      }

      // find clusters that overlap and merge those
      std::vector<bool> peripheral_cluster_merged(horizontally_peripheral_clusters.size(), false);
      for(size_t i = 0; i < horizontally_peripheral_clusters.size(); i++)
      {
        if(peripheral_cluster_merged[i])
          continue;

        for(size_t j = i + 1; j < horizontally_peripheral_clusters.size(); j++)
        {
          if(peripheral_cluster_merged[j])
            continue;

          // check for overlap
          Eigen::Array2i min_box_1(min_max_col_indices[i].first, min_max_row_indices[i].first);
          Eigen::Array2i max_box_1(min_max_col_indices[i].second + 1, min_max_row_indices[i].second + 1);
          Eigen::Array2i min_box_2(min_max_col_indices[j].first, min_max_row_indices[j].first);
          Eigen::Array2i max_box_2(min_max_col_indices[j].second + 1, min_max_row_indices[j].second + 1);

          Eigen::Array2i intersection_min_corner = min_box_2.max(min_box_1);
          Eigen::Array2i intersection_max_corner = max_box_2.min(max_box_1);

          Eigen::Array2i diff = (intersection_max_corner - intersection_min_corner).max(Eigen::Array2i::Zero());

          int intersection_volume = diff.prod();

          if(intersection_volume > 0)
          {
            peripheral_cluster_merged[j] = true;
            size_t first_cluster_index = horizontally_peripheral_clusters[i];
            size_t second_cluster_index = horizontally_peripheral_clusters[j];

            auto cluster_id_of_first = segments_cloud->points[cluster_indices[first_cluster_index].indices[0]].segmentation;
            for(int point_index = 0; point_index < (int)cluster_indices[second_cluster_index].indices.size(); point_index++)
            {
              const int current_point_index = cluster_indices[second_cluster_index].indices[point_index];
              segments_cloud->points[current_point_index].segmentation = cluster_id_of_first;
            }

            cluster_indices[first_cluster_index].indices.insert(cluster_indices[first_cluster_index].indices.end(),
                                                                cluster_indices[second_cluster_index].indices.begin(),
                                                                cluster_indices[second_cluster_index].indices.end());
            cluster_indices[second_cluster_index].indices.clear();
          }
        }
      }
    }
  }
  else
  {
    // filter out background points
    segments_cloud->header = input_cloud->header;

    for(const auto& point : *input_cloud)
    {
      if(point.segmentation < m_min_certainty_thresh)
        continue;

      segments_cloud->push_back(point);
    }
    ROS_DEBUG("Object_detector: segments_cloud size is %lu ", segments_cloud->size());

    if(segments_cloud->empty())
    {
      ROS_DEBUG("Object_detection: No positive segmentation points left for detection");
      publishEmpty(segments_cloud->header);
      return;
    }

    // workaround for usage with nodelet
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::copyPointCloud(*segments_cloud, *cloud_xyz);

    if(segments_cloud->size() != cloud_xyz->size())
      ROS_ERROR_STREAM("Object_detection: cloud sizes do not match after copy.");

    // Cluster segments by distance
    if(!euclideanClustering(cloud_xyz, cluster_indices, m_cluster_tolerance, m_min_cluster_size, m_max_cluster_size))
      ROS_DEBUG("Object_detection: Clustering resulted in no detections.");

  }

  // transform cloud to fixed frame
  if(!transformCloud(segments_cloud, m_fixed_frame))
  {
    ROS_ERROR("Object_detection: Transform error.");
    publishEmpty(segments_cloud->header);
    return;
  }

  // propagate all hypotheses' positions into the time of the current cloud
  std::vector<Eigen::Vector3d> predicted_states;
  {
    std::lock_guard<std::mutex> guard(m_hypotheses_mutex);
    predicted_states.reserve(m_current_hypotheses.states.size());

    double dt = (header.stamp - m_current_hypotheses.header.stamp).toSec();
    for(const auto& state : m_current_hypotheses.states)
    {
      predicted_states.emplace_back(Eigen::Vector3d(state.position.x + dt * state.velocity.x,
                                                    state.position.y + dt * state.velocity.y,
                                                    state.position.z + dt * state.velocity.z));
    }
  }

  multi_hypothesis_tracking_msgs::ObjectDetectionsPtr detections_msg(new multi_hypothesis_tracking_msgs::ObjectDetections());
  detections_msg->header.frame_id = m_fixed_frame;
  detections_msg->header.stamp = header.stamp;

  visualization_msgs::MarkerArray marker_array;

  geometry_msgs::PoseArray pose_msgs;
  pose_msgs.header = detections_msg->header;

  // filter clusters and publish as marker array
  for(std::size_t i = 0; i < cluster_indices.size(); ++i)
	{
	  if(cluster_indices[i].indices.empty())
	    continue;

		Eigen::Vector3f mean;
		Eigen::Array3f min, max;
		bool is_peripheral = false;
		getClusterProperties(segments_cloud, cluster_indices[i], mean, min, max, is_peripheral);

		Eigen::Vector3f size = max.matrix() - min.matrix();

		// search for a hypothesis that is close to this cluster
		bool loosen_thresholds = false;
    for(const auto& predicted_state : predicted_states)
    {
      float distance = (mean - predicted_state.cast<float>()).norm();
      if(distance < m_amplify_threshold)
      {
        loosen_thresholds = true;
        break;
      }
    }

    bool filter_cluster = false;

    // check if detection fits description
    if(min.z() > m_max_object_altitude)
      filter_cluster = true;

		if(size.z() < m_min_object_height && !is_peripheral)
      filter_cluster = true;

    if(size.z() > m_max_object_height)
      filter_cluster = true;

    float object_width = static_cast<float>(sqrt(pow(size.x(), 2) + pow(size.y(), 2)));
		if(object_width > m_max_object_width)
      filter_cluster = true;

		bool optimal_detection = !filter_cluster; // flag to mark those detections that don't fit the real description

		if(filter_cluster && loosen_thresholds)
    {
      if(object_width < m_max_object_width * 2)
        filter_cluster = false;

      // only consider max_object_height without min_object_height
      if(size.z() < m_max_object_height)
        filter_cluster = false;

      optimal_detection = false;
    }

    // if filter_cluster still true then filter this cluster out
    if(filter_cluster)
      continue;

		if(!optimal_detection)
      ROS_DEBUG_STREAM("loosening description.");

    // Fill object detection message
    if(m_pub_detections.getNumSubscribers() > 0)
    {
      multi_hypothesis_tracking_msgs::ObjectDetection msg_detection;
      msg_detection.centroid.x = mean.x();
      msg_detection.centroid.y = mean.y();
      msg_detection.centroid.z = mean.z();

      msg_detection.position_covariance_xx = 0.03f * 0.03f;
      msg_detection.position_covariance_yy = 0.03f * 0.03f;
      msg_detection.position_covariance_zz = 0.03f * 0.03f;

      pcl::PointCloud<pcl::PointXYZ>::Ptr detection_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      detection_cloud->points.reserve(cluster_indices[i].indices.size());
      detection_cloud->width = static_cast<uint32_t>(cluster_indices[i].indices.size());
      detection_cloud->height = 1;
      msg_detection.point_ids.clear();
      for(auto idx : cluster_indices[i].indices)
      {
        msg_detection.point_ids.push_back(idx);
        detection_cloud->points.emplace_back(pcl::PointXYZ(segments_cloud->points[idx].x, segments_cloud->points[idx].y, segments_cloud->points[idx].z));
      }
      pcl::toROSMsg(*detection_cloud, msg_detection.cloud);

      msg_detection.class_a_detection = optimal_detection;

      detections_msg->object_detections.push_back(msg_detection);
    }

    // Fill visualization message
    if(m_pub_vis_marker.getNumSubscribers() > 0)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = m_fixed_frame;
      marker.header.stamp = header.stamp;
      marker.ns = "object_detector_namespace";
      marker.id = i;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = mean.x();
      marker.pose.position.y = mean.y();
      marker.pose.position.z = mean.z();
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = std::max(static_cast<float>(size.x()), 0.2f);
      marker.scale.y = std::max(static_cast<float>(size.y()), 0.2f);
      marker.scale.z = std::max(static_cast<float>(size.z()), 0.2f);
      marker.color.a = 0.8f;
      marker.color.r = 1.0;
      marker.color.g = 0.5;
      marker.color.b = 0.0;
      marker.lifetime = ros::Duration(0, 100000000); // 0.1 seconds

      marker_array.markers.push_back(marker);
    }
	}

  ROS_DEBUG_STREAM("Object_detector: Number of detections after filtering " << marker_array.markers.size() << ". ");

  if(m_pub_detections.getNumSubscribers() > 0)
    m_pub_detections.publish(detections_msg);
  if(m_pub_vis_marker.getNumSubscribers() > 0)
    m_pub_vis_marker.publish(marker_array);
  if(m_pub_clustered_cloud.getNumSubscribers() > 0)
    m_pub_clustered_cloud.publish(segments_cloud);

  if(m_measure_time && !input_cloud->empty())
  {
    std::chrono::microseconds time_for_one_callback = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - callback_start_time);
    ROS_DEBUG_STREAM("Time for detection one cloud: " << time_for_one_callback.count() << " microseconds.");
    m_time_file << (double)time_for_one_callback.count()/1000.0 << std::endl;
    m_summed_time_for_callbacks += time_for_one_callback;
    m_number_of_callbacks++;
    ROS_DEBUG_STREAM("Mean time for detection one cloud: " << m_summed_time_for_callbacks.count() / m_number_of_callbacks << " microseconds.");
  }
}

void Detector::publishEmpty(const pcl::PCLHeader& header)
{
  multi_hypothesis_tracking_msgs::ObjectDetectionsPtr detections_msg(new multi_hypothesis_tracking_msgs::ObjectDetections());
  detections_msg->header.frame_id = m_fixed_frame;
  detections_msg->header.stamp = pcl_conversions::fromPCL(header.stamp);

  visualization_msgs::MarkerArray marker_array;

  geometry_msgs::PoseArray pose_msgs;
  pose_msgs.header = detections_msg->header;

  if(m_pub_detections.getNumSubscribers() > 0)
    m_pub_detections.publish(detections_msg);
  if(m_pub_vis_marker.getNumSubscribers() > 0)
    m_pub_vis_marker.publish(marker_array);
}

}

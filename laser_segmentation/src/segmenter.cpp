/** @file
 *
 * This class segments objects of a specified width in laser point clouds
 *
 * @author Jan Razlaw
 */

#include <laser_segmentation/segmenter.h>

namespace laser_segmentation
{
Segmenter::Segmenter(ros::NodeHandle node, ros::NodeHandle private_nh)
: PUCK_NUM_RINGS(16)
 , HOKUYO_NUM_RINGS(1)  
 , m_input_is_velodyne(true)       
 , m_circular_buffer_capacity(6000)
 , m_angle_between_scanpoints(0.2f) // 0.1 for 5Hz 0.2 for 10Hz 0.4 for 20Hz
 , m_max_kernel_size(100)
 , m_max_intensity_range(100.f)
 , m_certainty_threshold(0.f)
 , m_dist_weight(0.75f)
 , m_intensity_weight(0.25f)
 , m_weight_for_small_intensities(10.f)
 , m_min_object_width(0.2f)
 , m_max_object_width(1.2f)
 , m_delta_min(2.5f)
 , m_delta_low(5.f)
 , m_delta_high(200.f)
 , m_delta_max(200.f)
 , m_buffer_initialized(false)
 , m_max_range(130.f)
 , m_almost_max_range(m_max_range - 5.f)
 , m_is_organized_cloud(false)
 , m_first_cloud(true)
 , m_use_all_neighbors_for_median(false)
 , m_self_filter_radius(1.f)
 , m_measure_time(false)
 , m_number_of_callbacks(0)
{
   ROS_INFO("laser_segmentation::Segmenter: Init...");

   m_pub_detection_cloud = node.advertise<Cloud4Detection >("/laser_segments4detection", 1);

   private_nh.getParam("certainty_threshold", m_certainty_threshold);

   private_nh.getParam("dist_weight", m_dist_weight);
   private_nh.getParam("intensity_weight", m_intensity_weight);

   private_nh.getParam("min_object_width", m_min_object_width);
   private_nh.getParam("max_object_width", m_max_object_width);

   private_nh.getParam("delta_min", m_delta_min);
   private_nh.getParam("delta_low", m_delta_low);
   private_nh.getParam("delta_high", m_delta_high);
   private_nh.getParam("delta_max", m_delta_max);
   
   private_nh.getParam("circular_buffer_capacity", m_circular_buffer_capacity);
   private_nh.getParam("max_kernel_size", m_max_kernel_size);
   private_nh.getParam("angle_between_scanpoints", m_angle_between_scanpoints);
   private_nh.getParam("use_all_neighbors_for_median", m_use_all_neighbors_for_median);
   private_nh.getParam("self_filter_radius", m_self_filter_radius);
   private_nh.getParam("measure_time", m_measure_time);
   if(m_measure_time)
   {
      std::string path_to_results_file = "/tmp/times_laser_segmentation";
      m_time_file.open(path_to_results_file);
   }
          
   m_summed_time_for_callbacks = std::chrono::microseconds::zero();

   std::string input_topic = "/velodyne_points";
   private_nh.getParam("input_topic", input_topic);
   private_nh.getParam("input_is_velodyne", m_input_is_velodyne);
   if(m_input_is_velodyne)
      m_velodyne_sub = node.subscribe(input_topic, 1, &Segmenter::velodyneCallback, this);
   else
      m_hokuyo_sub = node.subscribe(input_topic, 1, &Segmenter::hokuyoCallback, this);
}

void Segmenter::initBuffer(int number_of_rings)
{
  for(int i = 0; i < number_of_rings; i++)
  {
    BufferMediansPtr median_filtered_circ_buffer(new BufferMedians(m_circular_buffer_capacity));
    m_median_filtered_circ_buffer_vector.push_back(median_filtered_circ_buffer);
  }

  m_median_iters_by_ring.resize(number_of_rings);
  m_segmentation_iters_by_ring.resize(number_of_rings);

  m_buffer_initialized = true;
}

void Segmenter::resetBuffer()
{
  for(int ring = 0; ring < (int)m_median_filtered_circ_buffer_vector.size(); ring++)
  {
    m_median_filtered_circ_buffer_vector.at(ring)->clear();
    m_median_iters_by_ring.at(ring).reset();
    m_segmentation_iters_by_ring.at(ring).reset();
  }
}

void Segmenter::hokuyoCallback(const sensor_msgs::LaserScanConstPtr& input_scan)
{
  if(m_pub_detection_cloud.getNumSubscribers() == 0)
  {
    ROS_DEBUG_STREAM("Segmenter::hokuyoCallback: No subscriber to laser_segmentation. Resetting buffer.");
    resetBuffer();
    return;
  }

  if(!m_buffer_initialized)
    initBuffer(HOKUYO_NUM_RINGS);

  sensor_msgs::PointCloud2 cloud;
  InputPointCloud::Ptr cloud_transformed(new InputPointCloud());

  std::string frame_id = "base_link";

  if (!m_tf_listener.waitForTransform(frame_id, input_scan->header.frame_id, input_scan->header.stamp + ros::Duration().fromSec((input_scan->ranges.size()) * input_scan->time_increment), ros::Duration(0.1)))
  {
    ROS_ERROR_THROTTLE(10.0, "Segmenter::hokuyoCallback: Could not wait for transform.");
    return;
  }

  // transform 2D scan line to 3D point cloud
  try
  {
    m_scan_projector.transformLaserScanToPointCloud(frame_id, *input_scan, cloud, m_tf_listener, 35.f,
                                                    (laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance));

    // fix fields.count member
    for (unsigned int i = 0; i < cloud.fields.size(); i++)
      cloud.fields[i].count = 1;

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");
    sensor_msgs::PointCloud2Iterator<float> iter_distance(cloud, "distances");

    cloud_transformed->points.reserve(cloud.height * cloud.width);
    for(; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_distance)
    {
      if(*iter_distance >= m_almost_max_range || std::isnan(*iter_x))
        continue;

      InputPoint point;
      point.x = *iter_x;
      point.y = *iter_y;
      point.z = *iter_z;
      point.intensity = *iter_intensity;
      point.distance = *iter_distance;
      point.ring = 0;

      m_median_filtered_circ_buffer_vector[point.ring]->push_back(point);
    }
  }
  catch (tf::TransformException& exc)
  {
    ROS_ERROR_THROTTLE(10.0, "Segmenter::hokuyoCallback: No transform found.");
    ROS_ERROR_THROTTLE(10.0, "message: '%s'", exc.what());
  }

  processScan(pcl_conversions::toPCL(cloud.header));
}

void Segmenter::velodyneCallback(const InputPointCloud::ConstPtr &input_cloud)
{
  if(m_pub_detection_cloud.getNumSubscribers() == 0)
  {
    ROS_DEBUG_STREAM("Segmenter::velodyneCallback: No subscriber to laser_segmentation. Resetting buffer");
    resetBuffer();
    return;
  }
  
  auto callback_start_time = std::chrono::high_resolution_clock::now();

  if(!m_first_cloud && m_is_organized_cloud != (input_cloud->height > 1))
    ROS_FATAL("\nCloud switched between being organized and not being organized!!!\n");

  m_is_organized_cloud = input_cloud->height > 1;

  if(!m_buffer_initialized)
    initBuffer(PUCK_NUM_RINGS);

  for(const auto& point : input_cloud->points)
  {
    // skip this point if we do not use max range values and point is either at max range or is not valid at all
    if(!m_is_organized_cloud && (point.distance >= m_almost_max_range || std::isnan(point.x)))
      continue;

    m_median_filtered_circ_buffer_vector[point.ring]->push_back(point);
    // if point is not valid, set distance and intensity to invalid values
    if(std::isnan(m_median_filtered_circ_buffer_vector[point.ring]->back().point.x))
    {
      m_median_filtered_circ_buffer_vector[point.ring]->back().point.distance = m_max_range;
      m_median_filtered_circ_buffer_vector[point.ring]->back().point.intensity = -1.f;
    }
  }

  processScan(input_cloud->header);

  if(!input_cloud->empty())
    m_first_cloud = false;

  if(m_measure_time && !input_cloud->empty())
  {
    std::chrono::microseconds time_for_one_callback = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - callback_start_time);
    ROS_DEBUG_STREAM("Time for segmenting one cloud with " << (int)input_cloud->size() << " points : " << time_for_one_callback.count() << " microseconds.");
    m_time_file << (double)time_for_one_callback.count()/1000.0 << std::endl;
    m_summed_time_for_callbacks += time_for_one_callback;
    m_number_of_callbacks++;
    ROS_DEBUG_STREAM("Mean time for segmenting one cloud: " << m_summed_time_for_callbacks.count() / m_number_of_callbacks << " microseconds.");
  }
}

void Segmenter::processScan(pcl::PCLHeader header)
{
   Cloud4Detection::Ptr cloud4detection (new Cloud4Detection);
   cloud4detection->header = header;

#pragma omp parallel for schedule(dynamic) num_threads(4)
   // counting backwards because for m600 setup scan rings with higher index are scanning the ground when m600 is standing -> these rings take more time during filtering because of shorter distances -> more efficient to filter these first when omp is on
   for(auto ring = (int)m_median_filtered_circ_buffer_vector.size() - 1; ring >= 0; --ring)
   {
      // initialize member iterators
      if(!m_median_filtered_circ_buffer_vector.at(ring)->empty() && !m_median_iters_by_ring[ring])
        m_median_iters_by_ring[ring] = m_median_filtered_circ_buffer_vector.at(ring)->begin();

      if(!m_median_filtered_circ_buffer_vector.at(ring)->empty() && !m_segmentation_iters_by_ring[ring])
        m_segmentation_iters_by_ring[ring] = m_median_filtered_circ_buffer_vector.at(ring)->begin();

      // apply median filters to ring
      if(m_median_iters_by_ring[ring])
        filterRing(m_median_filtered_circ_buffer_vector.at(ring), *m_median_iters_by_ring.at(ring));
   }

   if(m_is_organized_cloud)
   {
     // find min distance of segmentation iterators to median iterators per ring
     int min_iterator_distance = std::numeric_limits<int>::max();
     for(auto ring = 0; ring < (int)m_median_filtered_circ_buffer_vector.size(); ++ring)
     {
       int iterator_distance_for_ring = static_cast<int>(std::distance(*m_segmentation_iters_by_ring.at(ring), *m_median_iters_by_ring.at(ring)));
       min_iterator_distance = std::min(min_iterator_distance, iterator_distance_for_ring);
     }

     // compute segments for each ring
     for(auto ring = 0; ring < (int)m_median_filtered_circ_buffer_vector.size(); ++ring)
       if(m_segmentation_iters_by_ring[ring])
         segmentRing(*m_segmentation_iters_by_ring.at(ring),
                     min_iterator_distance,
                     cloud4detection);


     cloud4detection->height = (int)m_median_filtered_circ_buffer_vector.size();
     cloud4detection->width = min_iterator_distance;
     cloud4detection->is_dense = false;
   }
   else
   {
     // compute segments for each ring
     for(auto ring = 0; ring < (int)m_median_filtered_circ_buffer_vector.size(); ++ring)
       if(m_segmentation_iters_by_ring[ring])
         segmentRing(*m_segmentation_iters_by_ring.at(ring),
                     *m_median_iters_by_ring.at(ring),
                     cloud4detection);

   }

   if(m_pub_detection_cloud.getNumSubscribers() > 0)
      m_pub_detection_cloud.publish(cloud4detection);
      
   ROS_DEBUG_STREAM("cloud4detection cloud with " << (int)cloud4detection->size() << " points");  
}

void Segmenter::calcMedianFromBuffer(const int noise_filter_kernel_radius,
                                     const int object_filter_kernel_radius,
                                     const BufferMediansPtr& buffer,
                                     const median_const_iterator& current_element,
                                     std::function<float(Segmenter::InputPoint)> getValue,
                                     std::function<bool(float)> isValid,
                                     const float invalid_value,
                                     float& noise_filter_result,
                                     float& object_filter_result) const
{
  assert(std::distance(buffer->begin(), buffer->end()) > object_filter_kernel_radius * 2 + 1);

  median_const_iterator noise_kernel_start = current_element - noise_filter_kernel_radius;
  median_const_iterator noise_kernel_end = current_element + noise_filter_kernel_radius;
  median_const_iterator object_kernel_start = current_element - object_filter_kernel_radius;
  median_const_iterator object_kernel_end = current_element + object_filter_kernel_radius;
  long int noise_kernel_start_offset = -1;
  long int noise_kernel_end_offset = -1;

  // get distances of neighbors
  std::vector<float> neighborhood_values;
  neighborhood_values.reserve(object_filter_kernel_radius * 2 + 1);

  // use all values in the specified neighborhood for the median computation
  if(!m_is_organized_cloud || m_use_all_neighbors_for_median)
  {
    median_const_iterator it_tmp = object_kernel_start;
    noise_kernel_start_offset = std::distance(object_kernel_start, noise_kernel_start);
    noise_kernel_end_offset = std::distance(object_kernel_start, noise_kernel_end);
    // use advance to cast const
    while(it_tmp <= object_kernel_end)
      neighborhood_values.push_back(getValue((*it_tmp++).point));
  }
  // else use only valid points
  else
  {
    // we want to make sure that the same number of points left and right of the current are represented in the kernel
    std::vector<float> start_values;
    start_values.reserve(object_filter_kernel_radius);
    int start_nan_counter = 0;

    std::vector<float> end_values;
    end_values.reserve(object_filter_kernel_radius);
    int end_nan_counter = 0;

    // fill vectors starting from current element moving to begin for start_values and equivalently for end_values
    median_const_iterator start_iterator = object_kernel_start;
    while(std::distance(start_iterator, current_element) > 0)
    {
      if(isValid(getValue(start_iterator->point)))
        start_values.push_back(getValue(start_iterator->point));
      else
        start_nan_counter++;

      start_iterator++;
    }

    median_const_iterator end_iterator = std::next(current_element);
    while(std::distance(end_iterator, object_kernel_end) >= 0)
    {
      if(isValid(getValue(end_iterator->point)))
        end_values.push_back(getValue(end_iterator->point));
      else
        end_nan_counter++;

      end_iterator++;
    }

    // subtract max count from object_filter_kernel_radius
    int max_nan_counter = std::max(start_nan_counter, end_nan_counter);
    int adapted_object_filter_kernel_radius = object_filter_kernel_radius - max_nan_counter;

    // if new kernel_radius is smaller or eqaul to noise_filter_kernel_radius fill results with invalid values and return
    if(adapted_object_filter_kernel_radius <= noise_filter_kernel_radius)
    {
      noise_filter_result = invalid_value;
      object_filter_result = invalid_value;
      return;
    }
    // else fuse vectors and current element to neighborhood_values
    else
    {
      auto start_values_begin = start_values.end();
      std::advance(start_values_begin, -adapted_object_filter_kernel_radius);
      neighborhood_values.insert(neighborhood_values.end(), start_values_begin, start_values.end());
      neighborhood_values.push_back(getValue(current_element->point));
      auto end_values_end = end_values.begin() + adapted_object_filter_kernel_radius;
      neighborhood_values.insert(neighborhood_values.end(), end_values.begin(), end_values_end);

      noise_kernel_start_offset = (int)std::distance(object_kernel_start, noise_kernel_start) - max_nan_counter;
      noise_kernel_end_offset = noise_kernel_start_offset + noise_filter_kernel_radius * 2;
    }
  }

  // get median of neighborhood distances within range of noise kernel
  long int noise_kernel_middle_offset = (noise_kernel_end_offset + noise_kernel_start_offset) / 2;
  std::nth_element(neighborhood_values.begin() + noise_kernel_start_offset,
                   neighborhood_values.begin() + noise_kernel_middle_offset,
                   neighborhood_values.begin() + noise_kernel_end_offset + 1);

  noise_filter_result = neighborhood_values[noise_kernel_middle_offset];

  // get median of neighborhood distances within range of object kernel
  std::nth_element(neighborhood_values.begin(), neighborhood_values.begin() + neighborhood_values.size() / 2, neighborhood_values.end());

  object_filter_result = neighborhood_values[neighborhood_values.size() / 2];
}

void Segmenter::filterRing(std::shared_ptr<boost::circular_buffer<MedianFiltered> > buffer_median_filtered,
                          median_iterator& iter)
{
   if(buffer_median_filtered->empty())
      return;

   while(std::distance(iter, buffer_median_filtered->end()) > 1)
   {
      if(iter->point.distance > m_almost_max_range || iter->point.distance <= m_self_filter_radius)
      {
         iter->dist_noise_kernel = m_max_range;
         iter->dist_object_kernel = m_max_range;
         iter->intens_noise_kernel = -1.f;
         iter->intens_object_kernel = -1.f;
         ++iter;
         continue;
      }

      // compute how many points are approximately on the objects with the minimal specified width
      float alpha = static_cast<float>(std::atan((m_min_object_width/2.f)/iter->point.distance) * (180.0 / M_PI));
      float points_on_minimal_object = alpha * 2.f / m_angle_between_scanpoints;
      // the kernel size at which the target object gets filtered out is when there are more
      // non-object points than object points in the median filter kernel. To stay slightly below
      // this border we floor the result of the computation above and treat this result as the kernel
      // radius (half of the kernel size). This way only noise and objects smaller than the target
      // objects get filtered out.
      int noise_filter_kernel_radius = (int)std::floor(points_on_minimal_object);

      noise_filter_kernel_radius = std::max(noise_filter_kernel_radius, 0);
      noise_filter_kernel_radius = std::min(noise_filter_kernel_radius, m_max_kernel_size / 2 - 1);

      // do as for noise kernel but instead of floor do ceil
      alpha = static_cast<float>(std::atan((m_max_object_width/2.f)/iter->point.distance) * (180.0 / M_PI));
      float points_on_maximal_object = alpha * 2.f / m_angle_between_scanpoints;

      int object_filter_kernel_radius = (int)std::ceil(points_on_maximal_object);
      object_filter_kernel_radius = std::max(object_filter_kernel_radius, 1);
      object_filter_kernel_radius = std::min(object_filter_kernel_radius, m_max_kernel_size / 2);

      // if current element has enough neighbors to both sides, compute medians
      if(std::distance(buffer_median_filtered->begin(), iter) >= object_filter_kernel_radius &&
         std::distance(iter, buffer_median_filtered->end()) > object_filter_kernel_radius)
      {
         if(m_dist_weight != 0.f)
         {
            calcMedianFromBuffer(noise_filter_kernel_radius,
                                 object_filter_kernel_radius,
                                 buffer_median_filtered,
                                 median_const_iterator(iter),
                                 [&](const InputPoint &fn) -> float { return fn.distance; },
                                 [&](const float value) -> float { return value < m_almost_max_range; },
                                 m_max_range,
                                 iter->dist_noise_kernel,
                                 iter->dist_object_kernel);
         }
         else
            iter->dist_noise_kernel = iter->dist_object_kernel = m_max_range;

         if(m_intensity_weight != 0.f)
         {
            calcMedianFromBuffer(noise_filter_kernel_radius,
                                 object_filter_kernel_radius,
                                 buffer_median_filtered,
                                 median_const_iterator(iter),
                                 [&](const InputPoint &fn) -> float { return fn.intensity; },
                                 [&](const float value) -> float { return value >= 0.f; },
                                 -1.f,
                                 iter->intens_noise_kernel,
                                 iter->intens_object_kernel);
         }
         else
            iter->intens_noise_kernel = iter->intens_object_kernel = -1.f;
      }

      // if there are not enough neighboring points left in the circular to filter -> break
      if(std::distance(iter, buffer_median_filtered->end()) <= object_filter_kernel_radius)
         break;

      ++iter;
   }
}

float Segmenter::computeSegmentationProbability(float delta, bool do_weighting)
{
   if(do_weighting)
   {
      // cap absolute difference to 0 - m_max_intensity_range
      // and do some kind of weighting, bigger weight -> bigger weight for smaller intensity differences
      delta = std::max(0.f, delta);
      delta = std::min(delta, m_max_intensity_range/m_weight_for_small_intensities);
      delta *= m_weight_for_small_intensities;
   }

   if(delta >= m_delta_min && delta < m_delta_low)
      return (m_delta_min - delta) / (m_delta_min - m_delta_low);

   if(delta >= m_delta_low && delta < m_delta_high)
      return 1.f;

   if(delta >= m_delta_high && delta < m_delta_max)
     return (m_delta_max - delta) / (m_delta_max - m_delta_high);

   // if delta out of min-max-bounds
   return 0.f;
}

void Segmenter::segmentRing(median_iterator& median_it,
                            int steps,
                            Cloud4Detection::Ptr& cloud4detection)
{
  median_iterator end = median_it;
  std::advance(end, steps);
  segmentRing(median_it,
              end,
              cloud4detection);
}

void Segmenter::segmentRing(median_iterator& median_it,
                            median_iterator& end,
                            Cloud4Detection::Ptr& cloud4detection)
{
   for(; median_it != end; ++median_it)
   {
      // compute differences and resulting certainty value
      float certainty_value = 0.f;
      float distance_delta = 0.f;
      if(m_dist_weight > 0.f)
      {
         distance_delta = (*median_it).dist_object_kernel - (*median_it).dist_noise_kernel;
         certainty_value += computeSegmentationProbability(distance_delta) * m_dist_weight;
      }

      float intensity_delta = 0.f;
      if(certainty_value > 0.f && m_intensity_weight > 0.f)
      {
         // use absolute distance here because we are just searching for objects that are different to their background
         intensity_delta = fabsf((*median_it).intens_noise_kernel - (*median_it).intens_object_kernel);
         certainty_value += computeSegmentationProbability(intensity_delta, true) * m_intensity_weight;
      }

      certainty_value = std::min(certainty_value, 1.0f);
      certainty_value = std::max(certainty_value, 0.0f);

      const auto& current_point = (*median_it).point;

      if(m_pub_detection_cloud.getNumSubscribers() > 0)
      {
         Point4Detection point4detection;
         if(current_point.distance > m_self_filter_radius)
         {
            point4detection.x = current_point.x;
            point4detection.y = current_point.y;
            point4detection.z = current_point.z;
            point4detection.segmentation = (certainty_value < m_certainty_threshold) ? 0 : 1;
         }
         else
         {
            point4detection.x = std::nanf("");
            point4detection.y = std::nanf("");
            point4detection.z = std::nanf("");
            point4detection.segmentation = 0;
         }
         point4detection.ring = current_point.ring;
         if(m_is_organized_cloud || std::isfinite(point4detection.x))
            cloud4detection->push_back(point4detection);

         if(current_point.distance > m_almost_max_range && point4detection.segmentation > 0)
            ROS_ERROR("\nPoint with max range is a segment! This shouldn't happen.\n");
      }
   }
}

} // namespace laser_segmentation

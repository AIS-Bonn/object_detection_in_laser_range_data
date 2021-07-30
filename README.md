# Laser Segmentation

The ROS packages provide:
- the [segmentation method](/laser_segmentation) for LiDAR point clouds 
- and the [detection method](/object_detection) based on the generated segments

used in our paper ["Detection and Tracking of Small Objects in Sparse 3D Laser Range Data"](https://arxiv.org/abs/1903.05889).  
The accompanying [video](https://www.youtube.com/watch?v=rl49G1rY5Pk) visualizes the setup and data.

# Limitations

The implementation in it's current form supports:
- sensor_msgs::LaserScan messages from Hokuyo single-scan-ring laser range scanners,
- and point clouds from the Velodyne Puck.  

It can easily be extended to support any LiDAR providing 360 degrees scan rings. 

The assumption in our setup was that the sensor is scanning the environment mostly horizontally - e.g. mounted at the bottom of an unmanned aerial vehicle.  
This assumption allowed to specify a minimal and maximal width of an object and segment all scan rings individually into point groups of the specified size. 

The input point cloud has to consist of points providing a field named distance.  
This field has to contain the distance from the sensor to the measurement as a floating point number.  
The [Usage](#usage) section links to a bag file with exemplary data. 

# Installation

The object_detection package depends on the [multi_hypothesis_tracking_msgs](https://github.com/AIS-Bonn/multi_hypothesis_tracking_msgs) ROS package.  
Clone and build all three packages as usual.  

Tested with Ubuntu 18.04 and ROS Melodic.

# Usage

To see a demo:
1) Download the [example bag file](https://www.dropbox.com/s/cwuc7k2p5csrlw6/simulated_humans_scanned_by_moving_velodyne_puck.bag?dl=0).  
2) Update the path to the bag file in the [visualize_on_simulated_data.launch](/object_detection/launch/visualize_on_simulated_data.launch) file.
3) Launch the launch file -  
`roslaunch object_detection visualize_on_simulated_data.launch`

The bag file contains simulated data from a moving Velodyne Puck.  
The simulated environment is populated with 50 simulated humans moving with random speeds to random points on a plane.  

The packages are part of a pipeline to segment, detect, and track multiple objects.  
To see the full demo:
- also clone and build the [multi_hypothesis_tracking](https://github.com/AIS-Bonn/multi_hypothesis_tracking) package,
- and set the _use_multi_hypothesis_tracker_ parameter at the top of [visualize_on_simulated_data.launch](/object_detection/launch/visualize_on_simulated_data.launch) to true.

# Citation

```
@inproceedings{razlaw2019detection,
    title={Detection and Tracking of Small Objects in Sparse 3D Laser Range Data},
    author={Razlaw, Jan and Quenzel, Jan and Behnke, Sven},
    booktitle={2019 International Conference on Robotics and Automation (ICRA)},
    pages={2967--2973},
    year={2019},
    organization={IEEE}
}
```

# License

The packages are licensed under BSD-3.

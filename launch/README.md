# Launch Files

### Table of Contents

* [Overview and Setup](../#terrain-treadmill)
* [Nodes](../src#nodes)
* [Launch Files](#launch-files)
	* [start](#start)
	* [aruco_marker_finder](#aruco_marker_finder)
	* [flea3](#flea3)
	* [serial](#serial)
* [Arduino Code](../Arduino#arduino-code)

## start
[start.launch](./start.launch) is used to startup the entire apparatus. This launch file allows the apparatus to be started without certain nodes for debugging purposes (see launch file for options).

The file can also be used to setup data collection for the device. Setting the `record` arg to true causes a rosbag to be recorded. By default, this file will be saved in the `~/.ros` directory. To change which topics are recorded, add or delete them from the [arguments list](./start.launch#L68-L69).

For more information on using rosbag, see the [package documentation](http://wiki.ros.org/rosbag). For more information on using rosbag to playback data using command-line arguments, see [here](http://wiki.ros.org/rosbag/Commandline).

## aruco_marker_finder
[aruco_marker_finder.launch](./aruco_marker_finder.launch) is used to start the [simple_single](https://github.com/pal-robotics/aruco_ros/blob/indigo-devel/aruco_ros/src/simple_single.cpp) node from the [aruco_ros package](https://github.com/pal-robotics/aruco_ros)  with the proper settings for tag detection. Before using, this launch file confirm that:
* The [markerId](./aruco_marker_finder.launch#L3) is properly specified. To generate new markers, modify [this url](http://terpconnect.umd.edu/~jwelsh12/enes100/marker.html?id=1&size_mm=70&padding_mm=5).
* The [markerSize](./aruco_marker_finder.launch#L4) has been measured and specified.
* The [camera topics](./aruco_marker_finder.launch#L11-L12) have been properly remapped.

## flea3
[flea3.launch](./flea3.launch) is used to start a Point Grey [Flea3 camera](https://www.ptgrey.com/support/downloads/10120) using the [flea3_single_node](https://github.com/KumarRobotics/flea3/tree/master/src/single) from the [flea3](https://github.com/KumarRobotics/flea3/tree/master/src) package. This launch file allows the camera parameters to be set to desired values. Camera tuning is disucssed in the main package [readme](../README.md#terrain-treadmill). After tuning, this launch file can also be used to calibrate the camera. Print out a [calibration board](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration?action=AttachFile&do=view&target=check-108.pdf) and measure the size of one of the squares. Then, calibrate the camera by running:
* `roslaunch terrain_treadmill flea3.launch calib:=true size:=8x6 square:=[measured square size]`

After, overwrite the old [calibration file](../config/camera.yaml) with your new calibration. For more information on using ROS to calibrate your camera, see the [camera_calibration](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration) package.


## serial
[serial.launch](./serial.launch) is used to start a serial communication stream between ROS and the Arduino. In order to start the serial stream, the serial [port](./serial.launch#L3) and [baud](./serial.launch#L4) must be properly specified. Specifically, the baud must match the [Arduino](../Arduino/terrain_treadmill/terrain_treadmill.ino#L12). Note, for the Arduino Due, the max serial rate is `115200`, but the default rate has proven to be more reliable. If higher serial rates are required, consider switching to a different microcontroller or communication protocol.

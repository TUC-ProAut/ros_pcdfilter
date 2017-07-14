/** \mainpage ProAut PcdFilter
 *
 * \section intro_sec Introduction
 *
 * This package was designed to remove simple geometric shapes from pointclouds.
 * E.g. A cube or a sphere. Several of these filters can be applied to a
 * pointcloud in one shot. For more details on the types of filters and
 * their parameters see <a href="#filters_sec">Filters</a>.
 *
 * This package includes a full node and a ros interface for one's own node.
 * For more details about the node, like topic names and parameters, see
 * <a href="#node_sec">Node</a>.
 *
 * The package can handle three different input types of pointclouds:
 * + <a href="http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html">sensor_msgs/PointCloud2</a>
 *   (this is also the output type)
 * + <a href="http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud.html">sensor_msgs/PointCloud</a>
 * + <a href="http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html">sensor_msgs/LaserScan</a>
 *
 * Additionaly the input data can be throttled to reduce the cpu load.
 *
 * \section node_sec Node
 * \verbatim
rosrun pcdfilter_pa pcdfilter_pa_node
\endverbatim
 *
 * <b>Input and Output Topics:</b>
 *
 * Topic Name      | Type                    | Description
 * ----------------|-------------------------|---------------------------------
 * "~in_cloud"     | sensor_msgs/PointCloud2 | Input as <em>new</em> pointcloud type.
 * "~in_cloud_old" | sensor_msgs/PointCloud  | Input as <em>old</em> pointloud type. Will be converted to new pointcloud type.
 * "~in_laser"     | sensor_msgs/LaserScan   | Input as single laser scan. Will be converted to new pointcloud type by package "laser_geometry".
 * "~out_cloud"    | sensor_msgs/PointCloud2 | Output of filtering node.
 *
 * All topics can be remapped using parameters (see below).
 *
 * <b>Services:</b>
 *
 * Service Name      | Type                    | Description
 * ------------------|-------------------------|---------------------------------
 * "~filter"         | PcdFilterPaCloud.srv    | Forced filtering via service call. No throttling is done and input/output cloud is part of service message.
 * "~add_filters"    | PcdFilterPaFilter.srv   | Adding new filters. Old filters are kept.
 * "~change_filters" | PcdFilterPaFilter.srv   | Adding new filters, but removing all old filters before adding.
 * "~enable"         | std_srvs/Empty          | Disables this node. This also disconnects the node from all input messages.
 * "~disable"        | std_srvs/Empty          | Enables this node.
 *
 * <b>Parameters:</b>
 *
 * Parameter Name        | Type              | Description
 * ----------------------|-------------------|-------------------------------------
 * "~topic_in_cloud"     | string            | Name of input topic for new pointclouds.
 * "~topic_in_cloud_old" | string            | Name of input topic for old pointclouds.
 * "~topic_in_laser"     | string            | Name of input topic for laser scans.
 * "~topic_out_cloud"    | string            | Name of output topic for filtered pointcloud.
 * "~filters"            | vector of strings | All pointcloud filters as an array of strings.
 * "~skip_count"         | integer           | Input throttling. Number of skipped messages after each processed message.
 * "~skip_time"          | double            | Input throttling. Time intervall in seconds. After each processed message, this intervall starts and all input messages will be skipped.
 * "~tf_lookup_time"     | double            | Maximum time in seconds for waiting for a specific TF transform.
 * "~buffer_pointcloud"  | bool              | Flag for buffering the last received pointcloud. This can be used to test different filters without resending the some pointcloud.
 * "~debugging"          | bool              | Flag for enabling extented output.
 * "~enabled"            | bool              | Flag for setting start up behaviour. If node is filtering or not at startup.
 * "~laser_nan_replacement_value" | double   | If a nan-value is represented within the laser scan, it might indicate "no obstacle within range". Therefore this parameters will replace those values with a fixed number.
 *
 * See also
 * <a href="https://github.com/peterweissig/ros_pcdfilter/blob/master/config/parameter.yaml">this config file</a>.
 * It contains all parameters and their standard value.
 *
 * \section filters_sec Filters
 *
 * <b>Definition:</b>
 * This is the definition of every single filter as a string.
 * \verbatim
[<inversion>]<type> <separator> <dimensions> <tf> [<tf_offset>] [<tf2> [<tf2_offset>]] [# <comment>]
\endverbatim
 *
 * + inversion: (optional)\n
 *    Inversion | Description
 *    ----------|--------------------------------------------------------------
 *    "" (none) | normal behaviour; points within the volume will be <em>filtered out</em>; all points outside will be <em>kept</em>                                           |
 *    "!"       | reversed behaviour; points within the volume will be <em>kept</em>; all points outside will be <em>filtered out</em>                                   |
 *
 * + type:\n
 *    Type       | Description                       | Nr Parameters | Nr TF ids
 *    -----------|-----------------------------------|---------------|----------
 *    "cube"     | specified by its sidelength       | 1 parameter   | 1 TF
 *    "sphere"   | specified by its radius           | 1 parameter   | 1 TF
 *    "block"    | cuboid specified by its three sidelengths in x,y and z direction | 3 parameters | 1 TF
 *    "cylinder" | specified by its radius and its height | 2 parameters | 1 TF
 *    "link"     | cylinder centered around the two given tfs; specified by its radius and its overshoot | 2 parameters | 2 TF
 *    "cone"     | cone headed at fist tf and pointing to second tf; specified by its height and its ratio (radius/height) | 2 parameters | 2 TF
 *
 * + separator:\n
 *    Seperator | Description
 *    ----------|--------------------------------------------------------------
 *    ":"       | current filter is <em>required</em>; if some tf is missing the whole filtering is aborted and no output pointcloud is generated
 *    "?"       | current filter <em>may be skipped</em>; if some tf is missing then the current filter is not applied; but in any case the whole filtering and output process is continued
 *
 *    To control how long the filter should wait for the necessary tf there
 *    exists a parameter "~tf_lookup_time" - see also
 *    <a href="#node_sec">Node</a>.
 *
 * + dimensions:\n
 *     Number of dimensions depends on the type of the current filter
 *     (see above). At least one dimension is always needed, e.g. for a cube.
 *     Three dimensions must be set for a cuboid.\n
 *     Each dimension is a single number seperated by spaces.
 *
 * + tf:\n
 *     ROS frame id for the filtered objects.
 *
 * + tf_offset (optional):\n
 *     This allows for an additional offset relative to the given frame id.
 *     Order of parameters:\verbatim
x y z  qx qy qz qw
\endverbatim
 *     Missing parameters will be set to "0". If qw is not set then it will be
 *     set to <em>sqrt(1 - qx^2 - qy^2 - qz^2)</em>
 *
 * + tf2 and tf2_offset (optional):\n
 *     Only valid for type "link" and "cone".
 *     For description see above (tf and tf_offset).
 *
 * + comment:\n
 *     Every character following '#' is ignored. This can be used to give a
 *     short comment.
 *
 * <b>Examples:</b>
 *
 * \verbatim
cube: 3 base_footprint
\endverbatim
 * Filters every point within the following cube - the cube is removed
 * from the pointcloud.\n
 * The sidelength is 3 [meters].\n
 * The center is aligned with TF frame "base_footprint".\n
 * Corners will be at <em>[+/-1.5 , +/- 1.5, +/- 1.5]</em>.
 *
 * \verbatim
!sphere: 2.3 map
\endverbatim
 * Only points within sphere will stay.\n
 * The center of sphere is aligned with TF frame "map".\n
 * The radius is 2.3 [meters] and the diameter is 4.6 [meters].
 *
 * \verbatim
block: 1 2 3 base_link 0.5 1 1.5
\endverbatim
 * Removes every point within the cuboid.\n
 * One corner of the cuboid is at center of TF frame "base_link".
 * This is due to the additional offset of <em>[0.5, 1 1.5]</em>.\n
 * Two opposite corners are at <em>[0, 0, 0]</em> and <em>[1, 2, 3]</em>.
 *
 * \verbatim
link: 0.2 0 arm_link1 arm_link2
\endverbatim
 * Removes every point within the cylinder, which is attached to both
 * TF frames.\n
 * The center of the top and bottom surface is exactly at origin of
 * one frame (zero overshoot).\n
 * The radius is 0.2 [meters].\n
 * The height depends only the relative position of the two frames.\n
 * Also the orientation of the cylinder is not fixed to any frame.
 *
 * \verbatim
!cone: 10 1 laser pan_tilt_tilt_link
\endverbatim
 * Removes every point outside the cone.\n
 * The top of cone will be at the origin of the TF frame "laser" and the
 * centerline will go straight through the origin of the TF frame
 * "pan_tilt_tilt_link".\n
 * The height is 10 [meters].\n
 * The angle at the top is 90 degrees (ratio of radius to height is 1:1).
 *
 * \section links_sec Links
 *
 * Source code at github:
 *  + https://github.com/peterweissig/ros_pcdfilter
 *
 * Related packages:
 *  + https://github.com/peterweissig/ros_parameter
 *
 * \section doc_sec ROS Documentation
 *
 * ROS-Distribution | Documentation
 * -----------------|---------------
 * Indigo  | <a href="http://docs.ros.org/indigo/api/pcdfilter_pa/html/index.html">docs.ros.org</a>
 * Jade    | <a href="http://docs.ros.org/jade/api/pcdfilter_pa/html/index.html">docs.ros.org</a>
 * Kinetic | <a href="http://docs.ros.org/kinetic/api/pcdfilter_pa/html/index.html">docs.ros.org</a>
 * Lunar   | <a href="http://docs.ros.org/lunar/api/pcdfilter_pa/html/index.html">docs.ros.org</a>
 **/
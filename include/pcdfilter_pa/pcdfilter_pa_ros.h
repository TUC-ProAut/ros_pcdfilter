/******************************************************************************
*                                                                             *
* pcdfilter_pa_ros.h                                                          *
* ==================                                                          *
*                                                                             *
*******************************************************************************
*                                                                             *
* github repository                                                           *
*   https://github.com/peterweissig/ros_pcdfilter                             *
*                                                                             *
* Chair of Automation Technology, Technische Universität Chemnitz             *
*   https://www.tu-chemnitz.de/etit/proaut                                    *
*                                                                             *
*******************************************************************************
*                                                                             *
* New BSD License                                                             *
*                                                                             *
* Copyright (c) 2015-2017, Peter Weissig, Technische Universität Chemnitz     *
* All rights reserved.                                                        *
*                                                                             *
* Redistribution and use in source and binary forms, with or without          *
* modification, are permitted provided that the following conditions are met: *
*     * Redistributions of source code must retain the above copyright        *
*       notice, this list of conditions and the following disclaimer.         *
*     * Redistributions in binary form must reproduce the above copyright     *
*       notice, this list of conditions and the following disclaimer in the   *
*       documentation and/or other materials provided with the distribution.  *
*     * Neither the name of the Technische Universität Chemnitz nor the       *
*       names of its contributors may be used to endorse or promote products  *
*       derived from this software without specific prior written permission. *
*                                                                             *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" *
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE   *
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE  *
* ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY      *
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES  *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR          *
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER  *
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT          *
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY   *
* OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH *
* DAMAGE.                                                                     *
*                                                                             *
******************************************************************************/

#ifndef __PCD_FILTER_PA_ROS_H
#define __PCD_FILTER_PA_ROS_H

// local headers
#include "pcdfilter_pa/pcdfilter_pa.h"
#include "pcdfilter_pa/pcdfilter_pa_ros_parameter.h"
#include "pcdfilter_pa/pcdfilter_pa_ros_throttle.h"
#include "pcdfilter_pa/pcdfilter_pa_ros_filter.h"

// ros headers
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/transform_datatypes.h>

// additional libraries
#include <opencv2/highgui/highgui.hpp>

// standard headers
#include <string>

//**************************[cPcdFilterPaRos]**********************************
class cPcdFilterPaRos : public cPcdFilterPa {
  public:

    //! default constructor
    cPcdFilterPaRos();

    //! default destructor
    ~cPcdFilterPaRos();

    //! converts the given pointcloud to newer format
    sensor_msgs::PointCloud2Ptr convertCloud(
      const sensor_msgs::PointCloudConstPtr &msg) const;

    //! converts the given laserscan to a pointcloud
    sensor_msgs::PointCloud2Ptr convertCloud(
      const sensor_msgs::LaserScanConstPtr &msg) const;

    //! updates the internal transformations
    //! this should be done for each pointcloud
    bool updateTf( const tf::TransformListener &tf_listener,
        const std::string base_frame,
        const ros::Time time = ros::Time::now());

    //! updates the internal transformations for a buffered pointcloud
    //! this should be done for each pointcloud
    bool updateTf( const tf::TransformListener &tf_listener);

    //! filters the given pointcloud
    bool filterCloud(const sensor_msgs::PointCloud2ConstPtr &msg,
      sensor_msgs::PointCloud2Ptr &result);

    //! filters the buffered pointcloud
    bool filterCloud(sensor_msgs::PointCloud2Ptr &result);

    //! internal function
    //! extracts the x,y and z coordinates to opencv matrix of float
    //! if force_copy is set the returned matrix is never pointing to the
    //!   original pointcloud message
    const cv::Mat convertCloudToOpencv(
      const sensor_msgs::PointCloud2ConstPtr &msg,
      const bool force_copy = false) const;

    //! internal function
    //! applys the mask to the pointcloud and creates a new pointcloud
    //!   to store the results
    sensor_msgs::PointCloud2Ptr applyMasktoCloud(
      const sensor_msgs::PointCloud2ConstPtr cloud,
      const std::vector<bool> mask) const;


    //! object for input throttling - e.g. only every 5th pointcloud
    cPcdFilterPaRosThrottle input_throttle_;

    //! object for filter handling
    cPcdFilterPaRosFilters filters_;

    //! ros specific parameter
    cPcdFilterPaRosParameter rosparams_;

    //! buffered pointcloud
    //!   (usage depends on flag rosparams_.buffered_pointcloud)
    cv::Mat pcd_buffered_points_;
    //! frame id of last pointcloud
    //!   (usage depends on flag rosparams_.buffered_pointcloud)
    sensor_msgs::PointCloud2ConstPtr pcd_buffered_msg_;

};


#endif // __PCD_FILTER_PA_ROS_H

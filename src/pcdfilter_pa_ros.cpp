/******************************************************************************
*                                                                             *
* pcdfilter_pa_ros.cpp                                                        *
* ====================                                                        *
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

// local headers
#include "pcdfilter_pa_ros.h"

// ros headers
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_geometry/laser_geometry.h>

//**************************[cPcdFilterPaRos]**********************************
cPcdFilterPaRos::cPcdFilterPaRos() {
}

//**************************[~cPcdFilterPaRos]*********************************
cPcdFilterPaRos::~cPcdFilterPaRos() {
}

//**************************[convertCloud]*************************************
sensor_msgs::PointCloud2Ptr cPcdFilterPaRos::convertCloud (
    const sensor_msgs::PointCloudConstPtr &msg) const {

    sensor_msgs::PointCloud2Ptr result(new sensor_msgs::PointCloud2);
    sensor_msgs::convertPointCloudToPointCloud2(*msg, *result);

    return result;
}

//**************************[convertCloud]*************************************
sensor_msgs::PointCloud2Ptr cPcdFilterPaRos::convertCloud(
  const sensor_msgs::LaserScanConstPtr &msg) const {

    laser_geometry::LaserProjection projector;

    sensor_msgs::PointCloud2Ptr result (new sensor_msgs::PointCloud2);

    if (rosparams_.laser_nan_replacement_value_ < 0) {
        projector.projectLaser(*msg, *result, -1,
          laser_geometry::channel_option::Default);
    } else {
        sensor_msgs::LaserScan msg_changed(*msg);

        for (int i = 0; i < msg_changed.ranges.size(); i++) {
            if (std::isnan(msg_changed.ranges[i])) {
                msg_changed.ranges[i] =
                  rosparams_.laser_nan_replacement_value_;
            }
        }

        projector.projectLaser(msg_changed, *result, -1,
          laser_geometry::channel_option::Default);
    }

    return result;
}

//**************************[updateTf]*****************************************
bool cPcdFilterPaRos::updateTf( const tf::TransformListener &tf_listener,
  const std::string base_frame, const ros::Time time) {

    bool result = filters_.update(tf_listener, base_frame, time,
      params_.filters_);

    if (rosparams_.debugging_) {
        for (int i = 0; i < params_.filters_.size(); i++) {
            ROS_INFO("%s:   updated filter[%d] (%s)",
              ros::this_node::getName().data(), i,
              params_.filters_[i].toString().data());
        }
    }

    return result;
}

//**************************[updateTf]*****************************************
bool cPcdFilterPaRos::updateTf(
  const tf::TransformListener &tf_listener) {

    if (pcd_buffered_msg_.use_count() < 1) {
        params_.filters_.clear();
        return false;
    }
    return updateTf(tf_listener, pcd_buffered_msg_->header.frame_id,
      pcd_buffered_msg_->header.stamp);
}

//**************************[filterCloud]**************************************
bool cPcdFilterPaRos::filterCloud(const sensor_msgs::PointCloud2ConstPtr &msg,
  sensor_msgs::PointCloud2Ptr &result) {

    pcd_buffered_msg_ = msg;

    // convert pointcloud to opencv
    pcd_buffered_points_= convertCloudToOpencv(msg);

    if (pcd_buffered_points_.rows == 0) {
        ROS_WARN("%s: size of pointcloud after conversion to opencv is 0",
          ros::this_node::getName().data());

        pcd_buffered_points_.release();
        pcd_buffered_msg_.reset();
        return false;
    }

    // filter pointcloud
    bool result_bool = filterCloud(result);

    if (!rosparams_.buffer_pointcloud_) {
        pcd_buffered_points_.release();
        pcd_buffered_msg_.reset();
    }
    return result_bool;
}

//**************************[filterCloud]**************************************
bool cPcdFilterPaRos::filterCloud(sensor_msgs::PointCloud2Ptr &result) {

    if ((pcd_buffered_msg_.use_count() < 1) ||
      (pcd_buffered_points_.empty())) {
        return false;
    }

    std::vector<bool> mask;
    std::vector<int> count;


    // filter pointcloud
    count = pointcloudFilter(pcd_buffered_points_, mask);

    // apply mask
    result = applyMasktoCloud(pcd_buffered_msg_, mask);

    if (rosparams_.debugging_) {
        std::stringstream ss;
        ss << "remaining points " << result->height << " (" <<
          pcd_buffered_points_.rows;
        for (int i = 0; i < count.size(); i++) {
            ss << " - " << count[i];
        }
        ss << ")";
        ROS_INFO("%s: %s", ros::this_node::getName().data(), ss.str().data());
    }

    return true;
}

//**************************[convertCloudToOpencv]*****************************
const cv::Mat cPcdFilterPaRos::convertCloudToOpencv(
    const sensor_msgs::PointCloud2ConstPtr &msg,
    const bool force_copy) const {

    // number of points
    int count = msg->height * msg->width;
    int point_step = msg->point_step;

    // copy info about the resulting columns (x,y,z, dummy)
    int types[4] = {-1, -1, -1, -1};
    int offsets[4] = {0, 0, 0, 0};
    for (int i = 0; i < msg->fields.size(); i++) {
        int position = 3;
        if ((msg->fields[i].name == "x") || (msg->fields[i].name == "X")) {
            position = 0;
        }
        if ((msg->fields[i].name == "y") || (msg->fields[i].name == "Y")) {
            position = 1;
        }
        if ((msg->fields[i].name == "z") || (msg->fields[i].name == "Z")) {
            position = 2;
        }

        types[position]   = msg->fields[i].datatype;
        offsets[position] = msg->fields[i].offset  ;
    }
    // check if all resulting columns (x,y,z) exist
    for (int i = 0; i < 3; i++) {
        if (types[i] < 0) {
            return cv::Mat();
        }
    }

    // check if simple reassignment of data is possible (instead of copying)
    bool simple_assign = true;
    for (int i = 0; i < 3; i++) {
        if ((types[0] != types[i]) || (offsets[i] != i * offsets[1])) {
            simple_assign = false;
            break;
        }
    }
    if (simple_assign) {
        if ((types[0] == sensor_msgs::PointField::FLOAT32) &&
          (offsets[1] == 4)) {
            cv::Mat result(count, 3, CV_32FC1,
              const_cast<uint8_t*> (&(msg->data.front())), point_step);
            if (force_copy) {
                if (rosparams_.debugging_) {
                    ROS_INFO("%s: opencv - simple copy of pointcloud)",
                      ros::this_node::getName().data());
                }
                cv::Mat result_copy;
                result.copyTo(result_copy);
                return result_copy;
            } else {
                if (rosparams_.debugging_) {
                    ROS_INFO("%s: opencv - reassignment of pointcloud)",
                      ros::this_node::getName().data());
                }
                return result;
            }
        }
        if ((types[0] == sensor_msgs::PointField::FLOAT64) &&
          (offsets[1] == 8)) {
            if (rosparams_.debugging_) {
                ROS_INFO("%s: opencv - reassignment of pointcloud"
                  " + conversion", ros::this_node::getName().data());
            }
            cv::Mat result;
            cv::Mat (count, 3, CV_64FC1,
              const_cast<uint8_t*> (&(msg->data.front())),
              point_step).convertTo(result, CV_32FC1);
            return result;
        }
    }

    // copy data (and maybe change data_type)
    if (rosparams_.debugging_) {
        ROS_INFO("%s: opencv - complex copy of pointcloud",
          ros::this_node::getName().data());
    }
    cv::Mat result(count, 3, CV_32FC1);
    int current_pos = 0;
    for (int i = 0; i < count; i++) {

        for (int j = 0; j < 3; j++) {
            const void *ptr = &(msg->data[current_pos + offsets[j]]);
            double value = -1;
            switch (types[j]) {
                case sensor_msgs::PointField::FLOAT64:
                    value = float(*((const double*  ) ptr));
                    break;
                case sensor_msgs::PointField::FLOAT32:
                    value =       *((const float*   ) ptr) ;
                    break;
                case sensor_msgs::PointField::INT32:
                    value = float(*((const int32_t* ) ptr));
                    break;
                case sensor_msgs::PointField::UINT32:
                    value = float(*((const uint32_t*) ptr));
                    break;
                case sensor_msgs::PointField::INT16:
                    value = float(*((const int16_t* ) ptr));
                    break;
                case sensor_msgs::PointField::UINT16:
                    value = float(*((const uint16_t*) ptr));
                    break;
                case sensor_msgs::PointField::INT8:
                    value = float(*((const int8_t*  ) ptr));
                    break;
                case sensor_msgs::PointField::UINT8:
                    value = float(*((const uint8_t* ) ptr));
                    break;
            }
            result.at<double>(i,j) = value;
        }
        current_pos+= point_step;
    }

    return result;
}

//**************************[applyMasktoCloud]*********************************
sensor_msgs::PointCloud2Ptr cPcdFilterPaRos::applyMasktoCloud(
  const sensor_msgs::PointCloud2ConstPtr cloud,
  const std::vector<bool> mask) const {

    int point_step = cloud->point_step;

    int size_old = cloud->height * cloud->width;
    if ((size_old == 0) && (point_step > 0)) {
        size_old = cloud->data.size() / point_step;
    }

    int size_new = mask.size() <= size_old ? mask.size() : size_old;
    int count = 0;
    for (int i = 0; i < size_new; i++) {
        if (mask[i]) {count++;}
    }
    size_new = count;


    sensor_msgs::PointCloud2Ptr result (new sensor_msgs::PointCloud2);
    result->header.frame_id = cloud->header.frame_id;
    result->header.stamp    = cloud->header.stamp;
    result->is_bigendian    = cloud->is_bigendian;
    result->is_dense        = cloud->is_dense;
    result->height          = size_new;
    result->width           = 1;
    result->point_step      = cloud->point_step;
    result->row_step        = cloud->point_step;

    result->fields          = cloud->fields;
    result->data.resize(size_new * point_step);

    uint8_t *pos_new = &result->data[0];
    const uint8_t *pos_old_base = &cloud->data[0];
    for (int i = 0; i < size_old; i++) {
        if (mask[i]) {
            memcpy(pos_new, pos_old_base + i*point_step, point_step);
            pos_new+= point_step;
        }
    }

    return result;
}

/******************************************************************************
*                                                                             *
* pcdfilter_pa_node.cpp                                                       *
* =====================                                                       *
*                                                                             *
*******************************************************************************
*                                                                             *
* github repository                                                           *
*   https://github.com/TUC-ProAut/ros_pcdfilter                               *
*                                                                             *
* Chair of Automation Technology, Technische Universität Chemnitz             *
*   https://www.tu-chemnitz.de/etit/proaut                                    *
*                                                                             *
*******************************************************************************
*                                                                             *
* New BSD License                                                             *
*                                                                             *
* Copyright (c) 2015-2018, Peter Weissig, Technische Universität Chemnitz     *
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
#include "pcdfilter_pa/pcdfilter_pa_node.h"

// ros headers
#include <parameter_pa/parameter_pa_ros.h>

// standard headers
#include <string>
#include <math.h>

//**************************[main]*********************************************
int main(int argc, char **argv) {

    ros::init(argc, argv, "pcd_filter_pa_node");
    cPcdFilterPaNode pcd_filter;

    ros::spin();

    return 0;
}

//**************************[cPcdFilterPaNode]*********************************
cPcdFilterPaNode::cPcdFilterPaNode() {
    cParameterPaRos paramloader;

    std::vector<std::string> temp_filter;
    paramloader.load("~/filters"          , temp_filter);

    paramloader.load("~/skip_count"       , input_throttle_.skip_count_  );
    paramloader.load("~/skip_time"        , input_throttle_.skip_time_   );

    paramloader.load("~/tf_lookup_time"   , filters_.tf_lookup_time_     );
    paramloader.load("~/buffer_pointcloud", rosparams_.buffer_pointcloud_);
    paramloader.load("~/debugging"        , rosparams_.debugging_        );
    paramloader.load("~/laser_nan_replacement_value",
      rosparams_.laser_nan_replacement_value_);

    bool enabled = true;
    paramloader.load("~/enabled"          , enabled                      );


    bool remapping = false;
    remapping|= paramloader.loadTopic("~/topic_in_cloud"    ,
        nodeparams_.topic_in_cloud_     );
    remapping|= paramloader.loadTopic("~/topic_in_cloud_old",
        nodeparams_.topic_in_cloud_old_ );
    remapping|= paramloader.loadTopic("~/topic_in_laser"    ,
        nodeparams_.topic_in_laser_     );

    paramloader.loadTopic("~/topic_out_cloud"               ,
        nodeparams_.topic_out_cloud_    );

    // Publisher for filtered pointcloud
    pub_pcd_ = nh_.advertise<sensor_msgs::PointCloud2>(
        nodeparams_.topic_out_cloud_, 10, true);

    std::string str_service = paramloader.resolveRessourcename("~/");
    
    // Service for adding additional filters
    ser_filter_ = nh_.advertiseService( str_service + "filter",
        &cPcdFilterPaNode::filterCallbackSrv, this);

    // Service for adding additional filters
    ser_add_filters_ = nh_.advertiseService( str_service + "add_filters",
        &cPcdFilterPaNode::addFiltersCallbackSrv, this);
    // Service for changing all filters
    ser_change_filters_ = nh_.advertiseService( str_service + "change_filters",
        &cPcdFilterPaNode::changeFiltersCallbackSrv, this);

    // Service for enabling filter node
    ser_enable_ = nh_.advertiseService( str_service + "enable",
        &cPcdFilterPaNode::enableCallbackSrv, this);
    // Service for disabling filter node
    ser_disable_ = nh_.advertiseService( str_service + "disable",
        &cPcdFilterPaNode::disableCallbackSrv, this);

    if (enabled) { enable();} else { disable();}

    if (! remapping) {
        ROS_WARN_STREAM(ros::this_node::getName()                          <<
          ": no input topic was remapped\n"                                <<
          "this seems to be a mistake - try the following:\n"              <<
          "  rosrun pcdfilter_pa pcdfilter_pa_node _topic_in_cloud:="      <<
          "/new_in_topic _topic_out_cloud:=/new_out_topic");
    }

    addFilters(temp_filter);
}

//**************************[~cPcdFilterPaNode]********************************
cPcdFilterPaNode::~cPcdFilterPaNode() {
}

//**************************[setCloudCallbackSub]******************************
void cPcdFilterPaNode::setCloudCallbackSub(
  const sensor_msgs::PointCloud2ConstPtr &msg) {

    if (rosparams_.debugging_) {
        int count = (msg->width > 1 ? msg->width : 1) * msg->height;
        ROS_INFO("%s: #### received new pointcloud (%d)",
          ros::this_node::getName().data(), count);
    }
    if (! input_throttle_.checkNewInput(msg->header.stamp)) {
        if (rosparams_.debugging_) {
            ROS_INFO("%s: skipped (input throttle)",
              ros::this_node::getName().data());
        }

        return;
    }

    if (! updateTf(tf_listener_, msg->header.frame_id, msg->header.stamp)) {
        return;
    }

    sensor_msgs::PointCloud2Ptr result;
    if (filterCloud(msg, result)) {
        if (rosparams_.debugging_) {
            ROS_INFO("%s: publishing filtered pointcloud",
              ros::this_node::getName().data());
        }
        pub_pcd_.publish(result);
    }
}

//**************************[setCloudOldCallbackSub]***************************
void cPcdFilterPaNode::setCloudOldCallbackSub(
  const sensor_msgs::PointCloudConstPtr &msg) {

    setCloudCallbackSub(convertCloud(msg));
}

//**************************[setCloudLaserCallbackSub]*************************
void cPcdFilterPaNode::setCloudLaserCallbackSub(
  const sensor_msgs::LaserScanConstPtr &msg) {

    setCloudCallbackSub(convertCloud(msg));
}

//**************************[addFiltersCallbackSrv]****************************
bool cPcdFilterPaNode::addFiltersCallbackSrv(
  pcdfilter_pa::PcdFilterPaFilter::Request  &req,
  pcdfilter_pa::PcdFilterPaFilter::Response &res) {

    addFilters(req.new_filter);
    res.current_filter = filters_.get();

    if (rosparams_.buffer_pointcloud_) {
        if (! updateTf(tf_listener_)) {
            return true;
        }
        sensor_msgs::PointCloud2Ptr cloud;
        if (filterCloud(cloud)){
            pub_pcd_.publish(cloud);
        }
    }
    return true;
}

//**************************[filterCallbackSrv]********************************
bool cPcdFilterPaNode::filterCallbackSrv(
  pcdfilter_pa::PcdFilterPaCloud::Request  &req,
  pcdfilter_pa::PcdFilterPaCloud::Response &res) {

    res.ok = false;

    sensor_msgs::PointCloud2ConstPtr msg(&req.in_cloud);
    sensor_msgs::PointCloud2Ptr result;

    if (! updateTf(tf_listener_, msg->header.frame_id,
      msg->header.stamp)) {
        return true;
    }

    if (filterCloud(msg, result)) {
        ROS_INFO("%s: service call for filtering",
          ros::this_node::getName().data());

        sensor_msgs::PointCloud2_<std::allocator<void> >::_data_type temp;
        temp.swap(result->data);
        res.out_cloud = *result;
        res.out_cloud.data.swap(temp);

        res.ok = true;
    }
    return true;

}

//**************************[addFilters]***************************************
void cPcdFilterPaNode::addFilters(
  const std::vector<std::string> &new_filters) {

    for (int i = 0; i < new_filters.size(); i++) {
        if (filters_.add(new_filters[i])) {
            ROS_INFO("%s: added new filter definition \"%s\"",
              ros::this_node::getName().data(), filters_.getLast().data());
        } else {
            ROS_WARN("%s: error in filter definition \"%s\"",
              ros::this_node::getName().data(), filters_.getLast().data());
        }
    }
}

//**************************[changeFiltersCallbackSrv]*************************
bool cPcdFilterPaNode::changeFiltersCallbackSrv(
  pcdfilter_pa::PcdFilterPaFilter::Request  &req,
  pcdfilter_pa::PcdFilterPaFilter::Response &res) {

    int count = filters_.size();
    filters_.clear();

    ROS_INFO("%s: removed all filters (%d)",
      ros::this_node::getName().data(), count);
    return addFiltersCallbackSrv(req, res);
}

//**************************[enableCallbackSrv]********************************
bool cPcdFilterPaNode::enableCallbackSrv(std_srvs::Empty::Request  &req,
  std_srvs::Empty::Response &res) {

    enable();
    return true;
}

//**************************[disableCallbackSrv]*******************************
bool cPcdFilterPaNode::disableCallbackSrv(std_srvs::Empty::Request  &req,
  std_srvs::Empty::Response &res) {

    disable();
    return true;
}

//**************************[enable]*******************************************
void cPcdFilterPaNode::enable() {

    ROS_INFO("%s: node enabled :-)", ros::this_node::getName().data());

    // Subscriber for pointclouds
    if (nodeparams_.topic_in_cloud_ != "") {
        sub_pcd_ = nh_.subscribe<sensor_msgs::PointCloud2>(
          nodeparams_.topic_in_cloud_, 10,
          &cPcdFilterPaNode::setCloudCallbackSub, this);
    }

    // Subscriber for pointclouds (old format)
    if (nodeparams_.topic_in_cloud_old_ != "") {
        sub_pcd_old_ = nh_.subscribe<sensor_msgs::PointCloud>(
          nodeparams_.topic_in_cloud_old_, 10,
          &cPcdFilterPaNode::setCloudOldCallbackSub, this);
    }

    // Subscriber for laserscans
    if (nodeparams_.topic_in_laser_ != "") {
        sub_laser_ = nh_.subscribe<sensor_msgs::LaserScan>(
          nodeparams_.topic_in_laser_, 10,
          &cPcdFilterPaNode::setCloudLaserCallbackSub, this);
    }
}

//**************************[disable]******************************************
void cPcdFilterPaNode::disable() {

    ROS_INFO("%s: node disabled :-(", ros::this_node::getName().data());

    sub_pcd_.shutdown()    ;
    sub_pcd_old_.shutdown();
    sub_laser_.shutdown()  ;
}

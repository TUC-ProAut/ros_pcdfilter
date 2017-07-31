/******************************************************************************
*                                                                             *
* pcdfilter_pa_pa_node.h                                                      *
* ======================                                                      *
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

#ifndef __PCD_FILTER_PA_NODE_H
#define __PCD_FILTER_PA_NODE_H

// local headers
#include "pcdfilter_pa_ros.h"
#include "pcdfilter_pa_node_parameter.h"
#include "pcdfilter_pa/PcdFilterPaFilter.h"
#include "pcdfilter_pa/PcdFilterPaCloud.h"

// ros headers
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>

#include <tf/transform_listener.h>

// standard headers
#include <string>
#include <vector>

//**************************[main]*********************************************
int main(int argc, char **argv);

//**************************[cPcdFilterPaNode]*********************************
class cPcdFilterPaNode : public cPcdFilterPaRos {
  public:
    //! default constructor
    cPcdFilterPaNode();

    //! default destructor
    ~cPcdFilterPaNode();

  protected:
    //! ros specific variables, which will be read from Parameterserver
    cPcdFilterPaNodeParameter nodeparams_;

    //! node handler for topic subscription and advertising
    ros::NodeHandle nh_;

    //! for getting all necessary tfs
    tf::TransformListener tf_listener_;

    //! subscriber for a pointcloud
    ros::Subscriber sub_pcd_;
    //! subscriber for a pointcloud (old format)
    ros::Subscriber sub_pcd_old_;
    //! subscriber for a laserscan
    ros::Subscriber sub_laser_;

    //! publisher for filtered pointcloud
    ros::Publisher pub_pcd_;

    //! service for filtering
    ros::ServiceServer ser_filter_;

    //! service for adding additional filters
    ros::ServiceServer ser_add_filters_;
    //! service for changing filters
    ros::ServiceServer ser_change_filters_;

    //! service for enabling node
    ros::ServiceServer ser_enable_;
    //! service for disabling node
    ros::ServiceServer ser_disable_;

    //! callback function for receiving a pointcloud
    void setCloudCallbackSub(const sensor_msgs::PointCloud2ConstPtr &msg);
    //! callback function for receiving a pointcloud (old format)
    void setCloudOldCallbackSub(const sensor_msgs::PointCloudConstPtr &msg);
    //! callback function for receiving a laserscan
    void setCloudLaserCallbackSub(const sensor_msgs::LaserScanConstPtr &msg);

    //! callback function for filtering via service
    bool filterCallbackSrv(
        pcdfilter_pa::PcdFilterPaCloud::Request  &req,
        pcdfilter_pa::PcdFilterPaCloud::Response &res);

    //! callback function for adding additional filters
    bool addFiltersCallbackSrv(
        pcdfilter_pa::PcdFilterPaFilter::Request  &req,
        pcdfilter_pa::PcdFilterPaFilter::Response &res);
    //! callback function for changing filters
    bool changeFiltersCallbackSrv(
        pcdfilter_pa::PcdFilterPaFilter::Request  &req,
        pcdfilter_pa::PcdFilterPaFilter::Response &res);

    //! callback function for enabling filter node
    bool enableCallbackSrv(std_srvs::Empty::Request  &req,
        std_srvs::Empty::Response &res);
    //! callback function for disabling filter node
    bool disableCallbackSrv(std_srvs::Empty::Request  &req,
        std_srvs::Empty::Response &res);

    //! function for adding several filters and giving feedback
    void addFilters(const std::vector<std::string> &new_filters);

  private:
    //! function for activating this node (enabling inputs)
    void enable(void);
    //! function for deactivating this node (disabling inputs)
    void disable(void);
};

#endif // __PCD_FILTER_PA_NODE_H

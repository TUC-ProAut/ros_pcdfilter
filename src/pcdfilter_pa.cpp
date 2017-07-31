/******************************************************************************
*                                                                             *
* pcdfilter_pa.cpp                                                            *
* ================                                                            *
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
#include "pcdfilter_pa.h"

// additional libraries
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/photo/photo.hpp"

// standard headers
#include <string>
#include <math.h>

//**************************[cPcdFilterPa]*************************************
cPcdFilterPa::cPcdFilterPa() {
}

//**************************[~cPcdFilterPa]************************************
cPcdFilterPa::~cPcdFilterPa() {
}

//**************************[pointcloudFilter]*********************************
std::vector<int> cPcdFilterPa::pointcloudFilter(const cv::Mat &pointcloud,
    std::vector<bool> &mask) const {

    std::vector <int> count(params_.filters_.size(), 0);
    mask.resize(pointcloud.rows, true);

    for (int i_point = pointcloud.rows - 1; i_point >= 0; i_point--) {
        for (int i_filter = params_.filters_.size() - 1; i_filter >= 0;
          i_filter--) {
            if (params_.filters_[i_filter].type_ ==
              cPcdFilterPaFilter::ftNONE) {
                continue;
            }
            cv::Vec3f v((float*) pointcloud.ptr(i_point));
            v = params_.filters_[i_filter].rotation_ * v +
              params_.filters_[i_filter].translation_;

            bool check = false;
            switch (params_.filters_[i_filter].type_) {
                case cPcdFilterPaFilter::ftCUBE :
                    if (std::fabs(v[0]) >
                      params_.filters_[i_filter].parameter_[0]) {
                        check = true; break;
                    }
                    if (std::fabs(v[1]) >
                      params_.filters_[i_filter].parameter_[0]) {
                        check = true; break;
                    }
                    if (std::fabs(v[2]) >
                      params_.filters_[i_filter].parameter_[0]) {
                        check = true; break;
                    }
                    break;

                case cPcdFilterPaFilter::ftBLOCK :
                    if (std::fabs(v[0]) >
                      params_.filters_[i_filter].parameter_[0]) {
                        check = true; break;
                    }
                    if (std::fabs(v[1]) >
                      params_.filters_[i_filter].parameter_[1]) {
                        check = true; break;
                    }
                    if (std::fabs(v[2]) >
                      params_.filters_[i_filter].parameter_[2]) {
                        check = true; break;
                    }
                    break;

                case cPcdFilterPaFilter::ftSPHERE :
                    if (v[0] * v[0] + v[1] * v[1] + v[2] * v[2] >
                      params_.filters_[i_filter].parameter_[0]) {
                        check = true; break;
                    }
                    break;

                case cPcdFilterPaFilter::ftCYLINDER :
                    if (v[1] * v[1] + v[2] * v[2] >
                      params_.filters_[i_filter].parameter_[0]) {
                        check = true; break;
                    }
                    if (std::fabs(v[0]) >
                      params_.filters_[i_filter].parameter_[1]) {
                        check = true; break;
                    }
                    break;

                case cPcdFilterPaFilter::ftCONE :
                	if (v[0] <= 0) {
                        check = true; break;
                	}

                	if (v[0] > params_.filters_[i_filter].parameter_[0]) {
                        check = true; break;
                	}

                	if ((v[1] * v[1] + v[2] * v[2]) > (v[0] * v[0]) *
                	  params_.filters_[i_filter].parameter_[1]) {
                        check = true; break;
                    }
                    break;
            }
            if (params_.filters_[i_filter].inverse_) {
                check = ! check;
            }
            if (check == false) {
                count[i_filter]++;
                mask[i_point] = false;
                break;
            }
        }
    }

    return count;
}

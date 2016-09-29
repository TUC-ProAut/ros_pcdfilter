/******************************************************************************
*                                                                             *
* pcdfilter_pa_ros_throttle.cpp                                               *
* =============================                                               *
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
* Copyright (c) 2015-2016, Peter Weissig, Technische Universität Chemnitz     *
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
#include "pcdfilter_pa_ros_throttle.h"

//**************************[cPcdFilterPaRosThrottle]**************************
cPcdFilterPaRosThrottle::cPcdFilterPaRosThrottle() {
    skip_count_                = 0;
    skip_time_                 = 0.1;
    minimum_time_before_reset_ = 60;

    block_all_input_           = false;

    resetThrottle();
}

//**************************[~cPcdFilterPaRosThrottle]*************************
cPcdFilterPaRosThrottle::~cPcdFilterPaRosThrottle() {
}

//**************************[checkNewInput]************************************
bool cPcdFilterPaRosThrottle::checkNewInput(ros::Time time) {

    bool result = false;

    if (block_all_input_) {
        return false;
    }

    // check for time discontinuities
    if (time_ > ros::Time(1)) {
        if (time_ - time > ros::Duration(minimum_time_before_reset_)) {
            resetThrottle();
        }
    }

    // check counter
    count_input_++;
    if (skip_count_ > 0) {
        if (count_input_ % (skip_count_ + 1) != 1) {
            return false;
        }
    }

    // check time
    if ((skip_time_ > 0) && (time_ > ros::Time(1))) {
        if (time - time_ < ros::Duration(skip_time_)) {
            return false;
        }
    }

    time_  = time;
    return true;
}
//**************************[resetThrottle]************************************
void cPcdFilterPaRosThrottle::resetThrottle() {

    count_input_     = 0;
    time_            = ros::Time(0);
    block_all_input_ = false;
}

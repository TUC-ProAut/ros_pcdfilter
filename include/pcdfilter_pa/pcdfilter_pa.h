/******************************************************************************
*                                                                             *
* pcdfilter_pa.h                                                              *
* ==============                                                              *
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

#ifndef __PCD_FILTER_PA_H
#define __PCD_FILTER_PA_H

// local headers
#include "pcdfilter_pa/pcdfilter_pa_filter.h"
#include "pcdfilter_pa/pcdfilter_pa_parameter.h"

// additional libraries
#include <opencv2/highgui/highgui.hpp>

// standard headers
#include <string>
#include <vector>

//**************************[cPcdFilterPa]*************************************
class cPcdFilterPa {
  public:
    //! default constructor
    cPcdFilterPa();

    //! default destructor
    ~cPcdFilterPa();

    //! filters the given pointcloud based on the internal filters
    //! the input pointcloud must be a [n x 3] float matrix (0=x; 1=y; 2=z)
    //! the output mask will mark all valid points as true
    //! the return value is a vector of ints counting the number of
    //!   points replaced by each filter
    std::vector<int> pointcloudFilter(const cv::Mat &pointcloud,
        std::vector<bool> &mask) const;

    //! specific parameter
    cPcdFilterPaParameter params_;
};

#endif // __PCD_FILTER_PA_H

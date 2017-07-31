/******************************************************************************
*                                                                             *
* pcdfilter_pa_filter.h                                                       *
* =====================                                                       *
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

#ifndef __PCD_FILTER_PA_FILTER_H
#define __PCD_FILTER_PA_FILTER_H

// additional libraries
#include <opencv2/highgui/highgui.hpp>

// standard headers
#include <string>
#include <vector>

//**************************[cPcdFilterPaFilter]********************************
class cPcdFilterPaFilter {
  public:
    enum eFiltertype {ftNONE, ftCUBE, ftSPHERE, ftBLOCK, ftCYLINDER, ftCONE};

    cPcdFilterPaFilter();
    cPcdFilterPaFilter(const cPcdFilterPaFilter &other);
    cPcdFilterPaFilter& operator = (
      const cPcdFilterPaFilter &other);

    //! filter type
    eFiltertype type_;

    //! inverted filter
    bool inverse_;

    //! parameter for filter type
    //!   cube    : sidelength/2 -            -
    //!   sphere  : radius^2     -            -
    //!   block   : x/2          y/2          z/2
    //!   cylinder: radius^2     height/2     -
    //!   cone    : height       ratio^2      -
    double parameter_[3];

    //! transform of pointcloud - rotation
    //!   the transform moves the center to the origin and aligns it
    cv::Matx33f rotation_;
    //! transform of pointcloud - translation
    //!   the transform moves the center to the origin and aligns it
    cv::Vec3f translation_;

    //! creates an string representation of the filter
    //! (mainly used for debugging)
    std::string toString(void) const;
};

#endif // __PCD_FILTER_PA_FILTER_H

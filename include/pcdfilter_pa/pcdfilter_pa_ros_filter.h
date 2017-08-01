/******************************************************************************
*                                                                             *
* pcdfilter_pa_ros_filter.h                                                   *
* =========================                                                   *
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

#ifndef __PCD_FILTER_PA_ROS_FILTER_H
#define __PCD_FILTER_PA_ROS_FILTER_H

// local headers
#include "pcdfilter_pa/pcdfilter_pa_filter.h"

// ros headers
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// additional libraries
#include <opencv2/highgui/highgui.hpp>

// standard headers
#include <string>
#include <vector>

//**************************[cPcdFilterPaRosFilter]****************************
class cPcdFilterPaRosFilter {
  public:
    enum eFiltertype {
        ftNONE     = cPcdFilterPaFilter::ftNONE,
        ftCUBE     = cPcdFilterPaFilter::ftCUBE,
        ftSPHERE   = cPcdFilterPaFilter::ftSPHERE,
        ftBLOCK    = cPcdFilterPaFilter::ftBLOCK,
        ftCYLINDER = cPcdFilterPaFilter::ftCYLINDER,
        ftCONE     = cPcdFilterPaFilter::ftCONE,
        ftLINK
    };
    enum {
        COUNT_PARAMETER = 3,
        COUNT_FRAME     = 2
    };

    cPcdFilterPaRosFilter();
    cPcdFilterPaRosFilter(const cPcdFilterPaRosFilter &other);
    cPcdFilterPaRosFilter& operator = (
      const cPcdFilterPaRosFilter &other);

    //! inverted filter
    bool inverse_;

    //! filter type
    eFiltertype type_;

    //! required filter
    bool required_;

    //! single parameter of filter
    double parameter_[COUNT_PARAMETER];

    //! frame id(s)
    std::string frame_[COUNT_FRAME];

    // additional offsets relative to frames
    tf::Transform offset_[COUNT_FRAME];

    //! comment to filter
    std::string comment_;

    //! setting filter from string
    bool fromString(const std::string &filter);
    //! returning string equivalent of filter
    std::string toString(void) const;

    //! reseting this filter
    void reset(void);

    //! returning if filter is valid
    bool isValid(void) const;

  private:
    //! helper function for skipping whitespace
    bool _skipWhitespace(const std::string &str, int &pos) const;

    //! helper function for checking a certain symbol
    bool _checkSymbol(const std::string &str, int &pos,
        const char symbol) const;

    //! helper function for returning the next value
    std::string _getValue(const std::string &str, int &pos) const;

    //! helper function for returning the comment
    std::string _getComment(const std::string &str) const;

    //! helper function for converting a double
    std::string _floatToStr(const double &value) const;

    //! helper function for converting a double
    bool _StrToFloat(const std::string &str, double &value) const;
};

//**************************[cPcdFilterPaRosFilters]***************************
class cPcdFilterPaRosFilters {
  public:
    cPcdFilterPaRosFilters();


    // adds a new filter
    bool add(std::string filter);

    // clears all filters
    void clear(void);

    // return all filters
    std::vector<std::string> get(void) const;

    // returns simple filters based on numbers instead of frames
    bool update( const tf::TransformListener &tf_listener,
      const std::string base_frame, const ros::Time time,
      std::vector<cPcdFilterPaFilter> &result) const;

    // returns last added filter (also includes erroneous filters)
    std::string getLast(void) const;

    // return number of elements
    int size(void) const;

    // access single element
    cPcdFilterPaRosFilter& operator[](int i);
    // access single element (const)
    const cPcdFilterPaRosFilter& operator[](int i) const;

    // time before tf lookup fails - for updating filters
    double tf_lookup_time_;

  protected:
    // get tansform between two tfs
    double update_transform(const tf::Transform &start,
      const tf::Transform &goal, tf::Transform &result,
      const double relative_pos = 0.0) const;


    std::vector<cPcdFilterPaRosFilter> filters_;

    cPcdFilterPaRosFilter last_filter_;
};

#endif // __PCD_FILTER_PA_ROS_FILTER_H

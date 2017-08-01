/******************************************************************************
*                                                                             *
* pcdfilter_pa_ros_filter.cpp                                                 *
* ===========================                                                 *
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
#include "pcdfilter_pa/pcdfilter_pa_ros_filter.h"

// standard headers
#include <sstream>

//********************************************************************************
//**************************[cPcdFilterPaRosFilter]*******************************
//********************************************************************************

//**************************[cPcdFilterPaRosFilter]*******************************
cPcdFilterPaRosFilter::cPcdFilterPaRosFilter(void) {

    reset();
}

//**************************[cPcdFilterPaRosFilter]*******************************
cPcdFilterPaRosFilter::cPcdFilterPaRosFilter(
  const cPcdFilterPaRosFilter &other) {

    *this = other;
}

//**************************[operator = ]**************************************
cPcdFilterPaRosFilter& cPcdFilterPaRosFilter::operator = (
  const cPcdFilterPaRosFilter &other) {

    inverse_          = other.inverse_     ;
    type_             = other.type_        ;
    required_         = other.required_    ;

    for (int i = 0; i < COUNT_PARAMETER; i++) {
        parameter_[i] = other.parameter_[i];
    }

    for (int i = 0; i < COUNT_FRAME; i++) {
        frame_[i]     = other.frame_[i]    ;
        offset_[i]    = other.offset_[i]   ;
    }

    comment_          = other.comment_     ;

    return *this;
}


//**************************[fromString]***************************************
bool cPcdFilterPaRosFilter::fromString(const std::string &filter) {

    reset();

    int pos = 0;
    std::string s;

    _skipWhitespace(filter, pos);
    inverse_ = _checkSymbol(filter,pos,'!');

    _skipWhitespace(filter,pos);
    s = _getValue(filter, pos);
    if (s == "cube") {
        type_ = ftCUBE;
    } else if (s == "sphere") {
        type_ = ftSPHERE;
    } else if (s == "block") {
        type_ = ftBLOCK;
    } else if (s == "cylinder") {
        type_ = ftCYLINDER;
    } else if (s == "link") {
        type_ = ftLINK;
    } else if (s == "cone") {
        type_ = ftCONE;
    } else {
        return false;
    }

    _skipWhitespace(filter,pos);
    if (_checkSymbol(filter,pos,':')) {
        required_ = true;
    } else if (_checkSymbol(filter,pos,'?')) {
        required_ = false;
    } else {
        return false;
    }


    for (int i = 0; ; i++) {
        _skipWhitespace(filter,pos);
        s = _getValue(filter, pos);
        if (s == "") {
            return false;
        }
        if (i >= COUNT_PARAMETER) { break;}
        if (! _StrToFloat(s, parameter_[i])) { break;}
    }

    for (int i = 0; i < COUNT_FRAME; i++) {
        double d;
        if (_StrToFloat(s, d)) {
            return false;
        }
        frame_[i] = s;
        _skipWhitespace(filter, pos);
        s = _getValue(filter, pos);

        tf::Quaternion q = tf::Quaternion::getIdentity();
        for (int j = 0; ; j++) {
            if (! _StrToFloat(s, d)) {
                if (j > 3) {
                    if (j < 7) {
                        q.setW(0);
                    }
                    q.normalize();
                    offset_[i].setRotation(q);
                }
                offset_[i] = offset_[i].inverse();
                break;
            }
            if (j >= 7) {
                q.normalize();
                offset_[i].setRotation(q);
                offset_[i] = offset_[i].inverse();
                return false;
            } else {
                _skipWhitespace(filter, pos);
                s = _getValue(filter, pos);
            }
            switch (j) {
                case 0: offset_[i].getOrigin().setX(d); break;
                case 1: offset_[i].getOrigin().setY(d); break;
                case 2: offset_[i].getOrigin().setZ(d); break;

                case 3: q.setX(d); break;
                case 4: q.setY(d); break;
                case 5: q.setZ(d); break;
                case 6: q.setW(d); break;
            }
        }
        if (s == "") {
            break;
        }
    }

    comment_ = _getComment(filter);

    return isValid();
}

//**************************[toString]*****************************************
std::string cPcdFilterPaRosFilter::toString() const {

    std::stringstream result;

    if (inverse_) {result << '!';}

    int param_count = 1;
    int tf_count    = 1;
    switch (type_) {
        case ftCUBE:
            result << "cube";
            break;
        case ftSPHERE:
            result << "sphere";
            break;
        case ftBLOCK:
            result << "block";
            param_count = 3;
            break;
        case ftCYLINDER:
            result << "cylinder";
            param_count = 2;
            break;
        case ftLINK:
            result << "link";
            param_count = 2;
            tf_count = 2;
            break;
        case ftCONE:
            result << "cone";
            param_count = 2;
            break;
        case ftNONE:
            result << "-";
            param_count = 0;
            tf_count = 0;
            break;
        default:
            result << "error";
            param_count = 0;
            tf_count = 0;
            break;
    }

    if (required_) {
        result << ':';
    } else {
        result << "?";
    }

    if (param_count > COUNT_PARAMETER) { param_count = COUNT_PARAMETER;}
    for (int i = 0; i < param_count; i++) {
        result << ' ' << parameter_[i];
    }

    if (tf_count > COUNT_FRAME) { tf_count = COUNT_FRAME;}
    for (int i = 0; i < tf_count; i++) {
        tf::Transform temp_offset = offset_[i].inverse();
        result << ' ' << frame_[i];

        bool _x = false;
        bool _y = false;
        bool _z = false;
        bool _q = false;
        if (temp_offset.getOrigin().x() != 0) {
            _x = true;
        }
        if (temp_offset.getOrigin().y() != 0) {
            _x = _y = true;
        }
        if (temp_offset.getOrigin().z() != 0) {
            _x = _y = _z = true;
        }
        if ((temp_offset.getRotation().x() != 0) ||
          (temp_offset.getRotation().y() != 0) ||
          (temp_offset.getRotation().z() != 0) ||
          (temp_offset.getRotation().w() != 1)) {
            _x = _y = _z = _q = true;
        }

        if (_x) {
            result << ' ' << _floatToStr(temp_offset.getOrigin().x());
        }
        if (_y) {
            result << ' ' << _floatToStr(temp_offset.getOrigin().y());
        }
        if (_z) {
            result << ' ' << _floatToStr(temp_offset.getOrigin().z());
        }
        if (_q) {
            result << ' ' << _floatToStr(temp_offset.getRotation().x());
            result << ' ' << _floatToStr(temp_offset.getRotation().y());
            result << ' ' << _floatToStr(temp_offset.getRotation().z());
            result << ' ' << _floatToStr(temp_offset.getRotation().w());
        }
    }

    if (comment_ != "") {
        result << " #" << comment_;
    }

    return result.str();
}

//**************************[reset]********************************************
void cPcdFilterPaRosFilter::reset() {

    inverse_  = false;
    type_ = ftNONE;
    required_ = true;
    for (int i = 0; i < COUNT_FRAME; i++) {
        frame_[i] = "";
        offset_[i].setIdentity();
    }
    for (int i = 0; i < COUNT_PARAMETER; i++) {
        parameter_[i] = 0;
    }
    comment_ = "";
}

//**************************[isValid]******************************************
bool cPcdFilterPaRosFilter::isValid() const {

    return true;
}

//**************************[_skipWhitespace]**********************************
bool cPcdFilterPaRosFilter::_skipWhitespace(const std::string &str, int &pos)
  const {

    if (pos >= str.size()) {
        return false;
    }

    char c = str[pos];
    if ((c != ' ') && (c != '\t')) {
        return false;
    }

    while (1) {
        pos++;
        if (pos >= str.size()) {
            return false;
        }

        char c = str[pos];
        if ((c != ' ') && (c != '\t')) {
            break;
        }
    }

    return true;
}

//**************************[_checkSymbol]*************************************
bool cPcdFilterPaRosFilter::_checkSymbol(const std::string &str, int &pos,
  const char symbol) const {

    if (pos >= str.size()) {
        return false;
    }

    if (str[pos] == symbol) {
        pos++;
        return true;
    }

    return false;
}

//**************************[_getValue]****************************************
std::string cPcdFilterPaRosFilter::_getValue(const std::string &str, int &pos)
  const {

    int start = pos;

    while (1) {
        if (pos >= str.size()) {
            break;
        }

        char c = str[pos];
        if ((c == ' ') || (c == '\t') || (c == '#') || (c == ':') ||
          (c == '?')) {
            break;
        }
        pos++;
    }

    if (pos <= start) {
        return "";
    } else {
        return str.substr(start, pos - start);
    }
}

//**************************[_getComment]**************************************
std::string cPcdFilterPaRosFilter::_getComment(const std::string &str)
  const {

    int pos = (int)str.find('#');
    if (pos <= 0) {
        return "";
    }

    return str.substr(pos + 1);
}

//**************************[_floatToStr]**************************************
std::string cPcdFilterPaRosFilter::_floatToStr(const double &value) const {

    std::stringstream ss;

    ss << value;
    return ss.str();
}

//**************************[_StrToFloat]**************************************
bool cPcdFilterPaRosFilter::_StrToFloat(const std::string &str,
    double &value) const {

    std::stringstream ss;

    ss << str;
    try {
        ss >> value;
    } catch (std::runtime_error &e) {
        return false;
    }
    return ! ss.fail();
}

//*****************************************************************************
//**************************[cPcdFilterPaRosFilters]***************************
//*****************************************************************************

//**************************[cPcdFilterPaRosFilters]***************************
cPcdFilterPaRosFilters::cPcdFilterPaRosFilters(void) {

    tf_lookup_time_  = 0.2;
}

//**************************[add]**********************************************
bool cPcdFilterPaRosFilters::add(std::string filter) {

    cPcdFilterPaRosFilter f;

    bool result = f.fromString(filter);
    last_filter_ = f;
    if (result) {
        filters_.push_back(f);
    }

    return result;
}

//**************************[clear]********************************************
void cPcdFilterPaRosFilters::clear() {

    filters_.clear();
}

//**************************[get]**********************************************
std::vector<std::string> cPcdFilterPaRosFilters::get() const {

    std::vector<std::string> result;
    for (int i = 0; i < filters_.size(); i++) {
        result.push_back(filters_[i].toString());
    }
    return result;
}

//**************************[update]*******************************************
bool cPcdFilterPaRosFilters::update(
  const tf::TransformListener &tf_listener,
  const std::string base_frame, const ros::Time time,
  std::vector<cPcdFilterPaFilter> &result) const {

    result.clear();

    // buffer for frame_id/tf pairs (to increase speed)
    std::vector<std::pair<std::string, tf::Transform > > frames;

    ros::Time start_time = ros::Time::now();
    // iterate over all filters
    for (int i_filter = 0; i_filter < filters_.size(); i_filter++) {

        // check number of tfs (always 1 - except for "ftLink")
        int tf_count = 0;
        switch (filters_[i_filter].type_) {
            case cPcdFilterPaRosFilter::ftCUBE    :
            case cPcdFilterPaRosFilter::ftSPHERE  :
            case cPcdFilterPaRosFilter::ftBLOCK   :
            case cPcdFilterPaRosFilter::ftCYLINDER:
                tf_count = 1;
                break;
            case cPcdFilterPaRosFilter::ftLINK    :
                tf_count = 2;
                break;
            case cPcdFilterPaRosFilter::ftCONE    :
                tf_count = 2;
                break;
            case cPcdFilterPaRosFilter::ftNONE    :
                continue;
            default                               :
                return false;
        }

        // lookup tf based on frame id
        bool skip_filter = false;
        tf::Transform tf_temp[cPcdFilterPaRosFilter::COUNT_FRAME];
        for (int i_tf = 0; i_tf < tf_count; i_tf++) {
            std::string frame_temp = filters_[i_filter].frame_[i_tf];

            // check buffer first
            int i_frames = 0;
            for (; i_frames < frames.size(); i_frames++) {
                if (frames[i_frames].first == frame_temp) {
                    tf_temp[i_tf] = frames[i_frames].second;

                    if (tf_temp[i_tf].getBasis()[0][0] < -2) {
                        skip_filter = true;
                    }
                    break;
                }
            }

            // look up is necessary
            if (i_frames >= frames.size()) {
                tf::StampedTransform stf_temp;
                std::string error_msg;
                try {
                    bool check_transform = true;

                    // wait for transform
                    if (tf_lookup_time_ > 0) {
                        ros::Duration current_duration = ros::Time::now() - start_time;
                        if (current_duration < ros::Duration(tf_lookup_time_)) {
                            ros::Duration max_duration = ros::Duration(tf_lookup_time_) - current_duration;
                            if (tf_listener.waitForTransform(frame_temp,
                              base_frame, time, max_duration,
                              ros::Duration(0.01), &error_msg)) {
                                check_transform = false;
                            } else {
                                skip_filter = true;
                            }
                        }
                    }

                    // check if transform is possible
                    if ((! skip_filter) && check_transform) {
                        if(! tf_listener.canTransform(frame_temp,
                          base_frame, time, &error_msg)) {
                            skip_filter = true;
                        }
                    }

                    // get transform
                    if (! skip_filter) {
                        tf_listener.lookupTransform(frame_temp, base_frame,
                          time, stf_temp);
                    }
                } catch (tf::TransformException &ex){
                    skip_filter = true;
                    error_msg = ex.what();
                }

                // error handling
                if (skip_filter) {
                    if (filters_[i_filter].required_) {
                        ROS_DEBUG("%s",error_msg.data());
                    }

                    // save a invalid tf (rot[0,0] < -2)
                    tf_temp[i_tf].getBasis()[0][0] = -10;
                    frames.push_back(std::pair<std::string, tf::Transform>
                      (frame_temp, tf_temp[i_tf]));
                } else {
                    // save current tf
                    tf_temp[i_tf] = stf_temp;
                    frames.push_back(std::pair<std::string, tf::Transform>
                      (frame_temp, tf_temp[i_tf]));
                }
            }

            if (skip_filter) {
                if (filters_[i_filter].required_) {
                    return false;
                }
                break;
            }

            tf_temp[i_tf] = filters_[i_filter].offset_[i_tf] * tf_temp[i_tf];
        }

        // create final filters
        cPcdFilterPaFilter filter_temp;
        if (skip_filter) {
            filter_temp.type_ = cPcdFilterPaFilter::ftNONE;
            result.push_back(filter_temp);
            continue;
        }

        switch (filters_[i_filter].type_) {
            case cPcdFilterPaRosFilter::ftCUBE    :
                filter_temp.type_ = cPcdFilterPaFilter::ftCUBE;
                filter_temp.parameter_[0] =
                  filters_[i_filter].parameter_[0] / 2;
                break;
            case cPcdFilterPaRosFilter::ftSPHERE  :
                filter_temp.type_ = cPcdFilterPaFilter::ftSPHERE;
                filter_temp.parameter_[0] = filters_[i_filter].parameter_[0] *
                  filters_[i_filter].parameter_[0];
                break;
            case cPcdFilterPaRosFilter::ftBLOCK   :
                filter_temp.type_ = cPcdFilterPaFilter::ftBLOCK;
                for (int i = 0; i < 3; i++) {
                    filter_temp.parameter_[i] =
                      filters_[i_filter].parameter_[i] / 2;
                }
                break;
            case cPcdFilterPaRosFilter::ftCYLINDER:
                filter_temp.type_ = cPcdFilterPaFilter::ftCYLINDER;
                filter_temp.parameter_[0] = filters_[i_filter].parameter_[0] *
                  filters_[i_filter].parameter_[0];
                filter_temp.parameter_[1] =
                  filters_[i_filter].parameter_[1] / 2;
                break;
            case cPcdFilterPaRosFilter::ftLINK    :
                {
					filter_temp.type_ = cPcdFilterPaFilter::ftCYLINDER;

					double d = update_transform(tf_temp[0], tf_temp[1],
					  tf_temp[0], 0.5);
					if (d < 0) {
						filter_temp.type_ = cPcdFilterPaFilter::ftNONE;
						break;
					}

					filter_temp.parameter_[0] =
					  filters_[i_filter].parameter_[0] *
					  filters_[i_filter].parameter_[0];
					filter_temp.parameter_[1] = d / 2 +
					  filters_[i_filter].parameter_[1];
                }

                break;
            case cPcdFilterPaRosFilter::ftCONE    :
                filter_temp.type_ = cPcdFilterPaFilter::ftCONE;

                if (update_transform(tf_temp[0], tf_temp[1], tf_temp[0], 0.5) < 0) {
                    filter_temp.type_ = cPcdFilterPaFilter::ftNONE;
                    break;
                }

                filter_temp.parameter_[0] = filters_[i_filter].parameter_[0];
                filter_temp.parameter_[1] = filters_[i_filter].parameter_[1] * filters_[i_filter].parameter_[1];
                break;
        }

        filter_temp.inverse_ = filters_[i_filter].inverse_;
        filter_temp.translation_[0] = tf_temp[0].getOrigin().x();
        filter_temp.translation_[1] = tf_temp[0].getOrigin().y();
        filter_temp.translation_[2] = tf_temp[0].getOrigin().z();

        for (int y = 0; y < 3; y++) {
            for (int x = 0; x < 3; x++) {
                filter_temp.rotation_(y,x) = tf_temp[0].getBasis()[y][x];
            }
        }

        result.push_back(filter_temp);
    }

    return true;
}

//**************************[getLast]******************************************
std::string cPcdFilterPaRosFilters::getLast() const {

    return last_filter_.toString();
}

//**************************[size]*********************************************
int cPcdFilterPaRosFilters::size() const {
    return filters_.size();
}

//**************************[operator]*****************************************
cPcdFilterPaRosFilter& cPcdFilterPaRosFilters::operator[](int i) {
    return filters_[i];
}

//**************************[operator]*****************************************
const cPcdFilterPaRosFilter& cPcdFilterPaRosFilters::operator[](int i) const {
    return filters_[i];
}
//**************************[operator]*****************************************
double cPcdFilterPaRosFilters::update_transform(const tf::Transform &start,
  const tf::Transform &goal, tf::Transform &result, const double relative_pos) const {

    tf::Vector3 v = (start * goal.inverse()).getOrigin();
    double d = v.length();

    if (v.fuzzyZero()) {
        return -1;
    }
    tf::Vector3 axis = v;
    v.normalize();
    axis = tf::Vector3 (1,0,0).cross(v);

    tf::Transform tf_shift;
    if (! axis.fuzzyZero()) {
        tf::Quaternion q(axis.normalized(),tfAsin(axis.length()));
        q = q.inverse();
        tf_shift.setRotation(q);
    } else {
        tf_shift.setRotation(tf::Quaternion().getIdentity());
    }
    tf_shift.setOrigin(tf::Vector3(d * relative_pos,0,0));

    result = tf_shift * start;
    return d;
}


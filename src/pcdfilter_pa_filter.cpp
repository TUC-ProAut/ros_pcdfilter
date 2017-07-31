/******************************************************************************
*                                                                             *
* pcdfilter_pa_filter.cpp                                                     *
* =======================                                                     *
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
#include "pcdfilter_pa_filter.h"

// standard headers
#include <sstream>

//**************************[cPcdFilterPaFilter]*******************************
cPcdFilterPaFilter::cPcdFilterPaFilter(void) {

    type_    = ftNONE;
    inverse_ = false ;

    parameter_[0] = 0;
    parameter_[1] = 0;
    parameter_[2] = 0;

    rotation_     = cv::Matx33f::eye();
    //translation_  = cv::Vec3f();
}

//**************************[cPcdFilterPaFilter]*******************************
cPcdFilterPaFilter::cPcdFilterPaFilter(
  const cPcdFilterPaFilter &other) {

    *this = other;
}

//**************************[operator = ]**************************************
cPcdFilterPaFilter& cPcdFilterPaFilter::operator = (
  const cPcdFilterPaFilter &other) {

    type_         = other.type_        ;
    inverse_      = other.inverse_     ;

    parameter_[0] = other.parameter_[0];
    parameter_[1] = other.parameter_[1];
    parameter_[2] = other.parameter_[2];

    rotation_     = other.rotation_    ;
    translation_  = other.translation_ ;

    return *this;
}

//**************************[toString]*****************************************
std::string cPcdFilterPaFilter::toString(void) const {
    std::stringstream result;

    if (inverse_) {
        result << '!';
    }

    int count_para = 0;
    switch (type_) {
        case ftNONE    : result << "none"    ;                 break;
        case ftCUBE    : result << "cube"    ; count_para = 1; break;
        case ftSPHERE  : result << "sphere"  ; count_para = 1; break;
        case ftBLOCK   : result << "block"   ; count_para = 3; break;
        case ftCYLINDER: result << "cylinder"; count_para = 2; break;
        case ftCONE    : result << "cone"    ; count_para = 2; break;
        default        : result << "error"   ;                 break;
    }
    result << ":";

    for (int i = 0; i < count_para; i++) {
        result << ' ' << parameter_[i];
    }

    result << "; rot=[[";
    for (int y = 0; y < 3; y++) {
        if (y > 0) { result << "], [";}
        for (int x = 0; x < 3; x++) {
            if (x > 0) { result << ", ";}
            result << rotation_(y,x);
        }
    }
    result << "]]";

    result << "; trans=[";
    for (int x = 0; x < 3; x++) {
        if (x > 0) { result << ", ";}
        result << translation_(x);
    }
    result << "]";

    return result.str();
}

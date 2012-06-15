/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef MATLABIO_HPP_
#define MATLABIO_HPP_
#include <string>
#include <vector>
#include <cstdio>
#include <iostream>
#include <opencv2/core/core.hpp>
#include "MatlabIOContainer.hpp"
//#include "hdf5.h"
class MatlabIO {
private:
    // member variables
    static const int HEADER_LENGTH  = 116;
    static const int SUBSYS_LENGTH  = 8;
    static const int VERSION_LENGTH = 1;
    static const int ENDIAN_LENGTH  = 1;
    char header_[HEADER_LENGTH+1];
    char subsys_[SUBSYS_LENGTH];
    int16_t version_[VERSION_LENGTH];
    int16_t endian_[ENDIAN_LENGTH];
    string filename_;
    FILE *fid_;
    // internal methods
    void getHeader(void);
    void setHeader(void);
    MatlabIOContainer readVariable(void);
public:
    // constructors
    MatlabIO() {}
    // destructor
    ~MatlabIO() {}
    // get and set methods
    void filename(void) { return filename_; }
    // read and write routines
    bool open(string filename, string mode);
    bool close(void);
    std::vector<MatlabIOContainer> read(void);

    // templated functions (must be declared and defined in the header file)
    //template<class T> bool write(const T variable);
    //template<class T> bool write(const std::vector<MatlabIOContainer> variables);
    template<class T> T read(const string var_name);
};

#endif

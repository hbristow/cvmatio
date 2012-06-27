/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
#include <fstream>
#include <iostream>
#include <zlib.h>
#include <opencv2/core/core.hpp>
#include "MatlabIOContainer.hpp"
#include "esfstream.hpp"
#include "typetraits.hpp"

/*! @class MatlabIO
 *  @brief Matlab Mat file parser for C++ OpenCV
 *
 *  This class provides the capacity to read and write Mat files
 *  produced by Matlab. The OpenCV cv::Mat class is used for the
 *  internal storage of matrices. The class also provides methods
 *  to inspect the contents of a Mat-file, read all variables, or
 *  read a single variable by name.
 */
class MatlabIO {
private:
    // member variables
    static const int HEADER_LENGTH  = 116;
    static const int SUBSYS_LENGTH  = 8;
    static const int ENDIAN_LENGTH  = 2;
    char header_[HEADER_LENGTH+1];
    char subsys_[SUBSYS_LENGTH+1];
    char endian_[ENDIAN_LENGTH+1];
    int16_t version_;
    bool byte_swap_;
    int bytes_read_;
    std::string filename_;
    esfstream fid_;
    // internal methods
    void getHeader(void);
    void setHeader(void);
    bool hasVariable(void) { return fid_.peek() != EOF; }
	template<class T> MatlabIOContainer constructMatrix(std::vector<char>& name, std::vector<int32_t>& dims, std::vector<char>& real, std::vector<char>& imag, uint32_t stor_type);
	MatlabIOContainer constructString(std::vector<char>& name, std::vector<int32_t>& dims, std::vector<char>& real);
	MatlabIOContainer constructSparse(std::vector<char>& name, std::vector<int32_t>& dims, std::vector<char>& real, std::vector<char>& imag);
	MatlabIOContainer constructCell(std::vector<char>& name, std::vector<int32_t>& dims, std::vector<char>& real);
	MatlabIOContainer constructStruct(std::vector<char>& name, std::vector<int32_t>& dims, std::vector<char>& real);
	const char * readVariableTag(uint32_t &data_type, uint32_t &dbytes, uint32_t &wbytes, const char *data);
	MatlabIOContainer collateMatrixFields(uint32_t data_type, uint32_t nbytes, std::vector<char> data);
	std::vector<char> uncompressVariable(uint32_t& data_type, uint32_t& dbytes, uint32_t& wbytes, const std::vector<char> &data);
    MatlabIOContainer readVariable(uint32_t data_type, uint32_t nbytes, const std::vector<char> &data);
    MatlabIOContainer readBlock(void);
    MatlabIOContainer uncompressFromBin(std::vector<char> data, uint32_t nbytes);
    template<class T> MatlabIOContainer primitiveFromBin(std::vector<char> data, uint32_t nbytes);
public:
    // constructors
    MatlabIO() {}
    // destructor
    ~MatlabIO() { close(); }
    // get and set methods
    std::string filename(void) { return std::string(filename_); }
    // read and write routines
    bool open(std::string filename, std::string mode);
    bool close(void);
    std::vector<MatlabIOContainer> read(void);

    // templated functions (must be declared and defined in the header file)
    //template<class T> bool write(const T variable);
    //template<class T> bool write(const std::vector<MatlabIOContainer> variables);
    template<class T> T read(const std::string var_name);

    void whos(std::vector<MatlabIOContainer> variables) const;
};

enum {
    VERSION_5      = 5,
    VERSION_73     = 73
};

#endif

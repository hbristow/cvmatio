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

//#include "hdf5.h"
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
    template<class T> T swapEndian(const T &t);
    template<class T> std::vector<T> swapEndian(const std::vector<T> &t);
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
};

enum {
    MAT_INT8       = 1,
    MAT_UINT8      = 2,
    MAT_INT16      = 3,
    MAT_UINT16     = 4,
    MAT_INT32      = 5,
    MAT_UINT32     = 6,
    MAT_FLOAT      = 7,
    MAT_DOUBLE     = 9,
    MAT_INT64      = 12,
    MAT_UINT64     = 13,
    MAT_MATRIX     = 14,
    MAT_COMPRESSED = 15,
    MAT_UTF8       = 16,
    MAT_UTF16      = 17,
    MAT_UTF32      = 18
};

enum {
	MAT_CELL_CLASS 	   = 1,
	MAT_STRUCT_CLASS   = 2,
	MAT_OBJECT_CLASS   = 3,
	MAT_CHAR_CLASS     = 4,
	MAT_SPARSE_CLASS   = 5,
	MAT_DOUBLE_CLASS   = 6,
	MAT_FLOAT_CLASS    = 7,
	MAT_INT8_CLASS     = 8,
	MAT_UINT8_CLASS    = 9,
	MAT_INT16_CLASS    = 10,
	MAT_UINT16_CLASS   = 11,
	MAT_INT32_CLASS    = 12,
	MAT_UINT32_CLASS   = 13,
	MAT_INT64_CLASS    = 14,
	MAT_UINT64_CLASS   = 15
};

enum {
    VERSION_5      = 5,
    VERSION_73     = 73
};

#endif

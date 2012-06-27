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
#include <vector>
#include <cerrno>
#include <cstring>
#include <zlib.h>
#include "MatlabIO.hpp"
using namespace std;
using namespace cv;

bool MatlabIO::open(string filename, string mode) {

    // open the file
	filename_ = filename;
    if (mode.compare("r") == 0) fid_.open(filename.c_str(), fstream::in  | fstream::binary);
    if (mode.compare("w") == 0) fid_.open(filename.c_str(), fstream::out | fstream::binary);
    return !fid_.fail();
}

bool MatlabIO::close(void) {

    // close the file and release any associated objects
    fid_.close();
    return !fid_.fail();
}

template<class T>
T MatlabIO::swapEndian(const T &in) {
    char N = sizeof(T);
    T out = in;
    char *data = reinterpret_cast<char *>(&out);
    for (char n = 0; n < N; n+=2) swap(data[n], data[n+1]);
    return out;
}

template<class T>
vector<T> MatlabIO::swapEndian(const vector<T> &in) {
    int N = in.size()*sizeof(T);
    vector<T> out(in);
    char *data = reinterpret_cast<char *>(&(out[0]));
    for (int n = 0; n < N; n+=2) swap(data[n], data[n+1]);
    return out;
}


template<class T>
T product(const vector<T>& vec) {
	T acc = 1;
	for (int n = 0; n < vec.size(); ++n) acc *= vec[n];
	return acc;
}

template<class T1, class T2>
vector<T2> convertPrimitiveType(const vector<char>& in) {

	// firstly reinterpret the input as type T1
	int T1_size = in.size() / sizeof(T1);
	const T1* in_ptr = reinterpret_cast<const T1*>(&(in[0]));

	// construct the new vector
	vector<T2> out(in_ptr, in_ptr+T1_size);
	return out;

}

void MatlabIO::getHeader(void) {
    // get the header information from the Mat file
    for (int n = 0; n < HEADER_LENGTH+1; ++n) header_[n] = '\0';
    for (int n = 0; n < SUBSYS_LENGTH+1; ++n) subsys_[n] = '\0';
    for (int n = 0; n < ENDIAN_LENGTH+1; ++n) endian_[n] = '\0';
    fid_.read(header_, sizeof(char)*HEADER_LENGTH);
    fid_.read(subsys_, sizeof(char)*SUBSYS_LENGTH);
    fid_.read((char *)&version_, sizeof(int16_t));
    fid_.read(endian_, sizeof(char)*ENDIAN_LENGTH);

    // get the actual version
    if (version_ == 0x0100) version_ = VERSION_5;
    if (version_ == 0x0200) version_ = VERSION_73;

    // get the endianess
    if (strcmp(endian_, "IM") == 0) byte_swap_ = false;
    if (strcmp(endian_, "MI") == 0) byte_swap_ = true;
    // turn on byte swapping if necessary
    fid_.setByteSwap(byte_swap_);

    printf("Header: %s\nSubsys: %s\nVersion: %d\nEndian: %s\nByte Swap: %d\n", header_, subsys_, version_, endian_, byte_swap_);
    bytes_read_ = 128;
}


MatlabIOContainer MatlabIO::uncompressFromBin(vector<char> data, uint32_t nbytes) {

    printf("I am a: compressed matrix\n");
    MatlabIOContainer variable;
    return variable;
}

template<class T>
MatlabIOContainer MatlabIO::primitiveFromBin(vector<char> data, uint32_t nbytes) {


    printf("I am a: %s\n", typeid(T).name());
    MatlabIOContainer variable;
    return variable;
}

const char * MatlabIO::readVariableTag(uint32_t &data_type, uint32_t &dbytes, uint32_t &wbytes, const char *data) {
    
	bool small = false;
    const uint32_t *datai = reinterpret_cast<const uint32_t *>(data);
    data_type = datai[0];
    printf("Small format?: %x\n", data_type >> 16 != 0);
    if ((data_type >> 16) != 0) {
        // small data format
        printf("Casting to small format...\n");
        dbytes = data_type >> 16;
        data_type = (data_type << 16) >> 16;
        small = true;
    } else {
        // regular format
        dbytes = datai[1];
    }

    // get the whole number of bytes (wbytes) consumed by this variable, including header and padding
    if (small) wbytes = 8;
    else if (data_type == MAT_COMPRESSED) wbytes = 8 + dbytes;
    else wbytes = 8 + dbytes + (dbytes % 8);

    // return the seek head positioned over the data payload
    return data + (small ? 4 : 8);
}

MatlabIOContainer MatlabIO::constructStruct(vector<char>& name, vector<int32_t>& dims, vector<char>& real) {

	vector<MatlabIOContainer> strct;
	return MatlabIOContainer(string(&(name[0])), strct);
}

MatlabIOContainer MatlabIO::constructCell(vector<char>& name, vector<int32_t>& dims, vector<char>& real) {

	vector<MatlabIOContainer> cell;
	char* field_ptr = &(real[0]);
	for (int n = 0; n < product<int32_t>(dims); ++n) {
		MatlabIOContainer field;
		uint32_t data_type;
		uint32_t dbytes;
		uint32_t wbytes;
		const char* data_ptr = readVariableTag(data_type, dbytes, wbytes, field_ptr);
		field = collateMatrixFields(data_type, dbytes, vector<char>(data_ptr, data_ptr+dbytes));
		cell.push_back(field);
		field_ptr += wbytes;
	}
	return MatlabIOContainer(string(&(name[0])), cell);
}

MatlabIOContainer MatlabIO::constructSparse(vector<char>& name, vector<int32_t>& dims, vector<char>& real, vector<char>& imag) {

	MatlabIOContainer variable;
	return variable;
}

MatlabIOContainer MatlabIO::constructString(vector<char>& name, vector<int32_t>& dims, vector<char>& real) {
	// make sure the data is null terminated
	real.push_back('\0');
	return MatlabIOContainer(string(&(name[0])), string(&(real[0])));
}



template<class T>
MatlabIOContainer MatlabIO::constructMatrix(vector<char>& name, vector<int32_t>& dims, vector<char>& real, vector<char>& imag, uint32_t stor_type) {

	vector<T> vec_real;
	vector<T> vec_imag;
	vector<Mat> vec_mat;
	Mat flat;
	Mat mat;
	switch (stor_type) {
		case MAT_INT8:
			vec_real = convertPrimitiveType<int8_t, T>(real);
			vec_imag = convertPrimitiveType<int8_t, T>(imag);
			break;
		case MAT_UINT8:
			vec_real = convertPrimitiveType<uint8_t, T>(real);
			vec_imag = convertPrimitiveType<uint8_t, T>(imag);
			break;
		case MAT_INT16:
			vec_real = convertPrimitiveType<int16_t, T>(real);
			vec_imag = convertPrimitiveType<int16_t, T>(imag);
			break;
		case MAT_UINT16:
			vec_real = convertPrimitiveType<uint16_t, T>(real);
			vec_imag = convertPrimitiveType<uint16_t, T>(imag);
			break;
		case MAT_INT32:
			vec_real = convertPrimitiveType<int32_t, T>(real);
			vec_imag = convertPrimitiveType<int32_t, T>(imag);
			break;
		case MAT_UINT32:
			vec_real = convertPrimitiveType<uint32_t, T>(real);
			vec_imag = convertPrimitiveType<uint32_t, T>(imag);
			break;
		case MAT_INT64:
			vec_real = convertPrimitiveType<int64_t, T>(real);
			vec_imag = convertPrimitiveType<int64_t, T>(imag);
			break;
		case MAT_UINT64:
			vec_real = convertPrimitiveType<uint64_t, T>(real);
			vec_imag = convertPrimitiveType<uint64_t, T>(imag);
			break;
		case MAT_FLOAT:
			vec_real = convertPrimitiveType<float, T>(real);
			vec_imag = convertPrimitiveType<float, T>(imag);
			break;
		case MAT_DOUBLE:
			vec_real = convertPrimitiveType<double, T>(real);
			vec_imag = convertPrimitiveType<double, T>(imag);
			break;
		case MAT_UTF8:
			vec_real = convertPrimitiveType<char, T>(real);
			vec_imag = convertPrimitiveType<char, T>(imag);
			break;
		default:
			return MatlabIOContainer();
	}

	// assert that the conversion has not modified the number of elements
	uint32_t numel = 1;
	for (int n = 0; n < dims.size(); ++n) numel *= dims[n];
	assert(vec_real.size() == numel);

	// if the data is a scalar, don't write it to a matrix
	if (vec_real.size() == 1 && vec_imag.size() == 0) return MatlabIOContainer(string(&(name[0])), vec_real[0]);

	// get the number of channels
	int channels = dims.size() == 3 ? dims[2] : 1;
	bool complx = vec_imag.size() != 0;

	// put each plane of the image into a vector
	vector<Mat> sflat;
	flat = Mat(vec_real, true);
	for (int n = 0; n < channels; ++n)
		sflat.push_back(flat(Range(dims[0]*dims[1]*n, dims[0]*dims[1]*(n+1)), Range::all()));
	flat = Mat(vec_imag, true);
	for (int n = 0; n < channels*complx; ++n)
		sflat.push_back(flat(Range(dims[0]*dims[1]*n, dims[0]*dims[1]*(n+1)), Range::all()));

	// merge the planes into a matrix
	merge(sflat, flat);

	// reshape to the image dimensions
	mat = flat.reshape(flat.channels(), dims[1]);

	// transpose the matrix since matlab stores them in column major ordering
	transpose(mat, mat);

	return MatlabIOContainer(string(&(name[0])), mat);
}


MatlabIOContainer MatlabIO::collateMatrixFields(uint32_t data_type, uint32_t nbytes, vector<char> data) {

    // get the flags
    bool complx  = data[9] & (1 << 3);
    bool logical = data[9] & (1 << 1);
    
    // get the type of the encapsulated data
    char enc_data_type = data[8];
    // the preamble size is 16 bytes
    uint32_t pre_wbytes = 16;

    // get the dimensions
    uint32_t dim_type;
    uint32_t dim_dbytes;
    uint32_t dim_wbytes;
    const char* dim_data = readVariableTag(dim_type, dim_dbytes, dim_wbytes, &(data[pre_wbytes]));
    vector<int32_t> dims(reinterpret_cast<const int32_t *>(dim_data), reinterpret_cast<const int32_t *>(dim_data+dim_dbytes));
    printf("Complex?: %d\n", complx);
    printf("Logical?: %d\n", logical);
    printf("Dimensions: ");
    for(int n = 0; n < dims.size(); ++n) printf("%d  ", dims[n]);
    printf("\n");
    printf("Dim bytes: %d\n", dim_dbytes);

    // get the variable name
    uint32_t name_type;
    uint32_t name_dbytes;
    uint32_t name_wbytes;
    const char* name_data = readVariableTag(name_type, name_dbytes, name_wbytes, &(data[pre_wbytes+dim_wbytes]));
    vector<char> name(name_data, name_data+name_dbytes);
    name.push_back('\0');
    printf("The variable name is: %s\n", &(name[0]));

    // get the real data
    uint32_t real_type;
    uint32_t real_dbytes;
    uint32_t real_wbytes;
    const char* real_data = readVariableTag(real_type, real_dbytes, real_wbytes, &(data[pre_wbytes+dim_wbytes+name_wbytes]));
    vector<char> real(real_data,real_data+real_dbytes);
    printf("The variable type is: %d\n", enc_data_type);

    vector<char> imag;
    if (complx) {
    	// get the imaginery data
    	uint32_t imag_type;
    	uint32_t imag_dbytes;
    	uint32_t imag_wbytes;
    	const char* imag_data = readVariableTag(imag_type, imag_dbytes, imag_wbytes, &(data[pre_wbytes+dim_wbytes+name_wbytes+real_wbytes]));
    	assert(imag_type == real_type);
    	for (imag_data; imag_data != imag_data+imag_dbytes; imag_data++) imag.push_back(*imag_data);
    }

    // construct whatever object we happened to get
    MatlabIOContainer variable;
    switch (enc_data_type) {
    	// integral types
    	case MAT_INT8_CLASS:      variable = constructMatrix<int8_t>(name, dims, real, imag, real_type); break;
        case MAT_UINT8_CLASS:     variable = constructMatrix<uint8_t>(name, dims, real, imag, real_type); break;
        case MAT_INT16_CLASS:     variable = constructMatrix<int16_t>(name, dims, real, imag, real_type); break;
        case MAT_UINT16_CLASS:    variable = constructMatrix<uint16_t>(name, dims, real, imag, real_type); break;
        case MAT_INT32_CLASS:     variable = constructMatrix<int32_t>(name, dims, real, imag, real_type); break;
        case MAT_UINT32_CLASS:    variable = constructMatrix<uint32_t>(name, dims, real, imag, real_type); break;
        case MAT_FLOAT_CLASS:     variable = constructMatrix<float>(name, dims, real, imag, real_type); break;
        case MAT_DOUBLE_CLASS:    variable = constructMatrix<double>(name, dims, real, imag, real_type); break;
        case MAT_INT64_CLASS:     variable = constructMatrix<int64_t>(name, dims, real, imag, real_type); break;
        case MAT_UINT64_CLASS:    variable = constructMatrix<uint64_t>(name, dims, real, imag, real_type); break;
        case MAT_CHAR_CLASS:      variable = constructString(name, dims, real); break;
        // sparse types
        case MAT_SPARSE_CLASS:    variable = constructSparse(name, dims, real, imag); break;
        // recursive types
        case MAT_CELL_CLASS:	  variable = constructCell(name, dims, real); break;
        case MAT_STRUCT_CLASS:	  variable = constructStruct(name, dims, real); break;
        // non-handled types
        case MAT_OBJECT_CLASS:	  break;
        default: 				  break;
    }
    return variable;
}


vector<char> MatlabIO::uncompressVariable(uint32_t& data_type, uint32_t& dbytes, uint32_t& wbytes, const vector<char> &data) {
    // setup the inflation parameters
    char buf[8];
    z_stream infstream;
    infstream.zalloc = Z_NULL;
    infstream.zfree  = Z_NULL;
    infstream.opaque = Z_NULL;
    int ok = inflateInit(&infstream);
    printf("Inflate init: %d\n", ok == Z_OK);

    // inflate the variable header
    infstream.avail_in = data.size();
    infstream.next_in = (unsigned char *)&(data[0]);
    infstream.avail_out = 8;
    infstream.next_out = (unsigned char *)&buf;
    ok = inflate(&infstream, Z_NO_FLUSH);
    printf("Inflation: %d\n", data.size());

    // get the headers
    readVariableTag(data_type, dbytes, wbytes, buf);

    // inflate the remainder of the variable, now that we know its size
    char *udata_tmp = new char[dbytes];
    infstream.avail_out = dbytes;
    infstream.next_out = (unsigned char *)udata_tmp;
    inflate(&infstream, Z_FINISH);
    inflateEnd(&infstream);

    // convert to a vector
    vector<char> udata(udata_tmp, udata_tmp+dbytes);
    free(udata_tmp);
    return udata;

}

MatlabIOContainer MatlabIO::readVariable(uint32_t data_type, uint32_t nbytes, const vector<char> &data) {

    // interpret the data
    MatlabIOContainer variable;
    switch (data_type) {
        case MAT_INT8:      variable = primitiveFromBin<int8_t>(data, nbytes); break; 
        case MAT_UINT8:     variable = primitiveFromBin<uint8_t>(data, nbytes); break;
        case MAT_INT16:     variable = primitiveFromBin<int16_t>(data, nbytes); break;
        case MAT_UINT16:    variable = primitiveFromBin<uint16_t>(data, nbytes); break;
        case MAT_INT32:     variable = primitiveFromBin<int32_t>(data, nbytes); break;
        case MAT_UINT32:    variable = primitiveFromBin<uint32_t>(data, nbytes); break;
        case MAT_FLOAT:     variable = primitiveFromBin<float>(data, nbytes); break;
        case MAT_DOUBLE:    variable = primitiveFromBin<double>(data, nbytes); break;
        case MAT_INT64:     variable = primitiveFromBin<int64_t>(data, nbytes); break;
        case MAT_UINT64:    variable = primitiveFromBin<uint64_t>(data, nbytes); break;
        case MAT_UTF8:      variable = primitiveFromBin<char>(data, nbytes); break;
        case MAT_UTF16:     break;
        case MAT_UTF32:     break;
        case MAT_COMPRESSED:
        {
            // uncompress the data
            uint32_t udata_type;
            uint32_t udbytes;
            uint32_t uwbytes;
            vector<char> udata = uncompressVariable(udata_type, udbytes, uwbytes, data);
            variable = readVariable(udata_type, udbytes, udata);
            break;
        }
        case MAT_MATRIX:
        {
            // deserialize the matrix
            variable = collateMatrixFields(data_type, nbytes, data);
            break;
        }
        default: break;
    }
    printf("Read variable:\nData Type: %d, bytes: %d\n", data_type, nbytes); fflush(stdout);
    return variable;
}


MatlabIOContainer MatlabIO::readBlock(void) {

    // allocate the output
    MatlabIOContainer variable;

    // get the data type and number of bytes consumed
    // by this variable. Check to see if it's using
    // the small data format (seriously, who thought of that? You save at best 8 bytes...)
    uint32_t data_type;
    uint32_t dbytes;
    uint32_t wbytes;
    char buf[8];
    fid_.read(buf, sizeof(char)*8);
    readVariableTag(data_type, dbytes, wbytes, buf);

    // read the binary data block
    printf("\nReading binary data block...\n"); fflush(stdout);
    char *data_tmp = new char[dbytes];
    fid_.read(data_tmp, sizeof(char)*dbytes);
    vector<char> data(data_tmp, data_tmp+dbytes);
    free(data_tmp);

    // move the seek head position to the next 64-bit boundary
    // (but only if the data is uncompressed. Saving yet another 8 tiny bytes...)
    if (data_type != MAT_COMPRESSED) {
        printf("Aligning seek head to next 64-bit boundary...\n");
        streampos head_pos = fid_.tellg();
        int padding = head_pos % 8;
        fid_.seekg(padding, fstream::cur);
    }

    // now read the variable contained in the block
    return readVariable(data_type, dbytes, data);
}



std::vector<MatlabIOContainer> MatlabIO::read(void) {

    // allocate the output
    std::vector<MatlabIOContainer> variables;

    // read the header information
    getHeader();

    // get all of the variables
    while(hasVariable()) {

        MatlabIOContainer variable;
        variable = readBlock();
        variables.push_back(variable);
    }
    return variables;
}


void MatlabIO::whos(vector<MatlabIOContainer> variables) const {

	printf("-------------------------\n");
	printf("File: %s\n", filename_.c_str());
	printf("%s\n", header_);
	printf("Variables:\n");
	for (int n = 0; n < variables.size(); ++n) {
		printf("%s:  %s\n", variables[n].name().c_str(), variables[n].type().c_str());
	}
	printf("-------------------------\n");
}

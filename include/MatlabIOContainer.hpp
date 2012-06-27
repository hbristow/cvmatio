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
#ifndef MATLABIOCONTAINER_HPP_
#define MATLABIOCONTAINER_HPP_
#include <string>
#include <typeinfo>
#include <boost/any.hpp>

/*! @class MatlabIOContainer
 *  @brief A container class for storing type agnostic variables
 *
 *  MatlabIOContainer stores variables of any type using the boost::any type.
 *  This allows multiple MatlabIOContainers to be stored in a single vector
 *  when reading multiple variables from a file or constructing a Matlab struct.
 */
class MatlabIOContainer {
private: 
    std::string name_;
    boost::any data_;
public:
    // constructors
    MatlabIOContainer() {}
    /*! @brief constructor
     *
     * @param name the string name of the variable
     * @param data the associated data, of any type
     */
    MatlabIOContainer(const std::string name, const boost::any data) : name_(name), data_(data) {}
    // destructor
    ~MatlabIOContainer() {}
    // set methods
    void setName(const std::string name) { name_ = name; }
    void setData(const boost::any data) { data_ = data; }
    // get methods
    template<class T> bool typeEquals(void) const { return data_.type() == typeid(T); }
    std::string type(void) const { return data_.type().name(); }
    std::string name(void) const { return name_; }
    template<class T> T data(void) const { return boost::any_cast<T>(data_); }
};

#endif

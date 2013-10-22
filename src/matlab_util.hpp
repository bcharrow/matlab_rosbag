/**
   Copyright (c) 2010, Georgia Tech Research Corporation
   Atlanta, Georgia 30332-0415
   All Rights Reserved

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

   * Neither the name of the copyright holders nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * This file was taken from GTSAM and then modified.
 */

#include <string>
#include <sstream>

extern "C" {
#include <mex.h>
}

#if defined(__LP64__) || defined(_WIN64)
// 64-bit
#define mxUINT32OR64_CLASS mxUINT64_CLASS
#else
#define mxUINT32OR64_CLASS mxUINT32_CLASS
#endif

//*****************************************************************************
// Utilities
//*****************************************************************************

void error(const char* str) {
  mexErrMsgIdAndTxt("wrap:error", str);
}

mxArray *scalar(mxClassID classid) {
  return mxCreateNumericMatrix(1, 1, classid, mxREAL);
}

void checkScalar(const mxArray* array, const char* str) {
  int m = mxGetM(array), n = mxGetN(array);
  if (m!=1 || n!=1)
    mexErrMsgIdAndTxt("wrap: not a scalar in ", str);
}

// Replacement streambuf for cout that writes to the MATLAB console
// Thanks to http://stackoverflow.com/a/249008
class mstream : public std::streambuf {
protected:
  virtual std::streamsize xsputn(const char *s, std::streamsize n) {
    mexPrintf("%.*s",n,s);
    return n;
  }
  virtual int overflow(int c = EOF) {
    if (c != EOF) {
      mexPrintf("%.1s",&c);
    }
    return 1;
  }
};

//*****************************************************************************
// wrapping C++ basis types in MATLAB arrays
//*****************************************************************************

// Leave method undefined so compiler error is generated
template <typename Class, typename Input>
mxArray* mexWrap(const Input& value);

// specialization to string
// wraps into a character array
template<>
mxArray* mexWrap<std::string>(const std::string& value) {
  return mxCreateString(value.c_str());
}

// specialization for std::vector<std::string>.
// wrap into a cell array of strings if vector has more than one element,
// else wrap into a character array
template<>
mxArray* mexWrap<std::vector<std::string> >(const std::vector<std::string> &s) {
  if (s.size() == 1) {
    return mxCreateString(s[0].c_str());
  } else {
    mxArray *strings = mxCreateCellMatrix(1, s.size());
    for (int i = 0; i < s.size(); ++i) {
      mxSetCell(strings, i, mxCreateString(s[i].c_str()));
    }
    return strings;
  }
}

// specialization to char
template<>
mxArray* mexWrap<char>(const char& value) {
  mxArray *result = scalar(mxUINT32OR64_CLASS);
  *(char*)mxGetData(result) = value;
  return result;
}

// Macro to create template specialization for mexWrap() when passed a builtin
#define __CREATE_MEX_WRAP(CPP_TYPE, MATLAB_TYPE)        \
  template<>                                            \
  mxArray* mexWrap<CPP_TYPE>(const CPP_TYPE& value) {   \
    mxArray *result = scalar(MATLAB_TYPE);              \
    *static_cast<CPP_TYPE*>(mxGetData(result)) = value; \
    return result;                                      \
  }

__CREATE_MEX_WRAP(uint8_t, mxUINT8_CLASS);
__CREATE_MEX_WRAP(uint16_t, mxUINT16_CLASS);
__CREATE_MEX_WRAP(uint32_t, mxUINT32_CLASS);
__CREATE_MEX_WRAP(uint64_t, mxUINT64_CLASS);

__CREATE_MEX_WRAP(int8_t, mxINT8_CLASS);
__CREATE_MEX_WRAP(int16_t, mxINT16_CLASS);
__CREATE_MEX_WRAP(int32_t, mxINT32_CLASS);
__CREATE_MEX_WRAP(int64_t, mxINT64_CLASS);

__CREATE_MEX_WRAP(float, mxSINGLE_CLASS);
__CREATE_MEX_WRAP(double, mxDOUBLE_CLASS);

// Specialization for bool
template<>
mxArray* mexWrap<bool>(const bool& value) {
  mxArray *result = mxCreateLogicalMatrix(1, 1);
  *(bool*)mxGetData(result) = value;
  return result;
}

//*****************************************************************************
// unwrapping MATLAB arrays into C++ basis types
//*****************************************************************************

template <typename T>
T mexUnwrap(const mxArray* array);

#define __CREATE_MEX_UNWRAP(CPP_TYPE)                   \
  template<>                                            \
  CPP_TYPE mexUnwrap<CPP_TYPE>(const mxArray* array) {  \
    checkScalar(array, "mexUnwrap<CPP_TYPE>");          \
    switch (mxGetClassID(array)) {                      \
    case mxINT64_CLASS:                                 \
      return (CPP_TYPE) *(int64_t*) mxGetData(array);   \
    case mxUINT64_CLASS:                                \
      return (CPP_TYPE) *(uint64_t*) mxGetData(array);  \
    default:                                            \
      return (CPP_TYPE) mxGetScalar(array);             \
    }                                                   \
  }                                                     \

__CREATE_MEX_UNWRAP(bool)
__CREATE_MEX_UNWRAP(char)
__CREATE_MEX_UNWRAP(uint8_t)
__CREATE_MEX_UNWRAP(uint16_t)
__CREATE_MEX_UNWRAP(uint32_t)
__CREATE_MEX_UNWRAP(uint64_t)
__CREATE_MEX_UNWRAP(int8_t)
__CREATE_MEX_UNWRAP(int16_t)
__CREATE_MEX_UNWRAP(int32_t)
__CREATE_MEX_UNWRAP(int64_t)
__CREATE_MEX_UNWRAP(double)
__CREATE_MEX_UNWRAP(float)

// specialization to string
// expects a character array
// Warning: relies on mxChar==char
template<>
std::string mexUnwrap<std::string>(const mxArray* array) {
  char *data = mxArrayToString(array);
  if (data==NULL) error("mexUnwrap<string>: not a character array");
  std::string str(data);
  mxFree(data);
  return str;
}

// Convert Matlab strings to C++ vector of strings.  The input can be a string
// or a cell array of strings.  The vector's ith element is the contents of the
// cell array's ith linear index.
template<>
std::vector<std::string> mexUnwrap(const mxArray *array) {
  std::vector<std::string> strings;

  if (mxIsChar(array)) {
    strings.push_back(mexUnwrap<std::string>(array));
  } else if (mxIsCell(array)) {
    size_t numel = mxGetNumberOfElements(array);
    strings.resize(numel);
    for (size_t i = 0; i < numel; ++i) {
      mxArray *str = mxGetCell(array, i);
      if (!mxIsChar(str)) {
        error("mexUnwrap<vector<string>>: non-character array in cell");
      }
      strings[i] = mexUnwrap<std::string>(str);
    }
  } else {
    error("mexUnwrap<vector<string>>: not a cell array");
  }
  return strings;
}

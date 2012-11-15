#include "mex.h"

#include <string>
#include <map>

#include <boost/scoped_ptr.hpp>
#include <boost/ptr_container/ptr_map.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "parser.hpp"
#include "matlab_util.hpp"

using namespace std;

//======================== ROS to Matlab conversions ========================//

// Extend mexWrap() to handle converting from vectors of vectors of bytes to
// matlab types
#define __CREATE_MEX_WRAP_BYTES(CPP_TYPE, MATLAB_TYPE)                  \
  template<> mxArray*                                                   \
  mexWrap<CPP_TYPE, vector<vector<uint8_t> > >                          \
  (const vector<vector<uint8_t> > &b) {                                 \
    mxArray *result = mxCreateNumericMatrix(1, b.size(),                \
                                            MATLAB_TYPE, mxREAL);       \
    int8_t *data = static_cast<int8_t*>(mxGetData(result));             \
    for (int i = 0; i < b.size(); ++i) {                                \
      if (sizeof(CPP_TYPE) != b[i].size()) {                            \
        throw runtime_error("bad size");                                \
      }                                                                 \
      copy(b[i].begin(), b[i].end(),                                    \
           data + i * sizeof(CPP_TYPE));                                \
    }                                                                   \
    return result;                                                      \
  }

__CREATE_MEX_WRAP_BYTES(uint8_t, mxUINT8_CLASS);
__CREATE_MEX_WRAP_BYTES(uint16_t, mxUINT16_CLASS);
__CREATE_MEX_WRAP_BYTES(uint32_t, mxUINT32_CLASS);
__CREATE_MEX_WRAP_BYTES(uint64_t, mxUINT64_CLASS);

__CREATE_MEX_WRAP_BYTES(int8_t, mxINT8_CLASS);
__CREATE_MEX_WRAP_BYTES(int16_t, mxINT16_CLASS);
__CREATE_MEX_WRAP_BYTES(int32_t, mxINT32_CLASS);
__CREATE_MEX_WRAP_BYTES(int64_t, mxINT64_CLASS);

__CREATE_MEX_WRAP_BYTES(float, mxSINGLE_CLASS);
__CREATE_MEX_WRAP_BYTES(double, mxDOUBLE_CLASS);

mxArray* bytesToString(const vector<uint8_t> &bytes) {
  boost::scoped_array<char> chars(new char[bytes.size() + 1]);
  copy(bytes.begin(), bytes.end(), chars.get());
  *(chars.get() + bytes.size()) = '\0';
  mxArray *result = mxCreateString(chars.get());
  return result;
}

// Specialization for string.  Return a character array if dealing with a
// single array of bytes, else return a cell cell array of character arrays.
template<> mxArray*
mexWrap<string, vector<vector<uint8_t> > >(const vector<vector<uint8_t> > &b) {
  if (b.size() == 1) {
    return bytesToString(b[0]);
  } else {
    mxArray *cells = mxCreateCellMatrix(1, b.size());
    for (int i = 0; i < b.size(); ++i) {
      mxSetCell(cells, 0, bytesToString(b[i]));
    }
    return cells;
  }
}

// Specialization for ros::Time.  Return a struct with 'sec' and 'nsec' fields.
template<> mxArray*
mexWrap<ros::Time, vector<vector<uint8_t> > >(const vector<vector<uint8_t> > &b) {
  const char *fields[] = {"sec", "nsec"};
  mxArray *times = mxCreateStructMatrix(1, b.size(), 2, fields);
  for (int i = 0; i < b.size(); ++i) {
    if (b[i].size() != 8) {
        throw runtime_error("bad size");
    }
    int32_t sec = 0, nsec = 0;
    copy(b[i].begin(), b[i].begin() + 4, reinterpret_cast<uint8_t*>(&sec));
    copy(b[i].begin() + 4, b[i].end(), reinterpret_cast<uint8_t*>(&nsec));
    mxSetField(times, i, fields[0], mexWrap<int32_t>(sec));
    mxSetField(times, i, fields[1], mexWrap<int32_t>(nsec));
  }
  return times;
}

// Specialization for bools.  Return a logical array.
template<> mxArray*
mexWrap<bool, vector<vector<uint8_t> > >(const vector<vector<uint8_t> > &b) {
  mxArray *matrix = mxCreateLogicalMatrix(1, b.size());
  bool *bools = static_cast<bool*>(mxGetData(matrix));
  for (int i = 0; i < b.size(); ++i) {
    if (b[i].size() != 1) {
        throw runtime_error("bad size");
    } else {
      bools[i] = b[i][0];
    }
  }
  return matrix;
}


// Forward declare template specialization; mexWrap<ROSMessage> and
// mexWrap<vector<ROSMessage*> > call each other, so this breaks cycle
template<>
mxArray* mexWrap<ROSMessage>(const ROSMessage &m);

template<>
mxArray* mexWrap<vector<ROSMessage*> >(const vector<ROSMessage*> &msgs) {
  if (msgs[0]->type().is_builtin) {
    // Create the builtin type
    if (msgs.size() != 1) {
      throw runtime_error("Shouldn't have multiple arrays of builtins");
    }
    return mexWrap<ROSMessage>(*msgs[0]);
  } else {
    // Get fields for this message type
    const ROSMessage *msg = msgs[0];
    boost::scoped_array<const char*> fields(new const char*[msg->nfields()]);
    int i = 0;
    for (ROSMessage::const_iterator it = msg->begin(); it != msg->end(); ++it) {
      fields[i] = it->first.c_str();
      ++i;
    }

    // Create an array of structs and then populate it by iterating over each
    // field of each message
    mxArray *rv = mxCreateStructMatrix(1, msgs.size(),
                                       msg->nfields(), fields.get());
    for (i = 0; i < msgs.size(); ++i) {
      msg = msgs[i];
      for (ROSMessage::const_iterator it = msg->begin();
           it != msg->end();
           ++it) {
        const vector<ROSMessage*> &field_val = it->second;

        mxArray *val = mexWrap<vector<ROSMessage*> >(field_val);
        mxSetField(rv, i, it->first.c_str(), val);
      }
    }
    return rv;
  }
}

// Convert a ROSMessage to matlab.  If the ROSType is a builtin, return the
// equivalent matlab datatype.  Otherwise, return a matlab struct.
template<>
mxArray* mexWrap<ROSMessage>(const ROSMessage &msg) {
  if (!msg.type().is_builtin) {
    std::vector<ROSMessage*> m;
    m.push_back(const_cast<ROSMessage*>(&msg));
    return mexWrap<vector<ROSMessage*> >(m);
  } else {
    string type = msg.type().base_type;
    vector<vector<uint8_t> > bytes = msg.bytes();
    if (type == "bool")          { return mexWrap<bool>(bytes); }
    else if (type == "uint8")    { return mexWrap<uint8_t>(bytes); }
    else if (type == "uint16")   { return mexWrap<uint16_t>(bytes); }
    else if (type == "uint32")   { return mexWrap<uint32_t>(bytes); }
    else if (type == "uint64")   { return mexWrap<uint64_t>(bytes); }
    else if (type == "int8")     { return mexWrap<int8_t>(bytes); }
    else if (type == "int16")    { return mexWrap<int16_t>(bytes); }
    else if (type == "int32")    { return mexWrap<int32_t>(bytes); }
    else if (type == "int64")    { return mexWrap<int64_t>(bytes); }
    else if (type == "float32")  { return mexWrap<float>(bytes); }
    else if (type == "float64")  { return mexWrap<double>(bytes); }
    else if (type == "time")     { return mexWrap<ros::Time>(bytes); }
    else if (type == "duration") { return mexWrap<ros::Time>(bytes); }
    else if (type == "string")   { return mexWrap<string>(bytes); }
    else { throw invalid_argument("Not a fundamental type"); }
  }
}

//============================= Mex Interfaces ==============================//

class ROSBagWrapper {
public:
  ROSBagWrapper(const string &fname) : path_(fname), view_(NULL) {
    bag_.open(fname.c_str(), rosbag::bagmode::Read);
  }

  void mex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    string cmd = mexUnwrap<string>(prhs[0]);
    if (cmd == "resetView") {
      vector<string> topics = mexUnwrap<vector<string> >(prhs[1]);
      resetView(topics);
    } else if (cmd == "readMessage") {
      plhs[0] = readMessage();
    } else if (cmd == "hasNext") {
      plhs[0] = mexWrap<bool>(hasNext());
    } else {
      throw invalid_argument("ROSBagWrapper::mex() Unknown method");
    }
  }

  void resetView(const vector<string> &topics) {
    view_.reset(new rosbag::View(bag_, rosbag::TopicQuery(topics)));
    iter_ = view_->begin();
  }

  mxArray* readMessage() {
    const rosbag::MessageInstance &m = *iter_;
    boost::scoped_ptr<ROSMessage> msg(deser_.CreateMessage(m));
    ++iter_;
    return mexWrap<ROSMessage>(*msg);
  }

  bool hasNext() const {
    return view_ != NULL && iter_ != view_->end();
  }

private:
  string path_;
  rosbag::Bag bag_;
  boost::scoped_ptr<rosbag::View> view_;
  rosbag::View::iterator iter_;
  BagDeserializer deser_;
};

// Manage separate instances of bags
class InstanceManager {
public:
  InstanceManager() : id_ctr_(1) {}

  ~InstanceManager() {
    for (map<uint64_t, ROSBagWrapper*>::iterator it = handles_.begin();
         it != handles_.end();
         ++it) {
      delete (*it).second;
    }
  }

  void mex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    string cmd = mexUnwrap<string>(prhs[0]);
    if (cmd == "construct") {
      string bagname = mexUnwrap<string>(prhs[1]);
      handles_[id_ctr_] = new ROSBagWrapper(bagname);
      plhs[0] = mexWrap<uint64_t>(id_ctr_);
      id_ctr_++;
      id_ctr_ = id_ctr_ == 0 ? id_ctr_ + 1 : id_ctr_;
    } else if (cmd == "destruct") {
      uint64_t id = mexUnwrap<uint64_t>(prhs[1]);
      bool check_exists = nrhs == 3 ? mexUnwrap<bool>(prhs[2]) : true;
      if (check_exists) {
        get(id);
      }
      delete handles_[id];
      handles_.erase(id);
    } else {
      throw invalid_argument("InstanceManger::mex() Unknown method");
    }
  }

  ROSBagWrapper* get(uint64_t id) {
    if (handles_.count(id) == 0) {
      throw invalid_argument("InstanceManager::get() Invalid handle");
    } else {
      return handles_[id];
    }
  }

private:
  uint64_t id_ctr_;                  /**< Next valid ID */
  map<uint64_t, ROSBagWrapper*> handles_;
};

static InstanceManager manager;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs < 1) {
    mexErrMsgTxt("rosbag_wrapper id ...");
  }

  uint64_t id = mexUnwrap<uint64_t>(prhs[0]);
  if (id == 0) {
    manager.mex(nlhs, plhs, nrhs - 1, prhs + 1);
  } else {
    ROSBagWrapper *wrapper = manager.get(id);
    wrapper->mex(nlhs, plhs, nrhs - 1, prhs + 1);
  }
}

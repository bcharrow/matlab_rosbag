#include "mex.h"

#include <string>
#include <map>

#include <wordexp.h>

#include <boost/scoped_ptr.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "parser.hpp"
#include "matlab_util.hpp"

using namespace std;

// Specialization for ros::Time.
// Return a struct with integer 'sec', 'nsec', fields and a double 'time' field.
template<>
mxArray* mexWrap<ros::Time>(const ros::Time &t) {
  const char *fields[] = {"sec", "nsec", "time"};
  mxArray *time =
    mxCreateStructMatrix(1, 1, sizeof(fields) / sizeof(fields[0]), fields);
  mxSetField(time, 0, "sec", mexWrap<uint32_t>(t.sec));
  mxSetField(time, 0, "nsec", mexWrap<uint32_t>(t.nsec));
  mxSetField(time, 0, "time", mexWrap<double>(t.sec + 1e-9 * t.nsec));
  return time;
}

static uint32_t read_uint32(const uint8_t *bytes, int *beg) {
  uint32_t val = reinterpret_cast<const uint32_t*>(bytes + *beg)[0];
  *beg += 4;
  return val;
}

// Convert bytes to a matlab string.
mxArray* mexWrapString(const uint8_t *bytes, int *beg) {
  uint32_t elem_size = read_uint32(bytes, beg);
  boost::scoped_array<char> chars(new char[elem_size + 1]);
  copy(bytes + *beg, bytes + *beg + elem_size, chars.get());
  *beg += elem_size;
  *(chars.get() + elem_size) = '\0';
  mxArray *result = mxCreateString(chars.get());
  return result;
}

// Convert bytes to matlab strings.  If n_elem 1, return a matlab string.
// Otherwise, return a cell array of matlab strings.
mxArray* mexWrapStrings(size_t elem_num, const uint8_t *bytes, int *beg) {
  if (elem_num == 1) {
    return mexWrapString(bytes, beg);
  } else {
    mxArray *cells = mxCreateCellMatrix(elem_num, 1);
    for (int i = 0; i < elem_num; ++i) {
      mxSetCell(cells, i, mexWrapString(bytes, beg));
    }
    return cells;
  }
}

mxArray* mexWrapNumeric(mxClassID matid, size_t elem_size, size_t elem_num,
                        const uint8_t *bytes, int *beg) {
  mxArray *result = mxCreateNumericMatrix(elem_num, 1, matid, mxREAL);
  uint8_t *mxdata = static_cast<uint8_t*>(mxGetData(result));
  size_t nbytes = elem_num * elem_size;
  copy(bytes + *beg, bytes + *beg + nbytes, mxdata);
  *beg += nbytes;
  return result;
}

// Convert bytes to matlab struct with fields sec, nsec and time.
mxArray* mexWrapTime(size_t n_elem, const uint8_t *bytes, int *beg) {
  const char *fields[] = {"sec", "nsec", "time"};
  mxArray *times =
    mxCreateStructMatrix(n_elem, 1, sizeof(fields)/sizeof(fields[0]), fields);

  const uint32_t *nums = reinterpret_cast<const uint32_t*>(bytes + *beg);
  for (int i = 0; i < n_elem; ++i) {
    uint32_t sec = nums[2*i];
    uint32_t nsec = nums[2*i + 1];
    mxSetField(times, i, "sec", mexWrap<uint32_t>(sec));
    mxSetField(times, i, "nsec", mexWrap<uint32_t>(nsec));
    mxSetField(times, i, "time", mexWrap<double>(sec + 1e-9 * nsec));
  }
  *beg += 2 * sizeof(uint32_t) * n_elem;
  return times;
}

mxArray* mexWrapDuration(size_t n_elem, const uint8_t *bytes, int *beg) {
  const char *fields[] = {"sec", "nsec", "time"};
  mxArray *times =
    mxCreateStructMatrix(n_elem, 1, sizeof(fields)/sizeof(fields[0]), fields);

  const int32_t *nums = reinterpret_cast<const int32_t*>(bytes + *beg);
  for (int i = 0; i < n_elem; ++i) {
    int32_t sec = nums[2*i];
    int32_t nsec = nums[2*i + 1];
    mxSetField(times, i, "sec", mexWrap<int32_t>(sec));
    mxSetField(times, i, "nsec", mexWrap<int32_t>(nsec));
    mxSetField(times, i, "time", mexWrap<double>(sec + 1e-9 * nsec));
  }
  *beg += 2 * sizeof(int32_t) * n_elem;
  return times;
}

mxArray* mexWrapLogical(size_t elem_num, const uint8_t *bytes, int *beg) {
  mxArray *result = mxCreateLogicalMatrix(elem_num, 1);
  uint8_t *mxdata = static_cast<uint8_t*>(mxGetData(result));
  copy(bytes + *beg, bytes + *beg + elem_num, mxdata);
  *beg += elem_num;
  return result;
}

mxArray* decode_builtin(const ROSType &type, const uint8_t *bytes, int *beg) {
  int array_len = type.array_size;
  if (array_len == -1) {
    array_len = read_uint32(bytes, beg);
  }

  switch (type.id) {
  case ROSType::STRING:
    return mexWrapStrings(array_len, bytes, beg);
    break;
  case ROSType::TIME:
  case ROSType::DURATION:
    return mexWrapTime(array_len, bytes, beg);
    break;
  case ROSType::BOOL:
    return mexWrapLogical(array_len, bytes, beg);
    break;
  case ROSType::CHAR:
  case ROSType::UINT8:
    return mexWrapNumeric(mxUINT8_CLASS, type.type_size, array_len, bytes, beg);
    break;
  case ROSType::UINT16:
    return mexWrapNumeric(mxUINT16_CLASS, type.type_size, array_len, bytes, beg);
    break;
  case ROSType::UINT32:
    return mexWrapNumeric(mxUINT32_CLASS, type.type_size, array_len, bytes, beg);
    break;
  case ROSType::UINT64:
    return mexWrapNumeric(mxUINT64_CLASS, type.type_size, array_len, bytes, beg);
    break;
  case ROSType::BYTE:
  case ROSType::INT8:
    return mexWrapNumeric(mxINT8_CLASS, type.type_size, array_len, bytes, beg);
    break;
  case ROSType::INT16:
    return mexWrapNumeric(mxINT16_CLASS, type.type_size, array_len, bytes, beg);
    break;
  case ROSType::INT32:
    return mexWrapNumeric(mxINT32_CLASS, type.type_size, array_len, bytes, beg);
    break;
  case ROSType::INT64:
    return mexWrapNumeric(mxINT64_CLASS, type.type_size, array_len, bytes, beg);
    break;
  case ROSType::FLOAT32:
    return mexWrapNumeric(mxSINGLE_CLASS, type.type_size, array_len, bytes, beg);
    break;
  case ROSType::FLOAT64:
    return mexWrapNumeric(mxDOUBLE_CLASS, type.type_size, array_len, bytes, beg);
    break;
  default:
    throw invalid_argument(type.name + " not a fundamental type");
  }
}

double decode_builtin_double(const ROSType &type, const uint8_t *bytes,
                             int *beg) {
  if (!type.is_builtin || type.array_size == -1 || type.type_size == -1) {
    throw invalid_argument("Cannot convert type " + type.name + " to double");
  }
  const uint8_t *pos = bytes + *beg;
  *beg += type.type_size;
  // Note absence of string; that has type_size == -1
  switch (type.id) {
  case ROSType::BOOL:
    return static_cast<double>(*reinterpret_cast<const bool*>(pos));
    break;
  case ROSType::CHAR:
  case ROSType::UINT8:
    return static_cast<double>(*reinterpret_cast<const uint8_t*>(pos));
    break;
  case ROSType::UINT16:
    return static_cast<double>(*reinterpret_cast<const uint16_t*>(pos));
    break;
  case ROSType::UINT32:
    return static_cast<double>(*reinterpret_cast<const uint32_t*>(pos));
    break;
  case ROSType::UINT64:
    return static_cast<double>(*reinterpret_cast<const uint64_t*>(pos));
    break;
  case ROSType::BYTE:
  case ROSType::INT8:
    return static_cast<double>(*reinterpret_cast<const int8_t*>(pos));
    break;
  case ROSType::INT16:
    return static_cast<double>(*reinterpret_cast<const int16_t*>(pos));
    break;
  case ROSType::INT32:
    return static_cast<double>(*reinterpret_cast<const int32_t*>(pos));
    break;
  case ROSType::INT64:
    return static_cast<double>(*reinterpret_cast<const int64_t*>(pos));
    break;
  case ROSType::FLOAT32:
    return static_cast<double>(*reinterpret_cast<const float*>(pos));
    break;
  case ROSType::FLOAT64:
    return *reinterpret_cast<const double*>(pos);
    break;
  case ROSType::TIME:
    return static_cast<double>(reinterpret_cast<const uint32_t*>(pos)[0] +
                               reinterpret_cast<const uint32_t*>(pos)[1]*1e-9);
    break;
  case ROSType::DURATION:
    return static_cast<double>(reinterpret_cast<const int32_t*>(pos)[0] +
                               reinterpret_cast<const int32_t*>(pos)[1]*1e-9);
    break;
  default:
    throw invalid_argument(type.name + " not a fundamental type");
  }
}

bool is_flattenable(const ROSMessageFields &fields) {
  bool flattenable = true;
  for (int i = 0; i < fields.nfields(); ++i) {
    const ROSMessageFields::Field &rmf = fields.at(i);
    const ROSType &type = rmf.type;
    flattenable &= (!rmf.constant && type.is_builtin && !type.is_array &&
                    type.type_size != -1);
  }
  return flattenable;
}

mxArray* decode_rosmsg(const ROSType &type, const ROSTypeMap &typemap,
                       bool flatten, const uint8_t *bytes, int *beg) {
  const ROSMessageFields *rmfs = typemap.getMsgFields(type.base_type);
  if (rmfs == NULL) {
    throw invalid_argument("Couldn't resolve fields for " + type.base_type);
  }

  int array_len = type.array_size;
  if (array_len == -1) {
    array_len = read_uint32(bytes, beg);
  }

  if (flatten && is_flattenable(*rmfs)) {
    // Return message as matrix of doubles
    int nfields = rmfs->nfields();
    mxArray *matmsg = mxCreateDoubleMatrix(nfields, array_len, mxREAL);
    double *data = mxGetPr(matmsg);
    for (int array_ind = 0; array_ind < array_len; ++array_ind) {
      for (int field_ind = 0; field_ind < nfields; ++field_ind) {
        const ROSMessageFields::Field &rmf = rmfs->at(field_ind);
        double val = decode_builtin_double(rmf.type, bytes, beg);
        data[field_ind + array_ind * nfields] = val;
      }
    }
    return matmsg;
  } else {
    // Create struct of messages
    boost::scoped_array<const char*> fieldnames(new const char*[rmfs->nfields()]);
    for (int i = 0; i < rmfs->nfields(); ++i) {
      fieldnames[i] = rmfs->at(i).name.c_str();
    }
    mxArray *matmsg =
      mxCreateStructMatrix(1, array_len, rmfs->nfields(), fieldnames.get());

    // Populate fields
    for (int array_ind = 0; array_ind < array_len; ++array_ind) {
      for (int field_ind = 0; field_ind < rmfs->nfields(); ++field_ind) {
        const ROSMessageFields::Field &rmf = rmfs->at(field_ind);

        mxArray *field_val;
        if (!rmf.type.is_builtin) {
          field_val = decode_rosmsg(rmf.type, typemap, flatten, bytes, beg);
        } else if (!rmf.constant) {
          field_val = decode_builtin(rmf.type, bytes, beg);
        } else {
          boost::scoped_array<uint8_t> serbytes(new uint8_t[rmf.bytes.size()]);
          copy(rmf.bytes.begin(), rmf.bytes.end(), serbytes.get());
          int serbeg = 0;
          field_val = decode_builtin(rmf.type, serbytes.get(), &serbeg);
        }

        mxSetFieldByNumber(matmsg, array_ind, field_ind, field_val);
      }
    }
    return matmsg;
  }
}

mxArray* decode_rostype(const ROSTypeMap &typemap, const uint8_t *bytes,
                        bool flatten) {
  int beg = 0;
  const ROSMessageFields *fields = typemap.getMsgFields("0-root");
  if (fields == NULL) {
    throw invalid_argument("Typemap does not have root type");
  }
  return decode_rosmsg(fields->type(), typemap, flatten, bytes, &beg);
}

//============================= Mex Interfaces ==============================//

class ROSBagWrapper {
public:
  ROSBagWrapper(const string &fname) : path_(fname), view_(NULL) {
    // Word expansion of filename (e.g. tilde expands to home directory)
    wordexp_t fname_exp;
    if (wordexp(fname.c_str(), &fname_exp, 0) != 0) {
      throw invalid_argument("Invalid filename: " + fname);
    }
    string path = fname_exp.we_wordv[0];
    wordfree(&fname_exp);

    bag_.open(path.c_str(), rosbag::bagmode::Read);
    info_.setBag(&bag_);
  }

  void mex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    string cmd = mexUnwrap<string>(prhs[0]);
    if (cmd == "resetView") {
      vector<string> topics = mexUnwrap<vector<string> >(prhs[1]);
      resetView(topics);
    } else if (cmd == "read") {
      if (nrhs != 3) {
        error("ROSBagWrapper::mex() Expected two arguments");
      } else if (!hasNext()) {
        error("No more messages for current view");
      }

      bool meta = mexUnwrap<bool>(prhs[1]);
      bool flatten = mexUnwrap<bool>(prhs[2]);
      if (!meta) {
        read(flatten, &plhs[0]);
      } else {
        read(flatten, &plhs[0], &plhs[1]);
      }
    } else if (cmd == "readAll") {
      if (nrhs != 3) {
        error("ROSBagWrapper::mex() Expected two arguments");
      }
      bool meta = mexUnwrap<bool>(prhs[1]);
      bool flatten = mexUnwrap<bool>(prhs[2]);
      if (!meta) {
        readAll(flatten, &plhs[0]);
      } else {
        readAll(flatten, &plhs[0], &plhs[1]);
      }
    } else if (cmd == "hasNext") {
      plhs[0] = mexWrap<bool>(hasNext());
    } else if (cmd == "info") {
      plhs[0] = mexWrap<string>(info_.info());
    } else if (cmd == "definition") {
      if (nrhs != 3) {
        error("ROSBagWrapper::mex() Expected two arguments");
      }
      string msg_type = mexUnwrap<string>(prhs[1]);
      bool raw = mexUnwrap<bool>(prhs[2]);
      plhs[0] = mexWrap<string>(info_.definition(msg_type, raw));
    } else if (cmd == "topicType") {
      if (nrhs != 2) {
        error("ROSBagWrapper::mex() Expected one argument");
      }
      vector<string> topics = mexUnwrap<vector<string> >(prhs[1]);
      plhs[0] = mexWrap<vector<string> >(info_.topicType(topics));
    } else {
      throw invalid_argument("ROSBagWrapper::mex() Unknown method");
    }
  }

  void resetView(const vector<string> &topics) {
    view_.reset(new rosbag::View(bag_, rosbag::TopicQuery(topics)));
    iter_ = view_->begin();
  }

  void read(bool flatten, mxArray **msg) {
    const rosbag::MessageInstance &mi = *iter_;

    const ROSTypeMap *map = deser_.getTypeMap(mi);
    size_t sz = mi.size();
    boost::scoped_array<uint8_t> bytes(new uint8_t[sz]);
    ros::serialization::IStream stream(bytes.get(), sz);
    mi.write(stream);

    *msg = decode_rostype(*map, bytes.get(), flatten);

    ++iter_;
  }

  void read(bool flatten, mxArray **msg, mxArray **meta) {
    const rosbag::MessageInstance &mi = *iter_;
    const char *fields[] = {"topic", "time", "datatype"};
    mxArray *val =
      mxCreateStructMatrix(1, 1, sizeof(fields) / sizeof(fields[0]), fields);
    mxSetField(val, 0, "topic", mexWrap<string>(mi.getTopic()));
    mxSetField(val, 0, "time", mexWrap<ros::Time>(mi.getTime()));
    mxSetField(val, 0, "datatype", mexWrap<string>(mi.getDataType()));
    *meta = val;

    read(flatten, msg);
  }

  void readAll(bool flatten, mxArray **msg) {
    vector<mxArray*> msgs;
    while (hasNext()) {
      msgs.push_back(NULL);
      read(flatten, &msgs.back());
    }

    *msg = mxCreateCellMatrix(1, msgs.size());
    for (int i = 0; i < msgs.size(); ++i) {
      mxSetCell(*msg, i, msgs[i]);
    }
  }

  void readAll(bool flatten, mxArray **msg, mxArray **meta) {
    vector<mxArray*> msgs, metas;
    while (hasNext()) {
      msgs.push_back(NULL);
      metas.push_back(NULL);
      read(flatten, &msgs.back(), &metas.back());
    }
    *msg = mxCreateCellMatrix(1, msgs.size());
    *meta = mxCreateCellMatrix(1, metas.size());
    for (int i = 0; i < msgs.size(); ++i) {
      mxSetCell(*msg, i, msgs[i]);
      mxSetCell(*meta, i, metas[i]);
    }
  }

  bool hasNext() const {
    return view_ != NULL && iter_ != view_->end();
  }

private:
  string path_;
  rosbag::Bag bag_;
  BagInfo info_;
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

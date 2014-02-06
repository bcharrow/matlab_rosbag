
#include "rosbag_wrapper.hpp"

#include <wordexp.h>

#include "matlab_util.hpp"

using namespace std;

// Specialization for ros::Time.
// Return a struct with integer 'sec', 'nsec', fields and a double 'time' field.
mxArray* mexWrap(const ros::Time &t) {
  const char *fields[] = {"sec", "nsec", "time"};
  mxArray *time =
    mxCreateStructMatrix(1, 1, sizeof(fields) / sizeof(fields[0]), fields);
  mxSetField(time, 0, "sec", mexWrap(t.sec));
  mxSetField(time, 0, "nsec", mexWrap(t.nsec));
  mxSetField(time, 0, "time", mexWrap(t.sec + 1e-9 * t.nsec));
  return time;
}

mxArray* mexWrap(const geometry_msgs::Vector3 &v) {
  mxArray *result = mxCreateNumericMatrix(3, 1, mxDOUBLE_CLASS, mxREAL);
  double *data = static_cast<double*>(mxGetData(result));
  data[0] = v.x;
  data[1] = v.y;
  data[2] = v.z;
  return result;
}

mxArray* mexWrap(const geometry_msgs::Quaternion &q) {
  mxArray *result = mxCreateNumericMatrix(4, 1, mxDOUBLE_CLASS, mxREAL);
  double *data = static_cast<double*>(mxGetData(result));
  data[0] = q.x;
  data[1] = q.y;
  data[2] = q.z;
  data[3] = q.w;
  return result;
}

mxArray* mexWrap(const vector<geometry_msgs::TransformStamped> &t) {
  const char *fields[] = {"translation", "rotation"};
  mxArray *transform =
    mxCreateStructMatrix(1, t.size(), sizeof(fields) / sizeof(fields[0]), fields);
  for (size_t i = 0; i < t.size(); ++i) {
    mxSetField(transform, i, fields[0], mexWrap(t.at(i).transform.translation));
    mxSetField(transform, i, fields[1], mexWrap(t.at(i).transform.rotation));
  }
  return transform;
}

mxArray* mexWrap(const vector<geometry_msgs::Pose2D> &ps) {
  mxArray *result = mxCreateNumericMatrix(3, ps.size(), mxDOUBLE_CLASS, mxREAL);
  double *data = static_cast<double*>(mxGetData(result));
  for (size_t i = 0; i < ps.size(); ++i) {
    const geometry_msgs::Pose2D &pose = ps.at(i);
    data[i * 3 + 0] = pose.x;
    data[i * 3 + 1] = pose.y;
    data[i * 3 + 2] = pose.theta;
  }
  return result;
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

// Convert bytes to matlab strings.  If bytes represent an array, return a cell
// array of matlab strings, otherwise return a matlab string.
mxArray* mexWrapStrings(bool is_array, size_t elem_num, const uint8_t *bytes,
                        int *beg) {
  if (!is_array) {
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
    mxSetField(times, i, "sec", mexWrap(sec));
    mxSetField(times, i, "nsec", mexWrap(nsec));
    mxSetField(times, i, "time", mexWrap(sec + 1e-9 * nsec));
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
    mxSetField(times, i, "sec", mexWrap(sec));
    mxSetField(times, i, "nsec", mexWrap(nsec));
    mxSetField(times, i, "time", mexWrap(sec + 1e-9 * nsec));
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
    return mexWrapStrings(type.is_array, array_len, bytes, beg);
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
void MexWrapper::assertArgs(int expect, int actual) {
  if (expect != actual) {
    ostringstream ostringstream;
    ostringstream << "Expected " << expect << " arguments (got " <<
      actual << ")";
    error(ostringstream.str().c_str());
  }
}


InstanceManager::~InstanceManager() {
  for (map<uint64_t, MexWrapper*>::iterator it = handles_.begin();
       it != handles_.end();
       ++it) {
    delete (*it).second;
  }
}

// Manage separate instances of bags
void InstanceManager::mex(int nlhs, mxArray *plhs[],
                          int nrhs, const mxArray *prhs[]) {
  string cmd = mexUnwrap<string>(prhs[0]);
  if (cmd == "construct") {
    string classname = mexUnwrap<string>(prhs[1]);
    if (classname == "ROSBagWrapper") {
      string bagname = mexUnwrap<string>(prhs[2]);
      handles_[id_ctr_] = new ROSBagWrapper(bagname);
    } else if (classname == "TFWrapper") {
      handles_[id_ctr_] = new TFWrapper();
    } else {
      throw invalid_argument(string("Unknown class: ") + classname);
    }
    plhs[0] = mexWrap(id_ctr_);
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

static InstanceManager g_manager;

ROSBagWrapper::ROSBagWrapper(const string &fname) : path_(fname), view_(NULL) {
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

void ROSBagWrapper::resetView(const vector<string> &topics,
                              const ros::Time &start_time,
                              const ros::Time &end_time) {
  view_.reset(new rosbag::View(bag_, rosbag::TopicQuery(topics), start_time,
                               end_time));
  iter_ = view_->begin();
}

void ROSBagWrapper::read(bool flatten, mxArray **msg) {
  const rosbag::MessageInstance &mi = *iter_;

  const ROSTypeMap *map = deser_.getTypeMap(mi);
  size_t sz = mi.size();
  boost::scoped_array<uint8_t> bytes(new uint8_t[sz]);
  ros::serialization::IStream stream(bytes.get(), sz);
  mi.write(stream);

  *msg = decode_rostype(*map, bytes.get(), flatten);

  ++iter_;
}

void ROSBagWrapper::read(bool flatten, mxArray **msg, mxArray **meta) {
  const rosbag::MessageInstance &mi = *iter_;
  const char *fields[] = {"topic", "time", "datatype"};
  mxArray *val =
    mxCreateStructMatrix(1, 1, sizeof(fields) / sizeof(fields[0]), fields);
  mxSetField(val, 0, "topic", mexWrap(mi.getTopic()));
  mxSetField(val, 0, "time", mexWrap(mi.getTime()));
  mxSetField(val, 0, "datatype", mexWrap(mi.getDataType()));
  *meta = val;

  read(flatten, msg);
}

void ROSBagWrapper::readAll(bool flatten, mxArray **msg) {
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

void ROSBagWrapper::readAll(bool flatten, mxArray **msg, mxArray **meta) {
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

void ROSBagWrapper::mex(int nlhs, mxArray *plhs[], int nrhs,
                        const mxArray *prhs[]) {
  string cmd = mexUnwrap<string>(prhs[0]);
  if (cmd == "resetView") {
    vector<string> topics = mexUnwrap<vector<string> >(prhs[1]);
    ros::Time start_time, end_time;
    start_time.fromSec(mexUnwrap<double>(prhs[2]));
    end_time.fromSec(mexUnwrap<double>(prhs[3]));
    resetView(topics, start_time, end_time);
  } else if (cmd == "read") {
    assertArgs(3, nrhs);
    if (!hasNext()) {
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
    assertArgs(3, nrhs);
    bool meta = mexUnwrap<bool>(prhs[1]);
    bool flatten = mexUnwrap<bool>(prhs[2]);
    if (!meta) {
      readAll(flatten, &plhs[0]);
    } else {
      readAll(flatten, &plhs[0], &plhs[1]);
    }
  } else if (cmd == "hasNext") {
    plhs[0] = mexWrap(hasNext());
  } else if (cmd == "info") {
    plhs[0] = mexWrap(info_.info());
  } else if (cmd == "definition") {
    assertArgs(3, nrhs);
    string msg_type = mexUnwrap<string>(prhs[1]);
    bool raw = mexUnwrap<bool>(prhs[2]);
    plhs[0] = mexWrap(info_.definition(msg_type, raw));
  } else if (cmd == "topicType") {
    assertArgs(2, nrhs);
    vector<string> topics = mexUnwrap<vector<string> >(prhs[1]);
    plhs[0] = mexWrap(info_.topicType(topics));
  } else if (cmd == "topics") {
    assertArgs(2, nrhs);
    string regexp = mexUnwrap<string>(prhs[1]);
    vector<string> topics = info_.topics(regexp);
    plhs[0] = mexWrap(topics);
  } else if (cmd == "times") {
    plhs[0] = mexWrap(info_.getBeginTime());
    plhs[1] = mexWrap(info_.getEndTime());
  } else {
    throw invalid_argument("ROSBagWrapper::mex() Unknown method");
  }
}

void TFWrapper::mex(int nlhs, mxArray *plhs[],
                    int nrhs, const mxArray *prhs[]) {
  string cmd = mexUnwrap<string>(prhs[0]);

  if (cmd == "build") {
    assertArgs(5, nrhs);
    int bag_id = mexUnwrap<int>(prhs[1]);
    double start = mexUnwrap<double>(prhs[2]);
    double stop = mexUnwrap<double>(prhs[3]);
    string topic = mexUnwrap<string>(prhs[4]);
    const ROSBagWrapper* bag;
    bag = static_cast<ROSBagWrapper*>(g_manager.get(bag_id));
    if (bag == NULL) {
      throw invalid_argument("TFWrapper::mex() Bad reference to bag");
    }
    transformer_.build(bag->bag(), ros::Time(start), ros::Time(stop), topic);
    plhs[0] = mexWrap(transformer_.beginTime());
    plhs[1] = mexWrap(transformer_.endTime());
  } else if (cmd == "allFrames") {
    plhs[0] = mexWrap(transformer_.allFrames());
  } else if (cmd == "lookup") {
    assertArgs(5, nrhs);
    string target_frame = mexUnwrap<string>(prhs[1]);
    string source_frame = mexUnwrap<string>(prhs[2]);
    vector<double> times = mexUnwrap<vector<double> >(prhs[3]);
    bool just2d = mexUnwrap<bool>(prhs[4]);

    if (just2d) {
      vector<geometry_msgs::Pose2D> transforms;
      transformer_.lookup(target_frame, source_frame, times, &transforms);
      plhs[0] = mexWrap(transforms);
    } else {
      vector<geometry_msgs::TransformStamped> transforms;
      transformer_.lookup(target_frame, source_frame, times, &transforms);
      plhs[0] = mexWrap(transforms);
    }
  } else {
    throw invalid_argument("TFWrapper::mex() Unknown method");
  }
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs < 1) {
    mexErrMsgTxt("rosbag_wrapper id ...");
  }

  uint64_t id = mexUnwrap<uint64_t>(prhs[0]);
  if (id == 0) {
    g_manager.mex(nlhs, plhs, nrhs - 1, prhs + 1);
  } else {
    g_manager.get(id)->mex(nlhs, plhs, nrhs - 1, prhs + 1);
  }
}

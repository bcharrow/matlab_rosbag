#ifndef ROSBAG_WRAPPER_HPP
#define ROSBAG_WRAPPER_HPP

#include "mex.h"

#include <boost/scoped_ptr.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "parser.hpp"

class MexWrapper {
public:
  virtual ~MexWrapper() {};
  virtual void mex(int nlhs, mxArray *plhs[], int nrhs,
                   const mxArray *prhs[]) = 0;

  void assertArgs(int expect, int actual);
};

class InstanceManager {
public:
  InstanceManager() : id_ctr_(1) { }
  ~InstanceManager();

  void mex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);

  MexWrapper* get(uint64_t id) {
    if (handles_.count(id) == 0) {
      throw std::invalid_argument("InstanceManager::get() Invalid handle");
    } else {
      return handles_[id];
    }
  }

private:
  uint64_t id_ctr_;                  /**< Next valid ID */
  std::map<uint64_t, MexWrapper*> handles_;
};

class ROSBagWrapper : public MexWrapper {
public:
  ROSBagWrapper(const std::string &fname);

  void resetView(const std::vector<std::string> &topics,
                 ros::Time const &start_time = ros::TIME_MIN,
                 ros::Time const &end_time = ros::TIME_MAX);
  void read(bool flatten, mxArray **msg);
  void read(bool flatten, mxArray **msg, mxArray **meta);
  void readAll(bool flatten, mxArray **msg);
  void readAll(bool flatten, mxArray **msg, mxArray **meta);

  bool hasNext() const {
    return view_ != NULL && iter_ != view_->end();
  }

  const rosbag::Bag& bag() const { return bag_; }

  virtual void mex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);

private:
  std::string path_;
  rosbag::Bag bag_;
  BagInfo info_;
  boost::scoped_ptr<rosbag::View> view_;
  rosbag::View::iterator iter_;
  BagDeserializer deser_;
};

class TFWrapper : public MexWrapper {
public:
  TFWrapper() {}
  virtual void mex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);
private:
  BagTF transformer_;
};

#endif

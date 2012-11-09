#include "mex.h"

#include <string>
#include <map>

#include <boost/foreach.hpp>
#include <boost/scoped_ptr.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "parser.hpp"

using namespace std;

// Taken from GTSAM:
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

void printer(const ROSMessage &m, int indent = 0) {
  string spacer(indent, ' ');
  mexPrintf("%s%s\n",
           spacer.c_str(), m.type().name.c_str());

  for (ROSMessage::const_iterator it = m.begin(); it != m.end(); ++it) {
    mexPrintf("%s- %s:\n", spacer.c_str(), (*it).first.c_str());
    const std::vector<ROSMessage*> &fields = (*it).second;
    for (size_t i = 0; i < fields.size(); ++i) {
      printer(*fields[i], indent + 4);
    }
  }
}


void usage() {
    mexErrMsgTxt("read_bag bagpath topic");
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs != 2 ||
      !(mxIsChar(prhs[0]) && mxGetM(prhs[0]) == 1) ||
      !(mxIsChar(prhs[1]) && mxGetM(prhs[1]) == 1)) {
    usage();
  }
  char *fname_char = mxArrayToString(prhs[0]);
  char *topic_char = mxArrayToString(prhs[1]);
  if (fname_char == NULL || topic_char == NULL) {
    usage();
  }
  string fname(fname_char);
  string topic(topic_char);
  mxFree(fname_char);
  mxFree(topic_char);

  try {
    rosbag::Bag bag;
    bag.open(fname.c_str(), rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(topic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // Map from message name to ROSTypeMap
    BagDeserializer d;
    boost::scoped_ptr<ROSMessage> message;

    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
      message.reset(d.CreateMessage(m));
      printer(*message.get());
    }
  } catch (const exception& e) {
    mexErrMsgTxt(("Exception from read_bag:\n" + string(e.what()) + "\n").c_str());
  }
}

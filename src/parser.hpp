#ifndef MATLAB_BAG_PARSER_HPP
#define MATLAB_BAG_PARSER_HPP

#include <string>
#include <vector>
#include <map>

#include <boost/scoped_array.hpp>
#include <boost/scoped_ptr.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <tf2/buffer_core.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose2D.h>

struct ROSType {
  ROSType() {}

  void populate(const std::string &type_str);

  std::string name;       // Raw type definition
  std::string base_type;  // Concatenation of msg_name and pkg_name
  std::string msg_name;   // geometry_msgs/Pose[40] -> Pose
  std::string pkg_name;   // geometry_msgs/Pose[40] -> geometry_msgs
  bool is_array;          // True if the type is an array
  bool is_qualified;      // True if the type is builtin or fully qualified
  bool is_builtin;        // True if the type is ROS builtin
  int array_size;         // 1 if !is_array, -1 if is_array and array is
                          // variable length, otherwise length in name
  int type_size;          // If builtin, size of builtin, -1 means variable
                          // length (i.e. string).  If not builtin, undefined.
  enum BuiltinID {
    BOOL , BYTE, CHAR,
    UINT8, UINT16, UINT32, UINT64, INT8, INT16, INT32, INT64,
    FLOAT32, FLOAT64,
    TIME, DURATION, STRING
  };
  BuiltinID id;           // If type is builtin, type id.  Undefined otherwise.
private:
  static std::map<std::string, int> builtin_size_;
  static std::map<std::string, BuiltinID> builtin_typeid_;
};

// A container which which has info about which fields a message has and what
// ROSType those fields are
class ROSMessageFields {
public:
  struct Field {
    Field(const std::string &n) : name(n) {}
    std::string name;
    ROSType type;
    bool constant;     // True if field is a constant in message definition
    std::string value; // If constant, value of field, else undefined
    std::vector<uint8_t> bytes; // If constant, serialized version of value
  };

  ROSMessageFields() {}

  void populate(const std::string &msg_def);
  void setFieldPkgName(int i, const std::string &msg_name);

  const ROSType& type() const { return type_; }
  const Field& at(int i) const { return fields_.at(i); };
  int nfields() const { return fields_.size(); }

private:
  ROSType type_;
  std::vector<Field> fields_;
};

// A container for resolving the fields of non-builtin ROSTypes
class ROSTypeMap {
public:
  ROSTypeMap() {};
  ~ROSTypeMap();

  void populate(const std::string &msg_def);
  std::vector<std::string> resolve(const std::string &type) const;

  const ROSMessageFields* getMsgFields(const std::string &msg_type) const;

private:
  mutable std::map<std::string, std::vector<std::string> > resolver_;
  mutable std::map<std::string, ROSMessageFields*> type_map_;
};

// Convenience class to store message definitions from rosbag::MessageInstance's
class BagDeserializer {
public:
  BagDeserializer() {}
  ~BagDeserializer();

  const ROSTypeMap* getTypeMap(const rosbag::MessageInstance &m);

private:
  std::map<std::string, ROSTypeMap*> type_maps_;
};

class BagInfo {
public:
  BagInfo() : bag_(NULL), view_(NULL) {};
  ~BagInfo();

  // Populate info from a bag; bag must be valid as long as BagInfo is in use
  void setBag(const rosbag::Bag *bag);

  // Get a string summarizing the contents of the bag.  Outputs most of the
  // information found in "rosbag info"
  std::string info();
  // Check if input string is a topic name or message type.  Outputs the
  // definition of the resolved message type.
  std::string definition(const std::string &input, bool raw) const;
  // Check if string a topic name
  bool isTopic(const std::string &topic) const;
  // Get a string listing the type of message published on a topic
  std::string topicType(const std::string &topic) const;
  std::vector<std::string> topicType(const std::vector<std::string> &topic) const;
  // Return vector containing all topics in bag matching regular
  // expression regexp.
  std::vector<std::string> topics(const std::string &regexp) const;

  ros::Time getBeginTime() { return time_begin_; }
  ros::Time getEndTime() { return time_end_; }

private:
  struct TopicSummary {
    std::string topic;
    std::string type;
    size_t size;
    size_t nconnections;
  };

  // Get a nicely formatted string describing all TopicSummarys
  std::vector<std::string> formatSummaries(const std::vector<TopicSummary> &topics);
  // Populate raw_msg_defs_ using the connections
  void readRawMsgDefs(const std::vector<const rosbag::ConnectionInfo*> &conns);
  // Populate type_maps_ using the connections
  void readTypeMaps(const std::vector<const rosbag::ConnectionInfo*> &conns);

  const rosbag::Bag *bag_;
  boost::scoped_ptr<rosbag::View> view_;
  // Times of beginning and end of bag
  ros::Time time_begin_, time_end_;

  // Map from message type to raw message definition. The keys includes both
  // qualified names and unqualified names.
  std::map<std::string, std::string> msg_defs_;
  std::map<std::string, ROSTypeMap*> type_maps_;
  // Map from topic names to message type.
  std::map<std::string, std::string> topic_types_;
};

// Get a string which describes the message (basically "rosmsg show")
std::string msg_definition(const ROSTypeMap &tm);
std::string msg_definition(const ROSMessageFields *rmf, const ROSTypeMap &tm);

// Class to deal with transforms from a bag
class BagTF {
public:
  BagTF() : buffer_(NULL) {}
  ~BagTF() {}

  // Read in all transforms from the bag.  Must be called before using any
  // other function.
  void build(const rosbag::Bag &bag, const ros::Time &begin_time,
             const ros::Time &end_time, const std::string &topic);

  void lookup(const std::string &target, const std::string &source,
              const std::vector<double> &times,
              std::vector<geometry_msgs::TransformStamped> *tforms) const;
  void lookup(const std::string &target, const std::string &source,
              const std::vector<double> &times,
              std::vector<geometry_msgs::Pose2D> *tforms) const;

  std::string allFrames() {
    if (!buffer_) {
      throw std::runtime_error("Must call buildTree() before getting frames");
    }
    return buffer_->allFramesAsString();
  }

  // Remove leading '/' if it exists
  std::string cleanFrame(const std::string &frame) const {
    if (frame.size() == 0 || frame[0] != '/') {
      return frame;
    } else {
      return frame.substr(1);
    }
  }

  const ros::Time& beginTime() { return begin_; }
  const ros::Time& endTime() { return end_; }

private:
  boost::scoped_ptr<tf2::BufferCore> buffer_;
  ros::Time begin_, end_;
};

#endif

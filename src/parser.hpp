#ifndef MATLAB_BAG_PARSER_HPP
#define MATLAB_BAG_PARSER_HPP

#include <string>
#include <vector>
#include <map>

#include <boost/scoped_array.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>

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
private:
  static std::map<std::string, int> builtins_;
};

// A container which which has info about which fields a message has and what
// ROSType those fields are
class ROSMessageFields {
public:
  struct Field {
    Field(const std::string &n) : name(n) {}
    std::string name;
    ROSType type;
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

  const ROSMessageFields* getMsgFields(const std::string &msg_type) const;

private:
  std::map<std::string, std::vector<std::string> > resolver_;
  mutable std::map<std::string, ROSMessageFields*> type_map_;
};


// A representation of an instance of a specific message type
class ROSMessage {
public:
  class Field {
  public:
    Field(const std::string &name) : name_(name) {}

    const std::string& name() const { return name_; }
    const ROSMessage& at(int i) const { return *values_.at(i); }
    int size() const { return values_.size(); }
  private:
    std::string name_;
    std::vector<ROSMessage*> values_;

    friend class ROSMessage;
  };
  ROSMessage(const ROSType &type);
  ~ROSMessage();

  const std::vector<std::vector<uint8_t> >& bytes() const { return bytes_; }
  const ROSType& type() const { return type_; }
  int nfields() const { return fields_.size(); }
  const Field& lookupField(const std::string &key) const;
  const Field& at(int i) const { return *fields_.at(i); }

  void populate(const ROSTypeMap &types, const uint8_t *bytes, int *beg);
private:
  std::vector<Field*> fields_;
  std::vector<std::vector<uint8_t> > bytes_;
  const ROSType type_;
};

// Convenience class to create a ROSMessage from a rosbag::MessageInstance
class BagDeserializer {
public:
  BagDeserializer() : bytes_(NULL) {}
  ~BagDeserializer();

  ROSMessage* CreateMessage(const rosbag::MessageInstance &m);

private:
  const ROSTypeMap* getTypeMap(const rosbag::MessageInstance &m);
  void populateMsg(const ROSTypeMap &type_map,
                   const rosbag::MessageInstance &m,
                   ROSMessage *rm);

  std::map<std::string, ROSTypeMap*> type_maps_;
  boost::scoped_array<uint8_t> bytes_;
};

#endif

#include "parser.hpp"

#include <Eigen/Geometry>

#include <tf2_msgs/TFMessage.h>

#include <algorithm>

#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/foreach.hpp>

#include <inttypes.h>

using namespace std;

static const char *g_type_regexp = ("[a-zA-Z][a-zA-Z0-9_]*"
                                    "(/[a-zA-Z][a-zA-Z0-9_]*){0,1}"
                                    "(\\[[0-9]*\\]){0,1}");
static const char *g_field_regexp = "[a-zA-Z][a-zA-Z0-9_]*";
static const char* g_message_split_regexp = "^=+\\n+";

bool validTypeName(const std::string &ref) {
  static const boost::regex re(g_type_regexp);
  return boost::regex_match(ref, re);
}

map<string,int> init_builtins() {
  map<string,int> builtins;
  const char *types[] = {"bool", "byte", "char",
                         "uint8", "uint16", "uint32", "uint64",
                         "int8", "int16", "int32", "int64",
                         "float32", "float64",
                         "time", "duration",
                         "string"};
  const int sizes[] = {1, 1, 1,
                       1, 2, 4, 8,
                       1, 2, 4, 8,
                       4, 8,
                       8, 8,
                       -1};
  for (size_t i = 0; i < sizeof(types) / sizeof(types[0]); ++i) {
    builtins[types[i]] = sizes[i];
  }
  return builtins;
}

map<string, ROSType::BuiltinID> init_typeids() {
  map<string, ROSType::BuiltinID> typeids;
  typeids["bool"] = ROSType::BOOL;
  typeids["byte"] = ROSType::BYTE;
  typeids["char"] = ROSType::CHAR;
  typeids["uint8"] = ROSType::UINT8;
  typeids["uint16"] = ROSType::UINT16;
  typeids["uint32"] = ROSType::UINT32;
  typeids["uint64"] = ROSType::UINT64;
  typeids["int8"] = ROSType::INT8;
  typeids["int16"] = ROSType::INT16;
  typeids["int32"] = ROSType::INT32;
  typeids["int64"] = ROSType::INT64;
  typeids["float32"] = ROSType::FLOAT32;
  typeids["float64"] = ROSType::FLOAT64;
  typeids["time"] = ROSType::TIME;
  typeids["duration"] = ROSType::DURATION;
  typeids["string"] = ROSType::STRING;
  return typeids;
}

map<string,int> ROSType::builtin_size_(init_builtins());
map<string,ROSType::BuiltinID> ROSType::builtin_typeid_(init_typeids());

void ROSType::populate(const string &type_str) {
  if (!validTypeName(type_str)) {
    throw invalid_argument("Bad type string: " + type_str);
  }
  name = type_str;

  vector<string> fields;
  std::string type_field;
  boost::split(fields, type_str, boost::is_any_of("/"));
  if (fields.size() == 1) {
    pkg_name = "";
    type_field = fields[0];
  } else if (fields.size() == 2) {
    pkg_name = fields[0];
    type_field = fields[1];
  } else {
    throw runtime_error("Didn't catch bad type string: " + type_str);
  }

  static const boost::regex array_re("(.+)(\\[([0-9]*)\\])");
  boost::smatch what;
  if (boost::regex_search(type_field, what, array_re)) {
    is_array = true;
    msg_name = string(what[1].first, what[1].second);
    if (what.size() == 3) {
      array_size = -1;
    } else if (what.size() == 4) {
      string size(what[3].first, what[3].second);
      array_size = size.empty() ? -1 : atoi(size.c_str());
    } else {
      throw runtime_error("Didn't catch bad type string: " + type_str);
    }
  } else {
    is_array = false;
    msg_name = type_field;
    array_size = 1;
  }

  is_builtin = builtin_size_.count(msg_name) != 0;
  if (is_builtin) {
    type_size = builtin_size_[msg_name];
    id = builtin_typeid_[msg_name];
  }

  if (is_builtin) {
    is_qualified = true;
    base_type = msg_name;
  } else if (!pkg_name.empty()) {
    is_qualified = true;
    base_type = pkg_name + string("/") + msg_name;
  } else {
    is_qualified = false;
    base_type = msg_name;
  }
}

/**
 * Return vector where each element corresponds to a non-empty line of
 * non-commented out tokens
 *
 */
vector<vector<string> > tokenize(const string &msg_def) {
  static const boost::regex type_re(g_type_regexp);
  static const boost::regex field_re(g_field_regexp);
  static const boost::regex word_re("\\S+");
  vector<vector<string> > lines;
  stringstream ss(msg_def);
  string line;

  boost::match_results<std::string::const_iterator> what;

  // Read first line.  Should be of the form "MSG: <TYPE_NAME>"
  if (string(msg_def.begin(), msg_def.begin() + 4) == "MSG:") {
    getline(ss, line, '\n');
    string::const_iterator begin = line.begin(), end = line.end();
    if (regex_search(begin + 4, end, what, type_re)) {
      lines.push_back(vector<string>());
      lines.back().push_back(what[0]);
    } else {
      throw invalid_argument("Expected ROS type after 'MSG:'\n" + msg_def);
    }
  }

  // Read remaining lines to get fields
  while (getline(ss, line, '\n')) {
    string::const_iterator begin = line.begin(), end = line.end();

    // Skip empty line or one that is a comment
    if (regex_search(begin, end, what, boost::regex("(^\\s*$|^\\s*#)"))) {
      continue;
    }

    lines.push_back(vector<string>());

    // Get type and field
    string type, fieldname;
    if (regex_search(begin, end, what, type_re)) {
      type = what[0];
      begin = what[0].second;
    } else {
      throw invalid_argument("Bad type when parsing message\n" + msg_def);
    }

    if (regex_search(begin, end, what, field_re)) {
      fieldname = what[0];
      begin = what[0].second;
    } else {
      throw invalid_argument("Bad field when parsing message\n" + msg_def);
    }

    lines.back().push_back(type);
    lines.back().push_back(fieldname);

    // Determine next character
    // if '=' -> constant, if '#' -> done, if nothing -> done, otherwise error
    if (regex_search(begin, end, what, boost::regex("\\S"))) {
      if (what[0] == "=") {
        begin = what[0].second;
        // Copy constant
        string value;
        if (type == "string") {
          value.assign(begin, end);
        } else {
          if (regex_search(begin, end, what, boost::regex("\\s*#"))) {
            value.assign(begin, what[0].first);
          } else {
            value.assign(begin, end);
          }
          // TODO: Raise error if string is not numeric
        }

        boost::algorithm::trim(value);
        lines.back().push_back(value);
      } else if (what[0] == "#") {
        // Ignore comment
      } else {
        // Error
        throw invalid_argument("Unexpected character after type and field '" +
                               string(begin, end) + "':\n" + msg_def);
      }
    }
  }
  return lines;
}

// Convert a string representing the value of a constant to bytes
// TODO: Change this so it works on big endian machines
template <typename T>
bool serialize(const string &str_value, vector<uint8_t> *bytes) {
  istringstream ss(str_value);
  T val;
  ss >> val;
  if (ss.bad() || ss.fail() || !ss.eof()) {
    return false;
  }
  uint8_t *val_bytes = reinterpret_cast<uint8_t*>(&val);
  bytes->assign(val_bytes, val_bytes + bytes->size());
  return true;
}

template<>
bool serialize<uint8_t>(const string &str_value, vector<uint8_t> *bytes) {
  istringstream ss(str_value);
  uint16_t val_16;
  ss >> val_16;
  if (ss.bad() || ss.fail() || !ss.eof()) {
    return false;
  }
  uint8_t val = val_16;
  uint8_t *val_bytes = reinterpret_cast<uint8_t*>(&val);
  bytes->assign(val_bytes, val_bytes + bytes->size());
  return true;
}

template<>
bool serialize<int8_t>(const string &str_value, vector<uint8_t> *bytes) {
  istringstream ss(str_value);
  int16_t val_16;
  ss >> val_16;
  if (ss.bad() || ss.fail() || !ss.eof()) {
    return false;
  }
  int8_t val = val_16;
  uint8_t *val_bytes = reinterpret_cast<uint8_t*>(&val);
  bytes->assign(val_bytes, val_bytes + bytes->size());
  return true;
}

// Convert the value represented by "val" of type "type" into "bytes".
bool serialize_constant(const ROSType &type, const string &val,
                        vector<uint8_t> *bytes) {
  if (type.name == "string") {
    uint32_t sz = val.size();
    bytes->resize(sizeof(uint32_t) + sz);
    uint8_t *sz_bytes = reinterpret_cast<uint8_t*>(&sz);
    copy(sz_bytes, sz_bytes + sizeof(uint32_t), bytes->begin());
    copy(val.begin(), val.end(), bytes->begin() + sizeof(uint32_t));
    return true;
  } else {
    istringstream ss(val);
    bytes->resize(type.type_size);

    bool result = false;
    if (type.name == "bool") { result = serialize<bool>(val, bytes); }
    else if (type.name == "byte") { result = serialize<int8_t>(val, bytes); }
    else if (type.name == "char") { result = serialize<uint8_t>(val, bytes); }
    else if (type.name == "uint8") { result = serialize<uint8_t>(val, bytes); }
    else if (type.name == "uint16") { result = serialize<uint16_t>(val, bytes); }
    else if (type.name == "uint32") { result = serialize<uint32_t>(val, bytes); }
    else if (type.name == "uint64") { result = serialize<uint64_t>(val, bytes); }
    else if (type.name == "int8") { result = serialize<int8_t>(val, bytes); }
    else if (type.name == "int16") { result = serialize<int16_t>(val, bytes); }
    else if (type.name == "int32") { result = serialize<int32_t>(val, bytes); }
    else if (type.name == "int64") { result = serialize<int64_t>(val, bytes); }
    else if (type.name == "float32") { result = serialize<float>(val, bytes); }
    else if (type.name == "float64") { result = serialize<double>(val, bytes); }
    else { result = false; }

    return result;
  }
}

void ROSMessageFields::populate(const string &msg_def) {
  vector<vector<string> > lines = tokenize(msg_def);
  int start_ind = 0;
  if (lines.size() > 0 && lines.at(0).size() == 1) {
    type_.populate(lines.at(0).at(0));
    start_ind = 1;
  } else {
    type_.name = "0-root";
    type_.base_type = "0-root";
    type_.msg_name = "";
    type_.pkg_name = "0-root";
    type_.is_array = false;
    type_.is_qualified = true;
    type_.is_builtin = false;
    type_.array_size = 1;
  }

  for (size_t l = start_ind; l < lines.size(); ++l) {
    const vector<string> &toks = lines[l];
    if (toks.size() != 2 && toks.size() != 3) {
      throw runtime_error("Bad parsing of msg_def:\n" + msg_def);
    }
    const std::string &type = toks.at(0);
    const std::string &name = toks.at(1);
    fields_.push_back(Field(name));
    Field &field = fields_.back();

    field.type.populate(type);
    if (toks.size() == 3) {
      if (!field.type.is_builtin || field.type.is_array ||
          field.type.name == "time" || field.type.name == "duration") {
        throw invalid_argument("Invalid constant for " + field.name + ":\n" +
                               msg_def);
      }
      field.constant = true;
      field.value = toks.at(2);
      if (!serialize_constant(field.type, field.value, &(field.bytes))) {
        throw invalid_argument("Invalid constant value:\n" + msg_def);
      }
    } else {
      field.constant = false;
      field.value = "";
    }
  }
}

void ROSMessageFields::setFieldPkgName(int i, const std::string &pkg_name) {
  ROSType &type = fields_.at(i).type;
  type.pkg_name = pkg_name;
  type.base_type = type.pkg_name + string("/") + type.msg_name;
  type.is_qualified = true;
}


ROSTypeMap::~ROSTypeMap() {
  for (map<string, ROSMessageFields*>::iterator it = type_map_.begin();
       it != type_map_.end();
       ++it) {
    delete (*it).second;
  }
}

void ROSTypeMap::populate(const string &msg_def) {
  // Split into disparate message type definitions
  std::vector<std::string> split;
  boost::regex msg_sep_re(g_message_split_regexp);
  boost::split_regex(split, msg_def, msg_sep_re);

  // Parse individual message types and make map from msg_name -> qualified name
  std::vector<ROSMessageFields*> types(split.size());
  for (size_t i = 0; i < split.size(); ++i) {
    ROSMessageFields *rmf = new ROSMessageFields();

    try {
      rmf->populate(split[i]);
    } catch (exception &e) {
      delete rmf;
      throw;
    }

    // If multiple definitions, need to remove previous rmf
    if (type_map_.count(rmf->type().name) != 0) {
      delete type_map_[rmf->type().name];
    }
    type_map_[rmf->type().name] = rmf;
    resolver_[rmf->type().msg_name].push_back(rmf->type().pkg_name);

    // If we throw, rmf will be freed because it's in type_map_
    if (!rmf->type().is_qualified) {
      throw invalid_argument("Couldn't determine type in message:\n" +
                             split[i] + "\nFull message def is:\n" + msg_def);
    }
  }

  for (map<string, ROSMessageFields*>::iterator it = type_map_.begin();
       it != type_map_.end();
       ++it) {
    ROSMessageFields *rmf = (*it).second;
    for (int i = 0; i < rmf->nfields(); ++i) {
      const ROSMessageFields::Field &field = rmf->at(i);
      if (!field.type.is_qualified) {
        const vector<string> qualified_names = resolver_[field.type.msg_name];
        if (qualified_names.size() == 1) {
          rmf->setFieldPkgName(i, qualified_names[0]);
        } else {
          if (qualified_names.size() == 0) {
            throw invalid_argument("No definition for " + field.type.msg_name +
                                   "\nMessage def: \n" + msg_def);
          } else {
            throw invalid_argument("Multiple definitions for " + field.type.msg_name +
                                   "\nMessage def: \n" + msg_def);
          }
        }
      }
    }
  }
}

const ROSMessageFields* ROSTypeMap::getMsgFields(const string &msg_type) const {
  if (type_map_.count(msg_type) != 0) {
    return type_map_[msg_type];
  } else {
    return NULL;
  }
}

vector<string> ROSTypeMap::resolve(const std::string &type) const {
  if (resolver_.count(type) != 0) {
    return resolver_[type];
  } else {
    return vector<string>();
  }
}

BagDeserializer::~BagDeserializer() {
  for (map<string, ROSTypeMap*>::iterator it = type_maps_.begin();
       it != type_maps_.end();
       ++it) {
    delete (*it).second;
  }
}

const ROSTypeMap* BagDeserializer::getTypeMap(const rosbag::MessageInstance &m) {
  if (type_maps_.count(m.getDataType()) == 0) {
    ROSTypeMap *rtm = new ROSTypeMap();
    type_maps_[m.getDataType()] = rtm;
    rtm->populate(m.getMessageDefinition());
  }

  return type_maps_[m.getDataType()];
}

//================================= BagInfo =================================//

BagInfo::~BagInfo() {
  for (map<string, ROSTypeMap*>::iterator it = type_maps_.begin();
       it != type_maps_.end(); ++it) {
    delete it->second;
  }

}

// Format pairs of strings using a format string.  fmt is given (maxlen, first,
// second) where first and second are elements of a pair and maxlen is the size
// of the longest first element.
template<typename ForwardIterator>
vector<string> join(ForwardIterator begin, ForwardIterator end,
                    const char* fmt) {
  static char s[512];
  vector<string> joined;
  int maxlen = -1;
  for (ForwardIterator it = begin; it != end; ++it) {
    maxlen = max(maxlen, static_cast<int>(it->first.size()));
  }
  if (maxlen != -1) {
    for (ForwardIterator it = begin; it != end; ++it) {
      snprintf(s, sizeof(s), fmt,
               maxlen, it->first.c_str(), it->second.c_str());
      joined.push_back(string(s));
    }
  }
  return joined;
}

string human_size(double size) {
  const char *units[] = {"B", "KB", "MB", "GB", "TB", "PB"};
  int ind = 0;
  while (size > 1024.0 && ind + 1 < sizeof(units) / sizeof(units[0])) {
    size /= 1024.0;
    ++ind;
  }
  char s[80];
  snprintf(s, sizeof(s) / sizeof(s[0]), "%.1f %s", size, units[ind]);
  return string(s);
}

string duration_string(const ros::Duration &d) {
  char buf[40];
  boost::posix_time::time_duration td = d.toBoost();

  if (td.hours() > 0) {
    snprintf(buf, sizeof(buf) / sizeof(buf[0]), "%dhr %d:%02ds (%ds)",
             td.hours(), td.minutes(), td.seconds(), td.total_seconds());
  } else if (td.minutes() > 0) {
    snprintf(buf, sizeof(buf) / sizeof(buf[0]), "%d:%02ds (%ds)",
             td.minutes(), td.seconds(), td.total_seconds());
  } else {
    snprintf(buf, sizeof(buf) / sizeof(buf[0]), "%.1fs", d.toSec());
  }
  return string(buf);
}

string time_string(const ros::Time &rtime) {
  char s[60];
  const int buflen = sizeof(s) / sizeof(s[0]);
  time_t t = rtime.sec;
  tm *tm_time = localtime(&t);
  size_t len = strftime(s, buflen, "%b %d %Y %H:%M:%S", tm_time);
  if (len == 0) {
    return string("ERROR MAKING DATE!");
  } else {
    if (len < buflen - 1) {
      snprintf(s + len, buflen - len, ".%d (%0.2f)",
               (int)round(rtime.nsec / 10000000.0), rtime.toSec());
    }
    return string(s);
  }
}

string BagInfo::info() {
  vector<pair<string, string> > rows;
  rows.push_back(make_pair("path:", bag_->getFileName()));

  stringstream ss;
  ss << bag_->getMajorVersion() << "." << bag_->getMinorVersion();
  rows.push_back(make_pair("version:", ss.str()));

  // Don't populate time fields if no messages are present
  if (view_->size() > 0) {
    ros::Duration d = view_->getEndTime() - view_->getBeginTime();
    rows.push_back(make_pair("duration:", duration_string(d)));
    rows.push_back(make_pair("start:", time_string(view_->getBeginTime())));
    rows.push_back(make_pair("end:", time_string(view_->getEndTime())));
  }

  rows.push_back(make_pair("size:", human_size(bag_->getSize())));

  ss.str("");
  ss.clear();
  ss << view_->size();
  rows.push_back(make_pair("messages:", ss.str()));

  // Types + topics
  vector<const rosbag::ConnectionInfo*> connections = view_->getConnections();
  map<string, vector<const rosbag::ConnectionInfo*> > topic_conns;
  for (int i = 0; i < connections.size(); ++i) {
    const rosbag::ConnectionInfo* ci = connections[i];
    topic_conns[ci->topic].push_back(ci);
  }

  if (topic_conns.size() > 0) {
    set<pair<string, string> > type_md5s;
    vector<TopicSummary> topics;

    map<string, vector<const rosbag::ConnectionInfo*> >::iterator it;
    for (it = topic_conns.begin(); it != topic_conns.end(); ++it) {
      const rosbag::ConnectionInfo *ci = it->second.at(0);
      type_md5s.insert(make_pair(ci->datatype, ci->md5sum));

      vector<string> topic;
      topic.push_back(ci->topic);
      rosbag::View topicview(*bag_, rosbag::TopicQuery(topic));
      TopicSummary summary;
      summary.topic = ci->topic;
      summary.type = ci->datatype;
      summary.size = topicview.size();
      summary.nconnections = it->second.size();
      topics.push_back(summary);
    }

    // Type stanza
    vector<string> types = join(type_md5s.begin(), type_md5s.end(),
                                "%-*s [%s]");
    rows.push_back(make_pair("types:", types[0]));
    for (int i = 1; i < types.size(); ++i) {
      rows.push_back(make_pair("", types[i]));
    }

    // Topic stanza
    vector<string> topic_summary = formatSummaries(topics);
    rows.push_back(make_pair("topics:", topic_summary[0]));
    for (int i = 1; i < topics.size(); ++i) {
      rows.push_back(make_pair("", topic_summary[i]));
    }
  }

  ss.str("");
  ss.clear();
  vector<string> joined = join(rows.begin(), rows.end(), "%-*s %s");
  for (int i = 0; i < joined.size(); ++i) {
    ss << joined[i] << endl;
  }
  return ss.str();
}

void BagInfo::setBag(const rosbag::Bag *bag) {
  bag_ = bag;
  view_.reset(new rosbag::View(*bag_));

  time_begin_ = view_->getBeginTime();
  time_end_ = view_->getEndTime();

  msg_defs_.clear();
  type_maps_.clear();

  vector<const rosbag::ConnectionInfo*> connections = view_->getConnections();
  readRawMsgDefs(connections);
  readTypeMaps(connections);
}

void BagInfo::readRawMsgDefs(const vector<const rosbag::ConnectionInfo*> &connections) {
  for (int i = 0; i < connections.size(); ++i) {
    const rosbag::ConnectionInfo *ci = connections.at(i);

    if (msg_defs_.count(ci->datatype) == 0) {
      const boost::regex msg_sep_re(g_message_split_regexp);
      vector<string> split;
      boost::split_regex(split, ci->msg_def, msg_sep_re);
      ROSMessageFields rmf;
      boost::trim(split.at(0));

      ROSType type;
      type.populate(ci->datatype);
      // Add mapping from fully qualified type
      msg_defs_[type.base_type] = split.at(0);
      // Add mapping from name (i.e., no package)
      msg_defs_[type.msg_name] = "MSG: " + ci->datatype + "\n" + split.at(0);
      for (size_t i = 1; i < split.size(); ++i) {
        rmf.populate(split.at(i));
        size_t first_newline = split.at(i).find('\n');
        if (first_newline == string::npos) {
          throw invalid_argument(split.at(i));
        }
        string msg_def = split.at(i).substr(first_newline);
        boost::trim(msg_def);
        // Add mapping from fully qualified type
        msg_defs_[rmf.type().base_type] = msg_def;
        // Add mapping from name (i.e., no package)
        boost::trim(split.at(i));
        msg_defs_[rmf.type().msg_name] = split.at(i);
      }
    }
  }
}

void BagInfo::readTypeMaps(const vector<const rosbag::ConnectionInfo*> &connections) {
  set<string> seen;
  for (int i = 0; i < connections.size(); ++i) {
    const rosbag::ConnectionInfo *ci = connections.at(i);
    if (seen.count(ci->datatype) == 0) {
      seen.insert(ci->datatype);
      ROSTypeMap *rtm = new ROSTypeMap();
      type_maps_[ci->datatype] = rtm;
      // rtm is in type_maps_ so if populate throws destructor will free it
      rtm->populate(ci->msg_def);
    }
    if (topic_types_.find(ci->topic) == topic_types_.end()) {
      topic_types_[ci->topic] = ci->datatype;
    }
  }
}

vector<string> BagInfo::formatSummaries(const vector<TopicSummary> &topics) {
  size_t max_size = 0, max_topiclen = 0, max_typelen = 0;
  for (int i = 0; i < topics.size(); ++i) {
    const TopicSummary &ts = topics[i];
    stringstream ss;
    ss << ts.size;
    max_size = max(max_size, ss.str().size());
    max_topiclen = max(max_topiclen, ts.topic.size());
    max_typelen = max(max_typelen, ts.type.size());
  }

  vector<string> topic_summary;
  char s[512];
  const int bufsize = sizeof(s) / sizeof(s[0]);
  const char *msg = "msg";
  const char *msgs = "msgs";
  for (int i = 0; i < topics.size(); ++i) {
    const TopicSummary &ts = topics[i];
    int len = snprintf(s, bufsize, "%-*s  %*zu %-4s  : %-*s",
                       (int)max_topiclen, ts.topic.c_str(),
                       (int)max_size, ts.size, ts.size > 1 ? msgs : msg,
                       (int)max_typelen, ts.type.c_str());
    if (len < 0) {
      topic_summary.push_back("Problem summarizing topic!");
    } else {
      if (ts.nconnections > 1 && len < bufsize - 1) {
        snprintf(s + len, bufsize - len, " (%zu connections)",
                 ts.nconnections);
      }
      topic_summary.push_back(string(s));
    }
  }
  return topic_summary;
}

string BagInfo::definition(const std::string &input, bool raw) const {
  string msg_type;
  string msg_def;

  // Check if input is a topic name, else assume it is a message type
  if (isTopic(input)) {
    msg_type = topicType(input);
    msg_def = "MSG: " + msg_type + "\n";
  } else {
    msg_type = input;
  }

  if (raw) {
    stringstream ss;
    map<string, string>::const_iterator it = msg_defs_.find(msg_type);
    if (it == msg_defs_.end()) {
      throw invalid_argument("No topic or message definition for '" + input + "' found");
    }
    return msg_def + it->second;
  } else {
    ROSType type;
    type.populate(msg_type);
    string resolved_type;

    map<string, ROSTypeMap*>::const_iterator it;
    // Resolve name to absolute message type; just take first name that matches
    if (type.is_qualified) {
      resolved_type = type.base_type;
    } else {
      for (it = type_maps_.begin(); it != type_maps_.end(); ++it) {
        // Check if type is defined in the message definition or it's the root
        // type of the type map
        vector<string> pkg_names = it->second->resolve(type.base_type);
        string subtype = it->first.substr(it->first.find("/") + 1);
        if (pkg_names.size() != 0) {
          resolved_type = pkg_names.at(0) + "/" + type.base_type;
          msg_def = "MSG: " + resolved_type + "\n";
          break;
        } else if (subtype == type.base_type) {
          resolved_type = it->first;
          msg_def = "MSG: " + resolved_type + "\n";
          break;
        }
      }
      if (it == type_maps_.end()) {
        throw invalid_argument("No topic or message definition for '" + input + "' found");
      }
    }

    // Now that type is guaranteed to be resolved, lookup message definition
    it = type_maps_.find(resolved_type);
    if (it != type_maps_.end()) {
      return msg_def + msg_definition(*it->second);
    } else {
      for (map<string, ROSTypeMap*>::const_iterator it = type_maps_.begin();
           it != type_maps_.end(); ++it) {
        const ROSMessageFields *rmf = it->second->getMsgFields(resolved_type);
        if (rmf != NULL) {
          return msg_def + msg_definition(rmf, *it->second);
        }
      }
      throw invalid_argument("No topic or message definition for '" + input + "' found");
    }
  }
}

bool BagInfo::isTopic(const string &topic) const {
  map<string, string>::const_iterator it = topic_types_.find(topic);
  if (it != topic_types_.end()) {
    return true;
  } else {
    return false;
  }
}

string BagInfo::topicType(const string &topic) const {
  map<string, string>::const_iterator it = topic_types_.find(topic);
  if (it != topic_types_.end()) {
    return it->second;
  } else {
    throw invalid_argument("Couldn't find topic '" + topic + "'");
  }
}

vector<string> BagInfo::topicType(const vector<string> &topics) const {
  vector<string> types(topics.size());
  vector<string>::const_iterator it;
  int i = 0;
  for (it = topics.begin(), i = 0; it != topics.end(); ++it, ++i) {
    types[i] = topicType(*it);
  }
  return types;
}

vector<string> BagInfo::topics(const string &regexp) const {
  const boost::regex reg(regexp);
  vector<string> topic_vec;
  map<string, string>::const_iterator it;
  for (it = topic_types_.begin(); it != topic_types_.end(); ++it) {
    if (boost::regex_match(it->first, reg)) {
      topic_vec.push_back(it->first);
    }
  }
  return topic_vec;
}

//============================= msg_definition ==============================//

void msg_definition_helper(const ROSMessageFields *rmf, const ROSTypeMap &rtm,
                           int indent, stringstream *ss) {
  string spacer(indent, ' ');
  for (int i = 0; i < rmf->nfields(); ++i) {
    const ROSMessageFields::Field &field = rmf->at(i);
    stringstream type;
    type << field.type.base_type;
    if (field.type.is_array) {
      type << "[";
      if (field.type.array_size != -1) {
        type << field.type.array_size;
      }
      type << "]";
    }
    (*ss) << spacer << type.str() << " " << field.name << endl;
    if (!field.type.is_builtin) {
      msg_definition_helper(rtm.getMsgFields(field.type.base_type), rtm,
                            indent + 2, ss);
    }
  }
}

string msg_definition(const ROSTypeMap &tm) {
  stringstream ss;
  msg_definition_helper(tm.getMsgFields("0-root"), tm, 0, &ss);
  return ss.str();
}

string msg_definition(const ROSMessageFields *rmf, const ROSTypeMap &tm) {
  stringstream ss;
  msg_definition_helper(rmf, tm, 0, &ss);
  return ss.str();
}

//================================== BagTF ==================================//

void BagTF::build(const rosbag::Bag &bag, const ros::Time &begin,
                  const ros::Time &end, const string &tf_topic) {
  string static_topic = tf_topic + "_static";
  vector<string> topics;
  topics.push_back(tf_topic);
  topics.push_back(static_topic);
  rosbag::View view(bag, rosbag::TopicQuery(topics), begin, end);
  begin_ = view.getBeginTime();
  end_ = view.getEndTime();

  // Cache all messages in the selected time interval
  ros::Duration cache_time = view.getEndTime() - view.getBeginTime();
  cache_time += ros::Duration(1.0);

  buffer_.reset(new tf2::BufferCore(cache_time));
  BOOST_FOREACH(const rosbag::MessageInstance &m, view) {
    tf2_msgs::TFMessage::Ptr tf2_msg;
    vector<geometry_msgs::TransformStamped> *transforms;

    const string &datatype = m.getDataType();
    if (datatype == std::string("tf/tfMessage") ||
        datatype == ros::message_traits::datatype<tf2_msgs::TFMessage>()) {
      tf2_msg = m.instantiate<tf2_msgs::TFMessage>();
      transforms = &tf2_msg->transforms;
    } else {
      throw runtime_error(string("Non-transform datatype (") + datatype +
                          string(") on TF topic ") + m.getTopic());
    }

    bool is_static = m.getTopic() == static_topic;
    for (size_t t = 0; t < transforms->size(); ++t) {
      buffer_->setTransform(transforms->at(t), m.getCallerId(), is_static);
    }
  }
}

void BagTF::lookup(const string &target_in, const string &source_in,
                   const vector<double> &times,
                   vector<geometry_msgs::TransformStamped> *tforms) const {
  if (!buffer_) {
    throw std::runtime_error("Must call build() before getting frames");
  }
  string target = cleanFrame(target_in);
  string source = cleanFrame(source_in);

  tforms->resize(times.size());
  for (size_t i = 0; i < tforms->size(); ++i) {
    double t = times.at(i);
    tforms->at(i) = buffer_->lookupTransform(target, source, ros::Time(t));
  }
}

double getYaw(geometry_msgs::Quaternion &msg) {
  Eigen::Quaternion<double> q(msg.w, msg.x, msg.y, msg.z);
  Eigen::Vector3d rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);
  return rpy(2);
}

void BagTF::lookup(const string &target_in, const string &source_in,
                   const vector<double> &times,
                   vector<geometry_msgs::Pose2D> *tforms) const {
  if (!buffer_) {
    throw std::runtime_error("Must call build() before getting frames");
  }
  string target = cleanFrame(target_in);
  string source = cleanFrame(source_in);

  tforms->resize(times.size());
  for (size_t i = 0; i < tforms->size(); ++i) {
    double t = times.at(i);
    geometry_msgs::TransformStamped tform =
      buffer_->lookupTransform(target, source, ros::Time(t));
    geometry_msgs::Pose2D &pose = tforms->at(i);
    pose.x = tform.transform.translation.x;
    pose.y = tform.transform.translation.y;
    pose.theta = getYaw(tform.transform.rotation);
  }
}

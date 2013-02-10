#include "parser.hpp"

#include <algorithm>

#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>

#include <inttypes.h>

using namespace std;

static const char *g_type_regexp = ("[a-zA-Z][a-zA-Z1-9_]*"
                                    "(/[a-zA-Z][a-zA-Z1-9_]*){0,1}"
                                    "(\\[[0-9]*\\]){0,1}");
static const char *g_field_regexp = "[a-zA-Z][a-zA-Z1-9_]*";

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

map<string,int> ROSType::builtins_(init_builtins());

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

  is_builtin = builtins_.count(msg_name) != 0;
  if (is_builtin) {
    type_size = builtins_[msg_name];
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
  bool result = ss >> val;
  if (!result) {
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
  bool result = ss >> val_16;
  if (!result) {
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
  bool result = ss >> val_16;
  if (!result) {
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
    bytes->resize(val.size());
    copy(val.begin(), val.end(), bytes->begin());
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
  if (lines.at(0).size() == 1) {
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
  boost::regex msg_sep_re("=+\\n+");
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
          throw invalid_argument("Multiple types for " + field.type.msg_name +
                                 "\nMessage def: \n" + msg_def);
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

ROSMessage::ROSMessage(const ROSType &type)
  : fields_(0), bytes_(0), type_(type) {

}

ROSMessage::~ROSMessage() {
  vector<Field*>::iterator it;
  vector<ROSMessage*>::iterator mit;
  for (it = fields_.begin(); it != fields_.end(); ++it) {
    for (mit = (*it)->values_.begin(); mit != (*it)->values_.end(); ++mit) {
      delete (*mit);
    }
    delete (*it);
  }
}

uint32_t read_uint32(const uint8_t *bytes, int *beg) {
  uint32_t val = reinterpret_cast<const uint32_t*>(bytes + *beg)[0];
  *beg += 4;
  return val;
}

const ROSMessage::Field& ROSMessage::lookupField(const std::string &key) const {
  for (size_t i = 0; i < fields_.size(); ++i) {
    if (fields_[i]->name() == key) {
      return *fields_[i];
    }
  }
  throw invalid_argument("Field lookup failed: " + key);
}

void ROSMessage::populate(const ROSTypeMap &types, const uint8_t *bytes, int *beg) {
  if (type_.is_builtin) {
    int array_len = type_.array_size;
    if (array_len == -1) {
      array_len = read_uint32(bytes, beg);
    }

    for (int array_ind = 0; array_ind < array_len; ++array_ind) {
      int elem_size = type_.type_size;
      if (elem_size == -1) {
        elem_size = read_uint32(bytes, beg);
      }

      bytes_.push_back(std::vector<uint8_t>(elem_size));
      std::vector<uint8_t> &element = bytes_.back();
      for (size_t i = 0; i < element.size(); ++i) {
        element[i] = bytes[*beg + i];
      }
      *beg += elem_size;
    }
  } else {
    const ROSMessageFields *rmfs = types.getMsgFields(type_.base_type);
    if (rmfs == NULL) {
      throw invalid_argument("Couldn't resolve fields for " + type_.base_type);
    }

    for (int i = 0; i < rmfs->nfields(); ++i) {
      const ROSMessageFields::Field &rmf = rmfs->at(i);
      int array_len = rmf.type.is_builtin ? 1 : rmf.type.array_size;
      if (array_len == -1) {
        array_len = read_uint32(bytes, beg);
      }

      ROSMessage::Field *field = new ROSMessage::Field(rmf.name);
      fields_.push_back(field);

      // If field is not a constant, recursively populate it.  Otherwise
      // serialize the constant's value to bytes
      if (!rmf.constant) {
        for (int array_ind = 0; array_ind < array_len; ++array_ind) {
          ROSMessage *msg = new ROSMessage(rmf.type);
          field->values_.push_back(msg);
          // If populate throws, msg will be freed as it's in fields_
          msg->populate(types, bytes, beg);
        }
      } else {
        ROSMessage *msg = new ROSMessage(rmf.type);
        msg->bytes_.push_back(rmf.bytes);
        field->values_.push_back(msg);
      }
    }
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
    rtm->populate(m.getMessageDefinition());
    type_maps_[m.getDataType()] = rtm;
  }

  return type_maps_[m.getDataType()];
}

void BagDeserializer::populateMsg(const ROSTypeMap &type_map,
                                  const rosbag::MessageInstance &m,
                                  ROSMessage *rm) {
  bytes_.reset(new uint8_t[m.size()]);
  ros::serialization::IStream stream(bytes_.get(), m.size());
  m.write(stream);

  int beg = 0;
  rm->populate(type_map, bytes_.get(), &beg);
  if (beg != static_cast<int>(m.size())) {
    throw invalid_argument("Not all bytes were consumed while reading message");
  }
}

ROSMessage* BagDeserializer::CreateMessage(const rosbag::MessageInstance &m) {
  const ROSTypeMap *rtm = getTypeMap(m);

  const ROSMessageFields *rmf = rtm->getMsgFields("0-root");
  if (rmf == NULL) {
    throw invalid_argument("Couldn't get root message type");
  }

  ROSMessage *msg = new ROSMessage(rmf->type());

  try {
    populateMsg(*rtm, m, msg);
  } catch (std::exception &e) {
    delete msg;
    throw;
  }

  return msg;
}

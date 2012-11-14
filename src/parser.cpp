#include "parser.hpp"

#include <algorithm>

#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>

using namespace std;

map<string,int> init_builtins() {
  map<string,int> builtins;
  const char *types[] = {"uint8", "uint16", "uint32", "uint64",
                         "int8", "int16", "int32", "int64",
                         "float32", "float64",
                         "time", "duration",
                         "string"};
  const int sizes[] = {1, 2, 4, 8,
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
    throw invalid_argument("Bad type string: " + type_str);
  }

  static const boost::regex array_re("(.+)(\\[([0-9]*)\\])");
  boost::smatch what;
  if (boost::regex_search(type_field, what, array_re)) {
    is_array = true;
    msg_name = string(what[1].first, what[1].second);
    array_type = string(what[2].first, what[2].second);
    if (what.size() == 3) {
      array_size = -1;
    } else if (what.size() == 4) {
      string size(what[3].first, what[3].second);
      array_size = size.empty() ? -1 : atoi(size.c_str());
    } else {
      throw invalid_argument("Bad type string: " + type_str);
    }
  } else {
    is_array = false;
    msg_name = type_field;
    array_type = "";
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

void ROSMessageFields::populate(const string &msg_def) {
  static const boost::regex newline_re("\\n+");
  static const boost::regex whitespace_re("\\s+");
  vector<string> lines;
  boost::split_regex(lines, msg_def, newline_re);

  int start_ind = 0;
  if (boost::starts_with(lines[0], "MSG:")) {
    vector<string> parts;
    boost::split_regex(parts, lines[0], whitespace_re);
    type_.populate(parts[1]);
    start_ind = 1;
  } else {
    type_.name = "0-root";
    type_.base_type = "0-root";
    type_.msg_name = "";
    type_.pkg_name = "0-root";
    type_.array_type = "";
    type_.is_array = false;
    type_.is_qualified = true;
    type_.is_builtin = false;
    type_.array_size = 1;
  }

  boost::smatch what;
  for (size_t l = start_ind; l < lines.size(); ++l) {
    if (lines[l].size() == 0 || boost::starts_with(lines[l], "#")) {
      continue;
    }
    vector<string> elements;
    boost::split_regex(elements, lines[l], whitespace_re);
    if (elements.size() != 2) {
      throw invalid_argument("Bad line in message def: %s" + lines[l]);
    }
    fields_.push_back(Field(elements[1]));
    fields_.back().type.populate(elements[0]);
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
    ROSMessageFields *rmt = new ROSMessageFields();

    try {
      rmt->populate(split[i]);
    } catch (exception &e) {
      delete rmt;
      throw;
    }

    type_map_[rmt->type().name] = rmt;
    resolver_[rmt->type().msg_name].push_back(rmt->type().pkg_name);

    // If we throw, rmt will be freed because it's in type_map_
    if (!rmt->type().is_qualified) {
      throw invalid_argument("Couldn't determine type in message:\n" +
                             split[i] + "\nFull message def is:\n" + msg_def);
    }
  }

  for (map<string, ROSMessageFields*>::iterator it = type_map_.begin();
       it != type_map_.end();
       ++it) {
    ROSMessageFields *rmt = (*it).second;
    for (int i = 0; i < rmt->nfields(); ++i) {
      const ROSMessageFields::Field &field = rmt->at(i);
      if (!field.type.is_qualified) {
        const vector<string> qualified_names = resolver_[field.type.msg_name];
        if (qualified_names.size() == 1) {
          rmt->setFieldPkgName(i, qualified_names[0]);
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
  : bytes_(0), type_(type) {

}

ROSMessage::~ROSMessage() {
  for (map<string, vector<ROSMessage*> >::iterator it = fields_.begin();
       it != fields_.end();
       ++it) {
    for (vector<ROSMessage*>::iterator mit = (*it).second.begin();
         mit != (*it).second.end();
         ++mit) {
      delete (*mit);
    }
  }
}

uint32_t read_uint32(const uint8_t *bytes, int *beg) {
  uint32_t val = reinterpret_cast<const uint32_t*>(bytes + *beg)[0];
  *beg += 4;
  return val;
}

vector<const ROSMessage*> ROSMessage::getField(const std::string key) const {
  vector<const ROSMessage*> field;

  if (fields_.count(key) != 0) {
    field.assign(fields_[key].begin(), fields_[key].end());
  }
  return field;
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
    const ROSMessageFields *mt = types.getMsgFields(type_.base_type);
    if (mt == NULL) {
      throw invalid_argument("Couldn't populate fields");
    }

    for (int i = 0; i < mt->nfields(); ++i) {
      const ROSMessageFields::Field &field = mt->at(i);
      int array_len = field.type.is_builtin ? 1 : field.type.array_size;
      if (array_len == -1) {
        array_len = read_uint32(bytes, beg);
      }

      for (int array_ind = 0; array_ind < array_len; ++array_ind) {
        ROSMessage *msg = new ROSMessage(field.type);
        fields_[field.name].push_back(msg);
        // If populate throws, msg will be freed as it's in fields_
        msg->populate(types, bytes, beg);
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

#include <string>
#include <map>

#include <boost/foreach.hpp>
#include <boost/scoped_ptr.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "parser.hpp"

using namespace std;

void printer(const ROSMessage &m, int indent = 0) {
  string spacer(indent, ' ');
  ROS_INFO("%s%s",
           spacer.c_str(), m.type().name.c_str());

  for (int i = 0; i < m.nfields(); ++i) {
    const ROSMessage::Field &field = m.at(i);
    ROS_INFO("%s- %s:", spacer.c_str(), field.name().c_str());
    for (int j = 0; j < field.size(); ++j) {
      printer(field.at(j), indent + 4);
    }
  }
}

int main(int argc, char **argv) {
  if (argc != 3) {
    fprintf(stderr, "Usage: read_bag_test bagfile topic\n");
    return -1;
  }
  const char* fname = argv[1];
  rosbag::Bag bag;
  bag.open(fname, rosbag::bagmode::Read);
  
  std::vector<std::string> topics;
  topics.push_back(std::string(argv[2]));

  rosbag::View view(bag, rosbag::TopicQuery(topics));
  
  BagDeserializer bd;
  boost::scoped_ptr<ROSMessage> message;
  
  BOOST_FOREACH(rosbag::MessageInstance const m, view) {
    message.reset(bd.CreateMessage(m));
    if (message.get() == NULL) {
      ROS_ERROR("Problem parsing message");
      return -1;
    }
    printer(*message);
  }
}

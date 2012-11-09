mex -g read_bag.cpp parser.cpp  \
  ./librosbag.a \
  ./librosconsole.a \
  ./libroscpp_serialization.a \
  ./libros.a \
  ./librostime.a \
  ./liblog4cxx.a \
  ./libaprutil-1.a \
  ./libapr-1.a \
  ./libexpat.a \
  ../lib_boost/libboost_filesystem.a \
  ../lib_boost/libboost_system.a \
  ../lib_boost/libboost_signals.a \
  ../lib_boost/libboost_thread.a \
  ../lib_boost/libboost_regex.a \
-I/opt/ros/electric/stacks/ros_comm/tools/rosbag/include \
-I/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/include \
-I/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_serialization/include \
-I/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_traits/include \
-I/opt/ros/electric/stacks/ros_comm/utilities/xmlrpcpp/src \
-I/opt/ros/electric/stacks/ros_comm/tools/rosconsole/include \
-I/opt/ros/electric/stacks/ros_comm/utilities/rostime/include \
-I/opt/ros/electric/stacks/ros_comm/utilities/cpp_common/include \
  -lbz2 \
  -lrt

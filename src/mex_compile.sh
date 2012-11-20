mex -g rosbag_wrapper.cpp parser.cpp  \
  ../lib/librosbag.a \
  ../lib/librosconsole.a \
  ../lib/libroscpp_serialization.a \
  ../lib/libros.a \
  ../lib/librostime.a \
  ../lib/liblog4cxx.a \
  ../lib/libaprutil-1.a \
  ../lib/libapr-1.a \
  ../lib/libexpat.a \
  ../lib/libboost_filesystem.a \
  ../lib/libboost_system.a \
  ../lib/libboost_signals.a \
  ../lib/libboost_thread.a \
  ../lib/libboost_regex.a \
  ../lib/libbz2.a \
-I/opt/ros/electric/stacks/ros_comm/tools/rosbag/include \
-I/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/include \
-I/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_serialization/include \
-I/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_traits/include \
-I/opt/ros/electric/stacks/ros_comm/utilities/xmlrpcpp/src \
-I/opt/ros/electric/stacks/ros_comm/tools/rosconsole/include \
-I/opt/ros/electric/stacks/ros_comm/utilities/rostime/include \
-I/opt/ros/electric/stacks/ros_comm/utilities/cpp_common/include \
  -lrt

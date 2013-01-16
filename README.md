# Overview
matlab_rosbag is a library for reading ROS bags in Matlab.  It uses the C++ ROS API and returns messages as structs.  Because all of the work is done inside of a statically linked mex function, ROS does not need to be installed on a machine to use this library.  

Building this library requires some work, so you can download a pre-compiled version [here](http://www.seas.upenn.edu/~bcharrow/code.html).  If you want to compile things yourself see COMPILING.md

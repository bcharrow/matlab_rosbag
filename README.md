# Overview

matlab_rosbag is a library for reading ROS bags in Matlab.  It uses the C++ ROS API and returns messages as structs.  Because all of the work is done inside of a statically linked mex function, ROS does not need to be installed on a machine to use this library.

Building things requires some work and so I recommend you download a precompiled version of the code:

<b>Version 0.1</b>
 - Download [Mac OS X 64-bit](http://www.seas.upenn.edu/~bcharrow/media/code/matlab_rosbag-0.1-mac64.zip)
 - Download [Linux 64-bit](http://www.seas.upenn.edu/~bcharrow/media/code/matlab_rosbag-0.1-linux64.zip)

If you want to compile things yourself see <tt>COMPILING.md</tt>.  WARNING: This code won't work at all if your machine is big-endian.

# Usage

The only class you need to interact with is <tt>ros.Bag</tt> which can read ROS messages on topics from a bag and return them as Matlab structs.  Multiple messages are returned as cell arrays and it's possible to access some meta-data (e.g., topic, msg type). To get a basic idea of how the code works see <tt>example.m</tt> and <tt>example.bag</tt> which are both available in the downloads.

The fields in the structs are guaranteed to be in the same order as they are in the message definition.  There are also some utilities for converting messages from structs to matrices.

NOTE: Prior to [version 1.2](http://www.ros.org/wiki/Bags/Format/1.2), bags do not store message definitions.  As such, matlab_rosbag won't be able to do anything with these old bags.  Don't worry about this if you've been using a not terribly old version of ROS (i.e., C turtle and later).

#!/bin/bash
INCLUDE="$(pkg-config --cflags eigen3) $(pkg-config --cflags cpp_common)"
LINK="$(pkg-config --libs-only-L cpp_common)"
if [[ $(uname -s) == "Linux" ]]; then
    LINK="${LINK} -lrt"
fi

mex -O rosbag_wrapper.cpp parser.cpp \
    -lrosbag_storage \
    -lcpp_common \
    -lrostime \
    -lconsole_bridge \
    -ltf2 \
    -lroscpp_serialization \
    -lboost_regex \
    -lboost_signals \
    -lboost_system \
    -lbz2 \
    ${INCLUDE} ${LINK}

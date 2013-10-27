INSTALL="../../../install/lib"
INCLUDE="-I../../../install/include -I/usr/local/include"
LINK=""
if [[ $(uname -s) == "Linux" ]]; then
    LINK="${LINK} -lrt"
fi

mex -O rosbag_wrapper.cpp parser.cpp \
  ${INSTALL}/librosbag.a \
  ${INSTALL}/libroscpp.a \
  ${INSTALL}/librostime.a \
  ${INSTALL}/libtf2.a \
  ${INSTALL}/libroscpp_serialization.a \
  ${INSTALL}/libboost_regex.a \
  ${INSTALL}/libboost_system.a \
  ${INSTALL}/libbz2.a \
  ${INCLUDE} ${LINK}

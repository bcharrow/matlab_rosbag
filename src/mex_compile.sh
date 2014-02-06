INSTALL="../../../install/lib"
INCLUDE="-I../../../install/include $(pkg-config --cflags eigen3)"
LINK=""
if [[ $(uname -s) == "Linux" ]]; then
    LINK="${LINK} -lrt"
fi

mex -O rosbag_wrapper.cpp parser.cpp \
    ${INSTALL}/librosbag_storage.a \
    ${INSTALL}/libcpp_common.a \
    ${INSTALL}/librostime.a \
    ${INSTALL}/libconsole_bridge.a \
    ${INSTALL}/libtf2.a \
    ${INSTALL}/libroscpp_serialization.a \
    ${INSTALL}/libboost_regex.a \
    ${INSTALL}/libboost_signals.a \
    ${INSTALL}/libboost_system.a \
    ${INSTALL}/libbz2.a \
    ${INCLUDE} ${LINK}

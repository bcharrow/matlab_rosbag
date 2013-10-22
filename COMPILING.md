# Overview

In order to compile rosbag_wrapper into a mex function so that it can be used on a machine without ROS we need to statically compile all of the libraries that the C++ ROS bag API depends on.  To make things harder, [mex files are shared object files](http://www.mathworks.com/help/matlab/matlab_external/troubleshooting-mex-files.html#bsscx2j-1) and so each statically linked library must have been compiled with the -fPIC flag.  See [this page](http://www.gentoo.org/proj/en/base/amd64/howtos/index.xml?part=1&chap=3) for a description of -fPIC.

Even if you only want to compile code for your machine, Matlab comes with its own version of several libraries used by ROS -- most notably Boost -- and these versions may be incompatible with your system version.  So, you'll either need to statically compile boost, or compile and link against the version that Matlab uses.

## Using ROS Groovy

Install the [rosinstall_generator](http://wiki.ros.org/rosinstall_generator#Installation) package for ROS.

Generate install file and download necessary packages

    mkdir ~/matbag_ws && cd ~/matbag_ws
    rosinstall_generator rosbag --deps > base.rosinstall
    rosinstall --catkin -n src base.rosinstall
    rm src/CMakeLists.txt
    src/catkin/bin/catkin_init_workspace src

Modify src/CMakeLists.txt so that the lines after cmake_minimum_required() are:

    set(BUILD_SHARED_LIBS false)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
    add_definitions(-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_NONE)

By compiling out logging with ROSCONSOLE_SEVERITY_NONE, we don't need to build log4cxx or any of its dependencies.  Now build the ROS libraries.

    .src/catkin/bin/catkin_make

Now copy all of the libraries to <tt>matbag_ws/install/lib</tt>

    cp devel/lib/* ~/matbag_ws/install/lib/


### [Boost](http://www.boost.org/users/download/)

    ./bjam cxxflags='-fPIC'  --build-dir=mybuild --with-regex --with-system \
      --stagedir=$HOME/matab_ws/install/ link=static stage

Now copy all of the libraries to <tt>matbag_ws/install/lib</tt>

    cp stage/lib/* ~/matbag_ws/install/lib/

### [bz2](http://www.bzip.org/downloads.html)

Add <tt>-fPIC</tt> to the CFLAGS line in the Makefile.

    make install PREFIX=$HOME/matbag_ws/install

### matlab_rosbag
Clone the repository and use the <tt>mex_compile.sh</tt> build script

    cd ~/matbag_ws/src
    git clone git://github.com/bcharrow/matlab_rosbag.git
    cd matlab_rosbag/src
    bash mex_compile.sh


## Using ROS Electric

### ROS libraries

You'll need to compile static libraries for the following packages:

    rosbag rosconsole rostime roscpp_serialization roscpp

To compile static libraries go to a package's build folder and enter:

    cmake .. -DROS_BUILD_STATIC_LIBS:=true && make

Now check in ../lib and copy the static version of the library to LIB_DIR.
where LIB_DIR is a directory where you will put all of the static libraries.

### [Boost](http://www.boost.org/users/download/)

    ./bootstrap.sh
    ./bjam cxxflags='-fPIC'  --build-dir=mybuild \
        --with-regex --with-thread --with-signals --with-filesystem \
        --with-system link=static stage
     cp stage/lib/* LIB_DIR

### [bz2](http://www.bzip.org/downloads.html)

### [expat](http://sourceforge.net/projects/expat/)
    CFLAGS=-fPIC ./configure --prefix=$(pwd)/install
    make
    make install
    cp install/lib/libexpt.a LIB_DIR

### [APR](http://apr.apache.org/download.cgi)
    CFLAGS=-fPIC ./configure --prefix=$(pwd)/install
    make
    make install
    cp .libs/libapr-1.a LIB_DIR

### [APR-iconv](http://apr.apache.org/download.cgi)

IMPORTANT: On OS X I had to compile the APR implementation of iconv() and compile apr-util against that.  On Ubuntu 11.10, I didn't have to do this.


    CFLAGS=-fPIC ./configure --with-apr=../apr-1.4.6/ --prefix=$(pwd)/install
    make
    make install
    cp lib/.libs/libapriconv-1.a LIB_DIR

### [APR-util](http://apr.apache.org/download.cgi)

    CFLAGS=-fPIC ./configure \
                 --with-iconv=../apr-iconv-1.2.1/install \
                 --with-apr=../apr-1.4.6/install \
                 --with-expat=../expat-2.1.0/install
    make
    cp .libs/libapr-1.a LIB_DIR

### [log4cxx](http://logging.apache.org/log4cxx/download.html)

    CFLAGS=-fPIC ./configure --prefix=$(pwd)/install \
        --with-apr=../apr-1.4.6/install \
        --with-apr-util=../apr-util-1.5.1/install
    make
    make install
    cp install/lib/liblog4cxx.a LIB_DIR


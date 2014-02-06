# Overview

In order to compile rosbag_wrapper into a mex function so that it can be used on a machine without ROS we need to statically compile all of the libraries that the C++ ROS bag API depends on.  To make things harder, [mex files are shared object files](http://www.mathworks.com/help/matlab/matlab_external/troubleshooting-mex-files.html#bsscx2j-1) and so each statically linked library must have been compiled with the -fPIC flag.  See [this page](http://www.gentoo.org/proj/en/base/amd64/howtos/index.xml?part=1&chap=3) for a description of -fPIC.

Even if you only want to compile code for your machine, Matlab comes with its own version of several libraries used by ROS -- most notably Boost -- and these versions may be incompatible with your system version.  So, you'll either need to statically compile boost, or compile and link against the version that Matlab uses.

## Using ROS Hydro
You need to install each of these things in order.  First, make the workspace:

    mkdir ~/matbag_ws && cd ~/matbag_ws

### [Boost](http://www.boost.org/users/download/)
Download and unpack the latest version of Boost and go to that directory.

    ./bootstrap.sh
    ./b2 cxxflags='-fPIC' --build-dir=build --with-regex --with-system \
      --with-filesystem --with-program_options --with-date_time --with-thread \
      --with-signals link=static --prefix=$HOME/matbag_ws/install/ install

To compile things with multiple processors, add the flag <tt>-jNUM_PROCESSORS</tt>

### [bz2](http://www.bzip.org/downloads.html)

Add <tt>-fPIC</tt> to the CFLAGS line in the Makefile.

    make install PREFIX=$HOME/matbag_ws/install

### ROS
Install [rosinstall_generator](http://wiki.ros.org/rosinstall_generator#Installation) and [wstool](http://wiki.ros.org/wstool#Installation) for ROS.

Generate install file and download necessary packages

    rosinstall_generator --rosdistro hydro rosbag_storage tf2 --deps --wet-only --tar > matbag.rosinstall
    wstool init -j8 src matbag.rosinstall
    git clone -b hydro-devel https://github.com/bcharrow/matlab_rosbag.git src/matlab_rosbag
    rosdep install --from-paths src --ignore-src --rosdistro hydro -y

Next we need to compile.  Set ROSCONSOLE_SEVERITY_NONE, so we don't need to build log4cxx or any of its dependencies and use the downloaded version of boost and bz2 as opposed to the system's version.

    ./src/catkin/bin/catkin_make_isolated --merge --install --install-space install --cmake-args \
      -DBUILD_SHARED_LIBS=false \
      -DCMAKE_CXX_FLAGS=-fPIC \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DBoost_NO_SYSTEM_PATHS=ON \
      -DBOOST_ROOT=$(pwd)/install \
      -DBZIP2_INCLUDE_DIR=~/matbag_ws/install/include/ \
      -DBZIP2_LIBRARIES=~/matbag_ws/install/lib/libbz2a

### matlab_rosbag
Now use the <tt>mex_compile.sh</tt> build script

    cd ~/matbag_ws/src/matlab_rosbag/src
    bash mex_compile.sh

## Using ROS Groovy
You need to install each of these things in order.  First, make the workspace:

    mkdir ~/matbag_ws && cd ~/matbag_ws

### [Boost](http://www.boost.org/users/download/)
Download and unpack the latest version of Boost and go to that directory.

    ./bootstrap.sh
    ./b2 cxxflags='-fPIC' --build-dir=build --with-regex --with-system \
      --with-filesystem --with-program_options --with-date_time --with-thread \
      --with-signals link=static --prefix=$HOME/matbag_ws/install/ install

To compile things with multiple processors, add the flag <tt>-jNUM_PROCESSORS</tt>

### [bz2](http://www.bzip.org/downloads.html)

Add <tt>-fPIC</tt> to the CFLAGS line in the Makefile.

    make install PREFIX=$HOME/matbag_ws/install

### ROS
Install the [rosinstall_generator](http://wiki.ros.org/rosinstall_generator#Installation) package for ROS.

Generate install file and download necessary packages

    rosinstall_generator --rosdistro groovy --deps rosbag tf2 tf > base.rosinstall
    rosinstall --catkin -n src base.rosinstall
    rm src/CMakeLists.txt
    src/catkin/bin/catkin_init_workspace src

Modify src/CMakeLists.txt so that the lines after cmake_minimum_required() are:

    set(BUILD_SHARED_LIBS false)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
    add_definitions(-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_NONE)

    set(Boost_NO_SYSTEM_PATHS ON)
    set(BOOST_ROOT "${CMAKE_SOURCE_DIR}/../install")

By compiling out logging with ROSCONSOLE_SEVERITY_NONE, we don't need to build log4cxx or any of its dependencies.  Also, we're using the downloaded version of boost and not the system's version.  To build everything:

    src/catkin/bin/catkin_make install

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

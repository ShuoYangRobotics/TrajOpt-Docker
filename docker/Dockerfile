# we build this docker based on a docker with ros already installed
FROM ros:melodic-perception

LABEL maintainer="shuoyang@andrew.cmu.edu"
ENV REFRESH_AT 2020-10-26

# three folders for various files
# CATKIN_WS is the folder for ros pkgs
# SUPPORT_WS contains supporting software
# CPP_WS is the folder for non-ros pkgs, such as pure C++ projects
ENV CATKIN_WS=/root/catkin_ws
ENV CPP_WS=/root/cpp_ws
ENV SUPPORT_WS=/root/support_ws

# install necessary dependencies 
RUN   if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; \
      else export USE_PROC=$(($(nproc)/2)) ; fi && \
      apt-get update && apt-get install -y \
      cmake \
      vim \
      libatlas-base-dev \
      libeigen3-dev \
      libgoogle-glog-dev \
      libsuitesparse-dev \
      python-catkin-tools \
      gfortran \
      autoconf \
      coinor-libipopt-dev \
      libgfortran3 \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-message-filters \
      ros-${ROS_DISTRO}-grid-map \
      ros-${ROS_DISTRO}-tf-conversions \
      ros-${ROS_DISTRO}-tf \
      ros-${ROS_DISTRO}-orocos-kdl \
      ros-${ROS_DISTRO}-xpp \    
      ros-${ROS_DISTRO}-kdl-parser \ 
      ros-${ROS_DISTRO}-eigen-stl-containers \ 
      ros-${ROS_DISTRO}-graph-msgs \    
      ros-${ROS_DISTRO}-roslint \    
      ros-${ROS_DISTRO}-rviz-visual-tools \    
      ros-${ROS_DISTRO}-hebi-cpp-api \    
      ros-${ROS_DISTRO}-gazebo-msgs \  
      ros-${ROS_DISTRO}-depth-image-proc \  
      ros-${ROS_DISTRO}-gazebo-ros \ 
      ros-${ROS_DISTRO}-costmap-2d && \
      rm -rf /var/lib/apt/lists/* 

# set up code folders
RUN   mkdir -p $CATKIN_WS/src && \
      mkdir -p $SUPPORT_WS  && \ 
      mkdir -p $CPP_WS 


# install casadi we use version 3.5.1 as it is the lastest stable release
WORKDIR $CPP_WS
RUN   git clone https://github.com/casadi/casadi.git && \
      cd casadi/external_packages/superscs && \
      git clone https://github.com/jgillis/scs.git . && \
      cd ${CPP_WS}/casadi && \
      git checkout tags/3.5.1  &&\
      mkdir build && \
      cd build && \
      cmake -DWITH_COMMON=ON -DCASADI_INT_TYPE=int .. && \
      make -j8 && \
      make install 

# install gtsam we use version 4.0.3 as it is the lastest stable release    
WORKDIR $CPP_WS  
RUN     git clone https://github.com/borglab/gtsam.git && \
        cd gtsam && \
        git checkout tags/4.0.3 &&\
        mkdir build && \
        cd build && \
        cmake .. && \
        make -j8 && \
        make install 

# control toolbox depends on kindr
WORKDIR $CPP_WS
RUN   git clone https://github.com/ANYbotics/kindr.git && \
      cd kindr && \
      mkdir build && \
      cd build && \
      cmake ../ && \
      make -j8 && \
      make install 

# install control toolbox
WORKDIR $CPP_WS
RUN   git clone https://github.com/paulyang1990/control-toolbox.git && \      
      cd control-toolbox/ct && \
      ./build_ct.sh -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=false

# change timezone (this is very important otherwise many ROS topic time will be strange)
ENV TZ=America/New_York
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# build the ros workspace 
WORKDIR $CATKIN_WS
RUN   /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make'

RUN   echo "source "${CATKIN_WS}"/devel/setup.bash" >> ~/.bashrc

RUN   ldconfig
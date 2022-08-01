FROM ubuntu:20.04

# General container setup

ENV DEBIAN_FRONTEND=noninteractive



RUN apt-get update && apt-get install -y \
    apt-utils

RUN apt-get update && apt-get install -y \
    software-properties-common apt-transport-https sudo \
    psmisc tmux nano wget curl telnet gnupg gdb git gitk autoconf locales gdebi \
    terminator meld \
    python3 python3-dev python3-pip python3-setuptools

RUN locale-gen en_US.UTF-8

# Compact nvidia setup
ENV LOCAL_USER_ID=${LOCAL_USER_ID}
ENV NVIDIA_ENV=${NVIDIA_ENV}
RUN apt-get update && apt-get install -y --no-install-recommends \
        libxau6 libxdmcp6 libxcb1 libxext6 libx11-6 && \
    export NVIDIA_VISIBLE_DEVICES=all && \
    export NVIDIA_DRIVER_CAPABILITIES=graphics,utility && \
    /bin/sh -c echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf && \
    export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu:/usr/local/nvidia/lib:/usr/local/nvidia/lib64 && \
    apt-get update && apt-get install -y --no-install-recommends  \
        libglvnd0 libgl1 libglx0 libegl1 libgles2 && \
    apt-get update && apt-get install -y --no-install-recommends  \
        pkg-config libglvnd-dev libgl1-mesa-dev libegl1-mesa-dev libgles2-mesa-dev xterm

# Robotology build

# 
ARG ROBOTOLOGY_SUPERBUILD_RELEASE=v2022.05.1
ARG BUILD_TYPE=Release
# WARNING: building into /usr will NOT work and will result in some enigmatic errors
ARG ROBOTOLOGY_SUPERBUILD_INSTALL_DIR=/usr/local

# # Set up git (required by superbuild)
RUN git config --global user.name "GitHub Actions" && \
    git config --global user.email "actions@github.com"

# Install dependencies
RUN git clone https://github.com/robotology/robotology-superbuild.git --depth 1 --branch ${ROBOTOLOGY_SUPERBUILD_RELEASE} && \
    robotology-superbuild/scripts/install_apt_dependencies.sh


RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    apt-get update && \
    apt-get install -y libcgal-dev gazebo11 libgazebo11-dev

# Build robotology-superbuild
RUN cd robotology-superbuild && mkdir build && cd build && \
    cmake .. \
          -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
          -DYCM_EP_INSTALL_DIR=${ROBOTOLOGY_SUPERBUILD_INSTALL_DIR} \
          -DROBOTOLOGY_ENABLE_CORE:BOOL=ON \
          -DROBOTOLOGY_USES_GAZEBO:BOOL=ON && \
    make -j $(expr $(nproc) + 1 )
    # && \
    #cd ../.. && rm -Rf robotology-superbuild

# Clean up git configuration
RUN git config --global --unset-all user.name && \
    git config --global --unset-all user.email

RUN apt-get update && apt-get install -y mc

# caffeCoder requirements
# RUN apt-get update && apt-get install -y libcaffe-cpu-dev caffe-cpu

# stereo-vision requirements
RUN apt-get update && apt-get install -y nvidia-cuda-toolkit
# we will need nonfree opencv packages SIFT/SURF

# ----- opencv start ------

RUN apt-get remove -y opencv* libopencv*

RUN apt-get install -y libfreetype-dev libharfbuzz-dev \
                   libvtk7-dev libgtk-3-dev libgtkglext1-dev \
                   python3 python3-dev python3-numpy \
                   libavcodec-dev libavformat-dev libswscale-dev \
                   libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev \
                   libpng-dev libjpeg-dev libopenexr-dev libtiff-dev libwebp-dev \
                   libeigen3-dev libgflags-dev libgoogle-glog-dev libatlas-base-dev libsuitesparse-dev

RUN rm /usr/bin/gcc && \
    rm /usr/bin/g++ && \
    rm /usr/bin/cc && \
    rm /usr/bin/c++ && \
    ln -s /usr/bin/gcc-8 /usr/bin/gcc && \
    ln -s /usr/bin/gcc-8 /usr/bin/cc && \
    ln -s /usr/bin/g++-8 /usr/bin/g++ && \
    ln -s /usr/bin/g++-8 /usr/bin/c++


RUN mkdir -p /opencv && cd /opencv && \
    wget -O opencv.zip https://github.com/opencv/opencv/archive/4.2.0.zip && \
    wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.2.0.zip && \
    unzip opencv.zip && \
    unzip opencv_contrib.zip

RUN cd /opencv/opencv-4.2.0/ && \
    mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
        -D PYTHON_EXECUTABLE=/usr/bin/python3 \
        -D CMAKE_INSTALL_PREFIX=/usr \
        -D WITH_CUDA=ON \
        -D WITH_CUDNN=ON \
        -D WITH_CUBLAS=ON \
        -D WITH_TBB=ON \
        -D OPENCV_DNN_CUDA=ON \
        -D OPENCV_ENABLE_NONFREE=ON \
        -D WITH_FREETYPE=ON \
        -D WITH_VTK=ON \
        -D CUDA_ARCH_BIN=6.1 \
        -D OPENCV_EXTRA_MODULES_PATH=/opencv/opencv_contrib-4.2.0/modules \
        -D BUILD_EXAMPLES=OFF \
        -D HAVE_opencv_python3=ON .. && \
    make -j $(expr $(nproc) + 1 ) && \
    make install


# ----- opencv end ------

# for show3D from tool-incorporation
RUN apt-get update && apt-get install -y pcl-tools libpcl-dev

# install attmod/stereo-vision
RUN mkdir -p /attmod && cd /attmod && \
    git clone https://github.com/attmod/stereo-vision && \
    cd stereo-vision && mkdir -p build && cd build && \
    cmake ../ && \
    make -j $(expr $(nproc) + 1 ) && \
    make install

# install attmod/stereo-vision
RUN mkdir -p /attmod && cd /attmod && \
    git clone https://github.com/attmod/find-superquadric && \
    cd find-superquadric && mkdir -p build && cd build && \
    cmake ../ && \
    make -j $(expr $(nproc) + 1 ) && \
    make install

# install attmod/stereo-vision
RUN mkdir -p /attmod && cd /attmod && \
    git clone https://github.com/attmod/segmentation && \
    cd segmentation && mkdir -p build && cd build && \
    cmake ../ && \
    make -j $(expr $(nproc) + 1 ) && \
    make install

# Launch bash from /workdir
WORKDIR /workdir
CMD ["bash"]

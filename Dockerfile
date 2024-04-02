ARG UBUNTU_VERSION=jammy

FROM ubuntu:$UBUNTU_VERSION


ARG YARP_VERSION=3.7.2
ARG YCM_VERSION=0.15.0
ARG ED_VERSION=master
ARG SOURCE_FOLDER=/usr/local/src
ARG OPENGL=0

ENV DEBIAN_FRONTEND noninteractive 

RUN apt update

RUN apt install -y \
    apt-transport-https \
    ca-certificates \
    gnupg \
    software-properties-common \
    wget \
    lsb-core

RUN distro=`lsb_release -sr`; \
    comp=$(awk 'BEGIN{ print "'$distro'"<"'19.04'" }'); \
    if [ "$comp" -eq 1 ]; then \
        echo "WARNING: Getting newer CMake version from the kitware repository" ; \
        wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | apt-key add - ;\
        apt-add-repository "deb https://apt.kitware.com/ubuntu/ `lsb_release -cs` main";\
        apt update ;\
    fi

# Install useful packages
RUN apt install -y \
        build-essential \
        git \
        cmake \
        cmake-curses-gui \
        libssl-dev \
        iputils-ping \
        iproute2

#Install opencv
RUN apt install libopencv-dev -y

# Install yarp dependencies
RUN apt install -y \
    ca-certificates \
    build-essential \
    git \
    cmake \
    cmake-curses-gui \
    libace-dev \
    libassimp-dev \
    libglew-dev \
    libglfw3-dev \
    libglm-dev \
    libeigen3-dev

RUN export Qt5_DIR=/path/to/Qt/5.3/gcc_64/lib/cmake/Qt5/

RUN export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/path/to/Qt/5.3/gcc_64/

RUN apt update

# Install QT5 for GUIS 
# (NOTE: may be not compatible with nvidia drivers when forwarding screen)
RUN apt install -y \
        qtbase5-dev \
        qtdeclarative5-dev \
        qtmultimedia5-dev \
        qml-module-qtquick2 \
        qml-module-qtquick-window2 \
        qml-module-qtmultimedia \
        qml-module-qtquick-dialogs \
        qml-module-qtquick-controls \
# Install swig for python bindings
        swig -y \
# Configure virtual environment for python
        python3-pip -y \
        && apt-get autoremove \
        && apt-get clean \
        && rm -rf /tmp/* /var/lib/apt/lists/* /var/tmp/*

RUN pip3 install \
    virtualenv \
    virtualenvwrapper

# RUN ln -s /usr/bin/python3 /usr/bin/python && 
ENV VIRTUALENVWRAPPER_PYTHON /usr/bin/python3
RUN echo 'source /usr/local/bin/virtualenvwrapper.sh' | cat - /root/.bashrc > temp && mv temp /root/.bashrc

RUN cd $SOURCE_FOLDER && \
    git clone https://github.com/robotology/ycm.git && \
    cd ycm && \
    git checkout v$YCM_VERSION && \
    mkdir build && cd build && \
    cmake .. && \
    make -j `nproc` install

# Install YARP with GUIS and Python bindings
RUN cd $SOURCE_FOLDER && \
    git clone https://github.com/robotology/yarp.git &&\
    cd yarp &&\
    git checkout v$YARP_VERSION &&\
    mkdir build && cd build &&\
    cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
          -DYARP_COMPILE_BINDINGS=ON \
          -DCREATE_PYTHON=ON \
          -DYARP_USE_PYTHON_VERSION=3 \
          .. &&\
    make -j `nproc` install

RUN PY_VER=`python3 --version | awk '{print $2}' | awk -F "." 'BEGIN { OFS = "." }{print $1,$2}'` && \
    ln -s /usr/local/lib/python3/dist-packages/*yarp* /usr/local/lib/python$PY_VER/dist-packages/

RUN yarp check
EXPOSE 10000/tcp 10000/udp

# Some QT-Apps don't show controls without this
ENV QT_X11_NO_MITSHM 1

ARG ED_VERSION=master
RUN cd $SOURCE_FOLDER &&\
    git clone --depth 1 --branch $ED_VERSION https://github.com/robotology/event-driven.git &&\
    cd event-driven &&\
    mkdir build && cd build &&\
    cmake .. &&\
    make -j `nproc` install
    
RUN cd $SOURCE_FOLDER &&\
    git clone https://github.com/event-driven-robotics/four-dof-affine-tracking.git

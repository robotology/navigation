FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu18.04
LABEL maintainer="marco.randazzo@iit.it"

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive

# Update apt database
RUN apt update

# Install essentials
RUN apt install -y apt-utils software-properties-common sudo psmisc lsb-release \
                   tmux nano wget curl build-essential git gitk cmake cmake-curses-gui \
                   libedit-dev libxml2-dev autoconf

# Install graphics
RUN apt install -y xfce4 xfce4-goodies xserver-xorg-video-dummy xserver-xorg-legacy firefox
RUN sed -i 's/allowed_users=console/allowed_users=anybody/' /etc/X11/Xwrapper.config
COPY xorg.conf /etc/X11/xorg.conf

# We need the most recent x11vnc
RUN sed -Ei 's/^# deb-src /deb-src /' /etc/apt/sources.list && \
    apt update
RUN git clone https://github.com/LibVNC/x11vnc.git /opt/x11vnc && \
    cd /opt/x11vnc && \
    apt build-dep -y x11vnc && \
    autoreconf -fiv && \
    ./configure && \
    make && make install && \
    rm /opt/x11vnc -Rf

# Install Java
RUN apt install -y default-jdk

# Install Octave
RUN apt install -y octave epstool transfig

# Install markserv
RUN apt install -y nodejs npm
RUN npm install --global markserv

# Install jupyter
RUN apt install -y python python-dev python-pip ipython python-setuptools python3-setuptools
RUN pip install jupyter

# Install terminator
RUN apt install -y terminator

# Install websockify
RUN git clone https://github.com/novnc/websockify.git /opt/websockify && \
    cd /opt/websockify && \
    python3 setup.py install

# Clone noVNC
RUN git clone https://github.com/novnc/noVNC.git /opt/novnc
RUN echo "<html><head><meta http-equiv=\"Refresh\" content=\"0; url=vnc.html\"></head></html>" > /opt/novnc/index.html

# Install yarp dependencies
RUN echo "deb http://www.icub.org/ubuntu `lsb_release -cs` contrib/science" > /etc/apt/sources.list.d/icub.list
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 57A5ACB6110576A6
RUN apt update
RUN apt install -y icub-common

# Install Qt patch for Xfce
RUN apt install -y qml-module-qt-labs-folderlistmodel qml-module-qt-labs-settings

# other utilites
RUN apt-get install -y lsof iftop iputils-ping gdb

# Select branch + build type
ARG BRANCH=devel
ARG BUILD_TYPE=Release

# Build ycm
RUN git clone https://github.com/robotology/ycm.git -b $BRANCH
RUN cd ycm && mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE && \
    make install
RUN rm -Rf ycm

# Build robot-testing-framework
RUN git clone https://github.com/robotology/robot-testing-framework.git -b $BRANCH
RUN cd robot-testing-framework && mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE && \
    make install
RUN rm -Rf robot-testing-framework

# Build yarp
RUN git clone https://github.com/robotology/yarp.git -b $BRANCH
RUN cd yarp && mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    -DYARP_COMPILE_libYARP_math=ON \
    -DYARP_COMPILE_GUIS=ON \
    -DYARP_COMPILE_DEVICE_PLUGINS=ON && \
    make install
RUN cp yarp/scripts/yarp_completion /etc/bash_completion.d/.
#RUN rm -Rf yarp

# Build icub-main
RUN git clone https://github.com/robotology/icub-main.git -b $BRANCH
RUN cd icub-main && mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    -DENABLE_icubmod_cartesiancontrollerserver=ON \
    -DENABLE_icubmod_cartesiancontrollerclient=ON \
    -DENABLE_icubmod_gazecontrollerclient=ON && \
    make install
#RUN rm -Rf icub-main

# ROS installation
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt update
RUN apt install -y ros-melodic-desktop-full
RUN rosdep init

# create gitpod user
RUN useradd -l -u 33333 -G sudo -md /home/gitpod -s /bin/bash -p gitpod gitpod && \
    # passwordless sudo for users in the 'sudo' group
    sed -i.bkp -e 's/%sudo\s\+ALL=(ALL\(:ALL\)\?)\s\+ALL/%sudo ALL=NOPASSWD:ALL/g' /etc/sudoers
USER gitpod

# ROS installation (part2)
RUN rosdep update
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source ~/.bashrc"
USER root

# Build navigation
WORKDIR /home/gitpod
RUN git clone https://github.com/robotology/navigation.git -b $BRANCH
RUN cd /home/gitpod/navigation && mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE && \
    make -j4
ENV navigation_DIR=/home/gitpod/navigation

# gazebo plugins stuff
WORKDIR /home/gitpod
RUN git clone https://github.com/robotology/gazebo-yarp-plugins.git -b $BRANCH
RUN cd /home/gitpod/gazebo-yarp-plugins && mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE && \
    make install

# CER stuff
WORKDIR /home/gitpod
RUN git clone https://github.com/robotology/cer.git -b $BRANCH
RUN cd /home/gitpod/cer && mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    -DENABLE_cermod_cerDoubleLidar=ON \
    -DENABLE_cermod_tripodMotionControl=ON \
    -DGAZEBO_TRIPOD_PLUGIN=OFF && \ 
    make install
#IPOPT ISSUE WITH DGAZEBO_TRIPOD_PLUGIN
ENV CER_DIR=/home/gitpod/cer

# CER-sim stuff
WORKDIR /home/gitpod
RUN mkdir cer-sim
RUN git clone https://github.com/robotology/cer-sim.git -b master

USER gitpod

# Build icub-contrib-common
#WORKDIR /home/gitpod
#RUN mkdir iCubContrib
#RUN git clone https://github.com/robotology/icub-contrib-common.git
#RUN cd cd /home/gitpod/icub-contrib-common && mkdir build && cd build && \
#    cmake .. \
#    -DCMAKE_INSTALL_PREFIX=/home/gitpod/iCubContrib && \
#    make install
#RUN rm -Rf icub-contrib-common
#ENV ICUBcontrib_DIR=/home/gitpod/iCubContrib

ARG GITHUB_TOKEN

## Build audition-projects-helpers
# RUN git config --global url."https://$GITHUB_TOKEN:@github.com/".insteadOf "https://github.com/"
# RUN git clone https://github.com/dic-iit/audition-projects-helpers.git
# RUN git config --global --remove-section url."https://$GITHUB_TOKEN:@github.com/"
# RUN cd audition-projects-helpers && mkdir build && cd build && \
#   cmake .. \
#    -DCMAKE_BUILD_TYPE=$BUILD_TYPE && \
#    make install
# RUN rm -Rf audition-projects-helpers

# Set up .bashrc
RUN echo "PS1='\[\e]0;\u \w\a\]\[\033[01;32m\]\u\[\033[00m\] \[\033[01;34m\]\w\[\033[00m\] \$ '" >> ~/.bashrc
RUN echo "export PATH=$PATH:/home/gitpod/iCubContrib/bin" >> ~/.bashrc
ENV GAZEBO_PLUGIN_PATH=/usr/local/lib/
ENV GAZEBO_MODEL_PATH=/home/gitpod/cer-sim/gazebo/

# Switch back to root
USER root
RUN apt-get install -y geany meld ros-melodic-cartographer locate mercurial ros-melodic-teleop-tools ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard

# Set environmental variables
ENV DISPLAY=:1
ENV YARP_DATA_DIRS=/usr/local/share/yarp:/usr/local/share/iCub:/home/gitpod/iCubContrib/share/ICUBcontrib
ENV LD_LIBRARY_PATH=/usr/local/lib/yarp:/usr/local/lib/robottestingframework
ENV GAZEBO_PLUGIN_PATH=/usr/local/lib/
ENV GAZEBO_MODEL_PATH=/home/gitpod/cer-sim/gazebo/

# Set up script to launch markserv
COPY start-markserv.sh /usr/bin/start-markserv.sh
RUN chmod +x /usr/bin/start-markserv.sh

# Set up script to launch jupyter
COPY start-jupyter.sh /usr/bin/start-jupyter.sh
RUN chmod +x /usr/bin/start-jupyter.sh

# Set up script to launch graphics and vnc
COPY start-vnc-session.sh /usr/bin/start-vnc-session.sh
RUN chmod +x /usr/bin/start-vnc-session.sh

# Manage markserv port
EXPOSE 8080

# Manage jupyter port
EXPOSE 8888

# Manage x11vnc port
EXPOSE 5901

# Manage noVNC port
EXPOSE 6080

# Manage yarp port
EXPOSE 10000/tcp 10000/udp

#gazebo models
RUN mkdir -p ~/.gazebo
RUN hg clone https://bitbucket.org/osrf/gazebo_models ~/.gazebo/models
RUN echo "export PATH=$PATH:/home/gitpod/navigation/build/bin" >> ~/.bashrc
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

# additional
RUN apt update
RUN apt install -y synaptic ros-melodic-nav-core ros-melodic-map-server ros-melodic-gmapping ros-melodic-amcl ros-melodic-rtabmap ros-melodic-mapviz ros-melodic-moveit
RUN apt install -y net-tools 

#temp fix/updates
ARG FIX_NUMBER=1
WORKDIR /home/gitpod/cer-sim
RUN git pull
WORKDIR /root

# locate stuff
RUN updatedb

# Launch bash from /workspace
WORKDIR /workspace
COPY startup_commands.sh /workspace
RUN chmod +x startup_commands.sh
COPY stop_commands.sh /workspace
RUN chmod +x stop_commands.sh
COPY r1world.world /workspace

CMD ["bash"]

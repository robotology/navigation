FROM nvidia/cuda:10.1-cudnn7-devel
LABEL maintainer="marco.randazzo@iit.it"

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive

# Update apt database
RUN apt update

# Install essentials
RUN apt install -y apt-utils software-properties-common sudo psmisc lsb-release \
                   tmux nano wget curl build-essential git gitk cmake cmake-curses-gui \
                   libedit-dev libxml2-dev autoconf geany meld synaptic net-tools 

# Install graphics
RUN apt update
RUN apt install -y xfce4 xfce4-goodies xserver-xorg-video-dummy xserver-xorg-legacy
RUN apt update
RUN apt install -y firefox

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

# install cmake 3.12 
RUN sudo apt-get update
RUN sudo apt-get install -y apt-transport-https ca-certificates gnupg software-properties-common wget
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | sudo apt-key add -
RUN sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main'
RUN sudo apt-get update
RUN sudo apt-get install -y kitware-archive-keyring
RUN sudo apt-key --keyring /etc/apt/trusted.gpg del C1F34CDD40CD72DA
RUN sudo apt-get install -y cmake cmake-curses-gui

# ROS installation
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt update
RUN apt install -y ros-melodic-desktop-full
RUN apt install -y ros-melodic-cartographer locate mercurial ros-melodic-teleop-tools ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard
RUN apt install -y ros-melodic-nav-core ros-melodic-map-server ros-melodic-gmapping ros-melodic-amcl ros-melodic-rtabmap ros-melodic-mapviz ros-melodic-moveit
RUN rosdep init

# create gitpod user
user root
RUN useradd -l -u 33333 -G sudo -md /home/gitpod -s /bin/bash -p gitpod gitpod && \
    # passwordless sudo for users in the 'sudo' group
    sed -i.bkp -e 's/%sudo\s\+ALL=(ALL\(:ALL\)\?)\s\+ALL/%sudo ALL=NOPASSWD:ALL/g' /etc/sudoers

# create another user called user1
USER root 
RUN useradd -l -u 33334 -G sudo -md /home/user1 -s /bin/bash -p user1 user1 && \
    # passwordless sudo for users in the 'sudo' group
    sed -i.bkp -e 's/%sudo\s\+ALL=(ALL\(:ALL\)\?)\s\+ALL/%sudo ALL=NOPASSWD:ALL/g' /etc/sudoers

# ROS installation (part2)
USER gitpod
RUN rosdep update
RUN echo "source /opt/ros/melodic/setup.bash" >> /home/gitpod/.bashrc
RUN /bin/bash -c "source /home/gitpod/.bashrc"
USER user1
RUN sudo rosdep update
RUN echo "source /opt/ros/melodic/setup.bash" >> /home/user1/.bashrc
RUN /bin/bash -c "source /home/user1/.bashrc"

# set install_folder
USER user1
ENV robotology_install_folder /home/user1
ENV user1_home /home/user1
ENV robotology_install_user user1

#gazebo models
RUN mkdir -p $user1_home/.gazebo
RUN hg clone https://bitbucket.org/osrf/gazebo_models $user1_home/.gazebo/models
RUN echo "export PATH=$PATH:$robotology_install_folder/navigation/build/bin" >> $user1_home/.bashrc
RUN echo "source /opt/ros/melodic/setup.bash" >> $user1_home/.bashrc

# Build ycm
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/ycm.git -b $BRANCH
RUN cd ycm && mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE && \
    sudo make install

# Build robot-testing-framework
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/robot-testing-framework.git -b $BRANCH
RUN cd robot-testing-framework && mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE && \
    sudo make install

# Build yarp
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/yarp.git -b $BRANCH
RUN cd yarp && mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    -DYARP_COMPILE_libYARP_math=ON \
    -DYARP_COMPILE_GUIS=ON \
    -DYARP_COMPILE_DEVICE_PLUGINS=ON && \
    sudo make install
RUN sudo cp yarp/scripts/yarp_completion /etc/bash_completion.d/.

# Build icub-main
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/icub-main.git -b $BRANCH
RUN cd icub-main && mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    -DENABLE_icubmod_cartesiancontrollerserver=ON \
    -DENABLE_icubmod_cartesiancontrollerclient=ON \
    -DENABLE_icubmod_gazecontrollerclient=ON && \
    sudo make install

# Build navigation
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/navigation.git -b $BRANCH
RUN cd navigation && mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE && \
    sudo make -j4
ENV navigation_ROOT=$robotology_install_folder/navigation
ENV navigation_DIR=$robotology_install_folder/navigation/build

# gazebo plugins stuff
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/gazebo-yarp-plugins.git -b $BRANCH
RUN cd gazebo-yarp-plugins && mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE && \
    sudo make install

# fix for ipopt issue
USER $robotology_install_user
COPY simbody-ubuntu-1804-ipopt-fix-dropin.zip $robotology_install_folder
WORKDIR $robotology_install_folder
RUN unzip simbody-ubuntu-1804-ipopt-fix-dropin.zip
RUN sudo dpkg -i libsimbody3.5v5_3.5.4+dfsg2-1_amd64.deb
RUN sudo dpkg -i libsimbody-dev_3.5.4+dfsg2-1_amd64.deb
RUN rm libsimbody3.5v5_3.5.4+dfsg2-1_amd64.deb
RUN rm libsimbody-dev_3.5.4+dfsg2-1_amd64.deb
RUN rm simbody-ubuntu-1804-ipopt-fix-dropin.zip

# CER stuff
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/cer.git -b $BRANCH
RUN cd cer && mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    -DENABLE_cermod_cerDoubleLidar=ON \
    -DENABLE_cermod_tripodMotionControl=ON \
    -DGAZEBO_TRIPOD_PLUGIN=ON && \ 
    sudo make install
ENV CER_ROOT=$robotology_install_folder/cer
ENV CER_DIR=$robotology_install_folder/cer/build

# CER-sim stuff
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/cer-sim.git -b master

# Build icub-contrib-common
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/icub-contrib-common.git
RUN cd icub-contrib-common && mkdir build && cd build && cmake .. \
    -DCMAKE_INSTALL_PREFIX=$robotology_install_folder/iCubContrib && \
    make install
ENV ICUBcontrib_DIR=$robotology_install_folder/iCubContrib

# Build idyntree
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/idyntree -b $BRANCH
RUN cd idyntree && mkdir build && cd build && cmake .. && \
    sudo make install
    
USER $robotology_install_user
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
RUN echo "PS1='\[\e]0;\u \w\a\]\[\033[01;32m\]\u\[\033[00m\] \[\033[01;34m\]\w\[\033[00m\] \$ '" >> /home/$robotology_install_user/.bashrc
RUN echo "export PATH=\$PATH:\$robotology_install_folder/iCubContrib/bin" >> /home/$robotology_install_user/.bashrc
RUN echo "export PATH=\$PATH:\$navigation_DIR/bin" >> /home/$robotology_install_user/.bashrc
ENV GAZEBO_PLUGIN_PATH=/usr/local/lib/
ENV GAZEBO_MODEL_PATH=$robotology_install_folder/cer-sim/gazebo/

# Set environmental variables
ENV DISPLAY=:1
ENV YARP_DATA_DIRS=/usr/local/share/yarp:/usr/local/share/iCub:$robotology_install_folder/iCubContrib/share/ICUBcontrib
ENV LD_LIBRARY_PATH=/usr/local/lib/yarp:/usr/local/lib/robottestingframework
ENV GAZEBO_PLUGIN_PATH=/usr/local/lib/
ENV GAZEBO_MODEL_PATH=$robotology_install_folder/cer-sim/gazebo/

# Set up script to launch markserv
COPY --chown=$robotology_install_user:$robotology_install_user start-markserv.sh /usr/bin/start-markserv.sh
RUN chmod +x /usr/bin/start-markserv.sh

# Set up script to launch jupyter
COPY --chown=$robotology_install_user:$robotology_install_user start-jupyter.sh /usr/bin/start-jupyter.sh
RUN chmod +x /usr/bin/start-jupyter.sh

# Set up script to launch graphics and vnc
COPY --chown=$robotology_install_user:$robotology_install_user start-vnc-session.sh /usr/bin/start-vnc-session.sh
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

# locate stuff
RUN sudo updatedb

# Launch bash from /workspace
USER $robotology_install_user
WORKDIR /workspace
COPY --chown=$robotology_install_user:$robotology_install_user startup_commands.sh /workspace
COPY --chown=$robotology_install_user:$robotology_install_user startup_commands2.sh /workspace
RUN chmod +x startup_commands.sh
COPY --chown=$robotology_install_user:$robotology_install_user stop_commands.sh /workspace
RUN chmod +x stop_commands.sh
COPY --chown=$robotology_install_user:$robotology_install_user r1world.world /workspace
COPY --chown=$robotology_install_user:$robotology_install_user r1world_2.world /workspace

RUN sudo chown -R $robotology_install_user:$robotology_install_user /opt/ros 
RUN sudo chown -R $robotology_install_user:$robotology_install_user /home/$robotology_install_user/.ros

CMD ["bash"]

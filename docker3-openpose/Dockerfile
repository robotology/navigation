FROM randaz81/r1slam:cuda
LABEL maintainer="marco.randazzo@iit.it"

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive

# Update apt database
RUN sudo apt update

ENV robotology_install_folder /home/user1
ENV user1_home /home/user1
ENV robotology_install_user user1
USER $robotology_install_user

# openpose
WORKDIR $robotology_install_folder
RUN git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose.git
WORKDIR $robotology_install_folder/openpose/scripts/ubuntu
RUN bash ./install_deps.sh
WORKDIR $robotology_install_folder/openpose
RUN git checkout 1e4a7853
RUN pwd
RUN mkdir build && cd build && cmake .. && make -j4
ENV openpose_ROOT=$robotology_install_folder/openpose

# Build yarpopenpose
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/human-sensing.git -b master
WORKDIR $robotology_install_folder/human-sensing/yarpOpenPose
RUN mkdir build && cd build && cmake .. && make install

# Add VTK
RUN sudo apt-get install -y libvtk6-dev

# Build skeleton retriever
WORKDIR $robotology_install_folder
RUN git clone https://github.com/randaz81/assistive-rehab.git -b master
WORKDIR $robotology_install_folder/assistive-rehab
RUN mkdir build && cd build && cmake .. && make skeletonRetriever

COPY yarpOpenPose.ini /workspace
WORKDIR $user1_home/openpose/models
RUN  ./getModels.sh
WORKDIR /workspace

#CMD ["bash"]

FROM tiryoh/ros-desktop-vnc:noetic

ENV ROS_WORKSPACE=/home/ubuntu/catkin_ws

RUN apt update -q && \
    apt upgrade -y && \
    apt install -y dos2unix


COPY ./ ${ROS_WORKSPACE}/src/OpenHRC/
# COPY ./src/raspimouse_sim $ROS_WORKSPACE/src/raspimouse_sim
# COPY ./src/raspimouse_description $ROS_WORKSPACE/src/raspimouse_description
COPY ./configure.sh /home/ubuntu/configure.sh

RUN mkdir -p ${ROS_WORKSPACE} && \
    cd ${ROS_WORKSPACE} && \
    /bin/bash -c "find $ROS_WORKSPACE -type f -print0 | xargs -0 dos2unix" && \
    dos2unix $HOME/configure.sh && \
    chown -R ubuntu:ubuntu /home/ubuntu

USER ubuntu
RUN . /opt/ros/noetic/setup.sh && \
    # cd ${ROS_WORKSPACE} && \        
    # mkdir -p src && \
    # cd src && \
    # git clone https://github.com/itadera/OpenHRC.git && \
    cd ${ROS_WORKSPACE}/src/OpenHRC && \
    # git submodule update --init --recursive && \
    rosdep update && \
    rosdep install -i -y --from-paths ./  && \
    cd ${ROS_WORKSPACE}  && \
    catkin build -DCMAKE_BUILD_TYPE=Release && \
    echo ". \$HOME/catkin_ws/devel/setup.bash" >> /home/ubuntu/.bashrc && \
    echo ". \$HOME/configure.sh" >> /home/ubuntu/.bashrc

USER root
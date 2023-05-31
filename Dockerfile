FROM tiryoh/ros-desktop-vnc:noetic

ENV ROS_WORKSPACE=/home/ubuntu/catkin_ws

RUN apt update -q && \
    apt upgrade -y

COPY ./ ${ROS_WORKSPACE}/src/OpenHRC/

RUN mkdir -p ${ROS_WORKSPACE} && \
    cd ${ROS_WORKSPACE} && \
    chown -R ubuntu:ubuntu /home/ubuntu

USER ubuntu
RUN . /opt/ros/noetic/setup.sh && \
    cd ${ROS_WORKSPACE}/src/OpenHRC && \
    git submodule update --init --recursive && \
    rosdep update && \
    rosdep install -i -y --from-paths ./  && \
    cd ${ROS_WORKSPACE}  && \
    catkin build -DCMAKE_BUILD_TYPE=Release && \
    echo "source \$HOME/catkin_ws/devel/setup.bash" >> /home/ubuntu/.bashrc

USER root
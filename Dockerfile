FROM tiryoh/ros-desktop-vnc:noetic

ENV ROS_WORKSPACE=/home/ubuntu/catkin_ws

RUN apt update -q && \
    apt upgrade -y && \
    apt install -y libx11-xcb-dev libxcb-dri2-0-dev libxcb-glx0-dev libxcb-icccm4-dev libxcb-keysyms1-dev libxcb-randr0-dev

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
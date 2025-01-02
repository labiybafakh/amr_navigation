FROM osrf/ros:noetic-desktop

SHELL ["/bin/bash", "-c"]

ARG USER

ENV ROS_DISTRO noetic
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-catkin-tools \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /home/$USER && chmod a+rw /home/$USER
RUN mkdir -p /home/$USER/ros_ws && chmod a+rw /home/$USER/ros_ws

COPY amr_navigation /home/$USER/ros_ws/src

WORKDIR /home/$USER/ros_ws

# RUN cd src && /
#     git clone https://github.com/labiybafakh/amr_navigation

RUN sudo -E apt-get update && rosdep install --from-paths src --ignore-src --rosdistro humble -r -y

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin init && \
    # rosdep init && \
    # rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin build

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc





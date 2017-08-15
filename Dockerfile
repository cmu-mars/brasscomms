FROM cmu-mars/base

# install Gazebo
RUN sudo apt-get install -y ros-kinetic-gazebo-ros-pkgs \
                            ros-kinetic-gazebo-ros-control \
                            ros-kinetic-kobuki-gazebo

# add the source code for the shared "notifications" module
RUN git clone https://github.com/cmu-mars/notifications-p15 \
      --depth 1 \
      src/notifications

# install the navigation_msgs
ENV ROS_NAVIGATION_MSGS_VERSION 1.13.0
RUN wget -q "https://github.com/ros-planning/navigation_msgs/archive/${ROS_NAVIGATION_MSGS_VERSION}.tar.gz" && \
    tar -xvf "${ROS_NAVIGATION_MSGS_VERSION}.tar.gz" && \
    rm "${ROS_NAVIGATION_MSGS_VERSION}.tar.gz" && \
    mv "navigation_msgs-${ROS_NAVIGATION_MSGS_VERSION}" navigation_msgs && \
    rm navigation_msgs/README.md && \
    mv navigation_msgs/* src && \
    rm -rf navigation_msgs

# TODO: install gazebo_msgs

# TODO: install ig_action.msgs
ENV IG_ACTION_SERVER_REVISION b1f70a8
RUN git clone https://github.com/ChrisTimperley/ig-action-server-p15 \
              src/ig-action-server && \
    cd src/ig-action-server && \
    git checkout "${IG_ACTION_SERVER_REVISION}"

ADD . src/brasscomms

RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    catkin_make install

RUN sudo apt-get install -y python-requests \
                            python-flask

# CMD ["python src/brasscomms/src/brasscomms.py"]

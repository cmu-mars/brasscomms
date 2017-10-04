FROM cmu-mars/base

# install Gazebo
RUN sudo apt-get install -y ros-kinetic-gazebo-ros-pkgs \
                            ros-kinetic-gazebo-ros-control \
                            ros-kinetic-kobuki-gazebo

# add the source code for the shared "notifications" module
RUN git clone https://github.com/cmu-mars/notifications-p15 \
      --depth 1 \
      src/notifications

# install shared "models" module
ENV MARS_MODELS_REVISION 5dd3542
RUN git clone https://github.com/cmu-mars/cp-models-p15 \
              src/cp-models-p15 && \
    cd src/cp-models-p15 && \
    git checkout "${MARS_MODELS_REVISION}"

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
RUN git clone https://github.com/cmu-mars/ig-action-server-p15 \
              src/ig-action-server && \
    cd src/ig-action-server && \
    git checkout "${IG_ACTION_SERVER_REVISION}"

ADD . src/brasscomms

RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    catkin_make install

RUN sudo apt-get install -y python-requests \
                            python-flask

# CMD ["python src/brasscomms/src/brasscomms.py"]
ENV CP_GAZEBO_REVISION 421fade
RUN git clone https://github.com/cmu-mars/cp-gazebo-p15 \
              src/cp-gazebo && \
    cd src/cp-gazebo && \
    git checkout "${CP_GAZEBO_REVISION}"

# TODO: tidy this up; use volume sharing!
# TODO: add config file (from LLStaging/installation/mockup.sh)
# create a log and data file
RUN sudo mkdir /test && \
    sudo touch /test/log && \
    sudo mkdir /test/data
ADD ex_config.json /test/data/ex_config.json
RUN sudo chown -R $(whoami):$(whoami) /test

RUN rosdep update

FROM ros:humble

RUN apt-get update
RUN apt-get upgrade -y
RUN apt -y install python3-pip

# Install python packages
RUN python3 -m pip install bosdyn-client==3.2.1 bosdyn-mission==3.2.1 bosdyn-choreography-client==3.2.1
RUN python3 -m pip install setuptools==58.2.0

# Install deps
RUN apt -y install ros-humble-velodyne
RUN apt -y install ros-humble-pcl-conversions
RUN apt -y install libopencv-dev
RUN apt -y install wget

# Install GTSAM library
WORKDIR /libraries
RUN wget https://github.com/borglab/gtsam/archive/refs/tags/4.2a8.tar.gz
RUN tar -xvzf 4.2a8.tar.gz
WORKDIR /libraries/gtsam-4.2a8/build
RUN cmake -DGTSAM_USE_SYSTEM_EIGEN=ON ..
RUN make install
RUN ldconfig

# Copy src and build
WORKDIR /ros2_ws/src
COPY ./autonomy-packages/src .

WORKDIR /ros2_ws/launch
COPY ./autonomy-packages/launch .

WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.sh && \
	colcon build --packages-select autonomous_interfaces slam spot_driver
RUN echo '. install/setup.bash' >> bootstrap.bash
RUN echo 'ros2 launch launch/autonomy_launch.py' >> bootstrap.bash
RUN chmod +x bootstrap.bash

CMD ["bash", "bootstrap.bash"]
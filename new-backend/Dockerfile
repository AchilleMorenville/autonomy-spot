FROM ros:humble

RUN apt-get update
RUN apt-get upgrade -y
RUN apt -y install python3-pip wget unzip libgl-dev libtbb-dev

# Install python packages

RUN python3 -m pip install open3d laspy fastapi uvicorn uvloop requests

# Copy ros interface package and build

WORKDIR /src
RUN git clone https://github.com/AchilleMorenville/autonomy-packages.git

WORKDIR /ros2_ws/src
RUN cp -r /src/autonomy-packages/aut_msgs /ros2_ws/src

WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.sh && \
	colcon build --packages-select aut_msgs

# Install PotreeConverter

WORKDIR /src
RUN wget https://github.com/CSTC-WTCB-BBRI/PotreeConverter/archive/refs/heads/master.zip
RUN unzip master.zip
WORKDIR /src/PotreeConverter-master/build
RUN cmake ..
RUN make -j 4

WORKDIR /src/python
COPY ./python .


# Install softwares for 360° camera
RUN apt -y install gphoto2
#RUN apt -y install libusb-dev
#WORKDIR /src
#RUN wget https://github.com/tlobbri/liptp2/archive/refs/heads/main.zip
#RUN unzip main.zip
#WORKDIR /src/liptp2-main
#RUN sh ./configure > configure.log
#RUN make > make.log
#RUN make install >makeinstall.log
#RUN ldconfig


WORKDIR /src/
RUN echo 'cd /ros2_ws && . install/setup.bash' >> bootstrap.bash
RUN echo 'cd /src/python && uvicorn server:app --reload --host 0.0.0.0 --port 21800' >> bootstrap.bash
RUN chmod +x bootstrap.bash

CMD ["bash", "bootstrap.bash"]

# Dockerfile ported from https://github.com/ethz-asl/kalibr/blob/master/Dockerfile_ros1_20_04
# Modified by Pedro Soares
FROM osrf/ros:noetic-desktop-full


# Dependencies we use, catkin tools is very good build system
# https://github.com/ethz-asl/kalibr/wiki/installation
RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
	apt-get install -y \
	git wget autoconf automake nano \
	python3-dev python3-pip python3-scipy python3-matplotlib \
	ipython3 python3-wxgtk4.0 python3-tk python3-igraph python3-pyx \
	libeigen3-dev libboost-all-dev libsuitesparse-dev \
	doxygen \
	libopencv-dev \
	libpoco-dev libtbb-dev libblas-dev liblapack-dev libv4l-dev \
	python3-catkin-tools python3-osrf-pycommon


# Create the workspace and build kalibr in it
ENV WORKSPACE /catkin_ws

RUN mkdir -p $WORKSPACE/src && \
	cd $WORKSPACE && \
	catkin init && \
	catkin config --extend /opt/ros/noetic && \
	catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

WORKDIR $WORKSPACE/src/
RUN git clone https://github.com/ethz-asl/kalibr.git
RUN git clone https://github.com/ori-drs/allan_variance_ros

RUN	cd $WORKSPACE &&\
	catkin build -j$(nproc)

WORKDIR /catkin_ws

RUN echo "#!/bin/bash" >> /entrypoint.sh \
    && echo "source /opt/ros/noetic/setup.bash" >> /entrypoint.sh \
    && echo "source /catkin_ws/devel/setup.bash" >> /entrypoint.sh \
    && echo "export KALIBR_MANUAL_FOCAL_LENGTH_INIT=1" >> /entrypoint.sh \
    && echo 'exec "$@"' >> /entrypoint.sh \
    && chmod a+x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

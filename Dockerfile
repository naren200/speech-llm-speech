# docker run --gpus all -it csaba1padma/padma-ros-repo:latest
# docker build --ssh default -t padma-ros .
# docker run --gpus all -it padma-ros
# run for development:
# xhost +local:docker; docker run --init --privileged -v /dev/bus/usb:/dev/bus/usb -e DISPLAY=$DISPLAY  -v /tmp/.X11-unix:/tmp/.X11-unix --gpus all -it --rm csaba1padma/padma-ros-repo:latest

# Use an official NVIDIA runtime as a parent image
# https://hub.docker.com/r/nvidia/cuda/tags
# FROM nvidia/cuda:12.3.1-base-ubuntu22.04
# FROM nvidia/cuda:12.3.1-runtime-ubuntu22.04
FROM ubuntu:22.04

# use bash as assumed for install steps
SHELL ["/bin/bash", "-c"]

# # Set DEBIAN_FRONTEND to noninteractive
ENV TZ='America/New_York'
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=iron
# assume GPU available for inference
# ARG USE_GPU=false

# copy packages from S3 buckets
# COPY Vimba64_v6.0_Linux.tgz /root/

RUN apt-get update -y

# install ROS2
RUN locale  # check for UTF-8
RUN apt install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
RUN locale  # verify settings

RUN apt install -y software-properties-common
RUN add-apt-repository universe

RUN apt update -y && apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update -y && apt install -y ros-dev-tools

RUN apt update -y
RUN apt upgrade -y
RUN apt install -y dialog
RUN apt install -y ros-iron-desktop-full

RUN echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
# ENV ROS_DOMAIN_ID 0
RUN echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc

RUN apt install -y xterm

# this step is from: https://github.com/webfactory/ssh-agent
# COPY root-config /root/
# RUN sed 's|/home/runner|/root|g' -i.bak /root/.ssh/config

RUN apt install -y git
RUN mkdir -p /root/ros2_ws/src/speech-llm-speech

# copy padma-ros repo (local code)
COPY . /root/ros2_ws/src/speech-llm-speech/

WORKDIR /root/ros2_ws/src/whisper_asr/src/

# mount type is necessary here!
# RUN git clone https://github.com/ros-ai/ros2_whisper_cpp.git
# WORKDIR /root/ros2_ws/src/whisper_cpp/
# RUN make install

# Clean up SSH keys and configuration
RUN rm -rf /root/.ssh

WORKDIR /root/ros2_ws

RUN rosdep init
RUN rosdep update
# RUN rosdep install -i --from-path src --rosdistro iron -y

RUN mkdir /var/log/speech-llm-speech

RUN apt-get update -y
RUN apt-get install -y python3-pip
RUN pip3 install --upgrade pip

WORKDIR /root/ros2_ws/src/speech-llm-speech/

# Install any needed packages specified in requirements.txt
# make sure --ignore-installed doesnt break things; without it there was an error uninstalling blinker
RUN pip install --no-cache-dir --ignore-installed -r requirements.txt

RUN apt-get update -y
# RUN apt-get install -y python3-tk libprotoc-dev protobuf-compiler libopenblas-base libopenmpi-dev python3-pip python3-pyqt5
RUN apt-get install -y python3-colcon-common-extensions
# RUN apt-get install -y python3-matplotlib
RUN apt install -y ros-iron-ament-cmake-clang-format

# some other useful things
RUN apt-get update -y && apt install -y gedit

# Kaylee request, to suppress warnings
RUN pip3 install setuptools==58.2.0

RUN apt-get install portaudio19-dev -y
RUN pip3 install pyaudio

WORKDIR /root/ros2_ws/

# Build packages
# * Reference: https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html
# * Reference: https://docs.ros.org/en/iron/Tutorials/Colcon-Tutorial.html
 
# Install required ROS2 development packages
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-iron-ament-cmake \
    ros-iron-ament-cmake-auto \
    && rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c 'source /opt/ros/iron/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'

RUN apt install python3-rosdep


# # Source ROS2 axnd build in a single RUN command
# RUN /bin/bash -c 'source /opt/ros/iron/setup.bash && \
#     colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'


# Set the default command to an interactive login shell
CMD ["/bin/bash", "-l"]
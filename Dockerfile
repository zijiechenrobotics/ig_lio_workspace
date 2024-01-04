FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu20.04

# set time zone
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo '$TZ' > /etc/timezone

# config English environment
ENV LANG=en_US.UTF-8

RUN apt-get update

# unminimize ubuntu
RUN yes | unminimize

# install ROS
RUN apt-get install -y lsb-release
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt install curl gnupg -y
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt update
RUN apt install ros-noetic-desktop -y
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source ~/.bashrc"

# gcc9 g++9
RUN apt-get install -y software-properties-common
RUN add-apt-repository ppa:ubuntu-toolchain-r/test
RUN apt-get install -y gcc-9 g++-9
# switch to gcc9 & g++9
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 10
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 10

# livox
COPY ./docker/Livox-SDK /root/lib/Livox-SDK
WORKDIR /root/lib/Livox-SDK/build
RUN cmake .. && make -j10 && make install

# tools
RUN apt-get install -y libgoogle-glog-dev \
    git

# install lxde
RUN apt install dbus-x11 -y
RUN apt install fonts-wqy-microhei -y
RUN apt install -y \
    gnome-user-docs-zh-hans \
    language-pack-gnome-zh-hans

RUN apt-get install -y --no-install-recommends \
    lxde-core \
    terminator 

# install vnc
RUN apt-get install tigervnc-standalone-server x11vnc -y
WORKDIR /root/.vnc
COPY ./docker/xstartup ./
RUN chmod u+x ~/.vnc/xstartup

# set up noVNC
COPY ./docker/noVNC /usr/lib/noVNC
COPY ./docker/websockify /usr/lib/noVNC/utils/websockify

RUN apt-get install -y inxi ros-noetic-pcl-ros

# evo 
WORKDIR /root/
RUN apt-get install pip -y
RUN pip3 install evo --upgrade --no-binary evo
RUN pip3 install packaging

WORKDIR /
# create a terminator icon on the desktop
COPY ./docker/terminator.desktop ./root/Desktop/
COPY ./docker/startup.sh ./
RUN chmod u+x startup.sh
ENTRYPOINT ["./startup.sh"]
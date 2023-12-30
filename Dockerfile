FROM ubuntu:20.04

# set time zone
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo '$TZ' > /etc/timezone

RUN apt-get update

# unminimize ubuntu
RUN yes | unminimize

# config CN environment
RUN apt install language-pack-zh-hans -y

RUN echo LANG="zh_CN.UTF-8" >> /etc/environment
RUN echo LANGUAGE="zh_CN:zh:en_US:en" >> /etc/environment

RUN echo LANG="zh_CN.UTF-8" >> /etc/profile
RUN echo LANGUAGE="zh_CN:zh:en_US:en" >> /etc/profile

RUN echo LANG="zh_CN.UTF-8" >> ~/.bashrc
RUN echo LANGUAGE="zh_CN:zh:en_US:en" >> ~/.bashrc

RUN locale-gen
RUN /bin/bash -c "source ~/.bashrc"

# install lxde desktop
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

# install ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt install curl gnupg -y
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt update
RUN apt install ros-noetic-desktop-full -y
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source ~/.bashrc"

# livox
COPY ./docker/Livox-SDK /root/lib/Livox-SDK
WORKDIR /root/lib/Livox-SDK/build
RUN cmake .. && make -j10 && make install

# evo 
WORKDIR /root/
RUN apt-get install pip -y
RUN pip3 install evo --upgrade --no-binary evo
RUN pip3 install packaging

# tools
RUN apt-get install -y libgoogle-glog-dev \
    git


WORKDIR /
# create a terminator icon on the desktop
COPY ./docker/terminator.desktop ./root/Desktop/
COPY ./docker/startup.sh ./
RUN chmod u+x startup.sh
ENTRYPOINT ["./startup.sh"]
# ig_lio_workspace

This Docker is a standard development environment for [iG-LIO](https://github.com/zijiechenrobotics/ig_lio), which helps you quickly experience iG-LIO. The Docker is visualized via [VNC](https://github.com/TigerVNC/tigervnc) and [noVNC](https://github.com/novnc/noVNC), allowing users to see the trajectory and mapping in real time.

Since Docker utilizes the CPU to render frames, RViz may experience delays when visualizing dense point clouds. **It is advisable to use `sparse_map(docker)` to visualize the global map in RViz.**  If you want to visualize the dense map, it is recommended that you run iG-LIO on the local machine.

![run](figure/run.gif)

## 1. Quick Start
### 1.1 Install Docker

Please install Docker following the links. This Docker supports both Ubuntu and Windows.

- Ubuntu: https://docs.docker.com/engine/install/ubuntu/
- Windows: https://docs.docker.com/desktop/install/windows-install/

### 1.2 Pull Container

```bash
docker push zijiechenrobotic/ig-lio-workspace:latest

# for Chinese mainland
docker pull registry.cn-guangzhou.aliyuncs.com/zijiechenrobotics/ig-lio-workspace:latest
```

### 1.3 Launch Container

If you are not interested in the Docker's source code, **you do not need to git clone**. Simply create a new workspace in the following format.

![file](figure/file.png)

Enter the below contents in the `docker-compose.yml`

```yaml
version: '3.4'
services:
  ig-lio-workspace:
    build:
      context: .
    image: zijiechenrobotic/ig-lio-workspace:latest
    # or 
    # registry.cn-guangzhou.aliyuncs.com/zijiechenrobotics/ig-lio-workspace:latest
    environment:
      - VNC_PW=abc123 # user defined password
      - VNC_GEOMETRY=1280x720 # vnc resolution
      - VNC_DEPTH=24 # 16/24/32
    volumes:
      # Map ig-lio source code to Docker (please change to your own path)
      - ./workspace:/root/workspace
      # Map datasets to Docker (Please change to your own path)
      - /media/czj2020/LSLAM/lidar_dataset:/root/lidar_dataset
    ports:
      # noVNC port:
      - 46080:6080
      # standard VNC port:
      - 45901:5901
```

Launch container

```bash
docker compose up -d
```

Shut down container

```bash
docker compose down
```

### 1.4 Access Container

#### 1.4.1 Access via VNC

Enter the following address to access Docker via any VNC viewer ( recommend Windows uses MobaXterm and Ubuntu uses Remmina)

```
localhost:45901
```

#### 1.4.2 Access via noVNC (recommend)

Any browser, enter the following address in the address bar

```
localhost:46080/vnc.html
```

![noVNC](figure/noVNC.png)

### 1.5 Run

```bash
# launch the terminator in desktop
cd /root/workspace/ig_lio
catkin_make

# launch ig_lio
source devel/setup.bash
roslaunch ig_lio lio_<dataset name>.launch
```

Since we have mapped the datasets to the `lidar_dataset`, we can play the datasets inside Docker

```
cd /root/lidar_dataset
rosbag play <dataset name>
```

## Docker Environment

```
base: ubuntu 20.04
ROS noetic
evo
gcc9 & g++9
glog
git
```


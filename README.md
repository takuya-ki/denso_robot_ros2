# denso_robot_ros2

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.svg)](https://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)
![repo size](https://img.shields.io/github/repo-size/takuya-ki/denso_robot_ros2)

- Docker environment for DENSO Robot ROS2 driver. This repository was inspired by [haraisao/denso_robot_ros2](https://github.com/haraisao/denso_robot_ros2).
- Example scripts for COBOTTA.

## Dependencies

### Docker build environments

- [Ubuntu 22.04 PC](https://ubuntu.com/certified/laptops?q=&limit=20&vendor=Dell&vendor=Lenovo&vendor=HP&release=22.04+LTS)
  - Docker 27.4.1
  - Docker Compose 2.32.1

## Installation

1. Follow [official README](https://github.com/DENSORobot/denso_robot_ros2) to setup the robot controller
2. Connect an Ethernet cable between the host computer and the Ethernet port of the COBOTTA  
3. Set the IP as `192.168.0.X` to reach the IP `192.168.0.1` (robot controller side)  
4. Build the docker environment as below (if you use the docker, this must be set in docker container)  
    ```bash
    sudo apt install byobu && git clone git@github.com:takuya-ki/denso_robot_ros2.git --recursive --depth 1 && cd denso_robot_ros2 && COMPOSE_DOCKER_CLI_BUILD=1 DOCKER_BUILDKIT=1 docker compose build --no-cache --parallel  
    ```

## Usage with docker

1. Build and run the docker environment
    - Create and start docker containers in the initially opened terminal
    ```bash
    docker compose up
    ```
2. Kill the processes previously executed and execute the demos with the real robot
    - You can find scripts in denso_robot_ros2/utils to execute all necessary commands for each demonstration
    ```bash
    ./utils/XXX.sh
    ```

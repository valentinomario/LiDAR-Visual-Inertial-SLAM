
# Quick-Start Guide

The goal of this guide is to get you started using this project on the hardware described in [the hardware section](#hardware). Follow it thoroughly if you wish to run your own experiments. This project is shipped with a helper script based on docker compose to automatize the configuration process. Running it on the docker container is not strictly necessary, however it is highly recommended. Note that the *aarch64*-based container with CUDA support was used for the experiments on the NVIDIA Jetson Orin NX, while the *x64*-based containers were only used for offline development. The *x64* container with CUDA support may not work on all machines as it depends on the system's hardware specifications. If you decide to continue the development on your local machine, or on a custom container, you can skip to [the ros 
drivers section](#ros-drivers). All the requirements necessary to your environment are listed in [the requirements section](#requirements). 

## Run your container

To pull and launch the docker container, simply move to the repository root directory and run:

    ./docker_up.sh

The helper script launches the correct image based on the system architecture and mounts the camera in `dev/video0`, if available. 

If you don't want CUDA acceleration, add the flag:

    ./docker_up.sh --no-cuda

After the container is up, you can start a shell in it by using:

    docker exec -it lidar_visual_inertial_slam /bin/bash


## ROS Drivers 

This section explains the installation of the drivers required to publish the information coming from the sensors. Keep in mind that if you just want to test the system with a bag file, and you are using the shipped docker container, you can skip it completely. In this case, the [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) installation is required for its custom message definition, which is already installed in the container.  If you decided not to use the container, you must follow their installation guide for ROS Humble.

To work with the Livox MID360 LiDAR, the driver must be configured by modifying the file located in `~/livox_ws/src/livox_ros_driver2/config/MID360_config.json` to fit your specific network setup. After modifying the configuration file, remember to build the driver and source the workspace:

    ~/livox_ws/src/livox_ros_driver2/./build.sh humble
    source ~/livox_ws/install/setup.bash

To run the driver and receive the LiDAR Point Cloud using the Livox custom message, simply run:

    ros2 launch livox_ros_driver2 msg_MID360_launch.py 



The ROS camera driver must be installed. Two different drivers have been tested on the NVIDIA Jetson Orin NX: the gscam driver and a custom gstreamer wrapper with GPU image preprocessing. **If you use the container without CUDA support** you must install the gscam (CPU) driver. **Otherwise**, you can choose between any of the two.

### CPU only camera driver (gscam)
 
You can clone [valentinomario/gscam](https://github.com/valentinomario/gscam), which is a fork of the official [ros-drivers/gscam](https://github.com/ros-drivers/gscam) with a custom made pipeline tested on the Jetson board. Inside the docker container run:

    cd ~/ros2_ws/src/
    git clone https://github.com/valentinomario/gscam.git
    cd .. && colcon build --packages-select gscam

To later run the driver:

    source install/setup.bash
    ros2 launch gscam arducam.launch.xml 

To customize image format to fit your needs, modify the launch file or create a new one following  the `arducam.launch.xml` structure.

### GPU camera driver (camera_driver)

[valentinomario/camera_driver](https://github.com/valentinomario/camera_driver) is a gstreamer wrapper that uses opencv with CUDA acceleration for image resizing. You can choose to use this driver if you think your setup would benefit from CUDA acceleration of this step. To build and run the camera driver, run:

    cd ~/ros2_ws/src
    git clone https://github.com/valentinomario/camera_driver.git
    cd ..
    colcon build --packages-select camera_driver
    ros2 launch camera_driver camera_driver.launch.py

The options can be can be customized in `camera_driver/config/driver_options.yaml`.

## Run Project 

If you are using the docker container with the helper script, the node source code will be already in `~/ros2_ws/src`. Otherwise you need to clone this repository inside your environment. The configuration files are located in the `config_pkg/config/params_camera.yaml` and `config_pkg/config/params_lidar.yaml` files.

The project can be simply built using `colcon build`, however, it is recommended to build it using the following options to avoid unexpected freezes:

    export MAKEFLAGS="-j 4"
    colcon build --parallel-workers 2

Once the LiDAR and the camera are connected, and the drivers are running, you are ready to run the nodes. You can run the LiDAR Odometry Subsystem and the Visual Odometry Subsystem separately by starting two shells inside the container and running:

    source ~/ros2_ws/install/setup.bash
    ros2 launch lidar_odometry run.launch.py
    
And:

    source ~/ros2_ws/install/setup.bash
    ros2 launch vins_estimator run.launch.py

Notice that the Livox workspace is automatically sourced whenever you open a new shell in the shipped docker container.

If you wish to start both the subsystems in the same shell, use the following launch file:

    source ~/ros2_ws/install/setup.bash
    ros2 launch config_pkg full_system.launch.py 

To debug the output of the system, you can open the rviz2 presets:

    ros2 launch config_pkg rviz.launch.py 

## Hardware

This project is designed to work with some specific hardware. While it is not strictly necessary to have the same hardware to run it, some changes to the code base might be required to accommodate any differences. Below there is a list of the hardware used for the experiments:

 - Livox LiDAR MID360
 - Arducam IMX219
 - NVIDIA Jetson Orin NX
 - Seeedstudio A603 carrier board 




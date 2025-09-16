# Deployment Guide

This document provides a step-by-step guide to deploying ROS2 applications on a K3s cluster. 

## Config Files

Ensure you have the necessary configuration files in place. Below is an example of a `pod_spec.yaml` file used for deploying a ROS2 application. Here is the example for using ROS2 Publisher and Subscriber

```yaml
name: ros2_pub
image: ros:humble
command: "ros2 run demo_nodes_cpp talker"
namespace: ros2-pub-sub
deployment: edge # or 'local' for running on the local robot
```

```yaml
name: ros2_sub
image: ros:humble
command: "ros2 run demo_nodes_cpp listener"
namespace: ros2-pub-sub
deployment: edge # or 'local' for running on the local robot
```

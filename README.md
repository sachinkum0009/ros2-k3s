# ros2-k3s
Repo for using k3s with ROS2

## How to Run ros2_k3s_cli

1. Build the workspace:

```bash
colcon build --symlink-install
```

2. Source the ROS2 workspace:

```bash
source install/setup.bash
```

3. Run the CLI tool:

```bash
ros2 run ros2_k3s_cli cli --help
```

### Generate Robot Pod Config

Interactively create a robot pod YAML config:

```bash
ros2 run ros2_k3s_cli cli create-config --output robot_pod_generated.yaml
```

You will be prompted for name, image, command, namespace, and can select deployment (edge/local) using arrow keys.

### Example Commands

- Initialize cluster:
  ```bash
  ros2 run ros2_k3s_cli cli init --config cluster.yaml
  ```
- Validate cluster config:
  ```bash
  ros2 run ros2_k3s_cli cli validate --config cluster.yaml
  ```
- Apply (deploy) application:
  ```bash
  ros2 run ros2_k3s_cli cli deploy --config src/ros2-k3s/examples/ros2_sub.yaml
  ```

Replace `cluster.yaml` and the config path with your actual configuration files.

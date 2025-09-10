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

### Example Commands

- Initialize cluster:
  ```bash
  ros2 run ros2_k3s_cli cli init --config cluster.yaml
  ```
- Validate cluster config:
  ```bash
  ros2 run ros2_k3s_cli cli validate --config cluster.yaml
  ```
- Deploy application:
  ```bash
  ros2 run ros2_k3s_cli cli deploy --app myapp --config deploy.yaml
  ```

Replace `cluster.yaml`, `myapp`, and `deploy.yaml` with your actual configuration and application names.

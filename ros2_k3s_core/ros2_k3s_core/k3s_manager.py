# Copyright 2025 Sachin Kumar.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ros2_k3s_core.config import K3S_CONFIG_PATH
from kubernetes import client, config
from ros2_k3s_core.robot_pod import load_robot_pod_from_yaml
from ros2_k3s_core.create_pod import create_deployment


class K3SManager:
    """
    Manages Kubernetes resources for ROS2 robots using k3s.

    Initializes Kubernetes API clients and provides methods to deploy robot configurations.
    """

    def __init__(self):
        config.load_kube_config(config_file=K3S_CONFIG_PATH)
        self._apps_api = client.AppsV1Api()
        self._core_api = client.CoreV1Api()

    def deploy_config(self, yaml_path: str):
        """
        Deploys a robot pod configuration to the Kubernetes cluster using the provided YAML file.

        Args:
            yaml_path (str): The file path to the YAML configuration for the robot pod.

        Returns:
            V1Deployment: The result of the deployment creation operation from the Kubernetes API.

        Raises:
            ApiException: If there is an error during deployment creation.
        """
        robot_pod = load_robot_pod_from_yaml(yaml_path)
        deployment = create_deployment(robot_pod)
        result = self._apps_api.create_namespaced_deployment(
            namespace=robot_pod.namespace, body=deployment
        )
        return result

    def get_namespaces(self):
        """
        Retrieves the list of namespaces in the Kubernetes cluster.

        Returns:
            V1NamespaceList: The list of namespaces from the Kubernetes API.
        """
        return self._core_api.list_namespace()

    def get_nodes(self):
        """
        Retrieves the list of nodes in the Kubernetes cluster.

        Returns:
            V1NodeList: The list of nodes from the Kubernetes API.
        """
        return self._core_api.list_node()

    def get_pods(self, namespace: str):
        """
        Retrieves the list of pods in the specified namespace.

        Args:
            namespace (str): The namespace to list pods from.
        Returns:
            V1PodList: The list of pods in the specified namespace from the Kubernetes API.
        """
        return self._core_api.list_namespaced_pod(namespace=namespace)
    
    def get_logs(self, namespace: str, pod_name: str):
        """
        Retrieves the logs of a specific pod in the specified namespace for the last 5 seconds.

        Args:
            namespace (str): The namespace of the pod.
            pod_name (str): The name of the pod to retrieve logs from.

        Returns:
            str: The logs of the specified pod from the last 5 seconds.
        """
        return self._core_api.read_namespaced_pod_log(
            name=pod_name, 
            namespace=namespace, 
            since_seconds=5
        )

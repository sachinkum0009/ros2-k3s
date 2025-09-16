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


from kubernetes import client
from kubernetes.client import V1Deployment
from ros2_k3s_core.robot_pod import RobotPod


def create_deployment(robot_pod: RobotPod) -> V1Deployment:
    container = client.V1Container(
        name=robot_pod.name,
        image=robot_pod.image,
        command=["/bin/bash", "-c", robot_pod.command],
    )

    pod_spec = client.V1PodSpec(
        containers=[container],
        node_selector={"kubernetes.io/hostname": robot_pod.deployment},
    )

    template = client.V1PodTemplateSpec(
        metadata=client.V1ObjectMeta(labels={"app": robot_pod.name}), spec=pod_spec
    )

    spec = client.V1DeploymentSpec(
        replicas=1,
        selector=client.V1LabelSelector(match_labels={"app": robot_pod.name}),
        template=template,
    )

    deployment = client.V1Deployment(
        api_version="apps/v1",
        kind="Deployment",
        metadata=client.V1ObjectMeta(name=robot_pod.name),
        spec=spec,
    )
    return deployment

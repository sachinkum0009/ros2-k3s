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

from pydantic import BaseModel
from typing import Literal
import yaml


class RobotPod(BaseModel):
    name: str
    image: str
    command: str
    namespace: str
    deployment: Literal["edge", "local", "asus-pc", "mydevice2", "tb1", "tb2", "tb3"]


def load_robot_pod_from_yaml(yaml_path: str) -> RobotPod:
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)
    return RobotPod(**data)

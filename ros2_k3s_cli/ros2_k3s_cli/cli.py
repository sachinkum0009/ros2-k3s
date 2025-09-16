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

import argparse
import sys
import yaml
import questionary

from ros2_k3s_core.robot_pod import RobotPod
from ros2_k3s_core.k3s_manager import K3SManager

class K3SCLI:
    def __init__(self):
        self.parser = argparse.ArgumentParser(description='ROS2 K3S CLI')
        subparsers = self.parser.add_subparsers(dest='command', required=True)

        # init command
        parser_init = subparsers.add_parser('init', help='Initialize the ROS2 K3S cluster')
        parser_init.add_argument('--config', required=True, help='Path to cluster config file')
        parser_init.set_defaults(func=self.init)

        # validate command
        parser_validate = subparsers.add_parser('validate', help='Validate cluster configuration')
        parser_validate.add_argument('--config', required=True, help='Path to cluster config file')
        parser_validate.set_defaults(func=self.validate)

        # deploy command
        parser_deploy = subparsers.add_parser('deploy', help='Deploy application to cluster')
        parser_deploy.add_argument('--config', required=True, help='Path to deployment config file')
        parser_deploy.set_defaults(func=self.deploy)

        # create-config command
        parser_create_config = subparsers.add_parser('create-config', help='Interactively create a robot pod YAML config')
        parser_create_config.add_argument('--output', required=True, help='Output YAML file path')
        parser_create_config.set_defaults(func=self.create_config)

        self.k3s_manager = K3SManager()

    def run(self, argv=None):
        args = self.parser.parse_args(argv)
        return args.func(args)

    def init(self, args):
        if not args.config:
            print('Error: --config is required for init')
            return 1
        print(f'Initializing cluster with config: {args.config}')
        # TODO: Add actual initialization logic
        return 0

    def validate(self, args):
        if not args.config:
            print('Error: --config is required for validate')
            return 1
        print(f'Validating cluster config: {args.config}')
        # TODO: Add actual validation logic
        return 0

    def deploy(self, args):
        if not args.config:
            print('Error: --config is required for deploy')
            return 1
        print(f'Deploying with config: {args.config}')
        try:
            result = self.k3s_manager.deploy_config(args.config)
            print('Deployment successful:')
            # print(result)
        except Exception as e:
            print(f'Error during deployment: {e}')
            return 1
        return 0

    def create_config(self, args):
        print('Creating robot pod config...')
        name = input('Enter name: ')
        image = input('Enter image: ')
        command = input('Enter command: ')
        namespace = input('Enter namespace: ')
        deployment = questionary.select(
            'Select deployment:',
            choices=['edge', 'local']
        ).ask()

        try:
            pod = RobotPod(
                name=name,
                image=image,
                command=command,
                namespace=namespace,
                deployment=deployment
            )
        except Exception as e:
            print(f'Error: {e}')
            return 1

        data = {
            'name': pod.name,
            'image': pod.image,
            'command': pod.command,
            'namespace': pod.namespace,
            'deployment': pod.deployment
        }
        with open(args.output, 'w') as f:
            yaml.safe_dump(data, f, sort_keys=False)
        print(f'Config written to {args.output}')
        return 0

def main():
    cli = K3SCLI()
    sys.exit(cli.run())

if __name__ == '__main__':
    main()

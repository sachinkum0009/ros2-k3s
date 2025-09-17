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
import subprocess
from tabulate import tabulate

from ros2_k3s_core.robot_pod import RobotPod
from ros2_k3s_core.k3s_manager import K3SManager

class K3SCLI:
    def __init__(self):
        self.parser = argparse.ArgumentParser(description='ROS2 K3S CLI')
        subparsers = self.parser.add_subparsers(dest='command', required=True)

        # init command
        parser_init = subparsers.add_parser('init', help='Initialize the ROS2 K3S cluster')
        parser_init.add_argument('--server', action='store_true', help='Initialize as K3S server')
        parser_init.add_argument('--agent', action='store_true', help='Initialize as K3S agent')
        parser_init.add_argument('--token', help='Token for K3S agent (required when using --agent)')
        parser_init.add_argument('--url', help='K3S server URL for agent (required when using --agent)')
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

        # get command
        parser_get = subparsers.add_parser('get', help='Get cluster resources')
        get_subparsers = parser_get.add_subparsers(dest='resource', required=False)
        parser_get.set_defaults(func=self.get_interactive)
        
        # get namespace subcommand
        parser_get_namespace = get_subparsers.add_parser('namespace', help='Get namespaces')
        parser_get_namespace.set_defaults(func=self.get_namespace)
        
        # get pod subcommand
        parser_get_pod = get_subparsers.add_parser('pod', help='Get pods')
        parser_get_pod.add_argument('-n', '--namespace', required=True, help='Namespace to get pods from')
        parser_get_pod.set_defaults(func=self.get_pod)


    def run(self, argv=None):
        args = self.parser.parse_args(argv)
        return args.func(args)

    def init(self, args):
        # Check if both server and agent are specified
        if args.server and args.agent:
            print('Error: Cannot specify both --server and --agent')
            return 1
        
        # Check if neither server nor agent is specified
        if not args.server and not args.agent:
            print('Error: Must specify either --server or --agent')
            return 1
        
        if args.server:
            print('Installing K3S server...')
            try:
                # Install K3S server using shell command
                subprocess.run('curl -sfL https://get.k3s.io | sh -', 
                             shell=True, check=True)
                
                print('K3S server installed successfully')
                
                # Set permissions for k3s.yaml
                print('Setting permissions for k3s.yaml...')
                subprocess.run(['sudo', 'chmod', '644', '/etc/rancher/k3s/k3s.yaml'], check=True)
                print('Permissions set successfully')
                
            except subprocess.CalledProcessError as e:
                print(f'Error installing K3S server: {e}')
                return 1
        
        elif args.agent:
            if not args.token:
                print('Please find the token')
                return 1
            if not args.url:
                print('Error: --url is required when using --agent')
                return 1
            
            print(f'Installing k3s agent with token and URL: {args.url}')
            try:
                # Install K3S agent using shell command with environment variables
                cmd = f'curl -sfL https://get.k3s.io | K3S_URL={args.url} K3S_TOKEN={args.token} sh -'
                subprocess.run(cmd, shell=True, check=True)
                
                print('K3S agent installed successfully')
                
            except subprocess.CalledProcessError as e:
                print(f'Error installing K3S agent: {e}')
                return 1
        
        return 0

    def validate(self, args):
        if not args.config:
            print('Error: --config is required for validate')
            return 1
        print(f'Validating cluster config: {args.config}')
        # TODO: Add actual validation logic
        return 0

    def deploy(self, args):
        k3s_manager = K3SManager()
        if not args.config:
            print('Error: --config is required for deploy')
            return 1
        print(f'Deploying with config: {args.config}')
        try:
            result = k3s_manager.deploy_config(args.config)
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

    def get_namespace(self, args):
        """Get and display all namespaces"""
        try:
            k3s_manager = K3SManager()
            namespaces_list = k3s_manager.get_namespaces()
            
            namespaces = [
                [
                    ns.metadata.name,
                    ns.status.phase,
                    ns.metadata.creation_timestamp.strftime('%Y-%m-%d %H:%M:%S') if ns.metadata.creation_timestamp else ""
                ]
                for ns in namespaces_list.items
            ]
            
            print(tabulate(namespaces, headers=["Name", "Status", "Age"], tablefmt="grid"))
            
        except Exception as e:
            print(f'Error getting namespaces: {e}')
            return 1
        return 0

    def get_pod(self, args):
        """Get and display pods in specified namespace"""
        try:
            k3s_manager = K3SManager()
            pods_list = k3s_manager.get_pods(args.namespace)
            
            pods = [
                [
                    pod.metadata.name,
                    pod.status.phase,
                    pod.status.pod_ip if pod.status.pod_ip else "",
                    pod.spec.node_name if pod.spec.node_name else "",
                ]
                for pod in pods_list.items
            ]
            
            print(tabulate(pods, headers=["Pod Name", "Status", "Pod IP", "Node"], tablefmt="grid"))
            
        except Exception as e:
            print(f'Error getting pods: {e}')
            return 1
        return 0

    def get_interactive(self, args):
        """Interactive mode for getting cluster resources"""
        # If a specific resource is requested, delegate to the appropriate method
        if args.resource == 'namespace':
            return self.get_namespace(args)
        elif args.resource == 'pod':
            return self.get_pod(args)
        
        # Interactive mode - no specific resource requested
        try:
            k3s_manager = K3SManager()
            
            # Step 1: Get and display namespaces for selection
            print("Fetching namespaces...")
            namespaces_list = k3s_manager.get_namespaces()
            
            if not namespaces_list.items:
                print("No namespaces found.")
                return 1
            
            # Create choices for namespace selection
            namespace_choices = [ns.metadata.name for ns in namespaces_list.items]
            
            # Let user select a namespace
            selected_namespace = questionary.select(
                'Select a namespace:',
                choices=namespace_choices
            ).ask()
            
            if not selected_namespace:
                print("No namespace selected.")
                return 1
            
            print(f"\nSelected namespace: {selected_namespace}")
            
            # Step 2: Get and display pods in the selected namespace
            print(f"Fetching pods in namespace '{selected_namespace}'...")
            pods_list = k3s_manager.get_pods(selected_namespace)
            
            if not pods_list.items:
                print(f"No pods found in namespace '{selected_namespace}'.")
                return 0
            
            # Display pods in table format
            pods = [
                [
                    pod.metadata.name,
                    pod.status.phase,
                    pod.status.pod_ip if pod.status.pod_ip else "",
                    pod.spec.node_name if pod.spec.node_name else "",
                ]
                for pod in pods_list.items
            ]
            
            print(f"\nPods in namespace '{selected_namespace}':")
            print(tabulate(pods, headers=["Pod Name", "Status", "Pod IP", "Node"], tablefmt="grid"))
            
            # Step 3: Let user select a pod to view logs
            pod_choices = [pod.metadata.name for pod in pods_list.items]
            
            selected_pod = questionary.select(
                'Select a pod to view logs:',
                choices=pod_choices
            ).ask()
            
            if not selected_pod:
                print("No pod selected.")
                return 0
            
            print(f"\nFetching logs for pod '{selected_pod}' in namespace '{selected_namespace}'...")
            
            # Step 4: Get and display logs
            try:
                logs = k3s_manager.get_logs(selected_namespace, selected_pod)
                print(f"\n{'='*60}")
                print(f"LOGS FOR POD: {selected_pod}")
                print(f"NAMESPACE: {selected_namespace}")
                print(f"{'='*60}")
                print(logs)
                print(f"{'='*60}")
                
            except Exception as log_error:
                print(f'Error getting logs for pod {selected_pod}: {log_error}')
                return 1
            
        except Exception as e:
            print(f'Error in interactive mode: {e}')
            return 1
        
        return 0

def main():
    cli = K3SCLI()
    sys.exit(cli.run())

if __name__ == '__main__':
    main()

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

import threading
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nicegui import Client, app, ui, ui_run

from ros2_k3s_core.k3s_manager import K3SManager


class NiceGuiNode(Node):

    def __init__(self) -> None:
        super().__init__('nicegui')

        self.k3s_manager = K3SManager()

        self.namespaces = [ns.metadata.name for ns in self.k3s_manager.get_namespaces().items]
        
        # Get nodes with their actual status
        nodes_list = self.k3s_manager.get_nodes()
        self.nodes = []
        for node in nodes_list.items:
            ready_status = "Unknown"
            for condition in node.status.conditions:
                if condition.type == "Ready":
                    ready_status = "Ready" if condition.status == "True" else "NotReady"
                    break
            
            self.nodes.append({
                'name': node.metadata.name,
                'status': ready_status
            })
        
        # State for selected namespace and its pods
        self.selected_namespace = None
        self.namespace_pods = []

        with Client.auto_index_client:
            # Set dark theme and modern styling
            ui.dark_mode(True)
            ui.colors(primary='#3b82f6', secondary='#6366f1', accent='#8b5cf6', 
                     dark='#0f1419', positive='#10b981', negative='#ef4444', 
                     info='#06b6d4', warning='#f59e0b')
            
            # Modern header with gradient background
            with ui.element('div').classes('w-full bg-gradient-to-r from-blue-600 to-purple-600 p-8 mb-8'):
                ui.label('ROS2 K3S Dashboard').classes('text-center text-5xl font-bold text-white mb-2')
                ui.label('Kubernetes Cluster Management').classes('text-center text-xl text-blue-100 opacity-80')
            
            # Modern container with grid layout
            with ui.element('div').classes('container mx-auto px-6'):
                
                # Stats/Overview section
                with ui.element('div').classes('grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6 mb-8'):
                    
                    # Namespaces card
                    with ui.card().classes('bg-gray-800 border border-gray-700 shadow-xl hover:shadow-2xl transition-all duration-300'):
                        with ui.element('div').classes('p-6'):
                            with ui.element('div').classes('flex items-center justify-between mb-4'):
                                ui.icon('folder', size='2xl').classes('text-blue-400')
                                ui.label(str(len(self.namespaces))).classes('text-3xl font-bold text-white')
                            ui.label('Namespaces').classes('text-lg font-semibold text-gray-300')
                            ui.label('Active namespaces in cluster').classes('text-sm text-gray-500 mt-1')
                    
                    # Nodes card
                    with ui.card().classes('bg-gray-800 border border-gray-700 shadow-xl hover:shadow-2xl transition-all duration-300'):
                        with ui.element('div').classes('p-6'):
                            with ui.element('div').classes('flex items-center justify-between mb-4'):
                                ui.icon('computer', size='2xl').classes('text-orange-400')
                                ui.label(str(len(self.nodes))).classes('text-3xl font-bold text-white')
                            ui.label('Nodes').classes('text-lg font-semibold text-gray-300')
                            ui.label('Worker and master nodes').classes('text-sm text-gray-500 mt-1')
                    
                    # Cluster status card
                    with ui.card().classes('bg-gray-800 border border-gray-700 shadow-xl hover:shadow-2xl transition-all duration-300'):
                        with ui.element('div').classes('p-6'):
                            with ui.element('div').classes('flex items-center justify-between mb-4'):
                                ui.icon('check_circle', size='2xl').classes('text-green-400')
                                ui.label('Online').classes('text-3xl font-bold text-white')
                            ui.label('Cluster Status').classes('text-lg font-semibold text-gray-300')
                            ui.label('K3s cluster is running').classes('text-sm text-gray-500 mt-1')
                    
                    # Connection info card
                    with ui.card().classes('bg-gray-800 border border-gray-700 shadow-xl hover:shadow-2xl transition-all duration-300'):
                        with ui.element('div').classes('p-6'):
                            with ui.element('div').classes('flex items-center justify-between mb-4'):
                                ui.icon('cloud', size='2xl').classes('text-purple-400')
                                ui.label('K3s').classes('text-3xl font-bold text-white')
                            ui.label('Platform').classes('text-lg font-semibold text-gray-300')
                            ui.label('Kubernetes distribution').classes('text-sm text-gray-500 mt-1')
                
                # Grid layout for namespaces and nodes
                with ui.element('div').classes('grid grid-cols-1 lg:grid-cols-2 gap-8'):
                    
                    # Detailed namespaces section
                    with ui.card().classes('bg-gray-800 border border-gray-700 shadow-xl'):
                        with ui.element('div').classes('p-6'):
                            with ui.element('div').classes('flex items-center mb-6'):
                                ui.icon('folder', size='lg').classes('text-blue-400 mr-3')
                                ui.label('Cluster Namespaces').classes('text-2xl font-bold text-white')
                            
                            # Namespaces grid
                            with ui.element('div').classes('grid grid-cols-1 gap-3'):
                                for ns in self.namespaces:
                                    with ui.element('div').classes('bg-gray-700 border border-gray-600 rounded-lg p-4 hover:bg-blue-600 cursor-pointer transition-all duration-200').on('click', lambda e, ns=ns: self.show_namespace_pods(ns)):
                                        with ui.element('div').classes('flex items-center justify-between'):
                                            with ui.element('div').classes('flex items-center'):
                                                ui.icon('folder_open', size='md').classes('text-blue-300 mr-3')
                                                ui.label(ns).classes('text-white font-medium text-lg')
                                            ui.icon('arrow_forward_ios', size='sm').classes('text-gray-400')
                                            
                            # Empty state if no namespaces
                            if not self.namespaces:
                                with ui.element('div').classes('text-center py-12'):
                                    ui.icon('folder_off', size='4xl').classes('text-gray-600 mb-4')
                                    ui.label('No namespaces found').classes('text-xl text-gray-400 mb-2')
                                    ui.label('Check your cluster connection').classes('text-gray-500')
                    
                    # Detailed nodes section
                    with ui.card().classes('bg-gray-800 border border-gray-700 shadow-xl'):
                        with ui.element('div').classes('p-6'):
                            with ui.element('div').classes('flex items-center mb-6'):
                                ui.icon('computer', size='lg').classes('text-orange-400 mr-3')
                                ui.label('Cluster Nodes').classes('text-2xl font-bold text-white')
                            
                            # Nodes grid
                            with ui.element('div').classes('grid grid-cols-1 gap-3'):
                                for node in self.nodes:
                                    with ui.element('div').classes('bg-gray-700 border border-gray-600 rounded-lg p-4 hover:bg-gray-650 transition-colors duration-200'):
                                        with ui.element('div').classes('flex items-center justify-between'):
                                            with ui.element('div').classes('flex items-center'):
                                                ui.icon('memory', size='md').classes('text-orange-300 mr-3')
                                                ui.label(node['name']).classes('text-white font-medium text-lg')
                                            # Status indicator with dynamic colors
                                            with ui.element('div').classes('flex items-center'):
                                                if node['status'] == 'Ready':
                                                    ui.icon('check_circle', size='sm').classes('text-green-400 mr-1')
                                                    ui.label('Ready').classes('text-sm text-green-300')
                                                elif node['status'] == 'NotReady':
                                                    ui.icon('error', size='sm').classes('text-red-400 mr-1')
                                                    ui.label('Not Ready').classes('text-sm text-red-300')
                                                else:
                                                    ui.icon('help', size='sm').classes('text-yellow-400 mr-1')
                                                    ui.label('Unknown').classes('text-sm text-yellow-300')
                                            
                            # Empty state if no nodes
                            if not self.nodes:
                                with ui.element('div').classes('text-center py-12'):
                                    ui.icon('computer_off', size='4xl').classes('text-gray-600 mb-4')
                                    ui.label('No nodes found').classes('text-xl text-gray-400 mb-2')
                                    ui.label('Check your cluster connection').classes('text-gray-500')

    def show_namespace_pods(self, namespace: str):
        """Show pods for the selected namespace"""
        self.selected_namespace = namespace
        
        # Get pods for the selected namespace
        try:
            pods_list = self.k3s_manager.get_pods(namespace)
            self.namespace_pods = []
            
            for pod in pods_list.items:
                pod_info = {
                    'name': pod.metadata.name,
                    'status': pod.status.phase,
                    'ready': sum(1 for condition in (pod.status.conditions or []) 
                               if condition.type == 'Ready' and condition.status == 'True'),
                    'total_containers': len(pod.spec.containers) if pod.spec.containers else 0,
                    'node': pod.spec.node_name,
                    'created': pod.metadata.creation_timestamp,
                    'cpu_usage': 0.0,
                    'ram_usage': 0.0
                }
                
                # Try to get resource usage for running and ready pods only
                is_pod_ready = (pod.status.phase == 'Running' and 
                               pod_info['ready'] > 0 and 
                               pod_info['ready'] == pod_info['total_containers'])
                
                if is_pod_ready:
                    try:
                        ram_usage, cpu_usage = self.k3s_manager.get_resource_usage(namespace, pod.metadata.name)
                        print(f"Pod: {pod.metadata.name}, RAM: {ram_usage:.2f} MB, CPU: {cpu_usage:.2f} mCPU")
                        pod_info['ram_usage'] = ram_usage
                        pod_info['cpu_usage'] = cpu_usage
                    except Exception as e:
                        print(f"Could not fetch resource usage for pod {pod.metadata.name}: {e}")
                        # Keep default values of 0.0
                
                self.namespace_pods.append(pod_info)
        except Exception as e:
            self.namespace_pods = []
            print(f"Error fetching pods for namespace {namespace}: {e}")
        
        # Open dialog to show pods
        self.show_pods_dialog()
    
    def show_pods_dialog(self):
        """Display pods in a modal dialog"""
        with ui.dialog().props('maximized') as dialog:
            with ui.card().classes('w-full h-full bg-gray-900'):
                # Header
                with ui.element('div').classes('bg-gradient-to-r from-blue-600 to-purple-600 p-6 mb-6'):
                    with ui.element('div').classes('flex items-center justify-between'):
                        with ui.element('div').classes('flex items-center'):
                            ui.icon('folder_open', size='lg').classes('text-white mr-3')
                            ui.label(f'Pods in "{self.selected_namespace}" namespace').classes('text-2xl font-bold text-white')
                        ui.button('Close', on_click=dialog.close).props('flat').classes('text-white')
                
                # Content
                with ui.element('div').classes('p-6 overflow-auto'):
                    if self.namespace_pods:
                        # Pods grid
                        with ui.element('div').classes('grid grid-cols-1 lg:grid-cols-2 xl:grid-cols-3 gap-4'):
                            for pod in self.namespace_pods:
                                with ui.card().classes('bg-gray-800 border border-gray-700'):
                                    with ui.element('div').classes('p-4'):
                                        # Pod header
                                        with ui.element('div').classes('flex items-center justify-between mb-3'):
                                            with ui.element('div').classes('flex items-center'):
                                                # Status icon
                                                if pod['status'] == 'Running':
                                                    ui.icon('play_circle', size='md').classes('text-green-400 mr-2')
                                                elif pod['status'] == 'Pending':
                                                    ui.icon('schedule', size='md').classes('text-yellow-400 mr-2')
                                                else:
                                                    ui.icon('error', size='md').classes('text-red-400 mr-2')
                                                
                                                ui.label(pod['name']).classes('text-white font-semibold text-lg')
                                            
                                            # Status badge
                                            status_color = 'bg-green-600' if pod['status'] == 'Running' else 'bg-yellow-600' if pod['status'] == 'Pending' else 'bg-red-600'
                                            with ui.element('div').classes(f'{status_color} px-2 py-1 rounded-full'):
                                                ui.label(pod['status']).classes('text-white text-xs font-medium')
                                        
                                        # Pod details
                                        with ui.element('div').classes('space-y-2'):
                                            with ui.element('div').classes('flex items-center text-sm'):
                                                ui.icon('computer', size='sm').classes('text-gray-400 mr-2')
                                                ui.label(f"Node: {pod['node'] or 'Unknown'}").classes('text-gray-300')
                                            
                                            with ui.element('div').classes('flex items-center text-sm'):
                                                ui.icon('inventory', size='sm').classes('text-gray-400 mr-2')
                                                ui.label(f"Containers: {pod['ready']}/{pod['total_containers']}").classes('text-gray-300')
                                            
                                            if pod['created']:
                                                with ui.element('div').classes('flex items-center text-sm'):
                                                    ui.icon('schedule', size='sm').classes('text-gray-400 mr-2')
                                                    ui.label(f"Created: {pod['created'].strftime('%Y-%m-%d %H:%M:%S')}").classes('text-gray-300')
                                        
                                        # Resource usage section
                                        with ui.element('div').classes('mt-4 pt-3 border-t border-gray-600'):
                                            with ui.element('div').classes('flex items-center justify-between mb-2'):
                                                ui.label('Resource Usage').classes('text-sm font-semibold text-gray-300')
                                                # Only show refresh button for truly ready pods
                                                is_pod_ready = pod['status'] == 'Running' and pod['ready'] > 0 and pod['ready'] == pod['total_containers']
                                                if is_pod_ready:
                                                    ui.button('', icon='refresh', on_click=lambda p=pod: self.refresh_pod_resources(p)).props('flat dense').classes('text-gray-400 hover:text-white')
                                            
                                            # Check if pod is truly running (status = Running AND containers are ready)
                                            is_pod_ready = pod['status'] == 'Running' and pod['ready'] > 0 and pod['ready'] == pod['total_containers']
                                            
                                            if is_pod_ready:
                                                # CPU Usage for running and ready pods
                                                with ui.element('div').classes('flex items-center justify-between text-sm mb-1'):
                                                    with ui.element('div').classes('flex items-center'):
                                                        ui.icon('memory', size='sm').classes('text-purple-400 mr-2')
                                                        ui.label('CPU:').classes('text-gray-300')
                                                    cpu_usage = pod.get('cpu_usage', 0.0)
                                                    cpu_color = 'text-green-300' if cpu_usage < 50 else 'text-yellow-300' if cpu_usage < 80 else 'text-red-300'
                                                    # Create reactive label for CPU
                                                    cpu_label = ui.label(f"{cpu_usage:.1f} mCPU").classes(f'{cpu_color} font-mono')
                                                    pod['cpu_label'] = cpu_label  # Store reference for updates
                                                
                                                # RAM Usage for running and ready pods
                                                with ui.element('div').classes('flex items-center justify-between text-sm'):
                                                    with ui.element('div').classes('flex items-center'):
                                                        ui.icon('storage', size='sm').classes('text-blue-400 mr-2')
                                                        ui.label('RAM:').classes('text-gray-300')
                                                    ram_usage = pod.get('ram_usage', 0.0)
                                                    ram_color = 'text-green-300' if ram_usage < 100 else 'text-yellow-300' if ram_usage < 500 else 'text-red-300'
                                                    # Create reactive label for RAM
                                                    ram_label = ui.label(f"{ram_usage:.1f} MB").classes(f'{ram_color} font-mono')
                                                    pod['ram_label'] = ram_label  # Store reference for updates
                                            else:
                                                # Show appropriate message for non-ready pods
                                                with ui.element('div').classes('flex items-center justify-center py-2'):
                                                    if pod['status'] == 'Running' and pod['ready'] == 0:
                                                        # Pod is "Running" but containers are not ready
                                                        ui.icon('warning', size='sm').classes('text-yellow-400 mr-2')
                                                        ui.label('Containers Not Ready').classes('text-yellow-300 font-semibold')
                                                    elif pod['status'] == 'Pending':
                                                        ui.icon('schedule', size='sm').classes('text-yellow-400 mr-2')
                                                        ui.label('Starting...').classes('text-yellow-300 font-semibold')
                                                    else:
                                                        # Failed, Error, or other states
                                                        ui.icon('error', size='sm').classes('text-red-400 mr-2')
                                                        ui.label('Not Running').classes('text-red-300 font-semibold')
                    else:
                        # Empty state
                        with ui.element('div').classes('text-center py-16'):
                            ui.icon('inbox', size='4xl').classes('text-gray-600 mb-4')
                            ui.label('No pods found').classes('text-2xl text-gray-400 mb-2')
                            ui.label(f'The "{self.selected_namespace}" namespace contains no pods').classes('text-gray-500')
        
        dialog.open()
    
    def refresh_pod_resources(self, pod):
        """Refresh resource usage for a specific pod"""
        try:
            ram_usage, cpu_usage = self.k3s_manager.get_resource_usage(self.selected_namespace, pod['name'])
            pod['ram_usage'] = ram_usage
            pod['cpu_usage'] = cpu_usage

            print(f"Refreshed Pod: {pod['name']}, RAM: {ram_usage:.2f} MB, CPU: {cpu_usage:.2f} mCPU")
            
            # Update CPU label with new value and color
            if 'cpu_label' in pod:
                cpu_color = 'text-green-300' if cpu_usage < 50 else 'text-yellow-300' if cpu_usage < 80 else 'text-red-300'
                pod['cpu_label'].text = f"{cpu_usage:.1f} mCPU"
                pod['cpu_label'].classes(replace=f'{cpu_color} font-mono')
            
            # Update RAM label with new value and color
            if 'ram_label' in pod:
                ram_color = 'text-green-300' if ram_usage < 100 else 'text-yellow-300' if ram_usage < 500 else 'text-red-300'
                pod['ram_label'].text = f"{ram_usage:.1f} MB"
                pod['ram_label'].classes(replace=f'{ram_color} font-mono')
            
            ui.notify(f"Resource usage updated for {pod['name']}", type='positive')
            
        except Exception as e:
            ui.notify(f"Failed to refresh resources: {str(e)}", type='negative')
            print(f"Error refreshing resources for pod {pod['name']}: {e}")


def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py, but it's empty to enable NiceGUI auto-reloading
    pass


def ros_main() -> None:
    rclpy.init()
    node = NiceGuiNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


app.on_startup(lambda: threading.Thread(target=ros_main).start())
ui_run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name, so we need to specify it here
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖')
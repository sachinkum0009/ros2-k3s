"""
Copyright 2025 Sachin
Author: Sachin
License: Apache 2.0
"""
import unittest
from ros2_k3s_cli.cli import K3SCLI

class TestK3SCLI(unittest.TestCase):
    def setUp(self):
        self.cli = K3SCLI()

    def test_init_requires_config(self):
        with self.assertRaises(SystemExit) as cm:
            self.cli.run(['init'])
        self.assertNotEqual(cm.exception.code, 0)

    def test_init_success(self):
        result = self.cli.run(['init', '--config', 'cluster.yaml'])
        self.assertEqual(result, 0)

    def test_validate_requires_config(self):
        with self.assertRaises(SystemExit) as cm:
            self.cli.run(['validate'])
        self.assertNotEqual(cm.exception.code, 0)

    def test_validate_success(self):
        result = self.cli.run(['validate', '--config', 'cluster.yaml'])
        self.assertEqual(result, 0)

    def test_deploy_requires_app_and_config(self):
        with self.assertRaises(SystemExit) as cm:
            self.cli.run(['deploy'])
        self.assertNotEqual(cm.exception.code, 0)
        with self.assertRaises(SystemExit) as cm:
            self.cli.run(['deploy', '--app', 'myapp'])
        self.assertNotEqual(cm.exception.code, 0)
        with self.assertRaises(SystemExit) as cm:
            self.cli.run(['deploy', '--config', 'deploy.yaml'])
        self.assertNotEqual(cm.exception.code, 0)

    def test_deploy_success(self):
        result = self.cli.run(['deploy', '--app', 'myapp', '--config', 'deploy.yaml'])
        self.assertEqual(result, 0)

if __name__ == '__main__':
    unittest.main()

"""
Copyright 2025 Sachin
Author: Sachin
License: Apache 2.0
"""
import pytest
from ros2_k3s_cli.cli import K3SCLI

@pytest.fixture
def cli():
    return K3SCLI()

def test_init_requires_config(cli):
    with pytest.raises(SystemExit) as cm:
        cli.run(['init'])
    assert cm.value.code != 0

def test_init_success(cli):
    result = cli.run(['init', '--config', 'cluster.yaml'])
    assert result == 0

def test_validate_requires_config(cli):
    with pytest.raises(SystemExit) as cm:
        cli.run(['validate'])
    assert cm.value.code != 0

def test_validate_success(cli):
    result = cli.run(['validate', '--config', 'cluster.yaml'])
    assert result == 0

def test_deploy_requires_app_and_config(cli):
    with pytest.raises(SystemExit) as cm:
        cli.run(['deploy'])
    assert cm.value.code != 0
    with pytest.raises(SystemExit) as cm:
        cli.run(['deploy', '--app', 'myapp'])
    assert cm.value.code != 0
    with pytest.raises(SystemExit) as cm:
        cli.run(['deploy', '--config', 'deploy.yaml'])
    assert cm.value.code != 0

def test_deploy_success(cli):
    result = cli.run(['deploy', '--app', 'myapp', '--config', 'deploy.yaml'])
    assert result == 0

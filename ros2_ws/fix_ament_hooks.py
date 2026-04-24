#!/usr/bin/env python3
"""Corrige os hooks ament_prefix_path para pacotes Python que o colcon não gera."""
from pathlib import Path
from colcon_core.shell import create_environment_hook

ws = Path(__file__).parent

for pkg in ['fleet_orchestrator', 'fleet_data_collector']:
    install = ws / 'install' / pkg
    if not install.exists():
        print(f'{pkg}: install dir não encontrado, pulando')
        continue

    hooks = create_environment_hook('ament_prefix_path', install, pkg, 'AMENT_PREFIX_PATH', '', mode='prepend')

    dsv_path = install / f'share/{pkg}/package.dsv'
    if not dsv_path.exists():
        print(f'{pkg}: package.dsv não encontrado')
        continue

    content = dsv_path.read_text()
    if 'ament_prefix_path' not in content:
        dsv_path.write_text(
            f'source;share/{pkg}/hook/ament_prefix_path.dsv\n'
            f'source;share/{pkg}/hook/ament_prefix_path.sh\n'
            + content
        )
        print(f'{pkg}: ament_prefix_path adicionado ao package.dsv')
    else:
        print(f'{pkg}: já ok')

print('Pronto.')

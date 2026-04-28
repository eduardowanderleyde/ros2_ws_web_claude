#!/bin/bash
# Corrige os hooks ament_prefix_path em pacotes Python que o colcon não gera automaticamente.
# Corre após qualquer colcon build que inclua fleet_orchestrator ou fleet_data_collector.
set -e
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

for PKG in fleet_orchestrator fleet_data_collector; do
    HOOK_DIR="$WS/install/$PKG/share/$PKG/hook"
    DSV="$WS/install/$PKG/share/$PKG/package.dsv"

    mkdir -p "$HOOK_DIR"

    # Cria hook files se não existirem
    if [ ! -f "$HOOK_DIR/ament_prefix_path.sh" ]; then
        cat > "$HOOK_DIR/ament_prefix_path.sh" << 'EOF'
# generated from colcon_core/shell/template/hook_prepend_value.sh.em
_colcon_prepend_unique_value AMENT_PREFIX_PATH "$COLCON_CURRENT_PREFIX"
EOF
    fi
    if [ ! -f "$HOOK_DIR/ament_prefix_path.dsv" ]; then
        echo "prepend-non-duplicate;AMENT_PREFIX_PATH;" > "$HOOK_DIR/ament_prefix_path.dsv"
    fi

    # Adiciona as linhas ao package.dsv se ainda não estiverem lá
    if ! grep -q "ament_prefix_path" "$DSV" 2>/dev/null; then
        sed -i "s|source;share/$PKG/hook/pythonpath.sh|source;share/$PKG/hook/pythonpath.sh\nsource;share/$PKG/hook/ament_prefix_path.dsv\nsource;share/$PKG/hook/ament_prefix_path.sh|" "$DSV"
        echo "[fix_ament_hooks] $PKG: hook adicionado"
    else
        echo "[fix_ament_hooks] $PKG: hook já presente"
    fi
done

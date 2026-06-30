#!/bin/bash

# Restore original environments if running inside VS Code snap terminal
for var in LD_LIBRARY_PATH GTK_PATH QT_PLUGIN_PATH XDG_DATA_DIRS PATH; do
    orig_var="${var}_VSCODE_SNAP_ORIG"
    if [ -n "${!orig_var}" ]; then
        export "$var"="${!orig_var}"
    else
        if [ "$var" != "PATH" ]; then
            unset "$var"
        fi
    fi
done

# Also unset GTK-specific snap variables that can cause crashes
unset GTK_IM_MODULE_FILE GTK_EXE_PREFIX

export QT_NO_ACCESSIBILITY=1
export LD_LIBRARY_PATH="/home/grace/choreonoid_install/lib:${LD_LIBRARY_PATH:-}"

PROJECT_DIR="$(cd "$(dirname "$0")/project" && pwd)"
CHOREONOID_BIN="/home/grace/choreonoid_install/bin/choreonoid"

# Build controllers first
echo "Checking controller build..."
mkdir -p "${PROJECT_DIR}/controller/build"
cd "${PROJECT_DIR}/controller/build"
cmake -DCMAKE_PREFIX_PATH=/home/grace/choreonoid_install ..
make -j$(nproc)

cd "${PROJECT_DIR}"
echo "Starting simulation world_jalan_lari.cnoid..."
gdb -ex "run" -ex "bt" --batch --args "$CHOREONOID_BIN" world_jalan_lari.cnoid "$@" > "${PROJECT_DIR}/gdb_crash.log" 2>&1

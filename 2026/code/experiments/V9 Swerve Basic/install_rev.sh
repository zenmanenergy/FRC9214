#!/bin/bash
# Install robotpy-rev on RoboRIO

PYBIN="/usr/local/frc/3.14/bin/python3.14"

if [ -f "$PYBIN" ]; then
    $PYBIN -m pip install -I robotpy-rev==2026.0.2
    echo "robotpy-rev installed successfully"
else
    echo "Python not found at $PYBIN"
    # Try alternative paths
    for py in /usr/local/frc/*/bin/python* /opt/*/bin/python*; do
        if [ -f "$py" ]; then
            echo "Found Python at: $py"
            $py -m pip install -I robotpy-rev==2026.0.2
            break
        fi
    done
fi

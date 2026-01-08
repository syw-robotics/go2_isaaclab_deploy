#!/bin/bash
# Installation script for Go2 Isaac Lab Deploy Python

echo "================================================"
echo "Go2 Isaac Lab Deploy - Python Installation"
echo "================================================"
echo ""

# Check Python version
echo "Checking Python version..."
PYTHON_VERSION=$(python3 --version 2>&1 | awk '{print $2}')
echo "Python version: $PYTHON_VERSION"

REQUIRED_VERSION="3.8"
if [ "$(printf '%s\n' "$REQUIRED_VERSION" "$PYTHON_VERSION" | sort -V | head -n1)" != "$REQUIRED_VERSION" ]; then
    echo "Error: Python 3.8 or higher is required"
    exit 1
fi

echo "✓ Python version OK"
echo ""

# Install dependencies
echo "Installing Python dependencies..."
python3 -m pip install --upgrade pip --user
python3 -m pip install -r requirements.txt --user

# Verify installation
echo ""
echo "Verifying installation..."

# Check numpy
if python3 -c "import numpy" 2>/dev/null; then
    echo "✓ numpy installed"
else
    echo "✗ numpy failed to install"
    exit 1
fi

# Check pyyaml
if python3 -c "import yaml" 2>/dev/null; then
    echo "✓ pyyaml installed"
else
    echo "✗ pyyaml failed to install"
    exit 1
fi

# Check onnxruntime
if python3 -c "import onnxruntime" 2>/dev/null; then
    ONNX_VERSION=$(python3 -c "import onnxruntime; print(onnxruntime.__version__)")
    echo "✓ onnxruntime installed (version: $ONNX_VERSION)"
else
    echo "✗ onnxruntime failed to install"
    exit 1
fi

# Check unitree_sdk2py
if python3 -c "import unitree_sdk2py" 2>/dev/null; then
    echo "✓ unitree_sdk2py found"
else
    echo "⚠ unitree_sdk2py not found - please install Unitree SDK2 Python"
    echo "  Expected location: unitree_sdk2_python"
fi

# Verify project modules
echo ""
echo "Verifying project modules..."
if python3 -c "from fsm.ctrl_fsm import CtrlFSM; from isaaclab.envs import ManagerBasedRLEnv" 2>/dev/null; then
    echo "✓ All project modules can be imported"
else
    echo "✗ Failed to import project modules"
    exit 1
fi

echo ""
echo "================================================"
echo "Installation completed successfully!"
echo "================================================"
echo ""
echo "Usage:"
echo "  python3 main.py --network eth0"
echo "  ./run.sh --network eth0 --policy_dir loco_lab"
echo ""
echo "For more information, see README.md"

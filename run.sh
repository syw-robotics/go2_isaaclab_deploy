#!/bin/bash
# Run script for Go2 Isaac Lab Deploy Python

# Default values
NETWORK="eth0"
POLICY_DIR="unitree_rl_lab"
POLICY_NAME="policy.onnx"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --network)
            NETWORK="$2"
            shift 2
            ;;
        --policy_dir)
            POLICY_DIR="$2"
            shift 2
            ;;
        --policy_name)
            POLICY_NAME="$2"
            shift 2
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --network      Network interface (default: eth0)"
            echo "  --policy_dir   Policy directory (default: unitree_rl_lab)"
            echo "  --policy_name  Policy file name (default: policy.onnx)"
            echo "  --help         Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Run the controller
python3 main.py \
    --network "$NETWORK" \
    --policy_dir "$POLICY_DIR" \
    --policy_name "$POLICY_NAME" \

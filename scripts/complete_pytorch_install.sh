#!/bin/bash
# Complete PyTorch Installation for Jetson
# Use this if fix_pytorch_jetson.sh got interrupted

set -e

echo "======================================================================"
echo "COMPLETING JETSON PYTORCH INSTALLATION"
echo "======================================================================"

# Check virtual environment
if [ -z "$VIRTUAL_ENV" ]; then
    echo "⚠ Activate virtual environment first: source ~/ai/bin/activate"
    exit 1
fi

cd /tmp

# Download PyTorch 2.3.0 for JetPack 6.0
TORCH_WHEEL="torch-2.3.0-cp310-cp310-linux_aarch64.whl"
TORCH_URL="https://developer.download.nvidia.com/compute/redist/jp/v60/pytorch/${TORCH_WHEEL}"

echo ""
echo "[1/3] Downloading PyTorch for Jetson Orin..."
if [ ! -f "$TORCH_WHEEL" ]; then
    wget --no-check-certificate -q --show-progress "$TORCH_URL"
else
    echo "✓ Already downloaded"
fi

echo ""
echo "[2/3] Installing PyTorch..."
pip3 install "$TORCH_WHEEL"
echo "✓ PyTorch installed"

echo ""
echo "[3/3] Installing TorchVision..."
pip3 install torchvision==0.18.0
echo "✓ TorchVision installed"

# Cleanup
rm -f "$TORCH_WHEEL"

echo ""
echo "======================================================================"
echo "VERIFYING INSTALLATION"
echo "======================================================================"

python3 << 'EOF'
import torch
import sys

print(f"PyTorch version: {torch.__version__}")
print(f"CUDA available: {torch.cuda.is_available()}")

if torch.cuda.is_available():
    print(f"CUDA version: {torch.version.cuda}")
    print(f"GPU: {torch.cuda.get_device_name(0)}")
    
    # Critical test
    try:
        x = torch.randn(100, 100).cuda()
        y = x @ x
        print("")
        print("✓✓✓ SUCCESS! GPU IS WORKING! ✓✓✓")
        print("")
        sys.exit(0)
    except RuntimeError as e:
        if "no kernel image" in str(e):
            print("✗ Still getting 'no kernel image' error")
            sys.exit(1)
        else:
            raise
else:
    print("✗ CUDA not available")
    sys.exit(1)
EOF

if [ $? -eq 0 ]; then
    echo "======================================================================"
    echo "Installation successful!"
    echo ""
    echo "Next: python3 yolo_gpu_optimized.py"
    echo "Expected: 15-25 FPS with GPU acceleration"
    echo "======================================================================"
fi

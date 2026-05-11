#!/bin/bash
# Fix PyTorch for Jetson Orin (JetPack 6.0 / R36)
# The current PyTorch 2.5.1 is NOT compatible with Jetson!

set -e

echo "======================================================================"
echo "FIXING PYTORCH FOR JETSON ORIN"
echo "======================================================================"
echo ""
echo "Problem: PyTorch 2.5.1 is x86_64 build, not compatible with Jetson ARM"
echo "Solution: Install NVIDIA's official Jetson PyTorch build"
echo ""

# Check virtual environment
if [ -z "$VIRTUAL_ENV" ]; then
    echo "⚠ Not in virtual environment!"
    echo "Run: source ~/ai/bin/activate"
    exit 1
fi

echo "✓ Virtual environment: $VIRTUAL_ENV"
echo ""

# Detect JetPack
if [ -f /etc/nv_tegra_release ]; then
    JETPACK_VERSION=$(cat /etc/nv_tegra_release | grep -oP 'R\d+')
    echo "Detected JetPack: $JETPACK_VERSION"
else
    echo "Could not detect JetPack version"
    JETPACK_VERSION="R36"
fi

echo ""
echo "[1/4] Removing incompatible PyTorch..."
pip3 uninstall -y torch torchvision torchaudio
echo "✓ Removed"

echo ""
echo "[2/4] Installing dependencies..."
pip3 install numpy==1.26.4
echo "✓ Dependencies installed"

echo ""
echo "[3/4] Installing Jetson-specific PyTorch..."
echo "This will take a few minutes..."
echo ""

if [[ "$JETPACK_VERSION" == "R36" ]]; then
    # JetPack 6.0 (R36) - your version
    echo "Installing PyTorch 2.3.0 for JetPack 6.0..."
    
    # Download and install from NVIDIA
    cd /tmp
    
    # PyTorch wheel for JetPack 6.0
    TORCH_WHEEL="torch-2.3.0-cp310-cp310-linux_aarch64.whl"
    TORCH_URL="https://developer.download.nvidia.com/compute/redist/jp/v60/pytorch/${TORCH_WHEEL}"
    
    echo "Downloading from NVIDIA..."
    wget -q --show-progress "$TORCH_URL" -O "$TORCH_WHEEL"
    
    echo "Installing PyTorch..."
    pip3 install "$TORCH_WHEEL"
    
    echo "Installing TorchVision..."
    pip3 install torchvision==0.18.0
    
    rm -f "$TORCH_WHEEL"
    cd -
else
    # Fallback for other versions
    echo "Installing PyTorch for JetPack 5.x..."
    pip3 install --no-cache \
        torch torchvision \
        --index-url https://developer.download.nvidia.com/compute/redist/jp/v511
fi

echo "✓ PyTorch installed"

echo ""
echo "[4/4] Verifying installation..."
python3 << 'END'
import torch
import sys

print(f"PyTorch version: {torch.__version__}")
print(f"PyTorch file: {torch.__file__}")
print(f"CUDA available: {torch.cuda.is_available()}")

if torch.cuda.is_available():
    print(f"CUDA version: {torch.version.cuda}")
    print(f"GPU: {torch.cuda.get_device_name(0)}")
    
    # Critical test - this was failing before
    try:
        x = torch.randn(100, 100).cuda()
        y = x @ x  # Matrix multiplication on GPU
        print(f"✓✓✓ GPU tensor operations WORKING! ✓✓✓")
        sys.exit(0)
    except RuntimeError as e:
        if "no kernel image" in str(e):
            print(f"✗ Still getting 'no kernel image' error")
            print(f"✗ This PyTorch build is NOT compatible with Jetson")
            sys.exit(1)
        else:
            raise
else:
    print("✗ CUDA not available")
    sys.exit(1)
END

if [ $? -eq 0 ]; then
    echo ""
    echo "======================================================================"
    echo "SUCCESS! PyTorch is now working with GPU!"
    echo "======================================================================"
    echo ""
    echo "Next steps:"
    echo "  python3 jetson_cuda_diagnostic.py  # Should pass all tests"
    echo "  python3 yolo_gpu_optimized.py      # Should work with GPU"
    echo ""
    echo "Expected performance: 15-25 FPS with GPU acceleration"
    echo "======================================================================"
else
    echo ""
    echo "======================================================================"
    echo "Installation completed but GPU test failed"
    echo "======================================================================"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Check JetPack version: cat /etc/nv_tegra_release"
    echo "  2. Try different PyTorch version from:"
    echo "     https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048"
    echo "======================================================================"
fi

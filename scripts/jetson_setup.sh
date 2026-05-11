#!/bin/bash
# Jetson Nano Super - Complete Setup Script
# Run with: bash jetson_setup.sh

set -e  # Exit on error

echo "======================================================================"
echo "Jetson Nano Super - GPU-Accelerated YOLO Setup"
echo "======================================================================"

# Check if in virtual environment
if [ -z "$VIRTUAL_ENV" ]; then
    echo "⚠ Not in virtual environment. Activate it first:"
    echo "  source ai/bin/activate"
    exit 1
fi

echo ""
echo "✓ Virtual environment detected: $VIRTUAL_ENV"
echo ""

# Step 1: Setup CUDA environment
echo "[1/6] Setting up CUDA environment variables..."

# Add to current session
export CUDA_HOME=/usr/local/cuda
export PATH=$PATH:$CUDA_HOME/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CUDA_HOME/lib64

echo "✓ CUDA environment set for current session"
echo ""

# Ask to add to bashrc
read -p "Add CUDA to ~/.bashrc permanently? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    if ! grep -q "CUDA_HOME" ~/.bashrc; then
        echo "" >> ~/.bashrc
        echo "# CUDA Environment for Jetson" >> ~/.bashrc
        echo "export CUDA_HOME=/usr/local/cuda" >> ~/.bashrc
        echo "export PATH=\$PATH:\$CUDA_HOME/bin" >> ~/.bashrc
        echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:\$CUDA_HOME/lib64" >> ~/.bashrc
        echo "✓ Added CUDA to ~/.bashrc"
    else
        echo "✓ CUDA already in ~/.bashrc"
    fi
fi

# Step 2: Check JetPack version
echo ""
echo "[2/6] Detecting JetPack version..."
if [ -f /etc/nv_tegra_release ]; then
    JETPACK_VERSION=$(cat /etc/nv_tegra_release | grep -oP 'R\d+')
    echo "✓ JetPack: $JETPACK_VERSION"
    
    if [[ "$JETPACK_VERSION" == "R36" ]]; then
        PYTORCH_URL="https://developer.download.nvidia.com/compute/redist/jp/v60"
        echo "  Using PyTorch for JetPack 6.0"
    elif [[ "$JETPACK_VERSION" == "R35" ]]; then
        PYTORCH_URL="https://developer.download.nvidia.com/compute/redist/jp/v511"
        echo "  Using PyTorch for JetPack 5.1.1"
    else
        PYTORCH_URL="https://developer.download.nvidia.com/compute/redist/jp/v511"
        echo "  Using PyTorch for JetPack 5.x (default)"
    fi
else
    echo "⚠ Could not detect JetPack version, using default"
    PYTORCH_URL="https://developer.download.nvidia.com/compute/redist/jp/v511"
fi

# Step 3: Install system dependencies
echo ""
echo "[3/6] Installing system dependencies..."
sudo apt-get update
sudo apt-get install -y python3-pip python3-dev python3-setuptools
sudo apt-get install -y libjpeg-dev zlib1g-dev libpython3-dev
sudo apt-get install -y libopenblas-dev libopenmpi-dev libomp-dev
echo "✓ System dependencies installed"

# Step 4: Remove old PyTorch if exists
echo ""
echo "[4/6] Cleaning old PyTorch installation..."
pip3 uninstall -y torch torchvision torchaudio || true
echo "✓ Cleaned"

# Step 5: Install PyTorch with CUDA support
echo ""
echo "[5/6] Installing PyTorch with CUDA support..."
echo "  This may take several minutes..."

# For JetPack 6.0 (R36)
if [[ "$JETPACK_VERSION" == "R36" ]]; then
    echo "  Installing PyTorch for JetPack 6.0..."
    pip3 install --no-cache-dir \
        torch torchvision torchaudio \
        --index-url https://download.pytorch.org/whl/cu124
else
    # For JetPack 5.x
    echo "  Installing PyTorch for JetPack 5.x..."
    pip3 install --no-cache-dir \
        numpy==1.26.4
    
    pip3 install --no-cache-dir \
        torch torchvision \
        --index-url $PYTORCH_URL
fi

echo "✓ PyTorch installed"

# Step 6: Install other dependencies
echo ""
echo "[6/6] Installing additional dependencies..."
pip3 install --no-cache-dir ultralytics opencv-python

echo "✓ All dependencies installed"

# Verification
echo ""
echo "======================================================================"
echo "VERIFICATION"
echo "======================================================================"

echo ""
echo "Testing PyTorch CUDA support..."
python3 << END
import torch
print(f"PyTorch version: {torch.__version__}")
print(f"CUDA available: {torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"CUDA version: {torch.version.cuda}")
    print(f"GPU: {torch.cuda.get_device_name(0)}")
    print(f"✓✓✓ SUCCESS! GPU is ready! ✓✓✓")
else:
    print("✗ CUDA not available - installation may have failed")
END

echo ""
echo "======================================================================"
echo "SETUP COMPLETE!"
echo "======================================================================"
echo ""
echo "Next steps:"
echo "  1. Test GPU: python3 jetson_cuda_diagnostic.py"
echo "  2. Run detection: python3 yolo_gpu_optimized.py"
echo ""
echo "Expected performance: 15-25 FPS (vs 3-6 FPS on CPU)"
echo "======================================================================"

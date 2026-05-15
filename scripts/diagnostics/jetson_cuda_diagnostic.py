#!/usr/bin/env python3
"""
Jetson CUDA Diagnostic Script
Checks CUDA installation and PyTorch GPU support
"""

import os
import pathlib
import subprocess
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))
import config as cfg


def main() -> int:
    print("="*70)
    print("JETSON CUDA & GPU DIAGNOSTIC")
    print("="*70)

    # ==================== System Info ====================
    print("\n[1/7] System Information")
    try:
        with open('/etc/nv_tegra_release', 'r') as f:
            jetpack_info = f.read().strip()
            print(f"  Jetpack: {jetpack_info}")
    except Exception:
        print("  ⚠ Could not read Jetpack version")

    try:
        result = subprocess.run(['uname', '-a'], capture_output=True, text=True)
        print(f"  Kernel: {result.stdout.strip()}")
    except Exception:
        pass

    # ==================== NVIDIA Driver ====================
    print("\n[2/7] NVIDIA Driver & GPU")
    try:
        result = subprocess.run(['nvidia-smi'], capture_output=True, text=True)
        if result.returncode == 0:
            print("  ✓ nvidia-smi working:")
            lines = result.stdout.split('\n')[:10]
            for line in lines:
                if line.strip():
                    print(f"    {line}")
        else:
            print("  ✗ nvidia-smi failed")
            print(f"    Error: {result.stderr}")
    except FileNotFoundError:
        print("  ✗ nvidia-smi not found")
        print("    This is normal for Jetson, trying tegrastats instead...")

        try:
            result = subprocess.run(['tegrastats', '--interval', '1000', '--stop'],
                                  capture_output=True, text=True, timeout=2)
            print("  ✓ Tegrastats working (Jetson GPU detected)")
        except Exception:
            print("  ⚠ Could not run tegrastats")

    # ==================== CUDA Compiler ====================
    print("\n[3/7] CUDA Compiler (nvcc)")
    try:
        result = subprocess.run(['nvcc', '--version'], capture_output=True, text=True)
        if result.returncode == 0:
            print("  ✓ CUDA Compiler found:")
            for line in result.stdout.split('\n'):
                if 'release' in line.lower():
                    print(f"    {line.strip()}")
        else:
            print("  ✗ nvcc not working")
    except FileNotFoundError:
        print("  ✗ nvcc not found in PATH")
        print("    Check: /usr/local/cuda/bin/nvcc")

        try:
            result = subprocess.run(['/usr/local/cuda/bin/nvcc', '--version'],
                                  capture_output=True, text=True)
            if result.returncode == 0:
                print("  ✓ Found at /usr/local/cuda/bin/nvcc")
                print("    You need to add CUDA to PATH!")
        except Exception:
            print("  ✗ CUDA not found at /usr/local/cuda/bin/nvcc")

    # ==================== CUDA Environment ====================
    print("\n[4/7] CUDA Environment Variables")

    cuda_vars = {
        'CUDA_HOME': os.environ.get('CUDA_HOME'),
        'CUDA_PATH': os.environ.get('CUDA_PATH'),
        'LD_LIBRARY_PATH': os.environ.get('LD_LIBRARY_PATH'),
        'PATH': os.environ.get('PATH')
    }

    for var, value in cuda_vars.items():
        if value and 'cuda' in value.lower():
            print(f"  ✓ {var}: {value[:100]}...")
        elif value and var == 'PATH':
            if '/usr/local/cuda' in value:
                print(f"  ✓ {var} includes CUDA")
            else:
                print(f"  ⚠ {var} does NOT include CUDA")
        else:
            print(f"  ✗ {var}: Not set")

    # ==================== PyTorch ====================
    print("\n[5/7] PyTorch Installation")
    cuda_available = False
    try:
        import torch
        print(f"  ✓ PyTorch version: {torch.__version__}")
        print(f"  ✓ PyTorch location: {torch.__file__}")

        cuda_available = torch.cuda.is_available()
        print(f"  CUDA Available: {cuda_available}")

        if cuda_available:
            print(f"  ✓ CUDA Device Count: {torch.cuda.device_count()}")
            print(f"  ✓ CUDA Device Name: {torch.cuda.get_device_name(0)}")
            print(f"  ✓ CUDA Version (PyTorch): {torch.version.cuda}")

            try:
                x = torch.randn(100, 100).cuda()
                y = torch.randn(100, 100).cuda()
                _ = x + y
                print(f"  ✓ CUDA tensor operations working!")
            except Exception as e:
                print(f"  ✗ CUDA tensor test failed: {e}")
        else:
            print(f"  ✗ CUDA NOT AVAILABLE in PyTorch!")
            print(f"  ⚠ PyTorch was compiled WITHOUT CUDA support")
            print(f"  ⚠ You need Jetson-specific PyTorch wheel!")

            print(f"  cuDNN available: {torch.backends.cudnn.is_available()}")
            print(f"  cuDNN enabled: {torch.backends.cudnn.enabled}")

    except ImportError:
        print("  ✗ PyTorch not installed")
    except Exception as e:
        print(f"  ✗ PyTorch error: {e}")

    # ==================== TorchVision ====================
    print("\n[6/7] TorchVision")
    try:
        import torchvision
        print(f"  ✓ TorchVision version: {torchvision.__version__}")
    except ImportError:
        print("  ✗ TorchVision not installed")

    # ==================== Ultralytics ====================
    print("\n[7/7] Ultralytics YOLO")
    try:
        from ultralytics import YOLO
        import ultralytics
        print(f"  ✓ Ultralytics version: {ultralytics.__version__}")

        try:
            import torch as _torch
            model = YOLO(cfg.MODEL_PATH)
            print(f"  ✓ Model loaded")

            if hasattr(model, 'device'):
                print(f"  Model device: {model.device}")

            if _torch.cuda.is_available():
                try:
                    model.to('cuda:0')
                    print(f"  ✓ Model successfully moved to CUDA!")
                except Exception as e:
                    print(f"  ✗ Failed to move model to CUDA: {e}")
        except Exception as e:
            print(f"  ⚠ Model test skipped: {e}")

    except ImportError:
        print("  ✗ Ultralytics not installed")
    except Exception as e:
        print(f"  ✗ Ultralytics error: {e}")

    # ==================== Summary & Recommendations ====================
    print("\n" + "="*70)
    print("DIAGNOSIS & RECOMMENDATIONS")
    print("="*70)

    recommendations = []

    try:
        import torch as _torch
        if not _torch.cuda.is_available():
            recommendations.append("""
✗ CRITICAL: PyTorch has NO CUDA support!

SOLUTION - Install Jetson-specific PyTorch:
1. Uninstall current PyTorch:
   pip3 uninstall torch torchvision

2. Install Jetson PyTorch (choose your JetPack version):

   For JetPack 5.x (recommended):
   wget https://developer.download.nvidia.com/compute/redist/jp/v511/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl
   pip3 install torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl --break-system-packages

   Or use NVIDIA's index:
   pip3 install --no-cache-dir torch torchvision --index-url https://developer.download.nvidia.com/compute/redist/jp/v511 --break-system-packages

3. Verify:
   python3 -c "import torch; print('CUDA:', torch.cuda.is_available())"
""")
        else:
            recommendations.append("✓ PyTorch CUDA support is working!")
    except Exception:
        pass

    if not os.environ.get('CUDA_HOME') and not os.environ.get('CUDA_PATH'):
        recommendations.append("""
⚠ CUDA environment variables not set

Add to ~/.bashrc:
export CUDA_HOME=/usr/local/cuda
export PATH=$PATH:$CUDA_HOME/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CUDA_HOME/lib64

Then run: source ~/.bashrc
""")

    if recommendations:
        for rec in recommendations:
            print(rec)
    else:
        print("\n✓ All checks passed! Your Jetson is ready for GPU-accelerated inference.")
        print("\nExpected performance with GPU:")
        print("  - 1920x1080 → 640x480: ~15-20 FPS")
        print("  - With TensorRT: ~25-35 FPS")

    print("="*70)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

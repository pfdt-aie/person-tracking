#!/usr/bin/env python3
"""
System Diagnostic Script
Verifies all components before running tracking system
"""

import os
import pathlib
import socket
import struct
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))
import config as cfg


def main() -> int:
    print("="*70)
    print("YOLO11 + ZR30 System Diagnostic")
    print("="*70)

    # ==================== 1. Check Python Packages ====================
    print("\n[1/6] Checking Python packages...")
    required_packages = {
        'cv2': 'opencv-python',
        'numpy': 'numpy',
        'ultralytics': 'ultralytics'
    }

    missing_packages = []
    for module, package in required_packages.items():
        try:
            __import__(module)
            print(f"  ✓ {package}")
        except ImportError:
            print(f"  ✗ {package} - NOT INSTALLED")
            missing_packages.append(package)

    if missing_packages:
        print(f"\n⚠ Missing packages: {', '.join(missing_packages)}")
        print(f"Install with: pip3 install {' '.join(missing_packages)} --break-system-packages")
    else:
        print("  All packages installed!")

    # ==================== 2. Check YOLO Model ====================
    print("\n[2/6] Checking YOLO model...")
    model_path = cfg.MODEL_PATH

    if os.path.exists(model_path):
        print(f"  ✓ Model found: {model_path}")
        file_size = os.path.getsize(model_path) / (1024 * 1024)  # MB
        print(f"    Size: {file_size:.2f} MB")

        try:
            from ultralytics import YOLO
            print("  ✓ Attempting to load model...")
            model = YOLO(model_path)
            print(f"  ✓ Model loaded successfully!")
            print(f"    Model type: {type(model)}")
        except Exception as e:
            print(f"  ✗ Model loading failed: {e}")
    else:
        print(f"  ✗ Model NOT found at: {model_path}")
        print("    Please verify the model path!")

    # ==================== 3. Check Camera Network ====================
    print("\n[3/6] Checking camera network connectivity...")
    camera_ip = cfg.CAMERA_IP

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2)
        result = sock.connect_ex((camera_ip, cfg.GIMBAL_PORT))
        sock.close()

        if result == 0:
            print(f"  ✓ Camera reachable at {camera_ip}:{cfg.GIMBAL_PORT}")
        else:
            print(f"  ⚠ Camera port 37260 not responding (may be normal)")
            print(f"    Trying basic connectivity...")
            os.system(f"ping -c 1 -W 2 {camera_ip} > /dev/null 2>&1")

    except Exception as e:
        print(f"  ✗ Network error: {e}")

    # ==================== 4. Test Gimbal Control ====================
    print("\n[4/6] Testing gimbal control protocol...")

    def crc16_ccitt(data: bytes) -> int:
        crc = 0
        for byte in data:
            crc ^= (byte << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
                crc &= 0xFFFF
        return crc

    def build_packet(cmd_id: int, data: bytes = b'', seq: int = 0) -> bytes:
        stx = b'\x55\x66'
        ctrl = b'\x01'
        data_len = struct.pack("<H", len(data))
        sequence = struct.pack("<H", seq)
        cmd = struct.pack("B", cmd_id)

        packet_header = stx + ctrl + data_len + sequence + cmd
        payload = packet_header + data

        crc = crc16_ccitt(payload)
        return payload + struct.pack("<H", crc)

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(2.0)

        pkt = build_packet(0x00, b'', 0)
        sock.sendto(pkt, (camera_ip, cfg.GIMBAL_PORT))

        try:
            reply, addr = sock.recvfrom(1024)
            print(f"  ✓ Gimbal responding!")
            print(f"    Response length: {len(reply)} bytes")
            print(f"    Response: {' '.join(f'{b:02X}' for b in reply[:20])}...")
        except socket.timeout:
            print(f"  ⚠ No response from gimbal (this may be normal)")
            print(f"    The gimbal might still work during tracking")

        sock.close()

    except Exception as e:
        print(f"  ✗ Gimbal test error: {e}")

    # ==================== 5. Check RTSP Stream ====================
    print("\n[5/6] Checking RTSP stream...")
    rtsp_url = cfg.RTSP_URL

    try:
        import cv2
        print(f"  Testing: {rtsp_url}")

        cap = cv2.VideoCapture(rtsp_url)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                h, w = frame.shape[:2]
                print(f"  ✓ RTSP stream working!")
                print(f"    Resolution: {w}x{h}")
                print(f"    Frame shape: {frame.shape}")
            else:
                print(f"  ⚠ Stream opened but no frame received")
            cap.release()
        else:
            print(f"  ✗ Failed to open RTSP stream")
            print(f"    Try these alternatives:")
            print(f"      - rtsp://{camera_ip}:554/main.264")
            print(f"      - rtsp://{camera_ip}/main.264")
            print(f"      - rtsp://{camera_ip}:8554/stream1")

    except Exception as e:
        print(f"  ✗ RTSP test error: {e}")

    # ==================== 6. System Resources ====================
    print("\n[6/6] Checking system resources...")

    try:
        with open('/proc/meminfo', 'r') as f:
            for line in f:
                if 'MemTotal' in line:
                    total_ram = int(line.split()[1]) / 1024  # MB
                    print(f"  RAM: {total_ram:.0f} MB")
                elif 'MemAvailable' in line:
                    avail_ram = int(line.split()[1]) / 1024  # MB
                    print(f"  Available RAM: {avail_ram:.0f} MB")

        import platform
        print(f"  Platform: {platform.machine()}")
        print(f"  Python: {sys.version.split()[0]}")

    except Exception as e:
        print(f"  ⚠ Could not check system resources: {e}")

    # ==================== Summary ====================
    print("\n" + "="*70)
    print("Diagnostic Summary")
    print("="*70)

    if not missing_packages and os.path.exists(model_path):
        print("✓ System looks good! You can proceed with testing.")
        print("\nNext steps:")
        print("  1. Run simple test: python3 utils/yolo_test_simple.py")
        print("  2. Run full tracker: python3 main.py")
    else:
        print("⚠ Issues detected. Please fix the above errors before proceeding.")
        if missing_packages:
            print(f"\n  Install missing packages:")
            print(f"  pip3 install {' '.join(missing_packages)} --break-system-packages")
        if not os.path.exists(model_path):
            print(f"\n  Verify model path: {model_path}")

    print("="*70)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""
SIYI ZR30 RTSP Stream Explorer
Finds and tests all available RTSP streams
"""

import pathlib
import sys
import time

import cv2

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))
import config as cfg


def main() -> int:
    camera_ip = cfg.CAMERA_IP

    test_urls = [
        # Port 8554 (common RTSP port)
        f"rtsp://{camera_ip}:8554/main.264",
        f"rtsp://{camera_ip}:8554/sub",
        f"rtsp://{camera_ip}:8554/stream1",
        f"rtsp://{camera_ip}:8554/stream2",
        f"rtsp://{camera_ip}:8554/main",
        f"rtsp://{camera_ip}:8554/sub.264",

        # Port 554 (standard RTSP port)
        f"rtsp://{camera_ip}:554/main.264",
        f"rtsp://{camera_ip}:554/sub",
        f"rtsp://{camera_ip}:554/stream1",
        f"rtsp://{camera_ip}:554/stream2",
        f"rtsp://{camera_ip}:554/main",

        # No port specified (default 554)
        f"rtsp://{camera_ip}/main.264",
        f"rtsp://{camera_ip}/sub",
        f"rtsp://{camera_ip}/stream1",
        f"rtsp://{camera_ip}/stream2",
    ]

    print("="*70)
    print("SIYI ZR30 RTSP Stream Explorer")
    print("="*70)
    print(f"\nTesting {len(test_urls)} possible RTSP URLs...")
    print("This may take a few minutes...\n")

    working_streams = []

    for i, url in enumerate(test_urls):
        print(f"[{i+1}/{len(test_urls)}] Testing: {url}")

        try:
            cap = cv2.VideoCapture(url)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            time.sleep(0.5)

            if cap.isOpened():
                start = time.time()
                ret, frame = cap.read()
                connect_time = time.time() - start

                if ret and frame is not None:
                    h, w = frame.shape[:2]
                    fps = cap.get(cv2.CAP_PROP_FPS)

                    print(f"  ✓ WORKING!")
                    print(f"    Resolution: {w}x{h}")
                    print(f"    FPS: {fps:.1f}")
                    print(f"    Connect time: {connect_time:.2f}s")

                    working_streams.append({
                        'url': url,
                        'resolution': f"{w}x{h}",
                        'width': w,
                        'height': h,
                        'fps': fps,
                        'connect_time': connect_time
                    })
                else:
                    print(f"  ✗ Opened but no frame received")
            else:
                print(f"  ✗ Failed to open")

            cap.release()

        except Exception as e:
            print(f"  ✗ Error: {e}")

        print()

    # Summary
    print("="*70)
    print("RESULTS SUMMARY")
    print("="*70)

    if working_streams:
        print(f"\n✓ Found {len(working_streams)} working stream(s):\n")

        for i, stream in enumerate(working_streams):
            print(f"{i+1}. {stream['url']}")
            print(f"   Resolution: {stream['resolution']}")
            print(f"   FPS: {stream['fps']:.1f}")
            print(f"   Connect time: {stream['connect_time']:.2f}s")
            print()

        print("\nRECOMMENDATIONS:")

        lowest_res = min(working_streams, key=lambda x: x['width'] * x['height'])
        print(f"\n1. FASTEST (lowest resolution):")
        print(f"   {lowest_res['url']}")
        print(f"   Resolution: {lowest_res['resolution']}")

        highest_res = max(working_streams, key=lambda x: x['width'] * x['height'])
        if highest_res != lowest_res:
            print(f"\n2. BEST QUALITY (highest resolution):")
            print(f"   {highest_res['url']}")
            print(f"   Resolution: {highest_res['resolution']}")

        print("\nUpdate your scripts with the recommended URL!")
        print(f"Example: RTSP_URL = \"{lowest_res['url']}\"")

    else:
        print("\n✗ No working RTSP streams found!")
        print("\nTroubleshooting:")
        print(f"  1. Verify camera IP: ping {camera_ip}")
        print("  2. Check camera web interface for RTSP settings")
        print("  3. Verify RTSP is enabled on the camera")
        print(f"  4. Try accessing camera web UI: http://{camera_ip}")
        print("  5. Check network connectivity")

    print("="*70)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

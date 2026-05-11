#!/usr/bin/env python3
"""
Simple YOLO11 Detection Test (No Gimbal Tracking)
For quick performance testing on Raspberry Pi 5
"""

import cv2
import time
from ultralytics import YOLO

import config as cfg


def main() -> int:
    camera_ip = cfg.CAMERA_IP
    rtsp_url = cfg.RTSP_URL
    model_path = cfg.MODEL_PATH
    confidence_threshold = 0.5

    print("="*60)
    print("YOLO11 Simple Detection Test")
    print("="*60)

    print(f"\nLoading model: {model_path}")
    model = YOLO(model_path)
    print("✓ Model loaded")

    print(f"\nConnecting to: {rtsp_url}")
    cap = cv2.VideoCapture(rtsp_url)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        raise RuntimeError(f"Failed to open stream: {rtsp_url}")

    print("✓ Stream connected")
    print(f"  Resolution: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
    print("\nPress 'q' to quit\n")

    frame_count = 0
    total_detections = 0
    fps_list = []
    start_time = time.time()

    cv2.namedWindow("YOLO11 Test", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("YOLO11 Test", 640, 480)

    try:
        while True:
            frame_start = time.time()

            ret, frame = cap.read()
            if not ret:
                print("Stream interrupted, reconnecting...")
                cap.release()
                time.sleep(1)
                cap = cv2.VideoCapture(rtsp_url)
                continue

            results = model(frame, conf=confidence_threshold, verbose=False, classes=[0])

            annotated_frame = results[0].plot()

            num_detections = len(results[0].boxes)
            total_detections += num_detections
            frame_count += 1

            frame_time = time.time() - frame_start
            fps = 1.0 / frame_time if frame_time > 0 else 0
            fps_list.append(fps)

            avg_fps = sum(fps_list[-30:]) / len(fps_list[-30:]) if fps_list else 0
            cv2.putText(annotated_frame, f"FPS: {avg_fps:.1f}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(annotated_frame, f"Detections: {num_detections}", (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(annotated_frame, f"Frame: {frame_count}", (10, 110),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow("YOLO11 Test", annotated_frame)

            if frame_count % 30 == 0:
                elapsed = time.time() - start_time
                print(f"Frames: {frame_count} | Avg FPS: {avg_fps:.1f} | "
                      f"Total Detections: {total_detections} | Elapsed: {elapsed:.1f}s")

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nInterrupted")

    finally:
        cap.release()
        cv2.destroyAllWindows()

        elapsed = time.time() - start_time
        avg_fps = sum(fps_list) / len(fps_list) if fps_list else 0

        print("\n" + "="*60)
        print("Test Summary:")
        print(f"  Total Frames: {frame_count}")
        print(f"  Total Detections: {total_detections}")
        print(f"  Average FPS: {avg_fps:.1f}")
        print(f"  Runtime: {elapsed:.1f}s")
        print(f"  Detections/Frame: {total_detections/frame_count if frame_count > 0 else 0:.2f}")
        print("="*60)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""
YOLO11 Debug & Optimized Test Script
Fixes: Resolution, FPS, Detection issues
"""

import pathlib
import sys
import time

import cv2
import numpy as np
import torch
from ultralytics import YOLO

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))
import config as cfg


def main() -> int:
    camera_ip = cfg.CAMERA_IP

    rtsp_urls = [
        f"rtsp://{camera_ip}:8554/sub",
        f"rtsp://{camera_ip}:8554/stream2",
        f"rtsp://{camera_ip}:554/sub",
        f"rtsp://{camera_ip}:8554/main.264",
    ]

    model_path = cfg.MODEL_PATH

    confidence_threshold = 0.25
    iou_threshold = 0.45
    yolo_input_size = 416

    target_width = 640
    target_height = 480
    skip_frames = 2

    print("="*70)
    print("YOLO11 Debug & Optimized Test")
    print("="*70)

    # ==================== CHECK GPU AVAILABILITY ====================
    print("\n[1/4] Checking GPU availability...")
    cuda_available = torch.cuda.is_available()
    if cuda_available:
        print(f"  ✓ CUDA available: {torch.cuda.get_device_name(0)}")
        print(f"    CUDA version: {torch.version.cuda}")
        device = 'cuda:0'
    else:
        print("  ⚠ No CUDA GPU detected - using CPU (will be slower)")
        device = 'cpu'

    # ==================== LOAD MODEL ====================
    print(f"\n[2/4] Loading model: {model_path}")
    try:
        model = YOLO(model_path)
        model.to(device)
        print(f"  ✓ Model loaded on {device}")
        print(f"    Model type: {model.task}")
        print(f"    Model classes: {len(model.names)} classes")
        print(f"    Class names: {list(model.names.values())[:5]}...")
    except Exception as e:
        print(f"  ✗ Model loading failed: {e}")
        return 1

    # ==================== TRY DIFFERENT RTSP STREAMS ====================
    print(f"\n[3/4] Testing RTSP streams...")

    cap = None
    selected_url = None

    for i, url in enumerate(rtsp_urls):
        print(f"  [{i+1}/{len(rtsp_urls)}] Trying: {url}")
        test_cap = cv2.VideoCapture(url)
        test_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        test_cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'H264'))

        if test_cap.isOpened():
            ret, frame = test_cap.read()
            if ret:
                h, w = frame.shape[:2]
                print(f"    ✓ Stream working! Resolution: {w}x{h}")

                if w <= 1280 or cap is None:
                    if cap is not None:
                        cap.release()
                    cap = test_cap
                    selected_url = url
                    if w <= 1280:
                        print(f"    ✓ Using this stream (good resolution)")
                        break
                else:
                    test_cap.release()
                    print(f"    ⚠ Resolution too high, trying next...")
            else:
                test_cap.release()
                print(f"    ✗ Stream opened but no frame received")
        else:
            test_cap.release()
            print(f"    ✗ Failed to open stream")

    if cap is None:
        print("\n✗ All RTSP streams failed!")
        print("Troubleshooting:")
        print(f"  1. Check if camera is on: ping {camera_ip}")
        print("  2. Check RTSP stream with VLC")
        print("  3. Verify camera RTSP settings")
        return 1

    original_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    original_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print(f"\n✓ Using stream: {selected_url}")
    print(f"  Original resolution: {original_width}x{original_height}")
    print(f"  Will resize to: {target_width}x{target_height}")

    # ==================== TEST SINGLE FRAME FIRST ====================
    print(f"\n[4/4] Testing single frame detection...")

    ret, test_frame = cap.read()
    if not ret:
        print("✗ Failed to read test frame")
        return 1

    test_frame_resized = cv2.resize(test_frame, (target_width, target_height))

    print(f"  Original frame: {test_frame.shape}")
    print(f"  Resized frame: {test_frame_resized.shape}")
    print(f"  Running detection with conf={confidence_threshold}...")

    start_time = time.time()
    results = model(test_frame_resized, conf=confidence_threshold,
                    iou=iou_threshold, imgsz=yolo_input_size,
                    verbose=False, device=device)
    inference_time = (time.time() - start_time) * 1000

    num_detections = len(results[0].boxes)
    print(f"  ✓ Detection complete in {inference_time:.1f}ms")
    print(f"  Detections found: {num_detections}")

    if num_detections > 0:
        print("\n  Detection details:")
        for i, box in enumerate(results[0].boxes):
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            cls_name = model.names[cls]
            print(f"    [{i+1}] {cls_name}: {conf:.3f}")
    else:
        print("\n  ⚠ No detections in test frame")
        print("  Possible reasons:")
        print("    1. No person in frame")
        print("    2. Model not trained for this view angle")
        print("    3. Confidence threshold too high")
        print("    4. Model expects different input format")

    # ==================== MAIN LOOP ====================
    print("\n" + "="*70)
    print("Starting continuous detection...")
    print("Controls: 'q' = quit, 's' = save frame, 'c' = toggle confidence")
    print("="*70 + "\n")

    cv2.namedWindow("YOLO Debug", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("YOLO Debug", target_width, target_height)

    frame_count = 0
    processed_count = 0
    total_detections = 0
    fps_list = []
    show_low_conf = False
    frame_skip_counter = 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Stream lost, attempting reconnect...")
                cap.release()
                time.sleep(1)
                cap = cv2.VideoCapture(selected_url)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                continue

            frame_count += 1

            frame_skip_counter += 1
            if frame_skip_counter < skip_frames:
                continue
            frame_skip_counter = 0

            loop_start = time.time()

            frame_resized = cv2.resize(frame, (target_width, target_height))

            current_conf = confidence_threshold if not show_low_conf else 0.15
            results = model(frame_resized, conf=current_conf,
                           iou=iou_threshold, imgsz=yolo_input_size,
                           verbose=False, device=device)

            annotated = results[0].plot()

            num_detections = len(results[0].boxes)
            total_detections += num_detections
            processed_count += 1

            loop_time = time.time() - loop_start
            fps = 1.0 / loop_time if loop_time > 0 else 0
            fps_list.append(fps)
            avg_fps = sum(fps_list[-30:]) / len(fps_list[-30:])

            cv2.putText(annotated, f"FPS: {avg_fps:.1f}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(annotated, f"Detections: {num_detections}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(annotated, f"Conf: {current_conf:.2f}", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(annotated, f"Device: {device}", (10, 120),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.putText(annotated, f"Resolution: {target_width}x{target_height}", (10, 140),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            if num_detections > 0:
                y_offset = target_height - 30
                for i, box in enumerate(results[0].boxes[:3]):
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])
                    cls_name = model.names[cls]
                    text = f"{cls_name}: {conf:.2f}"
                    cv2.putText(annotated, text, (10, y_offset - i*25),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            cv2.imshow("YOLO Debug", annotated)

            if processed_count % 30 == 0:
                print(f"Processed: {processed_count} | FPS: {avg_fps:.1f} | "
                      f"Detections: {total_detections} | Last frame: {num_detections}")

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                filename = f"debug_frame_{frame_count}.jpg"
                cv2.imwrite(filename, annotated)
                print(f"Saved: {filename}")
            elif key == ord('c'):
                show_low_conf = not show_low_conf
                print(f"Confidence threshold: {confidence_threshold if not show_low_conf else 0.15}")

    except KeyboardInterrupt:
        print("\nInterrupted")

    finally:
        cap.release()
        cv2.destroyAllWindows()

        print("\n" + "="*70)
        print("Test Summary:")
        print(f"  Total frames captured: {frame_count}")
        print(f"  Frames processed: {processed_count}")
        print(f"  Total detections: {total_detections}")
        print(f"  Average FPS: {sum(fps_list)/len(fps_list) if fps_list else 0:.1f}")
        print(f"  Detection rate: {total_detections/processed_count if processed_count > 0 else 0:.2f} per frame")
        print(f"  Device used: {device}")
        print("="*70)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

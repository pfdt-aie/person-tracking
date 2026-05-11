#!/usr/bin/env python3
"""
YOLO11 GPU-Optimized Detection for Jetson Nano Super
Forces GPU usage and optimizes for CUDA acceleration
"""

import cv2
import time
import numpy as np
import sys
import os

import config as cfg


def main() -> int:
    print("="*70)
    print("YOLO11 GPU-Optimized Test - Jetson Nano Super")
    print("="*70)

    # ==================== FORCE CUDA CHECK ====================
    print("\n[1/5] Checking GPU/CUDA availability...")

    try:
        import torch
        print(f"  PyTorch version: {torch.__version__}")
        print(f"  PyTorch location: {torch.__file__}")

        cuda_available = torch.cuda.is_available()
        print(f"  CUDA available: {cuda_available}")

        if cuda_available:
            print(f"  ✓ CUDA Device: {torch.cuda.get_device_name(0)}")
            print(f"  ✓ CUDA Version: {torch.version.cuda}")
            print(f"  ✓ cuDNN Version: {torch.backends.cudnn.version()}")

            total_mem = torch.cuda.get_device_properties(0).total_memory / 1024**3
            print(f"  ✓ GPU Memory: {total_mem:.2f} GB")

            device = 'cuda:0'

            try:
                test_tensor = torch.randn(1000, 1000).cuda()
                _ = test_tensor @ test_tensor
                print(f"  ✓ CUDA tensor operations verified!")
            except Exception as e:
                print(f"  ✗ CUDA tensor test failed: {e}")
                print("  ⚠ Falling back to CPU")
                device = 'cpu'
        else:
            print("\n  ✗✗✗ CRITICAL ERROR: NO CUDA DETECTED! ✗✗✗")
            print("  Your Jetson Nano Super HAS a GPU but PyTorch cannot access it!")
            print("  This means PyTorch was NOT compiled with CUDA support.")
            print("\n  SOLUTION:")
            print("  1. Run: python3 utils/jetson_cuda_diagnostic.py")
            print("  2. Follow the instructions to install Jetson-specific PyTorch")
            print("  3. Then re-run this script")
            print("\n  Current PyTorch is CPU-only. Continuing anyway but will be SLOW...")
            device = 'cpu'

    except ImportError:
        print("  ✗ PyTorch not installed!")
        return 1

    # ==================== CAMERA & MODEL CONFIG ====================
    rtsp_url = cfg.RTSP_URL
    model_path = cfg.MODEL_PATH

    confidence_threshold = 0.25
    iou_threshold = 0.45

    target_width = 640
    target_height = 480

    if device == 'cuda:0':
        yolo_input_size = 640
        skip_frames = 1
    else:
        yolo_input_size = 416
        skip_frames = 2

    print(f"\n[2/5] Configuration")
    print(f"  Device: {device}")
    print(f"  YOLO input size: {yolo_input_size}")
    print(f"  Frame skip: {skip_frames}")
    print(f"  Target resolution: {target_width}x{target_height}")

    # ==================== LOAD MODEL ====================
    print(f"\n[3/5] Loading YOLO model...")
    print(f"  Model path: {model_path}")

    try:
        from ultralytics import YOLO

        model = YOLO(model_path)
        print(f"  ✓ Model loaded")

        if device == 'cuda:0':
            print(f"  Moving model to GPU...")
            try:
                model.to(device)
                print(f"  ✓ Model on GPU: {device}")
            except Exception as e:
                print(f"  ✗ Failed to move model to GPU: {e}")
                print(f"  ⚠ Using CPU instead")
                device = 'cpu'

        print(f"  Model classes: {len(model.names)}")
        print(f"  Classes: {list(model.names.values())}")

    except Exception as e:
        print(f"  ✗ Model loading failed: {e}")
        return 1

    # ==================== SETUP VIDEO STREAM ====================
    print(f"\n[4/5] Connecting to camera...")
    print(f"  RTSP URL: {rtsp_url}")

    cap = cv2.VideoCapture(rtsp_url)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print(f"  ✗ Failed to open RTSP stream")
        return 1

    ret, test_frame = cap.read()
    if not ret:
        print(f"  ✗ Failed to read frame")
        return 1

    original_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    original_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print(f"  ✓ Stream connected")
    print(f"  Original resolution: {original_width}x{original_height}")
    print(f"  Processing resolution: {target_width}x{target_height}")

    # ==================== TEST SINGLE FRAME ====================
    print(f"\n[5/5] Testing GPU inference on single frame...")

    test_frame_resized = cv2.resize(test_frame, (target_width, target_height))

    print(f"  Running warm-up inference...")
    _ = model(test_frame_resized, conf=confidence_threshold, verbose=False, device=device)

    print(f"  Running timed inference...")
    start_time = time.time()
    results = model(test_frame_resized, conf=confidence_threshold,
                    iou=iou_threshold, imgsz=yolo_input_size,
                    verbose=False, device=device)
    inference_time = (time.time() - start_time) * 1000

    num_detections = len(results[0].boxes)

    print(f"  ✓ Inference time: {inference_time:.1f}ms")
    print(f"  ✓ Detections: {num_detections}")

    if device == 'cuda:0':
        if inference_time < 100:
            print(f"  ✓✓✓ EXCELLENT! GPU is working! ✓✓✓")
            print(f"  Expected FPS: ~{1000/inference_time:.1f}")
        elif inference_time < 200:
            print(f"  ✓ Good GPU performance")
            print(f"  Expected FPS: ~{1000/inference_time:.1f}")
        else:
            print(f"  ⚠ GPU is detected but inference is slow")
            print(f"  This might indicate:")
            print(f"    - GPU not being used (check model.device)")
            print(f"    - TensorRT optimization needed")
            print(f"    - Memory transfer overhead")
    else:
        print(f"  Running on CPU (will be slower)")

    if num_detections > 0:
        print(f"\n  Detected objects:")
        for i, box in enumerate(results[0].boxes):
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            cls_name = model.names[cls]
            print(f"    [{i+1}] {cls_name}: {conf:.3f}")

    # ==================== MAIN DETECTION LOOP ====================
    print("\n" + "="*70)
    print("Starting GPU-accelerated detection...")
    print("Controls: 'q'=quit, 's'=save frame, 'i'=show inference time")
    print("="*70 + "\n")

    cv2.namedWindow("GPU Detection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("GPU Detection", target_width, target_height)

    frame_count = 0
    processed_count = 0
    total_detections = 0
    inference_times = []
    fps_list = []
    frame_skip_counter = 0
    show_inference = False

    if device == 'cuda:0':
        import torch as _torch
        _torch.backends.cudnn.benchmark = True
        print("✓ CUDA optimizations enabled (cuDNN benchmark)")

    try:
        while True:
            loop_start = time.time()

            ret, frame = cap.read()
            if not ret:
                print("Stream lost, reconnecting...")
                cap.release()
                time.sleep(1)
                cap = cv2.VideoCapture(rtsp_url)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                continue

            frame_count += 1

            frame_skip_counter += 1
            if frame_skip_counter < skip_frames:
                continue
            frame_skip_counter = 0

            frame_resized = cv2.resize(frame, (target_width, target_height))

            inf_start = time.time()
            results = model(frame_resized, conf=confidence_threshold,
                           iou=iou_threshold, imgsz=yolo_input_size,
                           verbose=False, device=device)
            inf_time = (time.time() - inf_start) * 1000
            inference_times.append(inf_time)

            annotated = results[0].plot()

            num_detections = len(results[0].boxes)
            total_detections += num_detections
            processed_count += 1

            loop_time = time.time() - loop_start
            fps = 1.0 / loop_time if loop_time > 0 else 0
            fps_list.append(fps)
            avg_fps = sum(fps_list[-30:]) / len(fps_list[-30:])
            avg_inf = sum(inference_times[-30:]) / len(inference_times[-30:])

            color = (0, 255, 0) if device == 'cuda:0' else (0, 165, 255)
            device_text = "GPU (CUDA)" if device == 'cuda:0' else "CPU"

            cv2.putText(annotated, f"FPS: {avg_fps:.1f}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.putText(annotated, f"Detections: {num_detections}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.putText(annotated, f"Device: {device_text}", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            if show_inference:
                cv2.putText(annotated, f"Inference: {avg_inf:.1f}ms", (10, 120),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            if device == 'cuda:0' and processed_count % 30 == 0:
                try:
                    import torch as _torch
                    gpu_mem = _torch.cuda.memory_allocated(0) / 1024**2
                    cv2.putText(annotated, f"GPU Mem: {gpu_mem:.0f}MB", (10, 120),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                except Exception:
                    pass

            cv2.imshow("GPU Detection", annotated)

            if processed_count % 30 == 0:
                print(f"Processed: {processed_count} | FPS: {avg_fps:.1f} | "
                      f"Inference: {avg_inf:.1f}ms | Detections: {total_detections} | "
                      f"Device: {device_text}")

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                filename = f"gpu_frame_{frame_count}.jpg"
                cv2.imwrite(filename, annotated)
                print(f"Saved: {filename}")
            elif key == ord('i'):
                show_inference = not show_inference

    except KeyboardInterrupt:
        print("\nInterrupted")

    finally:
        cap.release()
        cv2.destroyAllWindows()

        avg_fps = sum(fps_list) / len(fps_list) if fps_list else 0
        avg_inf = sum(inference_times) / len(inference_times) if inference_times else 0

        print("\n" + "="*70)
        print("SESSION SUMMARY")
        print("="*70)
        print(f"  Device used: {device}")
        print(f"  Total frames captured: {frame_count}")
        print(f"  Frames processed: {processed_count}")
        print(f"  Total detections: {total_detections}")
        print(f"  Average FPS: {avg_fps:.1f}")
        print(f"  Average inference time: {avg_inf:.1f}ms")
        print(f"  Detection rate: {total_detections/processed_count if processed_count > 0 else 0:.2f} per frame")

        if device == 'cuda:0':
            print(f"\n  ✓ GPU acceleration was ACTIVE")
            if avg_inf < 100:
                print(f"  ✓ Performance: EXCELLENT!")
            elif avg_inf < 200:
                print(f"  ✓ Performance: GOOD")
            else:
                print(f"  ⚠ Performance: Could be better (consider TensorRT)")
        else:
            print(f"\n  ⚠ Ran on CPU only")
            print(f"  ⚠ Install Jetson-specific PyTorch for GPU acceleration")

        print("="*70)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

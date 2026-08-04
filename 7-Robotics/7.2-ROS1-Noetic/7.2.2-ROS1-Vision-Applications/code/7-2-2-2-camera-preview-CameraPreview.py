#!/usr/bin/env python3
import cv2
import glob
import time
import os

def list_video_devices():
    """Return the /dev/video* device list."""
    devs = sorted(glob.glob("/dev/video*"))
    return devs

def try_open_camera(dev):
    """Try to open a camera device."""
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        return None
    return cap

def main():
    devices = list_video_devices()
    if not devices:
        print("❌ No /dev/video* found")
        return

    print("🔍 Found video devices:")
    for d in devices:
        print("  ", d)

    cams = []
    for dev in devices:
        cap = try_open_camera(dev)
        if cap:
            print(f"✅ Opened {dev}")
            cams.append((dev, cap))
        else:
            print(f"❌ Failed {dev}")

    if not cams:
        print("❌ No usable cameras")
        return

    cam_index = 0
    last_time = time.time()
    fps = 0.0

    print("\nControls:")
    print("  n: next camera")
    print("  p: previous camera")
    print("  q / ESC: quit\n")

    while True:
        dev, cap = cams[cam_index]

        ret, frame = cap.read()
        if not ret:
            cv2.putText(
                frame,
                "Camera read failed",
                (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2,
            )
        else:
            now = time.time()
            fps = 1.0 / (now - last_time)
            last_time = now

            # FPS display
            cv2.putText(
                frame,
                f"FPS: {fps:.2f}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
            )

            # Device name display
            cv2.putText(
                frame,
                dev,
                (20, frame.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )

        cv2.imshow("CameraPreview", frame)

        key = cv2.waitKey(1) & 0xFF
        if key in [27, ord("q")]:  # ESC / q
            break
        elif key == ord("n"):
            cam_index = (cam_index + 1) % len(cams)
            print(f"➡ Switch to {cams[cam_index][0]}")
            time.sleep(0.2)
        elif key == ord("p"):
            cam_index = (cam_index - 1) % len(cams)
            print(f"⬅ Switch to {cams[cam_index][0]}")
            time.sleep(0.2)

    for _, cap in cams:
        cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

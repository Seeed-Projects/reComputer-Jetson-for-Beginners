#!/usr/bin/env python3
# simple_AR.py
# common lib
import os
import sys
import time
import cv2 as cv
import numpy as np
import yaml

print("import done")

cv_edition = cv.__version__
print("cv_edition: ",cv_edition)

def load_camera_from_ost(yaml_path: str):
    with open(yaml_path, 'r') as f:
        params = yaml.safe_load(f)
    camera_data = params['camera_matrix']['data']
    dist_data = params['distortion_coefficients']['data']
    camera_matrix = np.array(camera_data, dtype=np.float32).reshape(3, 3)
    dist_coeffs = np.array(dist_data, dtype=np.float32).reshape(-1, 1)
    return camera_matrix, dist_coeffs

def draw_stickman(img, img_pts):
    cv.line(img, tuple(img_pts[18]), tuple(img_pts[4]), (0, 0, 255), 3)
    cv.line(img, tuple(img_pts[18]), tuple(img_pts[6]), (0, 0, 255), 3)
    cv.line(img, tuple(img_pts[18]), tuple(img_pts[21]), (0, 0, 255), 3)
    cv.line(img, tuple(img_pts[21]), tuple(img_pts[19]), (0, 0, 255), 3)
    cv.line(img, tuple(img_pts[21]), tuple(img_pts[20]), (0, 0, 255), 3)
    cv.line(img, tuple(img_pts[21]), tuple(img_pts[22]), (0, 0, 255), 3)
    cv.circle(img, tuple(img_pts[22]), 15, (0, 0, 255), -1)
    cv.line(img, tuple(img_pts[74]), tuple(img_pts[72]), (0, 255, 0), 3)
    cv.line(img, tuple(img_pts[74]), tuple(img_pts[73]), (0, 255, 0), 3)
    cv.line(img, tuple(img_pts[74]), tuple(img_pts[37]), (0, 255, 0), 3)
    cv.line(img, tuple(img_pts[37]), tuple(img_pts[76]), (0, 255, 0), 3)
    cv.line(img, tuple(img_pts[37]), tuple(img_pts[77]), (0, 255, 0), 3)
    cv.line(img, tuple(img_pts[37]), tuple(img_pts[75]), (0, 255, 0), 3)
    cv.circle(img, tuple(img_pts[75]), 15, (0, 255, 0), -1)
    return img

def main():
    print("start")

    pattern_size = (8,6)
    yaml_path = os.path.join(os.path.dirname(__file__), 'sources', 'ost.yaml')
    camera_matrix, dist_coeffs = load_camera_from_ost(yaml_path)

    object_points = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    object_points[:,:2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)

    axis = np.float32([
            [0, 0, -1], [0, 8, -1], [5, 8, -1], [5, 0, -1],
            [1, 2, -1], [1, 6, -1], [4, 2, -1], [4, 6, -1],
            [1, 0, -4], [1, 8, -4], [4, 0, -4], [4, 8, -4],
            [1, 2, -4], [1, 6, -4], [4, 2, -4], [4, 6, -4],
            [0, 1, -4], [3, 2, -1], [2, 2, -3], [3, 2, -3],
            [1, 2, -3], [2, 2, -4], [2, 2, -5], [0, 4, -4],
            [2, 3, -4], [1, 3, -4], [4, 3, -5], [4, 5, -5],
            [1, 2, -3], [1, 6, -3], [5, 2, -3], [5, 6, -3],
            [3, 4, -5], [0, 6, -4], [5, 6, -4], [2, 8, -4],
            [3, 8, -4], [2, 6, -4], [2, 0, -4], [1, 5, -4],
            [3, 0, -4], [3, 2, -4], [0, 3, -4], [1, 2, -4],
            [4, 2, -4], [5, 3, -4], [2, 7, -4], [3, 7, -4],
            [3, 3, -1], [3, 5, -1], [1, 5, -1], [1, 3, -1],
            [3, 3, -3], [3, 5, -3], [1, 5, -3], [1, 3, -3],
            [1, 3, -6], [1, 5, -6], [3, 3, -4], [3, 5, -4],
            [0, 0, -4], [3, 1, -4], [1, 1, -4], [0, 2, -4],
            [2, 4, -4], [4, 4, -4], [0, 8, -4], [5, 8, -4],
            [5, 0, -4], [0, 4, -5], [5, 4, -4], [5, 4, -5],
            [2, 5, -1], [2, 7, -1], [2, 6, -3], [2, 6, -5],
            [2, 5, -3], [2, 7, -3]
            ])

    capture = cv.VideoCapture(0)
    if cv_edition[0] == '3':
        capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))
    else:
        capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(6, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

    last_time = time.time()
    fps = 0.0

    while True:
        ret, frame = capture.read()
        if not ret:
            break

        now = time.time()
        fps = 1.0 / max(1e-6, (now - last_time))
        last_time = now
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        retval, corners = cv.findChessboardCorners(
            gray,
            pattern_size,
            None,
            flags=cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_NORMALIZE_IMAGE + cv.CALIB_CB_FAST_CHECK,
        )

        if retval:
            corners = cv.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001),
            )
            retval, rvec, tvec, _inliers = cv.solvePnPRansac(
                object_points,
                corners,
                camera_matrix,
                dist_coeffs,
            )
            if retval:
                image_points, _jacobian = cv.projectPoints(
                    axis,
                    rvec,
                    tvec,
                    camera_matrix,
                    dist_coeffs,
                )
                img_pts = np.int32(image_points).reshape(-1, 2)
                frame = draw_stickman(frame, img_pts)

                cv.putText(frame, "AR Active - 2 Stickman", (10, 60), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            else:
                cv.putText(frame, "Pose estimation failed", (10, 60), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        else:
            cv.putText(frame, "No chessboard detected", (10, 60), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

        cv.putText(frame, f"FPS: {fps:.1f}", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv.imshow('frame', frame)

        action = cv.waitKey(1) & 0xFF
        if action == ord('q') or action == 113:
            break

    capture.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()

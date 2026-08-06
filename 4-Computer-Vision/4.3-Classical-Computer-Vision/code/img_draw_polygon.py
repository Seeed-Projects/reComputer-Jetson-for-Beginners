# Draw a polygon on an image using cv2.polylines.
import cv2
import numpy as np
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
img_path = str(SCRIPT_DIR / "imgs" / "test.png")
img = cv2.imread(img_path)

if img is not None:
    # cv2.polylines(img, points, is_closed, color, thickness)
    points = np.array([(35, 15), (725, 15), (725, 125), (35, 125)], np.int32)
    points = points.reshape((-1, 1, 2))
    cv2.polylines(img, [points], True, (0, 255, 0), 5)
    cv2.imshow('Original Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print('Image not found')

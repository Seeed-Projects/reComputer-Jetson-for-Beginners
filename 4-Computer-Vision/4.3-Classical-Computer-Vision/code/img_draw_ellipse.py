# Draw an ellipse on an image using cv2.ellipse.
import cv2
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
img_path = str(SCRIPT_DIR / "imgs" / "test.png")
img = cv2.imread(img_path)

if img is not None:
    # cv2.ellipse(img, center, axes, angle, start_angle, end_angle, color, thickness)
    cv2.ellipse(img, (100, 100), (20, 60), 0, 0, 360, (0, 255, 0), 2)
    cv2.imshow('Original Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print('Image not found')

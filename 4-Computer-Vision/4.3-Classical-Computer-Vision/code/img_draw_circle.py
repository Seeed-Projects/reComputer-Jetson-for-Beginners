# Draw a circle on an image using cv2.circle.
import cv2
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
img_path = str(SCRIPT_DIR / "imgs" / "test.png")
img = cv2.imread(img_path)

if img is not None:
    # cv2.circle(img, center, radius, color, thickness)
    cv2.circle(img, (50, 50), 30, (0, 0, 255), 2)
    cv2.imshow('Original Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print('Image not found')

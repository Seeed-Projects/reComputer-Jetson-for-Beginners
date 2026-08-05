# Draw a rectangle on an image using cv2.rectangle.
import cv2
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
img_path = str(SCRIPT_DIR / "imgs" / "test.png")
img = cv2.imread(img_path)

if img is not None:
    # cv2.rectangle(img, top_left, bottom_right, color, thickness)
    cv2.rectangle(img, (10, 10), (100, 100), (255, 255, 0), 5)
    cv2.imshow('Original Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print('Image not found')

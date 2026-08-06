# Binarize an image using cv2.threshold.
import cv2
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
img_path = str(SCRIPT_DIR / "imgs" / "test.png")
img = cv2.imread(img_path)

if img is not None:
    _, binary_image = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
    cv2.imshow('Image', binary_image)
    cv2.imshow('Original Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print('Image not found')

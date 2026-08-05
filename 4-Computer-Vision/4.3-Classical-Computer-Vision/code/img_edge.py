# Detect edges in an image using the Canny algorithm (cv2.Canny).
import cv2
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
img_path = str(SCRIPT_DIR / "imgs" / "test.png")
img = cv2.imread(img_path)

if img is not None:
    edges = cv2.Canny(img, 100, 255)
    cv2.imshow('Image', edges)
    cv2.imshow('Original Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print('Image not found')

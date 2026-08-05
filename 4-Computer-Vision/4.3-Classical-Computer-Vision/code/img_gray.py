# Convert an image to grayscale using cv2.cvtColor.
import cv2
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
img_path = str(SCRIPT_DIR / "imgs" / "test.png")
img = cv2.imread(img_path)

if img is not None:
    gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imshow('Image', gray_image)
    cv2.imshow('Original Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print('Image not found')

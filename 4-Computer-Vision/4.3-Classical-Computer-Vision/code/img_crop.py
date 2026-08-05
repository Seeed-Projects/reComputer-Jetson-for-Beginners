# Crop a region from an image using NumPy array slicing and display both versions.
import cv2
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
img_path = str(SCRIPT_DIR / "imgs" / "test.png")
img = cv2.imread(img_path)

if img is not None:
    img_cropped = img[50:100, 100:150]
    cv2.imshow('Image_cropped', img_cropped)
    cv2.imshow('Image_original', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print('Image not found')

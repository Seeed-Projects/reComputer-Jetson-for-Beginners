# Resize an image to 100x100 using cv2.resize and display both versions.
import cv2
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
img_path = str(SCRIPT_DIR / "imgs" / "test.png")
img = cv2.imread(img_path)

if img is not None:
    img_resize = cv2.resize(img, (100, 100))
    cv2.imshow('Image_resize', img_resize)
    cv2.imshow('Image_original', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print('Image not found')

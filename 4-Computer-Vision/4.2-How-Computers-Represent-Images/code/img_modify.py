# Modify image pixels by setting the top-left 100x100 region to white.
import cv2
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
img_path = str(SCRIPT_DIR / "sources" / "cv_world.png")
img = cv2.imread(img_path)

if img is not None:
    # Set the top-left 100x100 region to white
    img[:100, :100] = [255, 255, 255]
    cv2.imshow('Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print('Image not found')

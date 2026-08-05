# Read an image from disk and save a copy using cv2.imwrite.
import cv2
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
img_path = str(SCRIPT_DIR / "sources" / "cv_world.png")
img = cv2.imread(img_path)

if img is not None:
    save_path = str(SCRIPT_DIR / "sources" / "cv_world_saved.png")
    cv2.imwrite(save_path, img)
else:
    print('Image not found')

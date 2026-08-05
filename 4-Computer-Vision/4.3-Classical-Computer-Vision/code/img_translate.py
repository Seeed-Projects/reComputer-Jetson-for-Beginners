# Translate (shift) an image using cv2.warpAffine with an affine matrix.
import cv2
import numpy as np
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
img_path = str(SCRIPT_DIR / "imgs" / "test.png")
img = cv2.imread(img_path)

if img is not None:
    M = np.float32([[1, 0, 50], [0, 1, 50]])
    shifted_image = cv2.warpAffine(img, M, (img.shape[1], img.shape[0]))
    cv2.imshow('Image', shifted_image)
    cv2.imshow('Original Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print('Image not found')

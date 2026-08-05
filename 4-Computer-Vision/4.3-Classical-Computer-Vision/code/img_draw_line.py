# Draw a line on an image using cv2.line.
import cv2
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
img_path = str(SCRIPT_DIR / "imgs" / "test.png")
img = cv2.imread(img_path)

if img is not None:
    # cv2.line(img, start_point, end_point, color, thickness)
    cv2.line(img, (50, 150), (700, 150), (0, 0, 255), 5)
    cv2.imshow('Original Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print('Image not found')

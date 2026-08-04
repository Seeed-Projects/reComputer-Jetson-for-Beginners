import cv2
from pathlib import Path

# Load the input image from disk. If the file cannot be found,
# raise a clear error message immediately.
img_path = str(Path(__file__).resolve().parent / "sources" / "cv_world.png")
image = cv2.imread(img_path)
if image is None:
    raise FileNotFoundError(f"Unable to load image: {img_path}")

# Print the loaded image shape and data type for quick verification.
print("Shape:", image.shape)
print("Data type:", image.dtype)

# Generate derived color-space representations for display.
# The grayscale image is converted back to BGR so it can be concatenated
# seamlessly with the other three-channel images.
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
gray_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
hsv_bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

# Crop a representative region from the original image.
# Resize the original image to a standard preview size.
crop = image[100:300, 150:350]
small = cv2.resize(image, (640, 480), interpolation=cv2.INTER_AREA)

# Resize helper: scale an image to a given height while preserving aspect ratio.
# This prevents the image from appearing stretched or squashed when tiled.

def resize_to_height(img, height):
    h, w = img.shape[:2]
    return cv2.resize(img, (int(w * height / h), height), interpolation=cv2.INTER_AREA)

# Draw a white text label on a black background at the bottom center of each tile.
# This helps the viewer understand what processing was applied to that image.

def add_label(img, text, padding=8):
    out = img.copy()
    h, w = out.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.6
    thickness = 1
    text_size, baseline = cv2.getTextSize(text, font, font_scale, thickness)
    text_w, text_h = text_size
    rect_w = text_w + padding * 2
    rect_h = text_h + padding * 2 + baseline
    rect_x = int((w - rect_w) / 2)
    rect_y = h - rect_h
    cv2.rectangle(out, (rect_x, rect_y), (rect_x + rect_w, h), (0, 0, 0), -1)
    text_x = rect_x + padding
    text_y = h - padding - baseline
    cv2.putText(out, text, (text_x, text_y), font, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)
    return out

# Create a padded version of the original image to fill the available width.
# Reflection padding is used to avoid introducing an ugly black border.

def padded_width_image(img, width, height):
    resized = resize_to_height(img, height)
    h, w = resized.shape[:2]
    if w >= width:
        return resized[:, :width]

    pad_total = width - w
    pad_left = pad_total // 2
    pad_right = pad_total - pad_left
    return cv2.copyMakeBorder(resized, 0, 0, pad_left, pad_right, cv2.BORDER_REFLECT_101)

# Build the first row of tiles from the original and converted images.
row_height = 240
row1 = [add_label(resize_to_height(img, row_height), label) for img, label in (
    (image, "Original"),
    (gray_bgr, "Grayscale"),
    (hsv_bgr, "HSV")
)]
row1_combined = cv2.hconcat(row1)

# Build the second row using cropped, resized, and padded tiles.
crop_resized = resize_to_height(crop, row_height)
small_resized = resize_to_height(small, row_height)
remaining_width = row1_combined.shape[1] - (crop_resized.shape[1] + small_resized.shape[1])
remaining_width = max(1, remaining_width)

padded = padded_width_image(image, remaining_width, row_height)
row2 = [
    add_label(crop_resized, "Cropped"),
    add_label(small_resized, "Resized 640x480"),
    add_label(padded, "Padded")
]
row2_combined = cv2.hconcat(row2)

# Adjust the second row width if necessary so the two rows can be stacked cleanly.
if row2_combined.shape[1] < row1_combined.shape[1]:
    pad_width = row1_combined.shape[1] - row2_combined.shape[1]
    row2_combined = cv2.copyMakeBorder(
        row2_combined, 0, 0, 0, pad_width, cv2.BORDER_REFLECT_101
    )
elif row2_combined.shape[1] > row1_combined.shape[1]:
    row2_combined = row2_combined[:, : row1_combined.shape[1]]

overview = cv2.vconcat([row1_combined, row2_combined])

# Scale down the final overview image if it would otherwise be too large.
# Keep the aspect ratio intact so the result looks correct.
max_width, max_height = 1200, 800
h, w = overview.shape[:2]
scale = min(max_width / w, max_height / h, 1.0)
final_display = cv2.resize(overview, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)

# Show the final tiled image in a single OpenCV window.
cv2.imshow("Image Overview", final_display)
cv2.waitKey(0)
cv2.destroyAllWindows()
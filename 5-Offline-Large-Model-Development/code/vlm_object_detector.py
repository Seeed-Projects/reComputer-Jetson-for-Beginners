#!/usr/bin/env python3
"""VLM Object Detection with Ollama + OpenCV."""
import argparse
import json
import re
import sys

import cv2
import ollama
from PIL import Image


PROMPT = (
    'You are an expert object detection system. Your task is to identify and localize objects in images.\n'
    'IMPORTANT: Return ONLY a valid JSON array, no other text.\n'
    'Format: [{"label": "object_name", "bbox_2d": [x1, y1, x2, y2]}]\n'
    'IMPORTANT: Coordinates must be absolute PIXEL values (0-based), NOT normalized.\n'
    'x1,y1 is top-left corner, x2,y2 is bottom-right corner.\n'
    'x coordinate ranges from 0 to image width-1 (left to right).\n'
    'y coordinate ranges from 0 to image height-1 (top to bottom).\n'
    'Example for a 1920x1080 image: [{"label": "person", "bbox_2d": [100, 50, 400, 800]}]\n'
    'Be precise with bounding box coordinates - they should tightly fit around each object.'
)

COLORS = [(0, 255, 0), (255, 0, 0), (0, 0, 255), (255, 255, 0)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", default="./img/test.jpg", required=True)
    parser.add_argument("--model", default="qwen3.5:4b")
    args = parser.parse_args()

    # Query VLM
    img = Image.open(args.image)
    w, h = img.size

    resp = ollama.chat(model=args.model, messages=[
        {"role": "user", "content": PROMPT, "images": [args.image]}
    ])

    # Parse JSON
    text = resp["message"]["content"]
    print(f"Raw response:\n{text}\n")

    match = re.search(r'\[.*\]', text, re.DOTALL)
    if not match:
        print("Error: No valid JSON array found in response")
        sys.exit(1)
    try:
        detections = json.loads(match.group())
    except json.JSONDecodeError as e:
        print(f"Error: Failed to parse JSON: {e}")
        sys.exit(1)

    normalized = []
    for det in detections:
        if not isinstance(det, dict):
            continue
        label = det.get("label") or det.get("name")
        bbox = det.get("bbox_2d") or det.get("bbox") or det.get("coordinates")
        if not label or not bbox:
            for k, v in det.items():
                if isinstance(v, list) and len(v) == 4:
                    label = k
                    bbox = v
                    break
        if not label or not bbox or len(bbox) != 4:
            continue
        
        # Handle case where bbox is a string like "[141, 44, 960, 985]"
        if isinstance(bbox, str):
            try:
                bbox = json.loads(bbox)
            except json.JSONDecodeError:
                # Try stripping quotes and parsing
                cleaned = re.sub(r'["\'\[\]]', '', bbox)
                bbox = [float(x.strip()) for x in cleaned.split(',') if x.strip()]
        
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            continue
        
        x1, y1, x2, y2 = [float(v) for v in bbox]
        
        # VLM models often return normalized coordinates (0-1000 or 0-1 range)
        # Detect this by checking if coordinates are small relative to image size
        # If x2/y2 are < 1000 and image dimensions are much larger, assume normalized
        if max(x2, y2) <= 1000 and (w > 1000 or h > 1000):
            # Normalized to 0-1000 range, convert to absolute pixels
            x1, y1, x2, y2 = x1 * w / 1000, y1 * h / 1000, x2 * w / 1000, y2 * h / 1000
        elif max(x2, y2) <= 1 and (w > 1 or h > 1):
            # Normalized to 0-1 range, convert to absolute pixels
            x1, y1, x2, y2 = x1 * w, y1 * h, x2 * w, y2 * h
        
        # Ensure coordinates are valid (x2 > x1, y2 > y1)
        if x2 <= x1:
            x2 = x1 + 10
        if y2 <= y1:
            y2 = y1 + 10
        
        # Clamp to image bounds
        x1, y1 = max(0, min(x1, w-1)), max(0, min(y1, h-1))
        x2, y2 = max(0, min(x2, w-1)), max(0, min(y2, h-1))
        
        normalized.append({"label": label, "bbox": [int(x1), int(y1), int(x2), int(y2)]})
    detections = normalized

    # Draw
    image = cv2.imread(args.image)
    for i, det in enumerate(detections):
        label = det.get("label", "?")
        bbox = det.get("bbox", [])
        if not bbox or len(bbox) != 4:
            print(f"Warning: Skipping detection with invalid bbox: {det}")
            continue
        x1, y1, x2, y2 = [int(v) for v in bbox]
        color = COLORS[i % len(COLORS)]
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 4)
        cv2.putText(image, label, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    output_path = args.image.rsplit('.', 1)[0] + '_result.jpg'
    cv2.imwrite(output_path, image)
    print(f"Saved result to {output_path}")


if __name__ == "__main__":
    main()
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
    'List all objects in this image as JSON: '
    '[{"label": "name", "bbox": [x1, y1, x2, y2]}, ...]'
)

COLORS = [(0, 255, 0), (255, 0, 0), (0, 0, 255), (255, 255, 0)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True)
    parser.add_argument("--model", default="llava:7b")
    parser.add_argument("--output", default="output.jpg")
    args = parser.parse_args()

    # Query VLM
    img = Image.open(args.image)
    w, h = img.size

    resp = ollama.chat(model=args.model, messages=[
        {"role": "user", "content": PROMPT, "images": [args.image]}
    ])

    # Parse JSON
    text = resp["message"]["content"]
    match = re.search(r'\[.*\]', text, re.DOTALL)
    detections = json.loads(match.group()) if match else []

    # Draw
    image = cv2.imread(args.image)
    for i, det in enumerate(detections):
        label = det.get("label", "?")
        x1, y1, x2, y2 = [int(v) for v in det.get("bbox", [])]
        color = COLORS[i % len(COLORS)]
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        cv2.putText(image, label, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    cv2.imwrite(args.output, image)
    cv2.imshow("Result", image)
    cv2.waitKey(0)


if __name__ == "__main__":
    main()
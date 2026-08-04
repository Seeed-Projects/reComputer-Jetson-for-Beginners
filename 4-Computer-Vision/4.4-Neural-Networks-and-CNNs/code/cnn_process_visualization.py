from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
from PIL import Image, ImageOps, ImageTk
import tkinter as tk
from tkinter import ttk
try:
    import torch
    import torch.nn.functional as F

    TORCH_AVAILABLE = True
except Exception:
    torch = None  # type: ignore[assignment]
    F = None  # type: ignore[assignment]
    TORCH_AVAILABLE = False

PIL_RESAMPLING = getattr(Image, "Resampling", Image)


def load_image(path: Path, size: int) -> np.ndarray:
    image = Image.open(path).convert("L")
    image = ImageOps.fit(image, (size, size), method=PIL_RESAMPLING.BILINEAR)
    array = np.asarray(image, dtype=np.float32) / 255.0
    return array


def conv2d_multi_channel(
    x: np.ndarray,
    kernels: np.ndarray,
    bias: np.ndarray,
    stride: int = 1,
    padding: int = 1,
) -> np.ndarray:
    in_channels, in_h, in_w = x.shape
    out_channels, kernel_in_channels, k_h, k_w = kernels.shape
    if in_channels != kernel_in_channels:
        raise ValueError("Input channels and kernel channels do not match.")

    padded = np.pad(x, ((0, 0), (padding, padding), (padding, padding)), mode="constant")
    out_h = (in_h + 2 * padding - k_h) // stride + 1
    out_w = (in_w + 2 * padding - k_w) // stride + 1
    out = np.zeros((out_channels, out_h, out_w), dtype=np.float32)

    for oc in range(out_channels):
        kernel = kernels[oc]
        for oy in range(out_h):
            y_start = oy * stride
            for ox in range(out_w):
                x_start = ox * stride
                patch = padded[:, y_start : y_start + k_h, x_start : x_start + k_w]
                out[oc, oy, ox] = float(np.sum(patch * kernel) + bias[oc])
    return out


def relu(x: np.ndarray) -> np.ndarray:
    return np.maximum(x, 0.0)


def max_pool2d(x: np.ndarray, kernel_size: int = 2, stride: int = 2) -> np.ndarray:
    channels, in_h, in_w = x.shape
    out_h = (in_h - kernel_size) // stride + 1
    out_w = (in_w - kernel_size) // stride + 1
    out = np.zeros((channels, out_h, out_w), dtype=np.float32)
    for c in range(channels):
        for oy in range(out_h):
            y_start = oy * stride
            for ox in range(out_w):
                x_start = ox * stride
                patch = x[c, y_start : y_start + kernel_size, x_start : x_start + kernel_size]
                out[c, oy, ox] = float(np.max(patch))
    return out


def avg_pool2d(x: np.ndarray, kernel_size: int = 2, stride: int = 2) -> np.ndarray:
    channels, in_h, in_w = x.shape
    out_h = (in_h - kernel_size) // stride + 1
    out_w = (in_w - kernel_size) // stride + 1
    out = np.zeros((channels, out_h, out_w), dtype=np.float32)
    for c in range(channels):
        for oy in range(out_h):
            y_start = oy * stride
            for ox in range(out_w):
                x_start = ox * stride
                patch = x[c, y_start : y_start + kernel_size, x_start : x_start + kernel_size]
                out[c, oy, ox] = float(np.mean(patch))
    return out


def normalize_map_to_uint8(feature_map: np.ndarray) -> np.ndarray:
    minimum = float(feature_map.min())
    maximum = float(feature_map.max())
    if maximum - minimum < 1e-8:
        return np.zeros(feature_map.shape, dtype=np.uint8)
    normalized = (feature_map - minimum) / (maximum - minimum)
    return np.clip(normalized * 255.0, 0, 255).astype(np.uint8)


def map_to_image(feature_map: np.ndarray, scale: int = 2) -> Image.Image:
    array = normalize_map_to_uint8(feature_map)
    image = Image.fromarray(array, mode="L")
    if scale > 1:
        image = image.resize((image.width * scale, image.height * scale), PIL_RESAMPLING.NEAREST)
    return image.convert("RGB")


def stage_tensor_to_images(stage_tensor: np.ndarray, max_channels: int) -> List[Image.Image]:
    if stage_tensor.ndim == 2:
        stage_tensor = np.expand_dims(stage_tensor, axis=0)
    images: List[Image.Image] = []
    for idx in range(min(stage_tensor.shape[0], max_channels)):
        feature = stage_tensor[idx]
        scale = max(1, 200 // max(feature.shape[0], feature.shape[1]))
        images.append(map_to_image(feature, scale=scale))
    return images


def build_demo_kernels() -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    conv1 = np.array(
        [
            [[[1, 0, -1], [2, 0, -2], [1, 0, -1]]],
            [[[1, 2, 1], [0, 0, 0], [-1, -2, -1]]],
            [[[0, 1, 0], [1, -4, 1], [0, 1, 0]]],
            [[[1, 1, 1], [1, -8, 1], [1, 1, 1]]],
        ],
        dtype=np.float32,
    )
    bias1 = np.zeros(4, dtype=np.float32)

    rng = np.random.default_rng(42)
    conv2 = rng.normal(loc=0.0, scale=0.25, size=(6, 4, 3, 3)).astype(np.float32)
    bias2 = rng.normal(loc=0.0, scale=0.03, size=(6,)).astype(np.float32)
    conv3 = rng.normal(loc=0.0, scale=0.2, size=(3, 6, 3, 3)).astype(np.float32)
    bias3 = rng.normal(loc=0.0, scale=0.02, size=(3,)).astype(np.float32)
    return conv1, bias1, conv2, bias2, conv3, bias3


def run_pipeline_numpy(image_array: np.ndarray) -> Dict[str, np.ndarray]:
    conv1_k, conv1_b, conv2_k, conv2_b, conv3_k, conv3_b = build_demo_kernels()
    x0 = np.expand_dims(image_array, axis=0)
    x1 = conv2d_multi_channel(x0, conv1_k, conv1_b, stride=1, padding=1)
    x2 = relu(x1)
    x3 = max_pool2d(x2, kernel_size=2, stride=2)
    x4 = conv2d_multi_channel(x3, conv2_k, conv2_b, stride=1, padding=1)
    x5 = relu(x4)
    x6 = avg_pool2d(x5, kernel_size=2, stride=2)
    x7 = conv2d_multi_channel(x6, conv3_k, conv3_b, stride=2, padding=1)
    x8 = relu(x7)
    return {
        "00_input": x0,
        "01_conv1": x1,
        "02_relu1": x2,
        "03_maxpool1": x3,
        "04_conv2": x4,
        "05_relu2": x5,
        "06_avgpool2": x6,
        "07_strided_downsample": x7,
        "08_relu3": x8,
    }


def run_pipeline_torch(image_array: np.ndarray, device: "torch.device") -> Dict[str, np.ndarray]:
    conv1_k, conv1_b, conv2_k, conv2_b, conv3_k, conv3_b = build_demo_kernels()
    x0 = torch.from_numpy(image_array).to(device=device, dtype=torch.float32).unsqueeze(0).unsqueeze(0)
    k1 = torch.from_numpy(conv1_k).to(device=device, dtype=torch.float32)
    b1 = torch.from_numpy(conv1_b).to(device=device, dtype=torch.float32)
    k2 = torch.from_numpy(conv2_k).to(device=device, dtype=torch.float32)
    b2 = torch.from_numpy(conv2_b).to(device=device, dtype=torch.float32)
    k3 = torch.from_numpy(conv3_k).to(device=device, dtype=torch.float32)
    b3 = torch.from_numpy(conv3_b).to(device=device, dtype=torch.float32)

    x1 = F.conv2d(x0, k1, b1, stride=1, padding=1)
    x2 = F.relu(x1)
    x3 = F.max_pool2d(x2, kernel_size=2, stride=2)
    x4 = F.conv2d(x3, k2, b2, stride=1, padding=1)
    x5 = F.relu(x4)
    x6 = F.avg_pool2d(x5, kernel_size=2, stride=2)
    x7 = F.conv2d(x6, k3, b3, stride=2, padding=1)
    x8 = F.relu(x7)

    return {
        "00_input": x0[0].detach().cpu().numpy(),
        "01_conv1": x1[0].detach().cpu().numpy(),
        "02_relu1": x2[0].detach().cpu().numpy(),
        "03_maxpool1": x3[0].detach().cpu().numpy(),
        "04_conv2": x4[0].detach().cpu().numpy(),
        "05_relu2": x5[0].detach().cpu().numpy(),
        "06_avgpool2": x6[0].detach().cpu().numpy(),
        "07_strided_downsample": x7[0].detach().cpu().numpy(),
        "08_relu3": x8[0].detach().cpu().numpy(),
    }


def run_pipeline(image_array: np.ndarray) -> Tuple[Dict[str, np.ndarray], str]:
    if TORCH_AVAILABLE:
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        return run_pipeline_torch(image_array, device), f"torch-{device.type}"
    return run_pipeline_numpy(image_array), "numpy-cpu"


def show_pipeline_window(stages: Dict[str, np.ndarray], max_channels: int, backend: str) -> None:
    root = tk.Tk()
    root.title("CNN Process Visualization")
    root.geometry("1180x860")

    container = ttk.Frame(root, padding=10)
    container.pack(fill="both", expand=True)

    header = ttk.Label(
        container,
        text="CNN Visualization: Input -> Conv -> ReLU -> Pool -> Conv -> ReLU -> Pool -> Downsample",
        font=("Segoe UI", 11, "bold"),
    )
    header.pack(anchor="w", pady=(0, 6))

    instruction = ttk.Label(
        container,
        text="Start from the original image. Click 'Next Stage' to reveal one new CNN stage each time.",
    )
    instruction.pack(anchor="w", pady=(0, 8))
    ttk.Label(container, text=f"Compute backend: {backend.upper()}").pack(anchor="w", pady=(0, 8))

    stage_items = list(stages.items())
    current_stage_index = 0

    top_controls = ttk.Frame(container)
    top_controls.pack(fill="x", pady=(0, 8))
    stage_status_var = tk.StringVar()
    shape_status_var = tk.StringVar()
    progress_status_var = tk.StringVar()
    ttk.Label(top_controls, textvariable=stage_status_var, font=("Segoe UI", 10, "bold")).pack(side="left")
    ttk.Label(top_controls, textvariable=shape_status_var).pack(side="left", padx=(14, 0))
    next_button = ttk.Button(top_controls, text="Next Stage")
    next_button.pack(side="right")
    ttk.Label(top_controls, textvariable=progress_status_var).pack(side="right", padx=(0, 12))

    stage_frame = ttk.LabelFrame(container, text="Current Stage")
    stage_frame.pack(fill="both", expand=True)
    image_grid = ttk.Frame(stage_frame)
    image_grid.pack(fill="both", expand=True, padx=10, pady=10)

    photo_refs: List[ImageTk.PhotoImage] = []
    cached_stage_images: List[List[Image.Image]] = [
        stage_tensor_to_images(tensor, max_channels=max_channels) for _, tensor in stage_items
    ]
    cached_stage_photos: List[List[ImageTk.PhotoImage]] = [
        [ImageTk.PhotoImage(image) for image in stage_images] for stage_images in cached_stage_images
    ]

    def render_current_stage() -> None:
        stage_name, tensor = stage_items[current_stage_index]
        display_name = stage_name.replace("_", " ")
        stage_status_var.set(f"Stage {current_stage_index + 1}: {display_name}")
        if tensor.ndim == 3:
            shape_status_var.set(f"Channels: {tensor.shape[0]}, Size: {tensor.shape[1]} x {tensor.shape[2]}")
        else:
            shape_status_var.set(f"Size: {tensor.shape[-2]} x {tensor.shape[-1]}")
        progress_status_var.set(f"{current_stage_index + 1} / {len(stage_items)}")

        for child in image_grid.winfo_children():
            child.destroy()

        photo_refs.clear()
        stage_images = cached_stage_images[current_stage_index]
        stage_photos = cached_stage_photos[current_stage_index]
        columns = 4
        for idx, image in enumerate(stage_images):
            photo = stage_photos[idx]
            photo_refs.append(photo)
            cell = ttk.Frame(image_grid)
            row = idx // columns
            col = idx % columns
            cell.grid(row=row, column=col, padx=6, pady=6, sticky="nw")
            ttk.Label(cell, text=f"Channel {idx}", font=("Segoe UI", 9, "bold")).pack(anchor="w")
            ttk.Label(cell, image=photo).pack(anchor="w")

        if current_stage_index >= len(stage_items) - 1:
            next_button.configure(state="disabled")
        else:
            next_button.configure(state="normal")

        root.photo_refs = photo_refs

    def show_next_stage() -> None:
        nonlocal current_stage_index
        if current_stage_index < len(stage_items) - 1:
            current_stage_index += 1
            render_current_stage()

    next_button.configure(command=show_next_stage)
    render_current_stage()
    root.mainloop()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="CNN process visualization demo")
    parser.add_argument(
        "--image",
        type=str,
        default=str(Path(__file__).resolve().parent / "img" / "cat.jpg"),
    )
    parser.add_argument("--size", type=int, default=160)
    parser.add_argument("--max-channels", type=int, default=8)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    image_path = Path(args.image).expanduser().resolve()
    if not image_path.exists():
        raise FileNotFoundError(f"Input image not found: {image_path}")

    image_array = load_image(image_path, args.size)
    stages, backend = run_pipeline(image_array)
    show_pipeline_window(stages, max_channels=args.max_channels, backend=backend)


if __name__ == "__main__":
    main()

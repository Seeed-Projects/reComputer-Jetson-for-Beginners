from __future__ import annotations

import argparse
import gc
import statistics
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_PT_MODEL = SCRIPT_DIR / "yolo26s.pt"
DEFAULT_ENGINE_MODEL = SCRIPT_DIR / "yolo26s.engine"
BASE_CANVAS_WIDTH = 1920
BASE_CANVAS_HEIGHT = 1080


@dataclass
class BenchmarkResult:
    label: str
    model_path: Path
    load_time_ms: float
    warmup_frames: int
    benchmark_frames: int
    wall_times_ms: List[float] = field(default_factory=list)
    preprocess_times_ms: List[float] = field(default_factory=list)
    inference_times_ms: List[float] = field(default_factory=list)
    postprocess_times_ms: List[float] = field(default_factory=list)

    @property
    def avg_wall_ms(self) -> float:
        return statistics.fmean(self.wall_times_ms) if self.wall_times_ms else 0.0

    @property
    def median_wall_ms(self) -> float:
        return statistics.median(self.wall_times_ms) if self.wall_times_ms else 0.0

    @property
    def p95_wall_ms(self) -> float:
        return percentile(self.wall_times_ms, 95.0)

    @property
    def avg_fps(self) -> float:
        return 1000.0 / self.avg_wall_ms if self.avg_wall_ms > 0 else 0.0

    @property
    def avg_preprocess_ms(self) -> float:
        return statistics.fmean(self.preprocess_times_ms) if self.preprocess_times_ms else 0.0

    @property
    def avg_inference_ms(self) -> float:
        return statistics.fmean(self.inference_times_ms) if self.inference_times_ms else 0.0

    @property
    def avg_postprocess_ms(self) -> float:
        return statistics.fmean(self.postprocess_times_ms) if self.postprocess_times_ms else 0.0


@dataclass
class LoadedModel:
    label: str
    model_path: Path
    model: object
    result: BenchmarkResult


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare YOLO .pt vs TensorRT .engine video inference speed on the same local video.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--video",
        required=True,
        help="Path to the local input video file.",
    )
    parser.add_argument(
        "--pt-model",
        default=str(DEFAULT_PT_MODEL),
        help="Path to the YOLO .pt model.",
    )
    parser.add_argument(
        "--engine-model",
        default=str(DEFAULT_ENGINE_MODEL),
        help="Path to the YOLO TensorRT .engine model.",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=640,
        help="Inference image size. Keep this aligned with the engine export size.",
    )
    parser.add_argument(
        "--device",
        default="0",
        help="Ultralytics device string, for example 0, 0,1, or cpu.",
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=0.25,
        help="Confidence threshold.",
    )
    parser.add_argument(
        "--iou",
        type=float,
        default=0.45,
        help="IoU threshold for NMS.",
    )
    parser.add_argument(
        "--warmup-frames",
        type=int,
        default=10,
        help="Number of frames used for warmup and excluded from benchmarking.",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Maximum number of benchmarked frames after warmup. Use 0 to keep running until the user exits.",
    )
    parser.add_argument(
        "--frame-stride",
        type=int,
        default=1,
        help="Only benchmark every Nth frame. Useful for very long videos.",
    )
    parser.add_argument(
        "--half",
        action="store_true",
        help="Enable FP16 inference when supported. This mainly affects the .pt model.",
    )
    parser.add_argument(
        "--no-loop",
        action="store_true",
        help="Stop when the video reaches the end instead of restarting from the first frame.",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Disable the comparison window. If combined with looping, stop the script with Ctrl+C.",
    )
    parser.add_argument(
        "--window-scale",
        type=float,
        default=1.0,
        help="Extra scale multiplier for the comparison window.",
    )
    parser.add_argument(
        "--layout",
        choices=("combined", "separate"),
        default="combined",
        help="Display each model in its own window or combine both into one window.",
    )
    return parser.parse_args()


def import_dependencies():
    try:
        import cv2  # type: ignore
    except ImportError as exc:
        raise SystemExit("OpenCV is required. Install it with: pip install opencv-python") from exc

    try:
        from ultralytics import YOLO  # type: ignore
    except ImportError as exc:
        raise SystemExit("Ultralytics is required. Install it with: pip install ultralytics") from exc

    return cv2, YOLO


def validate_args(args: argparse.Namespace) -> Tuple[Path, Path, Path]:
    video_path = Path(args.video).expanduser().resolve()
    pt_model_path = Path(args.pt_model).expanduser().resolve()
    engine_model_path = Path(args.engine_model).expanduser().resolve()

    ensure_file_exists(video_path, "Video")
    ensure_file_exists(pt_model_path, ".pt model")
    ensure_file_exists(engine_model_path, ".engine model")

    if args.imgsz <= 0:
        raise SystemExit("--imgsz must be greater than 0.")
    if args.warmup_frames < 0:
        raise SystemExit("--warmup-frames must be 0 or greater.")
    if args.max_frames < 0:
        raise SystemExit("--max-frames must be 0 or greater.")
    if args.frame_stride <= 0:
        raise SystemExit("--frame-stride must be greater than 0.")
    if args.window_scale <= 0:
        raise SystemExit("--window-scale must be greater than 0.")
    if not 0.0 <= args.conf <= 1.0:
        raise SystemExit("--conf must be between 0 and 1.")
    if not 0.0 <= args.iou <= 1.0:
        raise SystemExit("--iou must be between 0 and 1.")

    return video_path, pt_model_path, engine_model_path


def ensure_file_exists(path: Path, label: str) -> None:
    if not path.exists():
        raise SystemExit(f"{label} not found: {path}")
    if not path.is_file():
        raise SystemExit(f"{label} is not a file: {path}")


def get_video_info(video_path: Path, cv2) -> Dict[str, float]:
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise SystemExit(f"Failed to open video: {video_path}")

    info = {
        "fps": float(capture.get(cv2.CAP_PROP_FPS) or 0.0),
        "total_frames": float(capture.get(cv2.CAP_PROP_FRAME_COUNT) or 0.0),
        "width": float(capture.get(cv2.CAP_PROP_FRAME_WIDTH) or 0.0),
        "height": float(capture.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0.0),
    }
    capture.release()
    return info


def maybe_synchronize_cuda() -> None:
    try:
        import torch  # type: ignore

        if torch.cuda.is_available():
            torch.cuda.synchronize()
    except Exception:
        pass


def maybe_release_cuda() -> None:
    try:
        import torch  # type: ignore

        if torch.cuda.is_available():
            torch.cuda.empty_cache()
    except Exception:
        pass


def percentile(values: List[float], q: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]

    position = (len(ordered) - 1) * (q / 100.0)
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def extract_speed_components(results) -> Dict[str, float]:
    if not results:
        return {}

    speed = getattr(results[0], "speed", None)
    if not isinstance(speed, dict):
        return {}

    extracted = {}
    for key in ("preprocess", "inference", "postprocess"):
        value = speed.get(key)
        if isinstance(value, (int, float)):
            extracted[key] = float(value)
    return extracted


def build_predict_kwargs(args: argparse.Namespace, frame) -> Dict[str, object]:
    kwargs: Dict[str, object] = {
        "source": frame,
        "imgsz": args.imgsz,
        "conf": args.conf,
        "iou": args.iou,
        "device": args.device,
        "verbose": False,
    }
    if args.half:
        kwargs["half"] = True
    return kwargs


def load_model(label: str, model_path: Path, YOLO) -> LoadedModel:
    load_start = time.perf_counter()
    model = YOLO(str(model_path))
    maybe_synchronize_cuda()
    load_time_ms = (time.perf_counter() - load_start) * 1000.0
    print(f"{label} loaded in {load_time_ms:.2f} ms")
    return LoadedModel(
        label=label,
        model_path=model_path,
        model=model,
        result=BenchmarkResult(
            label=label,
            model_path=model_path,
            load_time_ms=load_time_ms,
            warmup_frames=0,
            benchmark_frames=0,
        ),
    )


def open_video_capture(video_path: Path, cv2):
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise SystemExit(f"Failed to open video for benchmarking: {video_path}")
    return capture


def record_measurement(result: BenchmarkResult, wall_ms: float, speed_components: Dict[str, float]) -> None:
    result.wall_times_ms.append(wall_ms)
    result.benchmark_frames += 1

    if "preprocess" in speed_components:
        result.preprocess_times_ms.append(speed_components["preprocess"])
    if "inference" in speed_components:
        result.inference_times_ms.append(speed_components["inference"])
    if "postprocess" in speed_components:
        result.postprocess_times_ms.append(speed_components["postprocess"])


def render_prediction_frame(results, frame):
    if not results:
        return frame.copy()

    min_dim = min(frame.shape[0], frame.shape[1])
    line_width = max(2, int(round(min_dim / 320.0)))

    try:
        return results[0].plot(line_width=line_width)
    except TypeError:
        return results[0].plot()


def get_canvas_size(args: argparse.Namespace) -> Tuple[int, int]:
    canvas_width = max(640, int(round(BASE_CANVAS_WIDTH * args.window_scale)))
    canvas_height = max(360, int(round(BASE_CANVAS_HEIGHT * args.window_scale)))
    return canvas_width, canvas_height


def infer_frame(
    loaded_model: LoadedModel,
    frame,
    args: argparse.Namespace,
    render: bool,
    record_stats: bool,
) -> Tuple[Optional[object], float]:
    predict_kwargs = build_predict_kwargs(args, frame)

    maybe_synchronize_cuda()
    frame_start = time.perf_counter()
    results = loaded_model.model.predict(**predict_kwargs)
    maybe_synchronize_cuda()
    wall_ms = (time.perf_counter() - frame_start) * 1000.0

    if record_stats:
        record_measurement(loaded_model.result, wall_ms, extract_speed_components(results))

    display_frame = None
    if render:
        display_frame = render_prediction_frame(results, frame)

    return display_frame, wall_ms


def add_overlay(frame, lines: List[str], cv2):
    panel = frame.copy()
    overlay = panel.copy()
    frame_height, frame_width = panel.shape[:2]
    font_scale = max(0.65, min(frame_width, frame_height) / 900.0)
    thickness = max(2, int(round(font_scale * 2)))
    line_height = max(28, int(34 * font_scale))
    top = max(16, int(20 * font_scale))
    left = max(16, int(20 * font_scale))
    margin_right = max(16, int(20 * font_scale))
    padding = max(10, int(12 * font_scale))
    box_height = padding * 2 + len(lines) * line_height

    cv2.rectangle(
        overlay,
        (left - padding, top - padding),
        (frame_width - margin_right, top + box_height),
        (8, 8, 8),
        -1,
    )
    cv2.addWeighted(overlay, 0.50, panel, 0.50, 0, panel)

    for index, line in enumerate(lines):
        y = top + index * line_height + line_height - max(8, int(10 * font_scale))
        cv2.putText(
            panel,
            line,
            (left + 1, y + 1),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (0, 0, 0),
            thickness + 2,
            cv2.LINE_AA,
        )
        cv2.putText(
            panel,
            line,
            (left, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (255, 255, 255),
            thickness,
            cv2.LINE_AA,
        )

    return panel


def prepare_panel(
    frame,
    loaded_model: LoadedModel,
    current_wall_ms: float,
    playback_round: int,
    warmup_text: Optional[str],
    source_fps: float,
    effective_display_fps: float,
    cv2,
):
    lines = [
        loaded_model.label,
        f"Current latency: {current_wall_ms:.2f} ms",
        f"Average latency: {loaded_model.result.avg_wall_ms:.2f} ms",
        f"Model FPS: {loaded_model.result.avg_fps:.2f}",
        f"Video FPS: {source_fps:.2f}",
        f"Display target FPS: {effective_display_fps:.2f}",
        f"Benchmarked frames: {loaded_model.result.benchmark_frames}",
        f"Playback round: {playback_round}",
    ]
    if warmup_text:
        lines.append(warmup_text)
    lines.append("Press q or Esc to exit")
    return add_overlay(frame, lines, cv2)


def resize_to_cover(frame, target_width: int, target_height: int, cv2):
    source_height, source_width = frame.shape[:2]
    if source_width <= 0 or source_height <= 0:
        return frame

    scale = max(target_width / source_width, target_height / source_height)
    resized_width = max(target_width, int(round(source_width * scale)))
    resized_height = max(target_height, int(round(source_height * scale)))
    interpolation = cv2.INTER_LINEAR if scale >= 1.0 else cv2.INTER_AREA
    resized = cv2.resize(frame, (resized_width, resized_height), interpolation=interpolation)

    start_x = max(0, (resized_width - target_width) // 2)
    start_y = max(0, (resized_height - target_height) // 2)
    end_x = start_x + target_width
    end_y = start_y + target_height
    return resized[start_y:end_y, start_x:end_x]


def prepare_display_frame(
    plotted_frame,
    loaded_model: LoadedModel,
    current_wall_ms: float,
    playback_round: int,
    warmup_text: Optional[str],
    source_fps: float,
    effective_display_fps: float,
    target_width: int,
    target_height: int,
    cv2,
):
    resized_frame = resize_to_cover(plotted_frame, target_width, target_height, cv2)
    return prepare_panel(
        resized_frame,
        loaded_model,
        current_wall_ms,
        playback_round,
        warmup_text,
        source_fps,
        effective_display_fps,
        cv2,
    )


def show_windows(
    pt_window_name: str,
    pt_frame,
    engine_window_name: str,
    engine_frame,
    combined_window_name: str,
    args: argparse.Namespace,
    wait_ms: int,
    cv2,
) -> bool:
    wait_ms = max(1, wait_ms)

    try:
        if args.layout == "separate":
            cv2.imshow(pt_window_name, pt_frame)
            cv2.imshow(engine_window_name, engine_frame)
        else:
            combined_frame = cv2.hconcat([pt_frame, engine_frame])
            cv2.imshow(combined_window_name, combined_frame)
        key = cv2.waitKey(wait_ms) & 0xFF
    except cv2.error as exc:
        raise SystemExit(
            "Failed to open the display window. If you are on Jetson over SSH, set DISPLAY first "
            "or rerun with --no-show."
        ) from exc

    if key in (ord("q"), 27):
        return True

    try:
        if args.layout == "separate":
            if cv2.getWindowProperty(pt_window_name, cv2.WND_PROP_VISIBLE) < 1:
                return True
            if cv2.getWindowProperty(engine_window_name, cv2.WND_PROP_VISIBLE) < 1:
                return True
        else:
            if cv2.getWindowProperty(combined_window_name, cv2.WND_PROP_VISIBLE) < 1:
                return True
    except cv2.error:
        pass

    return False


def cleanup_loaded_models(*loaded_models: LoadedModel) -> None:
    for loaded_model in loaded_models:
        del loaded_model.model
    gc.collect()
    maybe_release_cuda()


def benchmark_live_compare(
    pt_model_path: Path,
    engine_model_path: Path,
    video_path: Path,
    video_info: Dict[str, float],
    args: argparse.Namespace,
    cv2,
    YOLO,
) -> Tuple[BenchmarkResult, BenchmarkResult]:
    print("\n" + "=" * 72)
    print("Loading models for live comparison")
    pt_loaded = load_model("YOLO PT", pt_model_path, YOLO)
    engine_loaded = load_model("YOLO TensorRT Engine", engine_model_path, YOLO)

    capture = open_video_capture(video_path, cv2)
    pt_window_name = "YOLO PT"
    engine_window_name = "YOLO TensorRT Engine"
    combined_window_name = "YOLO PT vs TensorRT Engine"
    decoded_frames = 0
    warmup_used = 0
    completed_loops = 0
    windows_positioned = False
    source_fps = float(video_info.get("fps", 0.0) or 0.0)
    effective_display_fps = source_fps / max(args.frame_stride, 1) if source_fps > 0 else 0.0
    target_interval_s = 1.0 / effective_display_fps if effective_display_fps > 0 else 0.0
    next_present_time: Optional[float] = None
    canvas_width, canvas_height = get_canvas_size(args)
    panel_width = canvas_width // 2 if args.layout == "combined" else canvas_width
    panel_height = canvas_height

    if not args.no_show:
        window_flags = cv2.WINDOW_NORMAL
        if hasattr(cv2, "WINDOW_FREERATIO"):
            window_flags |= cv2.WINDOW_FREERATIO
        if args.layout == "separate":
            cv2.namedWindow(pt_window_name, window_flags)
            cv2.namedWindow(engine_window_name, window_flags)
            cv2.resizeWindow(pt_window_name, panel_width, panel_height)
            cv2.resizeWindow(engine_window_name, panel_width, panel_height)
        else:
            cv2.namedWindow(combined_window_name, window_flags)
            cv2.resizeWindow(combined_window_name, canvas_width, canvas_height)

    try:
        while True:
            success, frame = capture.read()
            if not success:
                if args.no_loop:
                    break

                completed_loops += 1
                capture.release()
                capture = open_video_capture(video_path, cv2)
                continue

            decoded_frames += 1
            if (decoded_frames - 1) % args.frame_stride != 0:
                continue

            record_stats = warmup_used >= args.warmup_frames

            pt_frame, pt_wall_ms = infer_frame(pt_loaded, frame, args, render=not args.no_show, record_stats=record_stats)
            engine_frame, engine_wall_ms = infer_frame(
                engine_loaded,
                frame,
                args,
                render=not args.no_show,
                record_stats=record_stats,
            )

            warmup_text = None
            if not record_stats:
                warmup_used += 1
                warmup_text = f"Warmup {warmup_used}/{args.warmup_frames}"

            if not args.no_show and pt_frame is not None and engine_frame is not None:
                playback_round = completed_loops + 1
                pt_panel = prepare_display_frame(
                    pt_frame,
                    pt_loaded,
                    pt_wall_ms,
                    playback_round,
                    warmup_text,
                    source_fps,
                    effective_display_fps,
                    panel_width,
                    panel_height,
                    cv2,
                )
                engine_panel = prepare_display_frame(
                    engine_frame,
                    engine_loaded,
                    engine_wall_ms,
                    playback_round,
                    warmup_text,
                    source_fps,
                    effective_display_fps,
                    panel_width,
                    panel_height,
                    cv2,
                )

                if args.layout == "separate" and not windows_positioned:
                    try:
                        cv2.moveWindow(pt_window_name, 40, 40)
                        cv2.moveWindow(engine_window_name, pt_panel.shape[1] + 80, 40)
                    except cv2.error:
                        pass
                    windows_positioned = True

                wait_ms = 1
                if target_interval_s > 0:
                    now = time.perf_counter()
                    if next_present_time is None:
                        next_present_time = now
                    scheduled_time = next_present_time + target_interval_s
                    remaining_s = scheduled_time - now
                    wait_ms = max(1, int(round(remaining_s * 1000.0))) if remaining_s > 0 else 1
                    next_present_time = scheduled_time if remaining_s > 0 else now

                if show_windows(
                    pt_window_name,
                    pt_panel,
                    engine_window_name,
                    engine_panel,
                    combined_window_name,
                    args,
                    wait_ms,
                    cv2,
                ):
                    break

            if record_stats and args.max_frames > 0 and pt_loaded.result.benchmark_frames >= args.max_frames:
                break
    finally:
        capture.release()
        if not args.no_show:
            cv2.destroyAllWindows()

    pt_loaded.result.warmup_frames = warmup_used
    engine_loaded.result.warmup_frames = warmup_used

    if pt_loaded.result.benchmark_frames == 0 or engine_loaded.result.benchmark_frames == 0:
        cleanup_loaded_models(pt_loaded, engine_loaded)
        raise SystemExit(
            "No frames were benchmarked. Try lowering --warmup-frames, using a longer video, "
            "or increasing --max-frames."
        )

    print(f"Warmup frames: {warmup_used}")
    print(f"Benchmarked frames: {pt_loaded.result.benchmark_frames}")

    pt_result = pt_loaded.result
    engine_result = engine_loaded.result
    cleanup_loaded_models(pt_loaded, engine_loaded)
    return pt_result, engine_result


def print_video_info(video_path: Path, info: Dict[str, float], args: argparse.Namespace) -> None:
    total_frames_text = "unknown"
    if info["total_frames"] > 0:
        total_frames_text = str(int(info["total_frames"]))
    effective_display_fps = info["fps"] / max(args.frame_stride, 1) if info["fps"] > 0 else 0.0
    canvas_width, canvas_height = get_canvas_size(args)

    print("=" * 72)
    print("Video benchmark configuration")
    print(f"Video:         {video_path}")
    print(f"Resolution:    {int(info['width'])} x {int(info['height'])}")
    print(f"Video FPS:     {info['fps']:.2f}")
    print(f"Total frames:  {total_frames_text}")
    print(f"Image size:    {args.imgsz}")
    print(f"Device:        {args.device}")
    print(f"Warmup frames: {args.warmup_frames}")
    print(f"Max frames:    {'until user exits' if args.max_frames == 0 else args.max_frames}")
    print(f"Frame stride:  {args.frame_stride}")
    print(f"Half:          {args.half}")
    print(f"Loop video:    {not args.no_loop}")
    print(f"Show window:   {not args.no_show}")
    print(f"Layout:        {args.layout}")
    print(f"Canvas size:   {canvas_width} x {canvas_height}")
    if effective_display_fps > 0:
        print(f"Display FPS:   {effective_display_fps:.2f} (limited by source video/frame stride)")


def print_comparison(pt_result: BenchmarkResult, engine_result: BenchmarkResult) -> None:
    print("\n" + "=" * 72)
    print("Comparison summary")
    print(
        f"{'Metric':<24}"
        f"{'.pt':>14}"
        f"{'.engine':>14}"
    )
    print("-" * 52)
    print(f"{'Load time (ms)':<24}{pt_result.load_time_ms:>14.2f}{engine_result.load_time_ms:>14.2f}")
    print(f"{'Avg wall (ms)':<24}{pt_result.avg_wall_ms:>14.2f}{engine_result.avg_wall_ms:>14.2f}")
    print(f"{'Median wall (ms)':<24}{pt_result.median_wall_ms:>14.2f}{engine_result.median_wall_ms:>14.2f}")
    print(f"{'P95 wall (ms)':<24}{pt_result.p95_wall_ms:>14.2f}{engine_result.p95_wall_ms:>14.2f}")
    print(f"{'Avg FPS':<24}{pt_result.avg_fps:>14.2f}{engine_result.avg_fps:>14.2f}")
    print(f"{'Avg preprocess (ms)':<24}{pt_result.avg_preprocess_ms:>14.2f}{engine_result.avg_preprocess_ms:>14.2f}")
    print(f"{'Avg inference (ms)':<24}{pt_result.avg_inference_ms:>14.2f}{engine_result.avg_inference_ms:>14.2f}")
    print(f"{'Avg postprocess (ms)':<24}{pt_result.avg_postprocess_ms:>14.2f}{engine_result.avg_postprocess_ms:>14.2f}")

    wall_speedup = pt_result.avg_wall_ms / engine_result.avg_wall_ms if engine_result.avg_wall_ms > 0 else 0.0
    fps_speedup = engine_result.avg_fps / pt_result.avg_fps if pt_result.avg_fps > 0 else 0.0

    print("-" * 52)
    print(f"TensorRT engine wall-latency speedup: {wall_speedup:.2f}x")
    print(f"TensorRT engine FPS speedup:          {fps_speedup:.2f}x")


def main() -> int:
    args = parse_args()
    video_path, pt_model_path, engine_model_path = validate_args(args)
    cv2, YOLO = import_dependencies()

    video_info = get_video_info(video_path, cv2)
    print_video_info(video_path, video_info, args)
    if not args.no_show:
        print("Comparison window will loop the video continuously. Press q or Esc to stop.")
        print("Note: visible playback smoothness is capped by the video's own FPS, even if model FPS is higher.")
    else:
        print("Display window disabled. If looping is enabled, use Ctrl+C to stop.")

    pt_result, engine_result = benchmark_live_compare(pt_model_path, engine_model_path, video_path, video_info, args, cv2, YOLO)

    print_comparison(pt_result, engine_result)
    print("\nNote: timing excludes video decode time and measures the model call per frame.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

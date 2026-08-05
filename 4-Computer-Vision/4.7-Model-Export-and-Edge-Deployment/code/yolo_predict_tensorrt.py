# YOLO11 TensorRT 模型预测示例
# 本脚本演示如何使用训练好的 TensorRT 引擎模型（best.engine）进行实时物体检测。
# TensorRT 引擎模型经过优化，在 Jetson 等边缘设备上具有更快的推理速度。
#
# 使用前请确保已通过模型转换将 .pt 模型导出为 .engine 模型。
# 参考 7.02.10 模型转换 章节。

from pathlib import Path

import cv2
from ultralytics import YOLO

SCRIPT_DIR = Path(__file__).resolve().parent


def predict_with_tensorrt(engine_path: str = None):
    """使用 TensorRT 引擎模型进行实时物体检测。

    Args:
        engine_path: TensorRT 引擎模型文件路径（best.engine）
    """
    if engine_path is None:
        engine_path = str(SCRIPT_DIR / "best.engine")

    # 检查模型文件是否存在
    if not Path(engine_path).exists():
        print(f"错误：未找到模型文件 {engine_path}")
        print("请先训练模型并导出为 TensorRT 引擎格式（.engine）。")
        print("参考 7.02.10 模型转换 章节将 .pt 模型导出为 .engine 模型。")
        return

    # 加载 TensorRT 引擎模型
    model = YOLO(engine_path)

    # 初始化视频捕获设备，参数 0 表示使用系统默认的摄像头
    cap = cv2.VideoCapture(0)

    # 检查摄像头是否成功打开
    if not cap.isOpened():
        print("错误：无法打开摄像头")
        return

    print("开始实时检测... (按 'q' 退出)")

    # 主循环：持续从摄像头捕获帧并进行处理
    while True:
        # 从摄像头读取一帧图像
        success, frame = cap.read()
        if not success:
            print("错误：无法从摄像头读取帧")
            break

        # 使用 YOLO 模型对当前帧进行推理
        # verbose=False 禁用模型内部的详细日志输出，保持控制台整洁
        results = model(frame, verbose=False)

        # 可视化检测结果：绘制边界框、类别标签和置信度分数
        annotated_frame = results[0].plot()

        # 显示处理后的图像帧
        cv2.imshow("Result", annotated_frame)

        # 检查键盘输入，按 'q' 退出
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    # 释放资源并清理窗口
    cap.release()
    cv2.destroyAllWindows()
    print("检测结束")


if __name__ == "__main__":
    predict_with_tensorrt()

# YOLO26 物体检测示例
# 本脚本演示如何使用 Ultralytics YOLO26 进行物体检测，包含图片推理和摄像头实时检测两种方式。
# 模型文件会自动下载（如果本地不存在）或从本地加载。

from pathlib import Path

import cv2
from ultralytics import YOLO

SCRIPT_DIR = Path(__file__).resolve().parent


def detect_image(image_path: str = "image1.jpg"):
    """对单张图片进行物体检测推理，并显示和保存结果。"""
    # 加载预训练的 YOLO26n 模型（轻量级版本，适合边缘设备和实时应用）
    model = YOLO("yolo26n.pt")

    # 对图像列表进行批量推理，返回 Results 对象列表
    results = model([image_path])

    # 处理结果列表
    for result in results:
        # result.boxes: 边界框输出对象（xyxy、xywh、conf、cls 等）
        boxes = result.boxes
        # result.masks: 分割掩码输出对象（仅分割模型可用）
        masks = result.masks
        # result.keypoints: 关键点输出对象（仅姿态估计模型可用）
        keypoints = result.keypoints
        # result.probs: 分类概率输出对象（仅分类模型可用）
        probs = result.probs
        # result.obb: 定向边界框输出对象（仅 OBB 模型可用）
        obb = result.obb

        # 在屏幕上显示推理结果
        result.show()
        # 将结果保存到磁盘
        result.save(filename=str(SCRIPT_DIR / "result.jpg"))

        # 注意：以上属性（masks, keypoints, probs, obb）可能为 None，
        # 取决于模型类型。yolo26n.pt 是目标检测模型，通常只包含 boxes。


def detect_camera():
    """使用摄像头进行实时物体检测。"""
    # 加载预训练 YOLO26n 模型
    # 如需更高精度可替换为 'yolo26s.pt'、'yolo26m.pt' 或 'yolo26x.pt'
    model = YOLO("yolo26n.pt")

    # 初始化视频捕获设备，参数 0 表示系统默认摄像头
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
        # verbose=False 禁用模型内部的详细日志输出
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
    # 演示摄像头实时检测
    # 如需对图片进行推理，可调用 detect_image("image1.jpg")
    detect_camera()

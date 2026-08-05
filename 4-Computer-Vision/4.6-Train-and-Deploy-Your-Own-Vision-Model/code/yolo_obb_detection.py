# YOLO11 定向物体检测示例
# 本脚本演示如何使用 Ultralytics YOLO11 进行定向物体检测（Oriented Object Detection）。
# 使用 yolo26n-obb.pt 模型，使用旋转边界框（Rotated Bounding Box）来表示物体的位置和方向。

import cv2
from ultralytics import YOLO


def detect_obb_camera():
    """使用摄像头进行实时定向物体检测。"""
    # 加载预训练的 YOLO 定向物体检测模型
    # "yolo26n" 表示 YOLO 版本 26 的纳米（nano）模型，适合边缘设备
    # "obb" 代表 Oriented Bounding Boxes（定向边界框），支持检测带有方向的物体
    # 如果本地没有该模型文件，程序会尝试从 Ultralytics 的官方服务器下载
    model = YOLO("yolo26n-obb.pt")

    # 初始化视频捕获设备，参数 0 表示使用计算机的默认摄像头
    cap = cv2.VideoCapture(0)

    print("开始定向物体检测... (按 'q' 退出)")

    # 主循环：持续从摄像头捕获视频帧并进行处理
    while True:
        # 从摄像头读取一帧图像
        ret, frame = cap.read()
        if not ret:
            print("错误：无法从摄像头读取帧")
            break

        # 使用 YOLO 模型对当前帧进行推理
        # verbose=False 不输出模型的详细推理信息，使控制台输出更简洁
        results = model(frame, verbose=False)

        # 可视化检测结果：绘制定向边界框（旋转矩形框）、类别标签和置信度分数
        annotated_frame = results[0].plot()

        # 在窗口中显示处理后的图像帧，调整为 640x480 像素
        cv2.imshow("Result", cv2.resize(annotated_frame, (640, 480)))

        # 检查键盘输入，按 'q' 退出
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    # 释放资源并清理窗口
    cap.release()
    cv2.destroyAllWindows()
    print("检测结束")


if __name__ == "__main__":
    detect_obb_camera()

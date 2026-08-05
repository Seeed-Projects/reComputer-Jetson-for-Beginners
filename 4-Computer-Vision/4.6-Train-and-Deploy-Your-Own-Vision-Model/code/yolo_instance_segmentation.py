# YOLO11 实例分割示例
# 本脚本演示如何使用 Ultralytics YOLO11 进行实时实例分割。
# 使用 yolo26n-seg.pt 模型，不仅绘制边界框，还为每个物体实例生成像素级掩码。

import cv2
from ultralytics import YOLO


def segment_camera():
    """使用摄像头进行实时实例分割。"""
    # 加载预训练的 YOLO 实例分割模型
    # 模型文件后缀为 '-seg'，表示专门用于分割的模型
    # 可根据精度和速度需求替换为：
    # 'yolo26s-seg.pt' (小型), 'yolo26m-seg.pt' (中型), 'yolo26l-seg.pt' (大型), 'yolo26x-seg.pt' (超大型)
    model = YOLO("yolo26n-seg.pt")

    # 初始化视频捕获设备，参数 0 代表系统默认摄像头
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("错误：无法打开摄像头")
        return

    print("开始实时实例分割... (按 'q' 键退出)")

    # 主循环：持续处理视频流
    while True:
        # 从摄像头读取一帧
        success, frame = cap.read()
        if not success:
            print("错误：无法从摄像头读取帧")
            break

        # 使用 YOLO 分割模型对当前帧进行推理
        # verbose=False 关闭模型内部的冗长日志，保持控制台输出简洁
        results = model(frame, verbose=False)

        # 可视化结果：绘制边界框、类别标签、置信度以及半透明彩色掩码
        # 不同实例的掩码颜色不同，易于区分
        annotated_frame = results[0].plot()

        # （可选）访问原始的分割结果数据进行自定义处理：
        # result = results[0]  # 获取第一个结果对象
        # boxes = result.boxes   # 边界框数据 (xyxy, conf, cls)
        # masks = result.masks   # 分割掩码数据
        # class_ids = result.boxes.cls  # 类别ID

        # 显示处理后的帧
        cv2.imshow("YOLO Instance Segmentation", annotated_frame)

        # 监听键盘输入，按 'q' 退出循环
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    # 释放资源并清理窗口
    cap.release()
    cv2.destroyAllWindows()
    print("实例分割结束")


if __name__ == "__main__":
    segment_camera()

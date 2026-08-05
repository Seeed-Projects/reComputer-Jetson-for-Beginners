# YOLO11 图像分类示例
# 本脚本演示如何使用 Ultralytics YOLO11 进行实时图像分类。
# 使用 yolo11n-cls.pt 模型，在 ImageNet 数据集上训练，可识别 1000 个不同类别。

import cv2
from ultralytics import YOLO


def classify_camera():
    """使用摄像头进行实时图像分类。"""
    # 加载预训练的 YOLO 图像分类模型
    # '-cls' 表示这是分类模型，在 ImageNet 数据集上训练
    # 可识别 1000 个不同的物体类别（动物、日常物品、交通工具、食物等）
    # 如果本地没有模型文件，程序会自动从 Ultralytics 服务器下载
    model = YOLO("yolo11n-cls.pt")

    # 初始化视频捕获设备，参数 0 打开系统默认的摄像头
    cap = cv2.VideoCapture(0)

    # 检查摄像头是否成功打开
    if not cap.isOpened():
        print("错误：无法打开摄像头")
        return

    print("开始实时图像分类... (按 'q' 退出)")

    # 主循环：持续从摄像头捕获视频帧并进行实时处理
    while True:
        # 从摄像头读取一帧图像
        ret, frame = cap.read()
        if not ret:
            print("错误：无法从摄像头读取帧")
            break

        # 使用 YOLO 模型对当前帧进行图像分类推理
        # verbose=False 不输出详细的推理过程信息，保持控制台输出简洁
        results = model(frame, verbose=False)

        # 解析并获取分类结果
        result = results[0]

        # 检查结果对象是否包含分类概率信息（probs 属性）
        if hasattr(result, "probs"):
            # 获取概率最高的类别索引（top1 类别）
            top1_index = result.probs.top1
            # 获取最高概率值（置信度）
            top1_conf = result.probs.top1conf
            # 使用类别索引从 names 字典中获取对应的类别名称
            top1_class = result.names[top1_index]

            # 在当前帧上绘制分类结果文本：类别名称 + 置信度百分比
            text = f"{top1_class}: {top1_conf:.2%}"
            cv2.putText(
                frame,
                text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
            )

        # 在窗口中显示处理后的图像帧
        cv2.imshow("Real-time Classification", frame)

        # 检查键盘输入，按 'q' 退出
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    # 释放资源并清理窗口
    cap.release()
    cv2.destroyAllWindows()
    print("结束")


if __name__ == "__main__":
    classify_camera()

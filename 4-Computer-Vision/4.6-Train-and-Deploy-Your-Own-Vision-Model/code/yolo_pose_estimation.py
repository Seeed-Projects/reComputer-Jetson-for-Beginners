# YOLO26 姿态估计示例
# 本脚本演示如何使用 Ultralytics YOLO26 进行实时人体姿态估计。
# 使用 yolo26n-pose.pt 模型，检测人体的 17 个关键点并绘制骨架连接线。

import cv2
from ultralytics import YOLO


def estimate_pose():
    """使用摄像头进行实时姿态估计。"""
    # 加载预训练的 YOLO 姿态估计模型
    # 'yolo26n-pose.pt' 是纳米（nano）版本，专为姿态估计任务设计
    # 该模型在 COCO 数据集上训练，能够检测人体的 17 个关键点
    # 文件会自动从 Ultralytics 服务器下载（如果本地不存在）
    model = YOLO("yolo26n-pose.pt")

    # 初始化视频捕获设备，参数 0 表示打开系统默认的摄像头
    cap = cv2.VideoCapture(0)

    # 检查摄像头是否成功打开
    if not cap.isOpened():
        print("错误：无法打开摄像头")
        return

    print("开始姿态估计... (按 'q' 退出)")

    # 主循环：持续从摄像头读取帧并进行处理
    while True:
        # 从摄像头读取一帧图像
        ret, frame = cap.read()
        if not ret:
            print("错误：无法从摄像头读取帧")
            break

        # 使用 YOLO 模型对当前帧进行姿态估计推理
        # verbose=False 不输出详细的推理过程信息，保持控制台简洁
        results = model(frame, verbose=False)

        # 可视化推理结果：绘制人体边界框、17 个关键点、骨架连接线和置信度分数
        result_frame = results[0].plot()

        # 显示处理后的图像
        cv2.imshow("Pose Estimation", result_frame)

        # 检查键盘输入，按 'q' 退出
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    # 释放资源并清理窗口
    cap.release()
    cv2.destroyAllWindows()
    print("结束")


if __name__ == "__main__":
    estimate_pose()

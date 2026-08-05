# QR 二维码识别示例
# 本脚本演示如何使用 pyzbar 和 OpenCV 进行实时二维码识别。
# 通过摄像头捕获视频流，检测画面中的二维码并绘制检测框和识别内容。

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from pyzbar import pyzbar


def decode_display(image, font_path=None):
    """检测并识别图像中的二维码，在图像上绘制检测框和识别内容。

    Args:
        image: 输入图像（BGR 格式）
        font_path: 中文字体文件路径，用于在图像上绘制中文文本

    Returns:
        绘制了检测框和识别内容的图像
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # 解码二维码
    barcodes = pyzbar.decode(gray)

    for barcode in barcodes:
        # 提取二维码的边界框的位置
        (x, y, w, h) = barcode.rect
        # 画出图像中条形码的边界框
        cv2.rectangle(image, (x, y), (x + w, y + h), (225, 0, 0), 5)

        encoding = "UTF-8"
        # 将二维码数据转换成字符串
        barcodeData = barcode.data.decode(encoding)
        barcodeType = barcode.type

        # 使用 PIL 绘制中文文本
        pilimg = Image.fromarray(image)
        # 创建画笔
        draw = ImageDraw.Draw(pilimg)

        if font_path:
            # 参数1：字体文件路径，参数2：字体大小
            fontStyle = ImageFont.truetype(font_path, size=12, encoding=encoding)
            # 参数1：打印坐标，参数2：文本，参数3：字体颜色，参数4：字体
            draw.text(
                (x, y - 25),
                str(barcode.data, encoding),
                fill=(255, 0, 0),
                font=fontStyle,
            )

        # PIL 图片转 cv2 图片
        image = cv2.cvtColor(np.array(pilimg), cv2.COLOR_RGB2BGR)
        # 向终端打印条形码数据和条形码类型
        print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))

    return image


def detect_qr_camera(font_path=None):
    """使用摄像头进行实时二维码识别。

    Args:
        font_path: 中文字体文件路径（可选）
    """
    # 初始化视频捕获设备
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("错误：无法打开摄像头")
        return

    print("开始二维码识别... (按 'q' 退出)")

    while True:
        # 从摄像头读取一帧图像
        ret, frame = cap.read()
        if not ret:
            print("错误：无法从摄像头读取帧")
            break

        # 检测并识别二维码
        frame = decode_display(frame, font_path)

        # 显示处理后的图像
        cv2.imshow("QR Code Detection", frame)

        # 检查键盘输入，按 'q' 退出
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    # 释放资源并清理窗口
    cap.release()
    cv2.destroyAllWindows()
    print("识别结束")


if __name__ == "__main__":
    detect_qr_camera()

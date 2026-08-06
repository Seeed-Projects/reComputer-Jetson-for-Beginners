# QR 二维码生成示例
# 本脚本演示如何使用 qrcode 和 PIL 库生成带有自定义 Logo 的二维码。
# 生成的二维码会保存到 figures 目录下，并自动弹出预览窗口。

from pathlib import Path

import qrcode
from PIL import Image

# Pillow >= 10.0 compatibility
PIL_RESAMPLING = getattr(Image, "Resampling", Image)

SCRIPT_DIR = Path(__file__).resolve().parent


def add_logo(img, logo_path):
    """在二维码图片中心添加 Logo。

    Args:
        img: 二维码 PIL 图片对象
        logo_path: Logo 图片文件路径

    Returns:
        添加了 Logo 的 PIL 图片对象
    """
    # 打开 Logo 图片
    icon = Image.open(logo_path)
    img_w, img_h = img.size

    # 设置 Logo 的大小（为二维码的 1/6）
    factor = 6
    size_w = int(img_w / factor)
    size_h = int(img_h / factor)
    icon_w, icon_h = icon.size
    if icon_w > size_w:
        icon_w = size_w
    if icon_h > size_h:
        icon_h = size_h

    # 调整 Logo 大小
    icon = icon.resize((icon_w, icon_h), PIL_RESAMPLING.LANCZOS)

    # 将 Logo 居中粘贴到二维码上
    w = int((img_w - icon_w) / 2)
    h = int((img_h - icon_h) / 2)
    img.paste(icon, (w, h), mask=None)
    return img


def create_qrcode(data, file_name, logo_path):
    """生成带有 Logo 的二维码。

    参数含义：
        version: 值为 1~40 的整数，控制二维码的大小（最小值是 1，是个 12×12 的矩阵）。
                 如果想让程序自动确定，将值设置为 None 并使用 fit 参数即可。
        error_correction: 控制二维码的错误纠正功能。
            ERROR_CORRECT_L: 大约 7% 或更少的错误能被纠正。
            ERROR_CORRECT_M（默认）: 大约 15% 或更少的错误能被纠正。
            ERROR_CORRECT_H: 大约 30% 或更少的错误能被纠正。
        box_size: 控制二维码中每个小格子包含的像素数。
        border: 控制边框（二维码与图片边界的距离）包含的格子数（默认为 4，是相关标准规定的最小值）

    Args:
        data: 要编码到二维码中的文本数据
        file_name: 输出二维码图片的文件路径
        logo_path: Logo 图片文件路径
    """
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_H,
        box_size=10,
        border=4,
    )
    # 添加数据到二维码
    qr.add_data(data)
    print(data)
    qr.make(fit=True)
    img = qr.make_image(fill_color="green", back_color="white")

    # 如果 Logo 文件存在则添加 Logo
    if Path(logo_path).is_file():
        img = add_logo(img, logo_path)

    # 保存并显示图片
    img.save(file_name)
    img.show()
    return img


if __name__ == "__main__":
    # 输出目录和 Logo 图片路径
    figures_dir = SCRIPT_DIR / "figures"
    figures_dir.mkdir(parents=True, exist_ok=True)
    logo_path = str(figures_dir / "seeed_logo.png")
    out_img = str(figures_dir / "seeed_logo_qr.jpg")

    # 获取用户输入并生成二维码
    text = input("Please enter:")
    create_qrcode(text, out_img, logo_path)

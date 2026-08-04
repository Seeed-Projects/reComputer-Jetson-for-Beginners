import time
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont

# 初始化 OLED
I2C_BUS = 7
I2C_ADDR = 0x3C
serial = i2c(port=I2C_BUS, address=I2C_ADDR)
device = ssd1306(serial)
device.clear()

# 准备字体
FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
font = ImageFont.truetype(FONT_PATH, 16)

# 准备滚动文字
text = "你好，这是Jetson的I2C测试！"  # 中文 + 英文
text += "    "  # 尾部空格保证循环滚动

# 计算文字尺寸
bbox = font.getbbox(text)
text_width = bbox[2] - bbox[0]
text_height = bbox[3] - bbox[1]
scroll_pos = device.width

# 循环刷新 OLED
while True:
    img = Image.new("1", (device.width, device.height))
    draw = ImageDraw.Draw(img)

    draw.text((scroll_pos, (device.height - text_height)//2), text, font=font, fill=255)
    device.display(img)

    scroll_pos -= 2
    if scroll_pos + text_width < 0:
        scroll_pos = device.width

    time.sleep(0.05)


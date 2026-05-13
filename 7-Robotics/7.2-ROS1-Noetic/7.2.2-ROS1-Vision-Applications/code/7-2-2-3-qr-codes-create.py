#!/usr/bin/env python3
import os
import qrcode
from PIL import Image
from pathlib import Path


def add_logo(img, logo_path):
    # Add logo, open logo image
    icon = Image.open(    logo_path)
    img_w, img_h = img.size
    # Set the size of the logo
    factor = 6
    size_w = int(img_w / factor)
    size_h = int(img_h / factor)
    icon_w, icon_h = icon.size
    if icon_w > size_w: icon_w = size_w
    if icon_h > size_h: icon_h = size_h
    # Resize the logo
    icon = icon.resize((icon_w, icon_h), Image.Resampling.LANCZOS)
    # Center the logo
    w = int((img_w - icon_w) / 2)
    h = int((img_h - icon_h) / 2)
    # Paste the logo
    img.paste(icon, (w, h), mask=None)
    return img


def create_qrcode(data, file_name, logo_path):
    '''
    version: Integer from 1 to 40, controls the size of the QR code.
    error_correction: Controls the error correction function. Can be one of the following:
        ERROR_CORRECT_L: About 7% or fewer errors can be corrected.
        ERROR_CORRECT_M (default): About 15% or fewer errors can be corrected.
        ERROR_CORRECT_H: About 30% or fewer errors can be corrected.
    box_size: Controls the number of pixels in each box of the QR code.
    border: Controls the number of boxes for the border (the default is 4).
    '''
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_H,
        box_size=10,
        border=4)
    # Add data to the QR code
    qr.add_data(data)
    print(data)
    qr.make(fit=True)
    img = qr.make_image(fill_color="green", back_color="white")
    # Add logo if file exists
    if Path(logo_path).is_file():
        img = add_logo(img, logo_path)
    # Save and show the image
    img.save(file_name)
    img.show()
    return img


if __name__ == '__main__':
    file_path = "./figures"
    logo_path = file_path + "/seeed_logo.png"
    out_img = file_path + '/seeed_logo_qr.jpg'
    text = input("Please enter:")
    create_qrcode(text, out_img, logo_path)

def decodeDisplay(image, font_path):
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    # Convert decoded text to Unicode before drawing it on the image.
    barcodes = pyzbar.decode(gray)
    for barcode in barcodes:
        # Extract the QR code bounding box.
        (x, y, w, h) = barcode.rect
        # Draw the barcode bounding box on the image.
        cv.rectangle(image, (x, y), (x + w, y + h), (225, 0, 0), 5)
        encoding = 'UTF-8'
        # Convert the decoded bytes to a string before drawing.
        barcodeData = barcode.data.decode(encoding)
        barcodeType = barcode.type
        # Draw the decoded data and type on the image.
        pilimg = Image.fromarray(image)
        # Create a drawing object.
        draw = ImageDraw.Draw(pilimg)
        # Parameter 1: font file path. Parameter 2: font size.
        fontStyle = ImageFont.truetype(font_path, size=12, encoding=encoding)
        # Parameter 1: text position. Parameter 2: text. Parameter 3: text color. Parameter 4: font.
        draw.text((x, y - 25), str(barcode.data, encoding), fill=(255, 0, 0), font=fontStyle)
        # Convert the PIL image back to an OpenCV image.
        image = cv.cvtColor(np.array(pilimg), cv.COLOR_RGB2BGR)
        # Print barcode data and type to the terminal.
        print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
    return image

import cv2
import pytesseract
import os

image_path = '/root/erc_images/ocr_debug_thresh.jpg'
if not os.path.exists(image_path):
    print("File not found.")
    exit()

img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

for psm in [6, 11]:
    print(f"\n--- Testing image_to_boxes PSM {psm} ---")
    custom_config = f'--oem 3 --psm {psm} -c tessedit_char_whitelist=12345'
    boxes = pytesseract.image_to_boxes(img, config=custom_config)
    for b in boxes.splitlines():
        b = b.split()
        if len(b) >= 5:
            char = b[0]
            if char in ['1', '2', '3', '4', '5']:
                x = int(b[1]) + (int(b[3]) - int(b[1])) / 2.0
                print(f"Found {char} at x={x}")

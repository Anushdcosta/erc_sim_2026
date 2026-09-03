import cv2
import pytesseract
from pytesseract import Output
import os

image_path = '/root/erc_images/ocr_debug_thresh.jpg'
if not os.path.exists(image_path):
    print("File not found.")
    exit()

img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

for psm in [6, 11, 4, 12]:
    print(f"\n--- Testing PSM {psm} ---")
    custom_config = f'--oem 3 --psm {psm} -c tessedit_char_whitelist=12345'
    d = pytesseract.image_to_data(img, output_type=Output.DICT, config=custom_config)
    
    n_boxes = len(d['text'])
    for i in range(n_boxes):
        text = ''.join(filter(str.isdigit, d['text'][i]))
        if text:
            print(f"Found '{text}' at x={d['left'][i]} with conf {d['conf'][i]}")

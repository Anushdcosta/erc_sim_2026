import cv2
import pytesseract
import sys

img = cv2.imread('/root/erc_images/ocr_all_shelves.jpg')
h, w, _ = img.shape
crop_h = int(h / 3.0)
img = img[0:crop_h, :]
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
_, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)

print("--- image_to_data ---")
custom_config = '--oem 3 --psm 11 -c tessedit_char_whitelist=12345'
data = pytesseract.image_to_data(thresh, config=custom_config, output_type=pytesseract.Output.DICT)
for i in range(len(data['text'])):
    print(f"Conf: {data['conf'][i]}, Text: '{data['text'][i]}', x={data['left'][i]}, y={data['top'][i]}, w={data['width'][i]}, h={data['height'][i]}")

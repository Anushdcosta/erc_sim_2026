import cv2
import pytesseract
import numpy as np

image_path = '/root/erc_images/ocr_debug_thresh.jpg'
img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

# Find contours
contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

number_contours = []
for c in contours:
    x, y, w, h = cv2.boundingRect(c)
    # The numbers are at the top of the image (y < 200) and have a decent size
    if 10 < w < 100 and 15 < h < 100 and y < 180:
        number_contours.append((x, y, w, h))

print(f"Found {len(number_contours)} potential number contours.")

# Sort left to right
number_contours.sort(key=lambda item: item[0])

for i, (x, y, w, h) in enumerate(number_contours):
    # Extract ROI with padding
    pad = 10
    roi = img[max(0, y-pad):min(img.shape[0], y+h+pad), max(0, x-pad):min(img.shape[1], x+w+pad)]
    
    # Run tesseract on single char
    config = '--psm 10 -c tessedit_char_whitelist=12345'
    text = pytesseract.image_to_string(roi, config=config).strip()
    
    print(f"Contour {i} at x={x}: read '{text}'")

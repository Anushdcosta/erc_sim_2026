import cv2
import pytesseract
import numpy as np

thresh = cv2.imread('/root/erc_images/ocr_debug_thresh.jpg', cv2.IMREAD_GRAYSCALE)
if thresh is None:
    print("Could not read image")
    exit(1)

# Find contours
contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

found_items = {}

print(f"Found {len(contours)} contours")
for cnt in contours:
    x, y, w, h = cv2.boundingRect(cnt)
    # The digits are quite large, probably w > 15 and h > 20
    if w > 10 and h > 15 and w < 100 and h < 100:
        # Extract ROI
        roi = thresh[y:y+h, x:x+w]
        
        # Add a white border (Tesseract needs white padding around black text)
        # But wait, thresh is binary INV, meaning numbers are WHITE on BLACK background!
        # If numbers are WHITE, we need to invert it back for Tesseract (black text on white background)
        roi_inv = cv2.bitwise_not(roi)
        padded = cv2.copyMakeBorder(roi_inv, 10, 10, 10, 10, cv2.BORDER_CONSTANT, value=[255, 255, 255])
        
        custom_config = '--oem 3 --psm 10 -c tessedit_char_whitelist=12345'
        char = pytesseract.image_to_string(padded, config=custom_config).strip()
        
        print(f"Contour at x={x}, y={y}, w={w}, h={h} -> OCR: '{char}'")
        if char in ['1', '2', '3', '4', '5']:
            found_items[int(char)] = x + w/2.0

print(found_items)

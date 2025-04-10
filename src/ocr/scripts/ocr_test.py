import cv2
import numpy as np
import pytesseract

# 配置 pytesseract 路径 (WINDOWS需要)
#pytesseract.pytesseract.tesseract_cmd = r'C:\Program Files\Tesseract-OCR\tesseract'


# 读取图像
image_path = 'test.jpg'  # 替换为你的图像路径
image = cv2.imread(image_path)

# 转换为灰度图像
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# 获取图像尺寸并提取 ROI (感兴趣区域)
h, w = gray.shape
roi = gray[int(0.21*h):int(0.79*h), int(0.25*w):int(0.75*w)]

# 缩小图像（例如缩小为原来的 50%）
scale_percent = 70  # 缩小比例（你可以根据需要调整）
width = int(roi.shape[1] * scale_percent / 100)
height = int(roi.shape[0] * scale_percent / 100)
dim = (width, height)
resized_roi = cv2.resize(roi, dim, interpolation=cv2.INTER_AREA)

# 显示缩小后的 ROI
# cv2.imshow("Resized ROI", resized_roi)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
cv2.imwrite("resized_roi.jpg", resized_roi)
print("ROI 图像已保存到 resized_roi.jpg")

# 使用 pytesseract 识别数字
custom_config = r'--psm 7 -c tessedit_char_whitelist=0123456789'
recognized_text = pytesseract.image_to_string(resized_roi, config=custom_config)

print("OCR Number:", recognized_text.strip())
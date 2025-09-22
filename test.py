import cv2
import numpy as np
import math

# ================= HÀM TÍNH GÓC =================
def angle_with_ox(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    angle_rad = math.atan2(dy, dx)
    return math.degrees(angle_rad)

def angle_BAC(A, B, C):
    angle_AB = angle_with_ox(A, B)
    angle_AC = angle_with_ox(A, C)
    theta = abs(angle_AB - angle_AC)
    if theta > 180:
        theta = 360 - theta
    return theta, angle_AB, angle_AC

# ================= DỮ LIỆU =================

A = [1013, 829]
B = [980, 892]
C = [1023, 846]

theta, angle_AB, angle_AC = angle_BAC(A, B, C)
print("Angle AB =", angle_AB)
print("Angle AC =", angle_AC)
print("∠BAC =", theta)

# ================= VẼ =================
# Tạo ảnh to
img = np.ones((2000, 2000, 3), dtype=np.uint8) * 255

# Vẽ các điểm
cv2.circle(img, A, 10, (0, 0, 255), -1)  # đỏ
cv2.circle(img, B, 10, (0, 255, 0), -1)  # xanh lá
cv2.circle(img, C, 10, (255, 0, 0), -1)  # xanh dương

# Vẽ các đoạn thẳng
cv2.line(img, A, B, (0, 0, 0), 2)
cv2.line(img, A, C, (0, 0, 0), 2)

# Ghi nhãn
cv2.putText(img, "A", (A[0]+10, A[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2)
cv2.putText(img, "B", (B[0]+10, B[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2)
cv2.putText(img, "C", (C[0]+10, C[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2)

cv2.putText(img, f"Angle BAC = {theta:.2f}", (50, 50),
            cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

# ================= CROP ================
xs = [A[0], B[0], C[0]]
ys = [A[1], B[1], C[1]]

x_min, x_max = min(xs)-500, max(xs)+500
y_min, y_max = min(ys)-500, max(ys)+500

x_min = max(0, x_min)
y_min = max(0, y_min)
x_max = min(img.shape[1]-1, x_max)
y_max = min(img.shape[0]-1, y_max)

if x_max > x_min and y_max > y_min:
    cropped = img[y_min:y_max, x_min:x_max]
else:
    cropped = img.copy()

# ================= HIỂN THỊ ================
cv2.imshow("Angle BAC (cropped)", cropped)
cv2.imwrite("debug_full.png", img)       # lưu full ảnh để kiểm tra
cv2.imwrite("debug_crop.png", cropped)   # lưu vùng crop
cv2.waitKey(0)
cv2.destroyAllWindows()

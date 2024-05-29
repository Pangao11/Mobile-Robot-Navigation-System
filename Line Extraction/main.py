import numpy as np
import matplotlib.pyplot as plt
import cv2

# 步骤 1: 生成点云
# 定义x值和曲线
x = np.linspace(-10, 10, 4000)
y = 2 * x + 1  # 定义直线

# 在曲线周围添加随机噪声
noise_intensity = 0.3
x_noise = x + np.random.normal(0, noise_intensity, x.shape)
y_noise = y + np.random.normal(0, noise_intensity, y.shape)

# 将生成的点转换为适合OpenCV处理的格式
points = np.vstack((x_noise, y_noise)).T
img_shape = (500, 500)
img = np.zeros(img_shape, dtype=np.uint8)

# 转换点坐标以适应图像尺寸
scale = 20
offset = 250
points_img = np.int32(points * scale + offset)

# 在图像上绘制点
for point in points_img:
    cv2.circle(img, tuple(point), 1, 255, -1)

# 步骤 2: 使用霍夫变换检测直线
# lines = cv2.HoughLines(img, 1, np.pi / 180, 500)

# 霍夫空间参数设置
theta_resolution = np.pi / 180
theta = np.arange(0, np.pi, theta_resolution)
rho_max = int(np.sqrt(img.shape[0]**2 + img.shape[1]**2))

hough_space = np.zeros((2 * rho_max, len(theta)), dtype=np.int32)

# 对每个白点计算霍夫变换曲线
for x, y in points_img:
    for t_idx, t in enumerate(theta):
        rho = int(x * np.cos(t) + y * np.sin(t))
        rho_index = rho + rho_max  # 调整索引以避免负值
        if 0 <= rho_index < 2 * rho_max:
            hough_space[rho_index, t_idx] += 1

plt.imshow(hough_space, extent=[0, np.pi, -rho_max, rho_max], aspect='auto', cmap='gray')
plt.title('Hough Space Visualization')
plt.xlabel('Theta (radians)')
plt.ylabel('Rho (pixels)')
plt.show()

# 设置霍夫变换检测直线的阈值
threshold = 200  # 这个值可能需要根据实际情况调整

# 从霍夫空间中提取满足阈值的直线
lines = []
for rho_index in range(2 * rho_max):
    for theta_index in range(len(theta)):
        if hough_space[rho_index, theta_index] >= threshold:
            rho = rho_index - rho_max
            theta_value = theta[theta_index]
            lines.append((rho, theta_value))

# print(lines)
# 将检测到的直线绘制到图像上
if lines:
    print("Detected lines count:", len(lines))
    for (rho, theta) in lines:
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))
        cv2.line(img, (x1, y1), (x2, y2), 128, 1)

plt.imshow(img, cmap='gray')
plt.title("Point Cloud and Detected Lines")
plt.show()



def point_line_distance(x, y, slope, intercept):
    """计算点到直线的距离"""
    return abs(slope * x - y + intercept) / np.sqrt(slope**2 + 1)


# 设置直线的容差范围（距离阈值）
distance_threshold = 0.005

# 计算每条线的匹配点数和误判点数
TP = 0  # 真阳性：预测的直线上实际有点
FP = 0  # 假阳性：预测的直线上实际没有点
total_points = len(points_img)  # 总点数

# 统计每条预测直线的TP和FP
for rho, theta in lines:
    slope = -np.cos(theta) / np.sin(theta)
    intercept = rho / np.sin(theta)
    inliers = 0  # 落在直线上的点数

    for x, y in points_img:
        distance = point_line_distance(x, y, slope, intercept)
        if distance <= distance_threshold:
            inliers += 1

    if inliers > 0:
        TP += 1  # 至少有一个点落在直线上，认为是真阳性
    else:
        FP += 1  # 没有点落在直线上，认为是假阳性

# 如果没有检测到直线，则 FN = 1
FN = 1 if not lines else 0

# 计算 Precision 和 False Positive Rate
precision = TP / (TP + FP) if TP + FP > 0 else 0
fpr = FP / total_points  # 假阳性率

print(f"True Positives: {TP}")
print(f"False Positives: {FP}")
print(f"False Negatives: {FN}")
print(f"Precision: {precision}")
print(f"False Positive Rate: {fpr}")


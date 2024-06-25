import numpy as np
import pandas as pd

# Tạo dữ liệu giả định cho các biến x, y, z
data = {
    'x': [1, 2, 4],
    'y': [2, 5, 0],
    'z': [3, 7, 8]
}

df = pd.DataFrame(data)

# Tính toán ma trận hiệp phương sai
cov_matrix = df.cov()

# Tính toán ma trận tương quan
corr_matrix = df.corr()

# In ma trận hiệp phương sai
print("Covariance matrix:")
print(cov_matrix)

# In ma trận tương quan
print("\nCorrelation matrix:")
print(corr_matrix)

# Cách viết khác
# # Tạo dữ liệu mẫu
# X = np.array([
#     [x1, y1, z1],
#     [x2, y2, z2],
#     [x3, y3, z3]
# ])

# # Tính ma trận hiệp phương sai
# cov_matrix = np.cov(X.T, bias=False)  # Chuyển vị để có các cột là x, y, z

# print("Ma trận hiệp phương sai:")
# print(cov_matrix)


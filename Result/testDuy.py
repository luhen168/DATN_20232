import numpy as np
import matplotlib.pyplot as plt

# Thiết lập kỳ vọng và độ lệch chuẩn
mu = 2.5
sigma = 1

# Số lượng mẫu
n_samples = 418

# Tạo dữ liệu mẫu
samples = np.random.normal(mu, sigma, n_samples)

# Vẽ phân phối chuẩn
count, bins, ignored = plt.hist(samples, 30, density=True, alpha=0.6, color='g')

# Vẽ đường cong phân phối chuẩn
plt.plot(bins, 1/(sigma * np.sqrt(2 * np.pi)) * np.exp(-(bins - mu)**2 / (2 * sigma**2)), linewidth=2, color='r')

plt.title('Normal Distribution with Expected Value 2.5')
plt.xlabel('Value')
plt.ylabel('Frequency')

# Hiển thị biểu đồ
plt.show()

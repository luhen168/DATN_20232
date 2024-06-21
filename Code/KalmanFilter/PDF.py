import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm

def calculate_statistics(data):
    """Tính toán kỳ vọng và phương sai cho cột dữ liệu"""
    mean = data.mean()
    variance = data.var()
    return mean, variance

def normal_distribution(x, mean, variance):
    """Hàm phân phối chuẩn"""
    std_dev = np.sqrt(variance)
    return (1 / (np.sqrt(2 * np.pi * variance))) * np.exp(-((x - mean) ** 2) / (2 * variance))

def plot_distribution(data, columns):
    """Vẽ biểu đồ phân phối chuẩn cho nhiều cột dữ liệu"""
    fig, axes = plt.subplots(nrows=len(columns), ncols=1, figsize=(10, 6 * len(columns)))
    
    for i, column in enumerate(columns):
        if column in data.columns:
            mean, variance = calculate_statistics(data[column])
            x = np.linspace(data[column].min(), data[column].max(), 100)
            y = normal_distribution(x, mean, variance)
            axes[i].plot(x, y, label=f'Normal distribution\n($\\mu={mean:.2f}$, $\\sigma^2={variance:.2f}$)')
            axes[i].set_title(f'Distribution of {column}')
            axes[i].set_xlabel(column)
            axes[i].set_ylabel('Density')
            axes[i].legend()
            print(f'{column}: Kỳ vọng (Mean) = {mean:.2f}, Phương sai (Variance) = {variance:.2f}')
        else:
            print(f"Cột '{column}' không tồn tại trong dữ liệu.")
    
    plt.tight_layout()
    plt.show()

# Đọc dữ liệu từ file CSV
file_path = r'D:\DATN_20232\Code\DATN_20232-Haruuu\GSDC\kf_position_data.csv'  # Đường dẫn tới file CSV của bạn
data = pd.read_csv(file_path)

# Vẽ biểu đồ phân phối chuẩn cho các cột
columns_to_plot = ['Latitude', 'Longitude', 'utcTimeMillis']
plot_distribution(data, columns_to_plot)

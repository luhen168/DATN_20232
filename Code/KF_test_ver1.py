#   _______________________________________________________________________
#  |[] ThetaLogShell                                                 |X]|!"|
#  |"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""|"|
#  | > ThetaLog.com                                                      | |
#  | > ENV: Python 3.7.0 Anaconda                                        | |
#  | > Kalman Filter (ví dụ đơn giản)                                    | |
#  |_____________________________________________________________________|/

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import style

style.use('bmh')

def simulate_volts(x, G, R):
    """
    Hàm lấy giá trị quan sát y từ mô hình quan sát
    y = Gx + v
    (với v là vector ngẫu nhiên v ~ N(0,R))

    :param x: numpy.array - vector thực tế x
    :param G: ma trận mô hình quan sát
    :param R: ma trận hiệp phương sai nhiễu quan sát

    :return y: numpy.array - vector ngẫu nhiên quan sát được
    """
    return G @ x + np.random.multivariate_normal(np.zeros(shape=R.shape[0]), R)

# Số lượng vòng lặp
n_iteration = 100

# Thật sự là -0.28 vols
x = np.array([[-0.28]])

# Ma trận chuyển đổi trạng thái
A = np.array([[1]])
# Ma trận hiệp phương sai nhiễu hệ thống
Q = np.array([[1e-6]])
# Ma trận mô hình quan sát
G = np.array([[1]])
# Ma trận hiệp phương sai nhiễu quan sát
R = np.array([[0.01]])

list_of_x = []
list_of_y = []

# Thiết lập phân bố ban đầu
x_hat = np.array([[0.5]])
sigma = np.array([[1.0]])

for i in range(n_iteration):
    # Bước dự đoán (Prediction)
    x_hat = A @ x_hat
    sigma = A @ sigma @ A.T + Q
    print("Sigma: %r " % sigma)
    list_of_x.append(float(x_hat))

    # Bước đo lường (Measurement)
    y = simulate_volts(x, G, R)
    kalman_gain = sigma @ G.T @ np.linalg.inv(G @ sigma @ G.T + R)
    x_hat = x_hat + kalman_gain @ (y - G @ x_hat)
    sigma = (np.eye(len(sigma)) - kalman_gain @ G) @ sigma
    list_of_y.append(float(y))

# Vẽ nào
plt.plot(list_of_y,'k+',label='dữ liệu quan sát')
# plt.axhline(x, color='b',label='giá trị thực tế')
plt.plot(list_of_x,'r-',label='giá trị dự đoán')
plt.legend()

plt.xlabel('Thời gian $t$')
plt.ylabel('Hiệu điện thế (volts)')
plt.title('Kalman Filter')

plt.show()
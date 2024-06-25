import numpy as np

class CCS:
    def __init__(self, phi, theta, psi):
        """
        Hàm khởi tạo
        """
        self.phi = np.radians(phi)
        self.theta = np.radians(theta)
        self.psi = np.radians(psi)
        
    def body_to_ecef(self, x, y, z):
        """
        Ma trận chuyển đổi từ hệ tọa độ body sang ECEF
        """
        R = np.array([
            [np.cos(self.theta) * np.cos(self.psi), np.cos(self.theta) * np.sin(self.psi), -np.sin(self.theta)],
            [np.sin(self.phi) * np.sin(self.theta) * np.cos(self.psi) - np.cos(self.phi) * np.sin(self.psi), 
             np.sin(self.phi) * np.sin(self.theta) * np.sin(self.psi) + np.cos(self.phi) * np.cos(self.psi), 
             np.sin(self.phi) * np.cos(self.theta)],
            [np.cos(self.phi) * np.sin(self.theta) * np.cos(self.psi) + np.sin(self.phi) * np.sin(self.psi), 
             np.cos(self.phi) * np.sin(self.theta) * np.sin(self.psi) - np.sin(self.phi) * np.cos(self.psi), 
             np.cos(self.phi) * np.cos(self.theta)]
        ])
        ecef = R.dot(np.array([x, y, z]))
        return ecef
    
    def body_to_ned(self, x, y, z):
        """
        Ma trận chuyển đổi từ hệ tọa độ body sang NED
        """
        R = np.array([
            [np.cos(self.theta) * np.cos(self.psi), np.cos(self.theta) * np.sin(self.psi), -np.sin(self.theta)],
            [np.sin(self.phi) * np.sin(self.theta) * np.cos(self.psi) - np.cos(self.phi) * np.sin(self.psi), 
             np.sin(self.phi) * np.sin(self.theta) * np.sin(self.psi) + np.cos(self.phi) * np.cos(self.psi), 
             np.sin(self.phi) * np.cos(self.theta)],
            [np.cos(self.phi) * np.sin(self.theta) * np.cos(self.psi) + np.sin(self.phi) * np.sin(self.psi), 
             np.cos(self.phi) * np.sin(self.theta) * np.sin(self.psi) - np.sin(self.phi) * np.cos(self.psi), 
             np.cos(self.phi) * np.cos(self.theta)]
        ])
        ned = R.dot(np.array([x, y, z]))
        return ned

import numpy as np

class CCS:
    def __init__(self, phi, theta, psi, latitude, longitude ):
        """
        Hàm khởi tạo
        """
        self.phi = np.radians(phi)
        self.theta = np.radians(theta)
        self.psi = np.radians(psi)
        self.latitude = np.radians(latitude)
        self.longitude = np.radians(longitude )
        
    def rotation_matrix_body_to_intermediate(self):
        """
        Tạo ma trận quay từ khung tọa độ body sang khung tọa độ intermediate.
        """
        cp = np.cos(self.theta)
        sp = np.sin(self.theta)
        cr = np.cos(self.phi)
        sr = np.sin(self.phi)
        cy = np.cos(self.psi)
        sy = np.sin(self.psi)

        R_lb = np.array([
            [cy * cr - sy * sp * sr, -sy * cp, cy * sr + sy * sp * cr],
            [sy * cr + cy * sp * sr, cy * cp, sy * sr - cy * sp * cr],
            [-cp * sr, sp, cp * cr]
        ])

        return R_lb

    def rotation_matrix_intermediate_to_ecef(self):
        """
        Tạo ma trận quay từ khung tọa độ intermediate sang khung tọa độ ECEF.
        """
        sl = np.sin(self.latitude)
        cl = np.cos(self.latitude)
        sp = np.sin(self.longitude)
        cp = np.cos(self.longitude)

        R_el = np.array([
            [-sl, -sp * cl, cp * cl],
            [cl, -sp * sl, cp * sl],
            [0, cp, sp]
        ])

        return R_el
    
    def body_to_ecef(self, x, y, z):
        """
        Ma trận chuyển đổi từ hệ tọa độ body sang ECEF
        """
        R_lb = self.rotation_matrix_body_to_intermediate()
        R_el = self.rotation_matrix_intermediate_to_ecef()
        R_eb = np.dot(R_el, R_lb)
        ecef = R_eb.dot(np.array([x, y, z]))
        return ecef
    
    # def body_to_ned(self, x, y, z):
    #     """
    #     Ma trận chuyển đổi từ hệ tọa độ body sang NED
    #     """
    #     R = np.array([
    #         [np.cos(self.theta) * np.cos(self.psi), np.cos(self.theta) * np.sin(self.psi), -np.sin(self.theta)],
    #         [np.sin(self.phi) * np.sin(self.theta) * np.cos(self.psi) - np.cos(self.phi) * np.sin(self.psi), 
    #          np.sin(self.phi) * np.sin(self.theta) * np.sin(self.psi) + np.cos(self.phi) * np.cos(self.psi), 
    #          np.sin(self.phi) * np.cos(self.theta)],
    #         [np.cos(self.phi) * np.sin(self.theta) * np.cos(self.psi) + np.sin(self.phi) * np.sin(self.psi), 
    #          np.cos(self.phi) * np.sin(self.theta) * np.sin(self.psi) - np.sin(self.phi) * np.cos(self.psi), 
    #          np.cos(self.phi) * np.cos(self.theta)]
    #     ])
    #     ned = R.dot(np.array([x, y, z]))
    #     return ned

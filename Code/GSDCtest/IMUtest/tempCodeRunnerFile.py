import numpy as np
import pymap3d as pm


class CCS:
    '''
        phi:   roll
        theta: pitch 
        psi:   yaw
    '''
    def __init__(self, phi, theta, psi, latitude, longitude, altitude):
        """
        Hàm khởi tạo
        """
        self.phi = np.radians(phi)
        self.theta = np.radians(theta)
        self.psi = np.radians(psi)
        self.latitude = np.radians(latitude)
        self.longitude = np.radians(longitude)
        self.altitude = altitude
    # def rotation_matrix_body_to_intermediate(self):
    #     """
    #     Tạo ma trận quay từ khung tọa độ body sang khung tọa độ intermediate.
    #     """
    #     cp = np.cos(self.theta)
    #     sp = np.sin(self.theta)
    #     cr = np.cos(self.phi)
    #     sr = np.sin(self.phi)
    #     cy = np.cos(self.psi)
    #     sy = np.sin(self.psi)

    #     R_lb = np.array([
    #         [cy * cr - sy * sp * sr, -sy * cp, cy * sr + sy * sp * cr],
    #         [sy * cr + cy * sp * sr, cy * cp, sy * sr - cy * sp * cr],
    #         [-cp * sr, sp, cp * cr]
    #     ])

    #     return R_lb

    # def rotation_matrix_intermediate_to_ecef(self):
    #     """
    #     Tạo ma trận quay từ khung tọa độ intermediate sang khung tọa độ ECEF.
    #     """
    #     sl = np.sin(self.latitude)
    #     cl = np.cos(self.latitude)
    #     sp = np.sin(self.longitude)
    #     cp = np.cos(self.longitude)

    #     R_el = np.array([
    #         [-sl, -sp * cl, cp * cl],
    #         [cl, -sp * sl, cp * sl],
    #         [0, cp, sp]
    #     ])

    #     return R_el
    
    def rotation_matrix_ned_to_body(self):
        """
            Tạo ma trận quay từ khung tọa độ intermediate sang khung tọa độ ECEF.
        """
        # Tính toán các giá trị cosine và sine của các góc
        cp = np.cos(self.theta)
        sp = np.sin(self.theta)
        cr = np.cos(self.phi)
        sr = np.sin(self.phi)
        cy = np.cos(self.psi)
        sy = np.sin(self.psi)

        R_nb = np.array([
            [cp * cy, cp * sy, -sp],
            [sp * cy * sr - sy * cr, sp * sy * cr + cy * cr, cp * sr],
            [sp * cy * cr + sy * sr, sy * sy * cr - cy * sr, cp * cr]
        ])

        return R_nb
    
    # def body_to_ecef(self, x, y, z):
    #     """
    #     Ma trận chuyển đổi từ hệ tọa độ body sang ECEF
    #     """
    #     R_lb = self.rotation_matrix_body_to_intermediate()
    #     R_el = self.rotation_matrix_intermediate_to_ecef()
    #     R_eb = np.dot(R_el, R_lb)
    #     ecef = R_eb.dot(np.array([x, y, z]))
    #     return ecef
    
    def body_to_ned(self, x, y, z):
        """
        Ma trận chuyển đổi từ hệ tọa độ body sang ECEF
        """
        R_bn = np.linalg.inv(self.rotation_matrix_ned_to_body())
        ned = R_bn.dot(np.array([x, y, z]))
        return ned
    
    def ecef_to_ned(self, x_ecef, y_ecef, z_ecef):
        """
        Ma trận chuyển đổi từ hệ tọa độ ecef sang ned
        """
        e, n, u = pm.ecef2enu(x_ecef, y_ecef, z_ecef, self.latitude, self.longitude, self.altitude)
        
        # Then convert ENU to NED by negating the up component (u to -d)
        ned = (n, e, -u)
        
        return ned
    
    def ned_to_ecef(self, x_ned, y_ned, z_ned):
        """
        Ma trận chuyển đổi từ hệ tọa độ ecef sang ned
        x_ned, y_ned, z_ned = n,e,d
        """
        # Chuyển về ENU trước 
        e = y_ned
        n = x_ned
        u = -z_ned
        x, y, z = pm.enu2ecef(x_ned, y_ned, z_ned, self.latitude, self.longitude, self.altitude)
        
        # Then convert ENU to NED by negating the up component (u to -d)
        ecef = (x, y, z )
        
        return ecef 
    
    def vecef_to_ned(self, vx_ecef, vy_ecef, vz_ecef):
        """
        Ma trận chuyển đổi từ hệ tọa độ ecef sang ned
        """
        ve, vn, vu = pm.ecef2enuv(vx_ecef, vy_ecef, vz_ecef, self.latitude, self.longitude)
        
        # Then convert ENU to NED by negating the up component (u to -d)
        vned = (vn, ve, -vu)
        
        return vned
    
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

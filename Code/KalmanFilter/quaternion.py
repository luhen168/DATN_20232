import numpy as np

class Quaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def __mul__(self, other):
        if isinstance(other, Quaternion):
            return Quaternion(
                self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
                self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
                self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
                self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z
            )
        elif isinstance(other, float) or isinstance(other, int):
            return Quaternion(
                self.x * other,
                self.y * other,
                self.z * other,
                self.w * other
            )
        else:
            raise TypeError(f"Multiplication with type {type(other)} not supported")

    def __iadd__(self, other):
        self.x += other.x
        self.y += other.y
        self.z += other.z
        self.w += other.w
        return self

    def normalize(self):
        norm = np.sqrt(self.x**2 + self.y**2 + self.z**2 + self.w**2)
        self.x /= norm
        self.y /= norm
        self.z /= norm
        self.w /= norm

    def to_rotation_matrix(self):
        # Convert quaternion to rotation matrix
        pass  # Implement this based on your quaternion to rotation matrix conversion logic

    def to_euler_angles(self):
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        pass  # Implement this based on your quaternion to Euler angles conversion logic

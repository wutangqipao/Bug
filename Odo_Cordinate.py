import time

import numpy as np
import math
from Odometer import LS7366R

# constant

PI = 3.141592653589793
LEN_L = 9.94999  # length between left_wheel and move_center,unit is centimeters
LEN_R = 9.94999  # length between right_wheel and move_center,unit is centimeters
LEN_V = 3  # length between vertical_wheel and move_center,unit is centimeters
LEN_W = LEN_R + LEN_L  # length between right_wheel and left_wheel,unit is centimeters

WHEEL_RADIUS = 2.4  # the radius of Omni wheels,unit is centimeters
CM_PER_TICK = 2 * PI * WHEEL_RADIUS / 2048


# class
class DFO:  # DATA OF ODOMETRY

    def __init__(self):
        self.current_data = 0
        self.previous_data = 0
        self.delt_data = 0


class DFCOR:  # DATA OF self.Coordinate
    def __init__(self):
        self.COR_X = 0
        self.COR_Y = 0
        self.COR_Theta = 0

        self.delt_x = 0
        self.delt_y = 0
        self.delt_theta = 0


class PosOdometry:
    def __init__(self):
        self.encoder = LS7366R(0, 10000, 4)
        # 创建对象
        self.Omni_L = DFO()  # left encoder
        self.Omni_R = DFO()  # right encoder
        self.Omni_V = DFO()  # vertical encoder

        self.Coordinate = DFCOR()

    def read_data(self):
        self.encoder.Switch_device(0)
        data_L = self.encoder.readCounter()
        self.encoder.Switch_device(1)
        data_R = self.encoder.readCounter()
        self.encoder.Switch_device(2)
        data_V = self.encoder.readCounter()

        return data_L, data_R, -data_V  # the sign of data is related with the installation of encoders

    def position_add(self):
        self.Coordinate.COR_Theta = self.Coordinate.COR_Theta + self.Coordinate.delt_theta
        self.Coordinate.COR_X = self.Coordinate.COR_X + self.Coordinate.delt_x
        self.Coordinate.COR_Y = self.Coordinate.COR_Y + self.Coordinate.delt_y
        if self.Coordinate.COR_Theta > PI:
            self.Coordinate.COR_Theta = -2 * PI + self.Coordinate.COR_Theta
        if self.Coordinate.COR_Theta < -PI:
            self.Coordinate.COR_Theta = 2 * PI + self.Coordinate.COR_Theta

    def Calculate_Cor(self):

        data_current = self.read_data()
        self.Omni_L.current_data, self.Omni_R.current_data, self.Omni_V.current_data = data_current
        # 计算编码值变化量
        self.Omni_L.delt_data = (self.Omni_L.current_data - self.Omni_L.previous_data) * CM_PER_TICK
        self.Omni_R.delt_data = (self.Omni_R.current_data - self.Omni_R.previous_data) * CM_PER_TICK
        self.Omni_V.delt_data = (self.Omni_V.current_data - self.Omni_V.previous_data) * CM_PER_TICK

        self.Omni_L.previous_data, self.Omni_R.previous_data, self.Omni_V.previous_data = data_current
        self.Coordinate.delt_theta = (self.Omni_R.delt_data - self.Omni_L.delt_data) / LEN_W
        if self.Coordinate.delt_theta == 0:
            deltay = self.Omni_V.delt_data
            deltax = (self.Omni_R.delt_data + self.Omni_L.delt_data) / 2
        else:
            turnRadius = ((self.Omni_R.delt_data + self.Omni_L.delt_data) / self.Coordinate.delt_theta + LEN_L - LEN_R) / 2
            strafeRadius = self.Omni_V.delt_data / self.Coordinate.delt_theta - LEN_W
            if self.Omni_V.delt_data == 0:
                strafeRadius = 0
            motion_matrix = np.array([[-np.cos(self.Coordinate.delt_theta) + 1, -np.sin(self.Coordinate.delt_theta)],
                                      [np.sin(self.Coordinate.delt_theta), (1 - np.cos(self.Coordinate.delt_theta))]])
            Radius_matrix = np.array([turnRadius, strafeRadius])
            deltay, deltax = np.dot(motion_matrix, Radius_matrix)

        # 旋转
        self.Coordinate.delt_x, self.Coordinate.delt_y = error_coord_transform(deltax, deltay, self.Coordinate.COR_Theta)

        # 积分计算整体坐标
        self.position_add()

        return self.Coordinate.COR_X, self.Coordinate.COR_Y, self.Coordinate.COR_Theta




def error_coord_transform(error_x, error_y, angle_radians):
    #
    rotation_matrix = np.array([
        [np.cos(angle_radians), -np.sin(angle_radians)],
        [np.sin(angle_radians), np.cos(angle_radians)]
    ])
    original_vector = np.array([error_x, error_y])
    new_vector = np.dot(rotation_matrix, original_vector)
    new_x, new_y = new_vector
    return new_x, new_y



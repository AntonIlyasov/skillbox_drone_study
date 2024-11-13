import numpy as np
import multicopter_model.constants as cs
from multicopter_model.constants import FlightMode, States 
from multicopter_model.pid import PID
import os
import time as tm

class QuadCopterController:
    def __init__(self):

        self.x_coeff = []
        self.y_coeff = []
        self.z_coeff = []

        self.target_x = 50
        self.target_y = 75
        self.target_z = 100
        self.target_yaw = 0
        
        self.position_controller_x = PID()
        self.position_controller_y = PID()
        self.position_controller_z = PID()

        self.velocity_controller_x = PID()
        self.velocity_controller_y = PID()
        self.velocity_controller_z = PID()

        self.roll_controller = PID()
        self.pitch_controller = PID()
        self.yaw_controller = PID()

        self.roll_rate_controller = PID()
        self.pitch_rate_controller = PID()
        self.yaw_rate_controller = PID()

        # Пример простого маршрута
        # self._mission = [[0, 0, 10 , 0], [0, 10, 10, 1]]
        self._mission = [[0, 0, 10 , 0]]
        self._current_mission_index = 0


    def set_target_position(self, x, y, z, yaw):
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.target_yaw = yaw

    def trajectoryGenerator(self):
        T = 10  #TODO ms 

        A = np.array([[0,       0,       0,      0,    0,   1],
                      [T**5,    T**4,    T**3,   T**2, T,   1],
                      [0,       0,       0,      0,    1,   0],
                      [5*T**4,  4*T**3,  3*T**2, 2*T,  1,   0],
                      [0,       0,       0,      2,    0,   0],
                      [20*T**3, 12*T**2, 6*T,    2,    0,   0]])
        
        print("A: ", A)
        
        b_x = np.array([0, self.target_x, 0, 0, 0, 0])
        b_y = np.array([0, self.target_y, 0, 0, 0, 0])
        b_z = np.array([0, self.target_z, 0, 0, 0, 0])

        print("b_x: ", b_x)
        print("b_y: ", b_y)
        print("b_z: ", b_z)
        
        self.x_coeff = np.linalg.inv(A) @ b_x    # [c5, c4, c3, c2, c1, c0]
        self.y_coeff = np.linalg.inv(A) @ b_y    # [c5, c4, c3, c2, c1, c0]
        self.z_coeff = np.linalg.inv(A) @ b_z    # [c5, c4, c3, c2, c1, c0]
        # 6x3 подставляем эти коэффициенты в полином и подставляя туда время
        # получать наши желаемые параметры - желаемое положение скорость ускорение


    def update(self, state_vector, dt, t_cur) -> np.ndarray:
        # print("\n#######################################################################")
        # print("self.x_coeff: ", self.x_coeff)
        # print("self.y_coeff: ", self.y_coeff)
        # print("self.z_coeff: ", self.z_coeff)
        # #TODO Обновление целевой точки маршрута
        # print("t_cur: ", t_cur)
        # time_vec = np.array([t_cur**5, t_cur**4, t_cur**3, t_cur**2, t_cur, 1])        
        # print("time_vec: ", time_vec)

        # x_des = time_vec @ self.x_coeff
        # y_des = time_vec @ self.y_coeff
        # z_des = time_vec @ self.z_coeff
        # print("x_des: ", x_des)
        # print("y_des: ", y_des)
        # print("z_des: ", z_des)
        
        # self.set_target_position(x_des, y_des, z_des, self.target_yaw)

        # #TODO Расчет целевой скорости ЛА
        # print("self.target_x: ", self.target_x)
        # print("self.target_y: ", self.target_y)
        # print("self.target_z: ", self.target_z)
        # print("self.target_yaw: ", self.target_yaw)

        # print("state_vector[States.X]: ", state_vector[States.X])
        # print("state_vector[States.Y]: ", state_vector[States.Y])
        # print("state_vector[States.Z]: ", state_vector[States.Z])

        # vx_des = self.position_controller_x.update(state_vector[States.X], self.target_x, dt)
        # vy_des = self.position_controller_y.update(state_vector[States.Y], self.target_y, dt)
        # vz_des = self.position_controller_z.update(state_vector[States.Z], self.target_z, dt)

        # print("vx_des: ", vx_des)
        # print("vy_des: ", vy_des)
        # print("vz_des: ", vz_des)

        # #TODO Расчет тяги и углов крена и тангажа, перепроектирование углов в связную СК
        # target_roll = self.velocity_controller_x.update(state_vector[States.VX], vx_des, dt)
        # target_pitch = self.velocity_controller_y.update(state_vector[States.VY], vy_des, dt)
        # cmd_trust = self.velocity_controller_z.update(state_vector[States.VZ], vz_des, dt)

        # print("target_roll: ", target_roll)
        # print("target_pitch: ", target_pitch)
        # print("cmd_trust: ", cmd_trust)
        
        # rot_mat_2d = self._rotation2d(state_vector[States.YAW])
        
        # roll_pitch =  rot_mat_2d @ np.array([target_roll, target_pitch])
        # print("roll_pitch: ", roll_pitch)

        # target_roll = roll_pitch[0]
        # target_pitch = roll_pitch[1]

        # print("target_roll: ", target_roll)
        # print("target_pitch: ", target_pitch)
        
        # # Пример для контура управления угловым положением и угловой скоростью.
        # target_roll_rate = self.roll_controller.update(state_vector[States.ROLL], target_roll, dt)
        # target_pitch_rate = self.pitch_controller.update(state_vector[States.PITCH], target_pitch, dt)
        # target_yaw_rate = self.yaw_controller.update(state_vector[States.YAW], self.target_yaw, dt)

        cmd_trust = 5
        target_roll_rate = 20
        target_pitch_rate = 0
        target_yaw_rate = 0
        
        # os.system('clear')
        print("target_roll_rate: ", target_roll_rate)
        print("target_pitch_rate: ", target_pitch_rate)
        print("target_yaw_rate: ", target_yaw_rate)

        cmd_roll = self.roll_rate_controller.update(state_vector[States.ROLL_RATE], target_roll_rate, dt)
        cmd_pitch = self.pitch_rate_controller.update(state_vector[States.PITCH_RATE], target_pitch_rate, dt)
        cmd_yaw = self.yaw_rate_controller.update(state_vector[States.YAW_RATE], target_yaw_rate, dt)

        print("cmd_roll: ", cmd_roll)
        print("cmd_pitch: ", cmd_pitch)
        print("cmd_yaw: ", cmd_yaw)

        u = self._mixer(cmd_trust, cmd_roll, cmd_pitch, cmd_yaw)
        print("u: ", u)
        return u

    def _mixer(self, cmd_trust, cmd_roll, cmd_pitch, cmd_yaw) -> np.ndarray:
        u_1 = cmd_trust + cmd_roll - cmd_yaw        #TODO Реализуйте алгоритм смешивания команд
        u_2 = cmd_trust - cmd_pitch + cmd_yaw       #TODO 
        u_3 = cmd_trust - cmd_roll - cmd_yaw        #TODO 
        u_4 = cmd_trust + cmd_pitch + cmd_yaw       #TODO 
        return np.array([u_1, u_2, u_3, u_4])

    def _rotation2d(self, theta):
        return np.array([[float(np.cos(theta)), float(-np.sin(theta))], 
                         [float(np.sin(theta)),  float(np.cos(theta))]])
import numpy as np
import multicopter_model.constants as cs
from multicopter_model.constants import FlightMode, States 
from multicopter_model.pid import PID
import time as tm
import os

class QuadCopterController:
    def __init__(self):

        self.x_coeff = []
        self.y_coeff = []
        self.z_coeff = []

        self.target_x = 0
        self.target_y = 0
        self.target_z = 10
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
        # self._mission = [[5, 5, 5 , 0], [0, 0, 0, 0]]*5
        self._mission = [[0, 3, 3, 0]]
        self._current_mission_index = 0
        self.set_target_position(self._mission[self._current_mission_index][0],
                                    self._mission[self._current_mission_index][1],
                                    self._mission[self._current_mission_index][2],
                                    self._mission[self._current_mission_index][3])



    def set_target_position(self, x, y, z, yaw):        # вызываем, когда дрон входит в сферу вокруг точки
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.target_yaw = yaw


    def update(self, state_vector, dt) -> np.ndarray:

        print("self.target_x: ", self.target_x)
        print("self.target_y: ", self.target_y)
        print("self.target_z: ", self.target_z)
        print("self.target_yaw: ", self.target_yaw)

        # if (len(self._mission) > self._current_mission_index):
        #     if ((abs(self._mission[self._current_mission_index][0] - state_vector[States.X])) < 0.3 and \
        #         (abs(self._mission[self._current_mission_index][1] - state_vector[States.Y])) < 0.3 and \
        #         (abs(self._mission[self._current_mission_index][2] - state_vector[States.Z])) < 0.3):
        #         self._current_mission_index += 1
        #         if (len(self._mission) > self._current_mission_index):
        #             self.set_target_position(self._mission[self._current_mission_index][0],
        #                                     self._mission[self._current_mission_index][1],
        #                                     self._mission[self._current_mission_index][2],
        #                                     self._mission[self._current_mission_index][3])


        target_vel_x = self.position_controller_x.update(state_vector[States.X], self.target_x, dt) # position_controller_y
        target_vel_y = self.position_controller_y.update(state_vector[States.Y], self.target_y, dt)
        target_vel_z = self.position_controller_z.update(state_vector[States.Z], self.target_z, dt)

        ############################################################################################################################

        # # режим хз        
        # target_vel_x = 0.0                  # м/с
        # target_vel_y = 1.0                  # м/с
        # target_vel_z = 0.0                  # м/с
        # self.target_yaw = 0.0               # радиан

        # print("target_vel_x: ", target_vel_x)
        # print("target_vel_y: ", target_vel_y)
        # print("target_vel_z: ", target_vel_z)
        # print("self.target_yaw: ", self.target_yaw)

        # #TODO Расчет тяги и углов крена и тангажа, перепроектирование углов в связную СК
        target_roll = self.velocity_controller_x.update(state_vector[States.VX], target_vel_x, dt)    # velocity_controller_y
        target_pitch = self.velocity_controller_y.update(state_vector[States.VY], target_vel_y, dt)   # velocity_controller_x
        cmd_trust = self.velocity_controller_z.update(state_vector[States.VZ], target_vel_z, dt)          # здесь cmd_trust может быть очень маленькое

        print("cmd_trust_before_*scale: ", cmd_trust)
        cmd_trust *= cs.trust_scale

        cmd_trust = np.clip(cmd_trust, cs.min_rotors_rpm, cs.max_rotors_rpm)

        # print("cmd_trust_before_rot: ", cmd_trust)
        # print("target_roll_before_rot: ", target_roll)
        # print("target_pitch_before_rot: ", target_pitch)
        # print("self.target_yaw_before_rot: ", self.target_yaw)

        target_pitch_roll = self._rotation2d(state_vector[States.YAW][0]).transpose() @ \
                            np.array([[target_pitch][0],[target_roll][0]])
        target_roll = target_pitch_roll[0]
        target_pitch = target_pitch_roll[1]

        print("########################################")
        print("target_roll: ", target_roll)
        print("target_pitch: ", target_pitch)
        print("########################################")

        # режим стабилизации
        cmd_trust = 2100
        target_roll = 0.0             # радиан
        target_pitch = -0.5             # радиан
        self.target_yaw = 0.0          # радиан
        
        # # Пример для контура управления угловым положением и угловой скоростью.
        target_roll_rate = self.roll_controller.update(state_vector[States.ROLL], target_roll, dt)  # -target_roll
        target_pitch_rate = self.pitch_controller.update(state_vector[States.PITCH], target_pitch, dt)
        target_yaw_rate = self.yaw_controller.update(state_vector[States.YAW], self.target_yaw, dt)

        # cmd_trust = 3500
        # target_roll_rate = 1
        # target_pitch_rate = 0
        # target_yaw_rate = 0
        
        # print("cmd_trust: ", cmd_trust)
        # print("target_roll_rate: ", target_roll_rate)
        # print("target_pitch_rate: ", target_pitch_rate)
        # print("target_yaw_rate: ", target_yaw_rate)

        cmd_roll = self.roll_rate_controller.update(state_vector[States.ROLL_RATE], target_roll_rate, dt)
        cmd_pitch = self.pitch_rate_controller.update(state_vector[States.PITCH_RATE], target_pitch_rate, dt)
        cmd_yaw = self.yaw_rate_controller.update(state_vector[States.YAW_RATE], target_yaw_rate, dt)

        # cmd_trust = 500
        # cmd_roll  = 10
        # cmd_pitch = 20
        # cmd_yaw   = 30

        # print("cmd_trust: ", cmd_trust)
        # print("cmd_roll: ", cmd_roll)
        # print("cmd_pitch: ", cmd_pitch)
        # print("cmd_yaw: ", cmd_yaw)

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
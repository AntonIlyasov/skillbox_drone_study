import numpy as np
import multicopter_model.constants as cs
from multicopter_model.constants import FlightMode, States, target_radius
from multicopter_model.pid import PID

class QuadCopterController:
    def __init__(self):
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
        self._mission = [[0, 0, 10 , 0], [0, 10, 10, 1], [10, 10, 10, 1], [10, 0, 10, 1], [5, -5, 10, 0], [0, -5, 10, 0], [0, 0, 0, 0]]
        self._current_mission_index = 0


    def set_target_position(self, x, y, z, yaw):
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.target_yaw = yaw


    def update(self, state_vector, dt) -> np.ndarray:

        #TODO Обновление целевой точки маршрута
        # Устанавливаю цель, в соответствии с текущим индексом цели
        self.set_target_position(self._mission[self._current_mission_index][0],
                                 self._mission[self._current_mission_index][1],
                                 self._mission[self._current_mission_index][2],
                                 self._mission[self._current_mission_index][3])
        current_pos = state_vector[0:3]                                                 # Текущая позиция коптера
        current_target = self._mission[self._current_mission_index][0:3]                # Текущая цель миссии
        diff_target_pos = np.array([[current_target[0] - current_pos[0]],               # Вектор разницы координат
                                    [current_target[1] - current_pos[1]],
                                    [current_target[2] - current_pos[2]]])
        current_length = np.linalg.norm(diff_target_pos)                                # Расстояние до цели

        print(f'Current index: {self._current_mission_index}')
        print(f'Current target: x = {self.target_x}, y = {self.target_y}, z = {self.target_z}')
        print(f'Current pos: x = {current_pos[0]}, y = {current_pos[1]}, z = {current_pos[2]}')
        print(f'Length to target: {current_length}')

        # Если расстояние до цели меньше допуска (добавил в константы: target_radius), 
        # и если следующее значение индекса целей меньше общего количества целей в списке целей,
        # то увеличиваю индекс цели, т.е. перехожу к следующей цели по списку 
        if (current_length < target_radius) and ((self._current_mission_index + 1) < len(self._mission)):
            self._current_mission_index += 1
            # Обновляю цель,  в соответствии с новым индексом целей
            self.set_target_position(self._mission[self._current_mission_index][0],
                                    self._mission[self._current_mission_index][1],
                                    self._mission[self._current_mission_index][2],
                                    self._mission[self._current_mission_index][3])
            print(f'Current target {self._current_mission_index}: {self._mission[self._current_mission_index]}')

        #TODO Расчет целевой скорости ЛА
        target_velocity_x = self.position_controller_x.update(state_vector[States.X], self.target_x, dt)
        target_velocity_y = self.position_controller_y.update(state_vector[States.Y], self.target_y, dt)
        target_velocity_z = self.position_controller_z.update(state_vector[States.Z], self.target_z, dt)

        #TODO Расчет тяги и углов крена и тангажа, перепроектирование углов в связную СК
        target_roll = self.velocity_controller_x.update(state_vector[States.VX], target_velocity_x, dt)
        target_pitch = self.velocity_controller_y.update(state_vector[States.VY], target_velocity_y, dt)
        cmd_trust = self.velocity_controller_z.update(state_vector[States.VZ], target_velocity_z, dt)

        cmd_trust *= cs.trust_scale
        cmd_trust = np.clip(cmd_trust, cs.min_rotors_rpm, cs.max_rotors_rpm)

        # Поворачиваю полученные значения желаемого тангажа и крена на угол рысканья,
        # для получения их значений в связанной СК
        roll_pitch = self._rotation2d(state_vector[States.YAW][0]).transpose() @ \
                                            np.array([[target_pitch][0],
                                                      [target_roll][0]])
        target_roll = roll_pitch[0]
        target_pitch = roll_pitch[1]
        # target_roll = 0
        # target_pitch = 0
        # cmd_trust = 1000
        
        # Пример для контура управления угловым положением и угловой скоростью.
        # target_roll_rate = self.roll_controller.update(state_vector[States.ROLL], target_roll, dt)
        target_roll_rate = self.roll_controller.update(state_vector[States.ROLL], -target_roll, dt)
        target_pitch_rate = self.pitch_controller.update(state_vector[States.PITCH], target_pitch, dt)
        target_yaw_rate = self.yaw_controller.update(state_vector[States.YAW], self.target_yaw, dt)

        cmd_roll = self.roll_rate_controller.update(state_vector[States.ROLL_RATE], target_roll_rate, dt)
        cmd_pitch = self.pitch_rate_controller.update(state_vector[States.PITCH_RATE], target_pitch_rate, dt)
        cmd_yaw = self.yaw_rate_controller.update(state_vector[States.YAW_RATE], target_yaw_rate, dt)

        u = self._mixer(cmd_trust, cmd_roll, cmd_pitch, cmd_yaw)
        return u

    def _mixer(self, cmd_trust, cmd_roll, cmd_pitch, cmd_yaw) -> np.ndarray:
        # Для коптера схемы "+", как задано в симуляторе
        u_1 = cmd_trust + cmd_roll - cmd_yaw
        u_2 = cmd_trust - cmd_pitch + cmd_yaw
        u_3 = cmd_trust - cmd_roll - cmd_yaw
        u_4 = cmd_trust + cmd_pitch + cmd_yaw
        # u_1 = 0 #TODO Реализуйте алгоритм смешивания команд
        # u_2 = 0 #TODO 
        # u_3 = 0 #TODO 
        # u_4 = 0 #TODO 
        return np.array([u_1, u_2, u_3, u_4])

    def _rotation2d(self, theta):
        return np.array([[float(np.cos(theta)), float(-np.sin(theta))], 
                         [float(np.sin(theta)),  float(np.cos(theta))]])
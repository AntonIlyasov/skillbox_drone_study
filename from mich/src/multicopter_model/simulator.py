import socket 
import struct
import time as tm
from multicopter_model.model import QuadCopterModel
from multicopter_model.controller import QuadCopterController
from multicopter_model.constants import FlightMode, States 

class Simulator:
    def __init__(self, controller: QuadCopterController, model: QuadCopterModel, dt):
        self.controller = controller
        self.model = model
        self.dt = dt
        self.time = 0
        self._host = '127.0.0.1'
        self._port = 12346
        self.addr = (self._host, self._port)
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def run(self):  
        while True:
            # Получим вектор состояния от модели
            state = self.model.state_vector
            # Получим вектор управления на основе текущего состояния
            u = self.controller.update(state, self.dt)
            # Обновим состояние модели
            self.model.update_state(u, self.dt)

            print("state:")
            print(state)
            print("States.VY: ", state[States.VY])
            print("States.ROLL: ", state[States.ROLL])
            print("States.PITCH: ", state[States.PITCH])
            print()

            self._send_pose_data(state)
            self.time += self.dt
            tm.sleep(self.dt*2)

    def _send_pose_data(self, state):
        # Упаковка и отправка данных о состоянии ЛА для визуализации
        data = bytearray(struct.pack("ddddddddddddd", state[0], state[1], state[2], state[3], state[4],
                 state[5], state[6], state[7], state[8], state[9], state[10], state[11], self.time))
        self.udp_socket.sendto(data, self.addr)
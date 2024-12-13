import numpy as np

class PID:
    def __init__(self):
        self._k_p = 1
        self._k_i = 0
        self._k_d = 0.1
        self._integral = 0
        self._last_error = 0.
        self._integral_limit = 300
        self._min_value = -1
        self._max_value = 1

    def set_pid_gains(self, k_p: float, k_i: float, k_d: float) -> None: 
        self._k_p = k_p
        self._k_i = k_i
        self._k_d = k_d

    @property
    def integral_limit(self):
        return self._integral_limit
    
    @integral_limit.setter
    def integral_limit(self, limit: float):
        self._integral_limit = limit

    def set_saturation_limit(self, min:float, max:float) -> None: 
        self._min_value = min
        self._max_value = max

    def update(self, input_val: float, target_val: float, dt: float) -> float:
        # Расчет ошибки
        error = target_val - input_val
        print("error: ", error)
        # Обновление интегрального значения
        self._integral += error * dt
        print("dt: ", dt)
        print("_integral: ", self._integral)
        # saturate integral
        # self._integral = np.clip(self._integral, -self._integral_limit, self._integral_limit)
        # Расчет P I  D компонент
        P = self._k_p * error
        print("P: ", P)
        I = self._k_i * self._integral
        print("I: ", I)
        D = self._k_d * (error - self._last_error) / dt
        print("D: ", D)
        self._last_error = error
        output = P + I + D
        print("output: ", output)
        # Ограничиваем величину выходного значения
        # output = np.clip(output, self._min_value, self._max_value)
        return float(output)
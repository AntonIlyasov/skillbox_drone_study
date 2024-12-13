import numpy as np
from multicopter_model.constants import FlightMode, States 
import math

class QuadCopterModel:
	def __init__(self, inertia, mass: float, trust_coef: float, drag_coef: float, arm_length: float):
		# Зададим константные параметры модели 
		self._inertia = inertia
		self._inertia_inv = np.linalg.inv(inertia)
		self._mass = mass
		self._trust_coef = trust_coef
		self._drag_coef = drag_coef
		self._arm_length = arm_length
		# Запишем вектор силы тяжести
		self._g = np.array([[0.0], [0.0], [-9.81]])
		self._motor_trust = np.array([[0.0], [0.0], [0.0]])
		self._motor_moments = np.array([[0.0], [0.0], [0.0]])

		# Зададим вектор состояния, запишем начальные значения, в данном случае везде нули.
		self._state_vector = np.array([[0.0],  # 0 - X
		 							   [0.0],  # 1 - Y
									   [0.0],  # 2 - Z
									   [0.0],  # 3 - Roll
									   [0.0],  # 4 - Pitch
									   [0.0],  # 5 - Yaw
									   [0.0],  # 6 - VelX
									   [0.0],  # 7 - VelY
									   [0.0],  # 8 - VelZ
									   [0.0],  # 9 - RollRate
									   [0.0],  # 10 - PitchRate
									   [0.0]]) # 11 - YawRate

	@property
	def state_vector(self):
		return self._state_vector

	def update_state(self, u, dt: float) -> None:
		lin_acc, ang_acc = self._func_right(u)
		self._integrate(lin_acc, ang_acc, dt)

	def _integrate(self, linear_acceleration, angular_acceleration, dt: float):
		# Проинтегрируем линейное ускорение и обновим скорость
		self.state_vector[6:9] += linear_acceleration * dt
		# Проинтегрируем скорость и обновим положение
		self.state_vector[0:3] += self.state_vector[6:9] * dt
		# Проинтегрируем угловое ускорение и обновим угловую скорость
		self.state_vector[9:12] += angular_acceleration * dt
		# Проинтегрируем угловую скорость и обновим угловое положение ЛА
		self.state_vector[3:6] += self.state_vector[9:12] * dt

	def _func_right(self, u):

		self._motor_trust[2] = (u**2 * self._trust_coef).sum()

		R = self._rotation_matrix_3d(self.state_vector[3], self.state_vector[4], self.state_vector[5])
		R.resize(3,3)
		linear_acceleration = R.transpose() @ self._motor_trust / self._mass + self._g

		self._motor_moments[0] = self._arm_length * self._trust_coef * (u[0]**2 - u[2]**2)
		self._motor_moments[1] = self._arm_length * self._trust_coef * (u[3]**2 - u[1]**2)
		self._motor_moments[2] = self._drag_coef * (u[3]**2 + u[1]**2 - u[0]**2 - u[2]**2)

		angular_acceleration = self._inertia_inv @ (self._motor_moments - np.cross((self.state_vector[9:12]), (self._inertia @ self.state_vector[9:12]), axis=0))
		return linear_acceleration, angular_acceleration



	def _rotation_matrix_3d(self, pitch, roll, yaw):
		return np.array([[np.cos(yaw)*np.cos(roll), np.sin(yaw)*np.cos(roll), -np.sin(roll)],
						  [np.cos(yaw)*np.sin(pitch)*np.sin(roll) - np.sin(yaw)*np.cos(pitch),
						  	np.sin(yaw)*np.sin(pitch)*np.sin(roll) + np.cos(yaw)*np.cos(pitch),
							np.sin(pitch)*np.cos(roll)],
						  [np.cos(yaw)*np.sin(roll)*np.cos(pitch) + np.sin(yaw)*np.sin(pitch),
							np.sin(yaw)*np.cos(pitch)*np.sin(roll) - np.cos(yaw)*np.sin(pitch),
							np.cos(pitch)*np.cos(roll)]])

		

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
		self._g = np.array([0.0, 0.0, -9.81])
		self._motor_trust = np.array([[0.0], [0.0], [0.0]])
		self._motor_moments = np.array([[0.0], [0.0], [0.0]])

		# Зададим вектор состояния, запишем начальные значения, в данном случае везде нули.
		self._state_vector = np.array([0.0,  # pose X		0
		 							   0.0,  # pose Y		1
									   0.0,  # pose Z		2
									   0.0,  # roll 		3
									   0.0,  # pitch 		4
									   0.0,  # yaw 			5
									   0.0,  # velocity X	6
									   0.0,  # velocity Y	7
									   0.0,  # velocity Z	8
									   0.0,  # roll rate 	9
									   0.0,  # pitch rate   10
									   0.0]) # yaw rate		11

	@property
	def state_vector(self):
		return self._state_vector

	def update_state(self, u, dt: float) -> None:
		# Получим линейное и угловое ускорение.
		lin_acc, ang_acc = self._func_right(u)
		# Проинтегрируем полученные приращения и обновим вектор состояния
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
		# Рассчитаем линейное ускорение
		print("u: ", u)

		sumRotorsAngularVelocity = u[0]**2 + u[1]**2 + u[2]**2 + u[3]**2
		print("sumRotorsAngularVelocity: ", sumRotorsAngularVelocity)
		
		mass_coeff = 1./self._mass
		print("mass_coeff: ", mass_coeff)

		mat_rot_3d = self._rotation_matrix_3d(self.state_vector[States.ROLL],
										   	  self.state_vector[States.PITCH],
										   	  self.state_vector[States.YAW])
		
		print("mat_rot_3d: ", mat_rot_3d)

		self._motor_trust = np.array([0.0,
									  0.0,
									  self._trust_coef*sumRotorsAngularVelocity])
		
		print("self._motor_trust: ", self._motor_trust)

		print("self._g: ", self._g)
		
		linear_acceleration = mass_coeff*mat_rot_3d@self._motor_trust + self._g
		print("linear_acceleration: ", linear_acceleration)

		# Рассчитаем моменты создаваемые двигателями в связной СК
		self._motor_moments = np.array([self._arm_length * self._trust_coef * (u[0]**2 - u[2]**2),
								  		self._arm_length * self._trust_coef * (u[3]**2 - u[1]**2),
										self._drag_coef * (u[3]**2 + u[1]**2 - u[0]**2 - u[2]**2)])
		
		print("self._motor_moments: ", self._motor_moments)
		
		angular_vel = np.array([self.state_vector[States.ROLL_RATE], 
								self.state_vector[States.PITCH_RATE], 
								self.state_vector[States.YAW_RATE]]) 

		print("angular_vel: ", angular_vel)	
				
		# Рассчитаем угловое ускорение
		angular_acceleration = self._inertia_inv @ (self._motor_moments - np.cross(angular_vel, self._inertia @ angular_vel, axis=0))

		# (self._motor_moments - angular_vel @ (self._inertia @ angular_vel))
		print("angular_acceleration: ", angular_acceleration)

		return linear_acceleration, angular_acceleration



	def _rotation_matrix_3d(self, roll, pitch, yaw):
		sr = np.sin(roll)
		cr = np.cos(roll)
		sp = np.sin(pitch)
		cp = np.cos(pitch)
		sy = np.sin(yaw)
		cy = np.cos(yaw)

		Rx = np.array([[1,  0,   0],
				 	   [0, cr, -sr],
				 	   [0, sr, cr]])
		
		Ry = np.array([[cp,  0, sp],
				 	   [0,  1,  0],
				 	   [-sp,  0, cp]])

		Rz = np.array([[cy, -sy, 0],
				 	   [sy,  cy, 0],
				 	   [0,   0, 1]])
		
		R = Rz@Ry@Rx
		return R
		
